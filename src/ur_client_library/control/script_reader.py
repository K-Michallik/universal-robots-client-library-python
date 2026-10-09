"""Restricted external-control script template reader (not a Jinja engine)."""

from __future__ import annotations

import re
from pathlib import Path, PurePosixPath, PureWindowsPath
from typing import TypeAlias

from ur_client_library.exceptions import (
    ScriptFileError,
    ScriptIncludeError,
    ScriptSyntaxError,
    UnknownVariableError,
)
from ur_client_library.helpers import parse_boolean
from ur_client_library.ur.version_information import VersionInformation

ScriptValue: TypeAlias = str | float | int | bool | VersionInformation
ScriptData: TypeAlias = dict[str, ScriptValue]

_VARIABLE = re.compile(r"\{\{\s*([\w-]+)\s*\}\}")
_INCLUDE = re.compile(r"""\{\%\s*include\s*['"]([^'"]+)['"]\s*\%\}""")
_IF = re.compile(r"\{\%\s*if\s+([^\s].*[^\s])\s+\%\}")
_ELIF = re.compile(r"\{\%\s*elif\s+([^\s].*[^\s])\s+\%\}")
_ELSE = re.compile(r"\{\%\s*else\s*\%\}")
_ENDIF = re.compile(r"\{\%\s*endif\s*\%\}")
_EXPRESSION = re.compile(
    r"""([A-Za-z_][A-Za-z0-9_]*)\s*([!=<>]=?)\s*(["']?[^'"]*["']?)"""
)
_IDENTIFIER = re.compile(r"[A-Za-z_][A-Za-z0-9_]*")
_STRING = re.compile(r"""^['"]([^'"]*)['"]$""")
_NUMBER = re.compile(r"^-?(?:\d+\.?\d*|\.\d+)(?:[eE][+-]?\d+)?$")
_BOOLEAN = re.compile(r"^(true|false|yes|no|on|off)$", re.IGNORECASE)
_VERSION = re.compile(r"^v(\d+\.\d+(?:\.\d+)?(?:-\d+)?)$")


def _is_rooted(raw: str) -> bool:
    """Return whether an include path is absolute or rooted on POSIX or Windows.

    Windows spellings such as ``\\evil.txt``, ``Z:evil.txt`` and UNC paths are
    rejected on every platform, mirroring upstream's root-name/root-directory check.
    """
    return bool(PurePosixPath(raw).anchor or PureWindowsPath(raw).anchor)


class ScriptReader:
    """Apply only URCL's variables, includes, and line-based conditionals.

    Includes are resolved relative to the including file and must stay inside
    the directory of the top-level script. Rooted or absolute include paths,
    include cycles, and chains deeper than :attr:`MAX_INCLUDE_DEPTH` raise.
    """

    MAX_INCLUDE_DEPTH = 32

    def read_script_file(
        self, file_path: str | Path, data: ScriptData | None = None
    ) -> str:
        canonical = self._resolve(Path(file_path), str(file_path), "script file path")
        return self._read(canonical, data or {}, canonical.parent, ())

    def _read(
        self,
        canonical_path: Path,
        data: ScriptData,
        root_dir: Path,
        include_stack: tuple[Path, ...],
    ) -> str:
        if canonical_path in include_stack:
            chain = " -> ".join(str(item) for item in (*include_stack, canonical_path))
            raise ScriptSyntaxError(f"Cyclic script include: {chain}")
        if len(include_stack) >= self.MAX_INCLUDE_DEPTH:
            raise ScriptSyntaxError(
                f"Maximum include depth ({self.MAX_INCLUDE_DEPTH}) exceeded while "
                f"processing script file {str(canonical_path)!r}"
            )
        try:
            script = canonical_path.read_text()
        except OSError as exc:
            raise ScriptFileError(
                f"Could not open script file {str(canonical_path)!r}; "
                "check that it exists and is readable"
            ) from exc
        script = self._replace_variables(script, data)
        script = self._replace_conditionals(script, data)
        return self._replace_includes(
            script, canonical_path.parent, data, root_dir, (*include_stack, canonical_path)
        )

    def _replace_includes(
        self,
        script: str,
        current_dir: Path,
        data: ScriptData,
        root_dir: Path,
        include_stack: tuple[Path, ...],
    ) -> str:
        while True:
            match = _INCLUDE.search(script)
            if match is None:
                return script
            raw = match.group(1)
            if _is_rooted(raw):
                raise ScriptIncludeError(
                    f"Rooted or absolute include paths are not allowed: {raw!r}"
                )
            resolved = self._resolve(current_dir / raw, raw, "include path")
            if not resolved.is_relative_to(root_dir):
                raise ScriptIncludeError(
                    f"Include path {raw!r} resolves outside of the script root directory "
                    f"{str(root_dir)!r} (resolved to {str(resolved)!r})"
                )
            included = self._read(resolved, data, root_dir, include_stack)
            script = script[: match.start()] + included + script[match.end() :]

    @staticmethod
    def _resolve(path: Path, raw: str, context: str) -> Path:
        # Non-strict resolution collapses ``..`` and follows symlinks so containment
        # checks see the real target; a missing file is reported by the read instead.
        try:
            return path.resolve()
        except (OSError, RuntimeError) as exc:
            raise ScriptFileError(f"Could not resolve {context} {raw!r}: {exc}") from exc

    @staticmethod
    def _replace_variables(script: str, data: ScriptData) -> str:
        while True:
            match = _VARIABLE.search(script)
            if match is None:
                return script
            key = match.group(1)
            if key not in data:
                raise UnknownVariableError(f"Variable {key!r} not found in data")
            replacement = ScriptReader._format_value(data[key], key=key)
            script = script[: match.start()] + replacement + script[match.end() :]

    @staticmethod
    def _format_value(value: ScriptValue, *, key: str) -> str:
        if isinstance(value, bool):
            return "True" if value else "False"
        if isinstance(value, str):
            return value
        if isinstance(value, VersionInformation):
            return value.to_string()
        if isinstance(value, int):
            return str(value)
        if isinstance(value, float):
            return f"{value:.6f}"
        raise TypeError(f"Unsupported value type for variable {key!r}")

    @staticmethod
    def _replace_conditionals(script: str, data: ScriptData) -> str:
        rendered: list[str] = []
        stack: list[dict[str, bool | str]] = []
        for line in script.splitlines():
            match = _IF.search(line)
            if match is not None:
                result = ScriptReader.evaluate_expression(match.group(1), data)
                parent_render = True if not stack else bool(stack[-1]["render"])
                stack.append(
                    {
                        "kind": "if",
                        "matched": result,
                        "render": parent_render and result,
                        "parent": parent_render,
                    }
                )
                continue
            match = _ELIF.search(line)
            if match is not None:
                if not stack:
                    raise ScriptSyntaxError("elif without matching if")
                top = stack[-1]
                if top["kind"] == "else":
                    continue
                result = ScriptReader.evaluate_expression(match.group(1), data)
                top["kind"] = "elif"
                if not bool(top["matched"]) and result:
                    top["matched"] = True
                    top["render"] = bool(top["parent"])
                else:
                    top["render"] = False
                continue
            if _ELSE.search(line) is not None:
                if not stack:
                    raise ScriptSyntaxError("else without matching if")
                top = stack[-1]
                top["kind"] = "else"
                top["render"] = bool(top["parent"]) and not bool(top["matched"])
                continue
            if _ENDIF.search(line) is not None:
                if not stack:
                    raise ScriptSyntaxError("endif without matching if")
                stack.pop()
                continue
            if not stack or bool(stack[-1]["render"]):
                rendered.append(line)
        if stack:
            raise ScriptSyntaxError("if block is missing endif")
        return "\n".join(rendered)

    @staticmethod
    def evaluate_expression(expression: str, data: ScriptData) -> bool:
        text = expression.strip()
        match = _EXPRESSION.fullmatch(text)
        if match is not None:
            variable, operator, rhs_text = match.groups()
            if variable not in data:
                raise UnknownVariableError(variable)
            rhs = ScriptReader._parse_operand(rhs_text, data)
            lhs = data[variable]
            if operator == "==":
                return ScriptReader._equal(lhs, rhs)
            if operator == "!=":
                return not ScriptReader._equal(lhs, rhs)
            if operator == "<":
                return ScriptReader._less(lhs, rhs)
            if operator == ">":
                return not (
                    ScriptReader._less(lhs, rhs) or ScriptReader._equal(lhs, rhs)
                )
            if operator == "<=":
                return ScriptReader._less(lhs, rhs) or ScriptReader._equal(lhs, rhs)
            if operator == ">=":
                return not ScriptReader._less(lhs, rhs)
            raise ScriptSyntaxError(f"Unsupported comparison operator {operator!r}")
        if _IDENTIFIER.fullmatch(text):
            if text not in data:
                raise UnknownVariableError(text)
            value = data[text]
            if isinstance(value, bool):
                return value
        raise ScriptSyntaxError(
            f"Expression {text!r} must be a boolean variable or supported comparison"
        )

    @staticmethod
    def _parse_operand(text: str, data: ScriptData) -> ScriptValue:
        match = _STRING.fullmatch(text)
        if match is not None:
            return match.group(1)
        if _NUMBER.fullmatch(text):
            return float(text)
        if _BOOLEAN.fullmatch(text):
            return parse_boolean(text)
        match = _VERSION.fullmatch(text)
        if match is not None:
            return VersionInformation.from_string(match.group(1).replace("-", "."))
        if text in data:
            return data[text]
        raise UnknownVariableError(text)

    @staticmethod
    def _equal(lhs: ScriptValue, rhs: ScriptValue) -> bool:
        if type(lhs) is type(rhs):
            return lhs == rhs
        if isinstance(lhs, bool):
            if isinstance(rhs, float):
                return abs(float(lhs) - rhs) < 0.000001
            if isinstance(rhs, int) and not isinstance(rhs, bool):
                return int(lhs) == rhs
        if (
            isinstance(lhs, (int, float))
            and not isinstance(lhs, bool)
            and isinstance(rhs, (int, float))
            and not isinstance(rhs, bool)
        ):
            return float(lhs) == float(rhs)
        raise TypeError(
            f"Equality comparison is not allowed between "
            f"{type(lhs).__name__} and {type(rhs).__name__}"
        )

    @staticmethod
    def _less(lhs: ScriptValue, rhs: ScriptValue) -> bool:
        if isinstance(lhs, VersionInformation):
            if not isinstance(rhs, VersionInformation):
                raise TypeError("Version values can only be ordered against versions")
            return lhs < rhs
        if (
            isinstance(lhs, (int, float))
            and not isinstance(lhs, bool)
            and isinstance(rhs, (int, float))
            and not isinstance(rhs, bool)
        ):
            return float(lhs) < float(rhs)
        raise TypeError("Ordering comparisons are only allowed for numeric or version values")
