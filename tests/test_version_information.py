import pytest

from ur_client_library.exceptions import ProtocolError
from ur_client_library.ur.version_information import VersionInformation


@pytest.mark.parametrize(
    ("source", "expected"),
    [
        ("5.12", VersionInformation(5, 12, 0, 0)),
        ("5.12.1", VersionInformation(5, 12, 1, 0)),
        ("5.12.1.1234", VersionInformation(5, 12, 1, 1234)),
    ],
)
def test_parse_version(source: str, expected: VersionInformation) -> None:
    assert VersionInformation.from_string(source) == expected


@pytest.mark.parametrize("source", ["", "1", "1.2.3.4.5", "1.two.3", "1..3"])
def test_reject_invalid_version(source: str) -> None:
    with pytest.raises(ProtocolError):
        VersionInformation.from_string(source)


def test_version_order_and_properties() -> None:
    older = VersionInformation.from_string("5.5.0.1101318")
    newer = VersionInformation.from_string("5.5.0.1101319")
    assert older < newer
    assert newer.is_e_series
    assert str(newer) == "5.5.0.1101319"
    assert VersionInformation().is_empty
