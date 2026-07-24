#!/usr/bin/env bash
#
# Minimal Docker URSim launcher for the Python example matrix. Its model,
# image, network, and mount conventions follow the upstream URCL launcher.

set -euo pipefail

model="ur5e"
version="5.9.4"
container_name="ursim"
ip_address="192.168.56.101"
program_folder=""
replace_existing=false
dry_run=false

usage() {
  echo "Usage: $0 [-m model] [-v version] [-n container] [-i ip] [-p program-folder] [-r] [-t]"
}

while getopts ":hm:v:n:i:p:rt" option; do
  case "${option}" in
    h)
      usage
      exit 0
      ;;
    m) model="${OPTARG}" ;;
    v) version="${OPTARG}" ;;
    n) container_name="${OPTARG}" ;;
    i) ip_address="${OPTARG}" ;;
    p) program_folder="${OPTARG}" ;;
    r) replace_existing=true ;;
    t) dry_run=true ;;
    :)
      echo "Option -${OPTARG} requires an argument." >&2
      exit 2
      ;;
    \?)
      echo "Unknown option: -${OPTARG}" >&2
      usage >&2
      exit 2
      ;;
  esac
done

if [[ "${version}" == 3.* ]]; then
  series="cb3"
elif [[ "${version}" == 10.* ]]; then
  series="polyscopex"
else
  series="e-series"
fi

case "${model}" in
  ur3|ur3e|ur5|ur5e|ur7e|ur8long|ur10|ur10e|ur12e|ur15|ur16e|ur18|ur20|ur30) ;;
  *)
    echo "Unsupported robot model: ${model}" >&2
    exit 2
    ;;
esac

case "${series}" in
  cb3)
    if [[ "${model}" != "ur3" && "${model}" != "ur5" && "${model}" != "ur10" ]]; then
      echo "CB3 URSim requires one of: ur3, ur5, ur10." >&2
      exit 2
    fi
    minimum_version="3.14.3"
    ;;
  e-series)
    if [[ "${model}" == "ur3" || "${model}" == "ur5" || "${model}" == "ur10" ]]; then
      echo "e-Series URSim requires an e-Series model name such as ur5e." >&2
      exit 2
    fi
    minimum_version="5.9.4"
    case "${model}" in
      ur20) minimum_version="5.14.0" ;;
      ur30) minimum_version="5.15.0" ;;
      ur15) minimum_version="5.22.0" ;;
      ur7e|ur12e) minimum_version="5.22.0" ;;
      ur8long|ur18) minimum_version="5.23.0" ;;
    esac
    ;;
  polyscopex)
    if [[ "${model}" == "ur3" || "${model}" == "ur5" || "${model}" == "ur10" ]]; then
      echo "PolyScope X URSim requires an e-Series model name such as ur5e." >&2
      exit 2
    fi
    minimum_version="10.7.0"
    case "${model}" in
      ur15) minimum_version="10.8.0" ;;
      ur7e|ur12e) minimum_version="10.9.0" ;;
      ur8long|ur18) minimum_version="10.11.0" ;;
    esac
    ;;
esac

if [[ "${version}" != "latest" ]]; then
  earliest="$(printf '%s\n%s\n' "${minimum_version}" "${version}" | sort -V | head -n 1)"
  if [[ "${earliest}" != "${minimum_version}" ]]; then
    echo "${series} ${model} requires URSim ${minimum_version} or newer; got ${version}." >&2
    exit 2
  fi
fi

if [[ -z "${program_folder}" ]]; then
  script_directory="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
  program_folder="${script_directory}/../tests/resources/dockerursim/programs/${series}"
fi
program_folder="$(realpath "${program_folder}")"
mkdir -p "${program_folder}"

image="universalrobots/ursim_${series}:${version}"
upper_model="${model^^}"
controller_model="${upper_model}"
case "${model}" in
  ur3e|ur5e|ur10e|ur16e)
    controller_model="${upper_model%E}"
    ;;
  ur7e|ur12e)
    if [[ "${series}" == "polyscopex" ]]; then
      controller_model="${upper_model%E}e"
    else
      controller_model="${upper_model%E}"
    fi
    ;;
esac
docker_args=(
  run --rm -d
  --network ursim_net
  --ip "${ip_address}"
  --name "${container_name}"
)

if [[ "${series}" == "polyscopex" ]]; then
  docker_args+=(
    --privileged
    -v "${program_folder}:/ur/bin/backend/applications"
    -e "ROBOT_TYPE=${controller_model}"
  )
  if [[ "${version}" == "10.7.0" ]]; then
    docker_args+=(-e "ROBOT_TYPE_CONTROLLER=${controller_model}")
  fi
else
  docker_args+=(
    -v "${program_folder}:/ursim/programs"
    -e "ROBOT_MODEL=${controller_model}"
  )
fi

if [[ "${dry_run}" == true ]]; then
  printf 'docker'
  printf ' %q' "${docker_args[@]}" "${image}"
  printf '\n'
  exit 0
fi

if ! docker network inspect ursim_net >/dev/null 2>&1; then
  docker network create --subnet=192.168.56.0/24 ursim_net >/dev/null
fi

if docker container inspect "${container_name}" >/dev/null 2>&1; then
  if [[ "${replace_existing}" == true ]]; then
    docker rm -f "${container_name}" >/dev/null
  else
    echo "Container ${container_name} already exists; use -r to replace it explicitly." >&2
    exit 2
  fi
fi

started=false
cleanup_failed_start() {
  status=$?
  trap - EXIT
  if [[ ${status} -ne 0 && "${started}" == true ]]; then
    docker rm -f "${container_name}" >/dev/null 2>&1 || true
  fi
  exit "${status}"
}
trap cleanup_failed_start EXIT

echo "Starting ${image} as ${container_name} (${model} at ${ip_address})"
docker "${docker_args[@]}" "${image}" >/dev/null
started=true

deadline=$((SECONDS + 180))
until python3 - "${ip_address}" <<'PY'
import socket
import sys

try:
    with socket.create_connection((sys.argv[1], 30001), timeout=1.0):
        pass
except OSError:
    raise SystemExit(1)
PY
do
  if (( SECONDS >= deadline )); then
    echo "URSim did not expose the Primary interface within 180 seconds." >&2
    docker logs "${container_name}" >&2 || true
    exit 1
  fi
  sleep 2
done

echo "Primary port is open; waiting 20 seconds for controller stabilization."
sleep 20

trap - EXIT
echo "URSim Primary interface is ready."
