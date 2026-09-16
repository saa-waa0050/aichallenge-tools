#!/usr/bin/env bash
# Race telemetry launcher for an existing aichallenge-racingkart installation.
# This repository is intentionally independent from the official challenge repo.

set -u
set -o pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"

# Usage:
#   bash run.sh
#   bash run.sh /path/to/aichallenge-racingkart
#
# Override with environment variables if desired:
#   AICHALLENGE_REPO=/path/to/repo
#   TELEMETRY_BASE_SPEED=35
#   TELEMETRY_SAMPLE_HZ=20
#   TELEMETRY_ROS_DOMAIN_ID=1

AICHALLENGE_DIR="${1:-${AICHALLENGE_REPO:-$HOME/aichallenge-racingkart}}"
BASE_SPEED="${TELEMETRY_BASE_SPEED:-32}"
SAMPLE_HZ="${TELEMETRY_SAMPLE_HZ:-20}"
ROS_DOMAIN_ID_VALUE="${TELEMETRY_ROS_DOMAIN_ID:-1}"

if [[ ! -d "${AICHALLENGE_DIR}" ]]; then
    echo "[telemetry] AI Challenge repository was not found:"
    echo "  ${AICHALLENGE_DIR}"
    echo
    echo "Run with an explicit path, for example:"
    echo "  bash ${SCRIPT_DIR}/run.sh ~/aichallenge-racingkart"
    exit 1
fi

if [[ ! -f "${AICHALLENGE_DIR}/docker-compose.yml" ]]; then
    echo "[telemetry] docker-compose.yml was not found in:"
    echo "  ${AICHALLENGE_DIR}"
    exit 1
fi

if ! command -v docker >/dev/null 2>&1; then
    echo "[telemetry] docker command was not found."
    exit 1
fi

if ! docker compose version >/dev/null 2>&1; then
    echo "[telemetry] docker compose is not available."
    exit 1
fi

cd "${AICHALLENGE_DIR}"

echo "========================================"
echo " Race Telemetry"
echo "========================================"
echo "AI Challenge repo : ${AICHALLENGE_DIR}"
echo "Baseline speed    : ${BASE_SPEED} km/h"
echo "Sample rate       : ${SAMPLE_HZ} Hz"
echo "ROS_DOMAIN_ID     : ${ROS_DOMAIN_ID_VALUE}"
echo
echo "Waiting for the autoware container..."
echo "Start the simulator normally in another terminal:"
echo
echo "  cd ${AICHALLENGE_DIR}"
echo "  make dev"
echo

CID=""
while [[ -z "${CID}" ]]; do
    CID="$(docker compose ps --status running -q autoware 2>/dev/null || true)"
    [[ -n "${CID}" ]] || sleep 0.25
done

echo "[telemetry] Autoware detected: ${CID:0:12}"
echo "[telemetry] Preparing recorder..."

CONTAINER_SCRIPT="/tmp/aichallenge_race_telemetry.py"
CONTAINER_OUT="/tmp/aichallenge_telemetry_runs"
CONTAINER_PID="/tmp/aichallenge_race_telemetry.pid"
HOST_OUT="${SCRIPT_DIR}/runs"

docker exec "${CID}" bash -lc "rm -rf '${CONTAINER_OUT}'; mkdir -p '${CONTAINER_OUT}'"
docker cp "${SCRIPT_DIR}/race_telemetry.py" "${CID}:${CONTAINER_SCRIPT}" >/dev/null

echo "[telemetry] Recorder started."
echo "[telemetry] Run the simulation normally."
echo "[telemetry] When the run is finished, press Ctrl+C HERE to save the report."
echo

# Start the recorder in the container. The shell writes its PID and then execs
# Python, so the PID file becomes the Python PID.
docker exec "${CID}" bash -lc "
    set -e
    source /opt/ros/humble/setup.bash
    source /aichallenge/workspace/install/setup.bash
    export ROS_DOMAIN_ID='${ROS_DOMAIN_ID_VALUE}'
    echo \$\$ > '${CONTAINER_PID}'
    exec python3 '${CONTAINER_SCRIPT}' \
        --base-speed '${BASE_SPEED}' \
        --sample-hz '${SAMPLE_HZ}' \
        --out '${CONTAINER_OUT}'
" &
EXEC_PID=$!

stop_recorder() {
    echo
    echo "[telemetry] Stopping recorder cleanly..."
    docker exec "${CID}" bash -lc "
        if [[ -f '${CONTAINER_PID}' ]]; then
            kill -INT \"\$(cat '${CONTAINER_PID}')\" 2>/dev/null || true
        fi
    " >/dev/null 2>&1 || true
}

trap stop_recorder INT TERM

wait "${EXEC_PID}"
EXEC_RC=$?

trap - INT TERM

mkdir -p "${HOST_OUT}"

if docker inspect "${CID}" >/dev/null 2>&1; then
    if docker cp "${CID}:${CONTAINER_OUT}/." "${HOST_OUT}/" >/dev/null 2>&1; then
        LATEST="$(find "${HOST_OUT}" -mindepth 1 -maxdepth 1 -type d -name 'run_*' | sort | tail -n 1)"
        echo
        echo "[telemetry] Report copied to this tools repository."
        if [[ -n "${LATEST}" ]]; then
            echo "  ${LATEST}"
            if [[ -f "${LATEST}/telemetry.html" ]]; then
                echo "  HTML: ${LATEST}/telemetry.html"
            fi
        fi
    else
        echo "[telemetry] Could not copy the report from the container."
    fi
else
    echo "[telemetry] Autoware container disappeared before the report could be copied."
fi

# Ctrl+C commonly makes docker exec return 130 even though the recorder saved
# correctly. Treat that as a normal interactive stop.
if [[ "${EXEC_RC}" -eq 130 ]]; then
    exit 0
fi
exit "${EXEC_RC}"
