#!/usr/bin/env bash
# Race telemetry launcher for an existing aichallenge-racingkart installation.
# Independent from the official challenge repository.

set -u
set -o pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"

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
CONTAINER_LOG="/tmp/aichallenge_race_telemetry.log"
HOST_OUT="${SCRIPT_DIR}/runs"

docker exec "${CID}" bash -lc "
    rm -rf '${CONTAINER_OUT}'
    mkdir -p '${CONTAINER_OUT}'
    rm -f '${CONTAINER_PID}' '${CONTAINER_LOG}'
"
docker cp "${SCRIPT_DIR}/race_telemetry.py" "${CID}:${CONTAINER_SCRIPT}" >/dev/null

# Launch detached from the host terminal. This is intentional:
# Ctrl+C should stop the recorder inside the container, not kill docker exec
# before the HTML report has finished being written.
docker exec -d "${CID}" bash -lc "
    source /opt/ros/humble/setup.bash
    source /aichallenge/workspace/install/setup.bash
    export ROS_DOMAIN_ID='${ROS_DOMAIN_ID_VALUE}'
    echo \$\$ > '${CONTAINER_PID}'
    exec python3 '${CONTAINER_SCRIPT}' \
        --base-speed '${BASE_SPEED}' \
        --sample-hz '${SAMPLE_HZ}' \
        --out '${CONTAINER_OUT}' \
        > '${CONTAINER_LOG}' 2>&1
"

# Wait briefly for PID file.
for _ in $(seq 1 50); do
    if docker exec "${CID}" test -s "${CONTAINER_PID}" 2>/dev/null; then
        break
    fi
    sleep 0.1
done

echo "[telemetry] Recorder started."
echo "[telemetry] Run the simulation normally."
echo "[telemetry] When the run is finished, press Ctrl+C HERE to save the report."
echo

STOP_REQUESTED=0

recorder_alive() {
    docker exec "${CID}" bash -lc "
        test -s '${CONTAINER_PID}' &&
        kill -0 \"\$(cat '${CONTAINER_PID}')\" 2>/dev/null
    " >/dev/null 2>&1
}

stop_recorder() {
    if [[ "${STOP_REQUESTED}" -eq 1 ]]; then
        return
    fi
    STOP_REQUESTED=1

    # Ignore repeated Ctrl+C while we let Python finish CSV + HTML generation.
    trap '' INT TERM

    echo
    echo "[telemetry] Stopping recorder cleanly..."
    docker exec "${CID}" bash -lc "
        if test -s '${CONTAINER_PID}'; then
            kill -INT \"\$(cat '${CONTAINER_PID}')\" 2>/dev/null || true
        fi
    " >/dev/null 2>&1 || true

    echo "[telemetry] Waiting for CSV/HTML generation to finish..."

    # Give Python up to 20 seconds to finish report generation.
    for _ in $(seq 1 200); do
        if ! recorder_alive; then
            return
        fi
        sleep 0.1
    done

    echo "[telemetry] Recorder did not exit within 20 s; forcing termination."
    docker exec "${CID}" bash -lc "
        if test -s '${CONTAINER_PID}'; then
            kill -TERM \"\$(cat '${CONTAINER_PID}')\" 2>/dev/null || true
        fi
    " >/dev/null 2>&1 || true
}

trap stop_recorder INT TERM

# Keep this host process alive while the detached recorder runs.
while recorder_alive; do
    sleep 0.25
done

trap - INT TERM

echo
if docker exec "${CID}" test -f "${CONTAINER_LOG}" 2>/dev/null; then
    docker exec "${CID}" cat "${CONTAINER_LOG}" 2>/dev/null || true
fi

mkdir -p "${HOST_OUT}"

if docker inspect "${CID}" >/dev/null 2>&1; then
    if docker cp "${CID}:${CONTAINER_OUT}/." "${HOST_OUT}/" >/dev/null 2>&1; then
        LATEST="$(find "${HOST_OUT}" -mindepth 1 -maxdepth 1 -type d -name 'run_*' | sort | tail -n 1)"
        echo
        echo "[telemetry] Report copied to this tools repository."
        if [[ -n "${LATEST}" ]]; then
            echo "  ${LATEST}"
            [[ -f "${LATEST}/telemetry.csv" ]] && echo "  CSV : ${LATEST}/telemetry.csv"
            [[ -f "${LATEST}/telemetry.html" ]] && echo "  HTML: ${LATEST}/telemetry.html"

            if [[ ! -f "${LATEST}/telemetry.html" ]]; then
                echo "[telemetry] WARNING: telemetry.html was not generated."
                exit 1
            fi
        fi
    else
        echo "[telemetry] Could not copy the report from the container."
        exit 1
    fi
else
    echo "[telemetry] Autoware container disappeared before the report could be copied."
    exit 1
fi
