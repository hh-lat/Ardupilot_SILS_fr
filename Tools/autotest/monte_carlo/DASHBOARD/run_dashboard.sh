#!/usr/bin/env bash
# ---------------------------------------------------------------------------
# run_dashboard.sh — view ANY Monte Carlo campaign in the dashboard.
#
# Rebuilds the dashboard data files from the campaign folder you give it, then
# (re)launches the Streamlit dashboard on http://localhost:<port>.
#
# Usage:
#   ./run_dashboard.sh <campaign_dir> [port]
#
# The <campaign_dir> may be a Linux path OR a pasted Windows / WSL path:
#   ./run_dashboard.sh /home/lat-chinmay/.../mc_ustol_25k
#   ./run_dashboard.sh 'C:\Users\ChinmayBorker\Documents\MONTE_CARLO_USTOL\mc_20260618_171051'
#   ./run_dashboard.sh '\\wsl.localhost\Ubuntu-24.04\home\lat-chinmay\...\mc_ustol_25k'
#
# Default port is 8501. The campaign folder must contain case_* subdirs, each
# with a result.json (new format) or a top-level summary.csv (old format).
# ---------------------------------------------------------------------------
set -euo pipefail

DASH_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PORT="${2:-8501}"
RAW="${1:-}"

if [[ -z "$RAW" ]]; then
    echo "Usage: $0 <campaign_dir> [port]"
    exit 1
fi

# --- normalise a pasted Windows / WSL UNC path to a Linux path -------------
norm_path() {
    local p="${1//\\//}"                              # backslashes -> forward slashes
    if [[ "$p" =~ ^//wsl[^/]*/[^/]+/(.*)$ ]]; then    # \\wsl.localhost\Distro\home\... -> /home/...
        p="/${BASH_REMATCH[1]}"
    elif [[ "$p" =~ ^([A-Za-z]):/(.*)$ ]]; then       # C:\Users\... -> /mnt/c/Users/...
        local drive="${BASH_REMATCH[1],,}"
        p="/mnt/${drive}/${BASH_REMATCH[2]}"
    fi
    echo "${p%/}"                                     # strip any trailing slash
}

CAMPAIGN="$(norm_path "$RAW")"

if [[ ! -d "$CAMPAIGN" ]]; then
    echo "ERROR: campaign dir not found: $CAMPAIGN"
    echo "       (from input: $RAW)"
    exit 1
fi
if ! ls -d "$CAMPAIGN"/case_* >/dev/null 2>&1; then
    echo "ERROR: no case_* folders under $CAMPAIGN"
    exit 1
fi

echo ">> Campaign : $CAMPAIGN"
echo ">> Cases    : $(ls -d "$CAMPAIGN"/case_* | wc -l)"
echo ">> Building dashboard_data.csv / .xlsx / timeseries.parquet / oscillation.parquet ..."
cd "$DASH_DIR"
python3 build_dashboard_data.py "$CAMPAIGN"

# --- (re)launch Streamlit on $PORT (bound to localhost only) ---------------
echo ">> (Re)starting Streamlit on port $PORT ..."
pids="$(ss -ltnp 2>/dev/null | grep ":$PORT " | grep -o 'pid=[0-9]*' | cut -d= -f2 | sort -u || true)"
if [[ -n "$pids" ]]; then kill $pids 2>/dev/null || true; fi
for _ in $(seq 1 15); do ss -ltn 2>/dev/null | grep -q ":$PORT " || break; sleep 1; done

nohup streamlit run app.py --server.headless true --server.port "$PORT" \
      --server.address 127.0.0.1 \
      --browser.gatherUsageStats false > "$DASH_DIR/streamlit.log" 2>&1 &
disown

for _ in $(seq 1 30); do
    code="$(curl -s -o /dev/null -w '%{http_code}' "http://localhost:$PORT/_stcore/health" 2>/dev/null || true)"
    [[ "$code" == "200" ]] && break
    sleep 1
done
echo ">> Dashboard live:  http://localhost:$PORT   (log: $DASH_DIR/streamlit.log)"
