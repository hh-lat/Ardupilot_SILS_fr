#!/usr/bin/env bash
# ===========================================================================
# mc_watch.sh — wait for a Monte Carlo campaign to finish, then archive it
#               (optionally upload to S3, optionally power the box off) so you
#               can disconnect / shut the AWS instance down safely.
#
# Usage:
#   ./mc_watch.sh [MC_DIR] [s3://bucket/prefix] [--shutdown]
#       MC_DIR      campaign folder (default: newest mc_* beside this script)
#       s3://...    optional: upload the .tar.gz there (needs awscli + creds)
#       --shutdown  optional: `sudo shutdown -h now` ~60 s after a CLEAN finish
#
# Run it DETACHED so it survives your SSH logout:
#   cd Tools/autotest/monte_carlo
#   nohup ./mc_watch.sh "" s3://my-bucket/mc --shutdown > mc_watch.log 2>&1 &
#   disown
#   tail -f mc_watch.log        # (optional) watch it; Ctrl-C just stops tailing
#
# Safety: it will NOT shut down if the runner died early (no DONE.txt) or if an
# S3 upload fails — so you never power off with un-saved data.
# ===========================================================================
set -uo pipefail

HERE="$(cd "$(dirname "$0")" && pwd)"
MC_DIR=""; S3_DEST=""; DO_SHUTDOWN=0
for a in "$@"; do
    case "$a" in
        s3://*)     S3_DEST="$a" ;;
        --shutdown) DO_SHUTDOWN=1 ;;
        "")         : ;;                       # allow empty positional placeholder
        *)          MC_DIR="$a" ;;
    esac
done

# Default to the newest mc_<datetime> directory next to this script.
# (Trailing slash in the glob matches directories only — so it never picks up
#  mc_watch.sh / mc_watch.log; the [0-9] further restricts to mc_<timestamp>.)
if [ -z "$MC_DIR" ]; then
    MC_DIR="$(ls -dt "$HERE"/mc_[0-9]*/ 2>/dev/null | head -1 || true)"
fi
if [ -z "$MC_DIR" ] || [ ! -d "$MC_DIR" ]; then
    echo "[mc_watch] ERROR: no campaign directory found (looked for $HERE/mc_[0-9]*/)."
    echo "[mc_watch]        pass it explicitly:  ./mc_watch.sh /path/to/mc_<ts>"
    exit 1
fi
MC_DIR="${MC_DIR%/}"
echo "[mc_watch] $(date '+%F %T')  watching: $MC_DIR"
echo "[mc_watch]   S3 dest : ${S3_DEST:-<none>}"
echo "[mc_watch]   shutdown: $([ "$DO_SHUTDOWN" = 1 ] && echo yes || echo no)"

# ---- Wait until DONE.txt appears, or the runner process disappears ----------
clean_finish=0
while true; do
    if [ -f "$MC_DIR/DONE.txt" ]; then
        clean_finish=1
        break
    fi
    if ! pgrep -f "aws_monte_carlo.py" >/dev/null 2>&1; then
        echo "[mc_watch] $(date '+%F %T')  WARNING: runner not running and no DONE.txt."
        echo "[mc_watch]   Archiving PARTIAL results; will NOT shut down."
        DO_SHUTDOWN=0
        break
    fi
    done_n=$(find "$MC_DIR" -name result.json 2>/dev/null | wc -l)
    echo "[mc_watch] $(date '+%F %T')  still running — $done_n cases finished so far"
    sleep 60
done

if [ "$clean_finish" = 1 ]; then
    echo "[mc_watch] $(date '+%F %T')  campaign complete:"
    sed 's/^/[mc_watch]   /' "$MC_DIR/DONE.txt"
fi

# ---- Archive ----------------------------------------------------------------
ARCHIVE="${MC_DIR}_$(date '+%Y%m%d_%H%M%S').tar.gz"
echo "[mc_watch] archiving -> $ARCHIVE"
if tar -czf "$ARCHIVE" -C "$(dirname "$MC_DIR")" "$(basename "$MC_DIR")"; then
    echo "[mc_watch] archive OK ($(du -h "$ARCHIVE" | cut -f1))"
else
    echo "[mc_watch] ERROR: archive failed — NOT shutting down."
    exit 1
fi

# ---- Optional upload to S3 --------------------------------------------------
if [ -n "$S3_DEST" ]; then
    echo "[mc_watch] uploading to $S3_DEST/ ..."
    if command -v aws >/dev/null 2>&1 && aws s3 cp "$ARCHIVE" "$S3_DEST/"; then
        echo "[mc_watch] upload OK"
    else
        echo "[mc_watch] ERROR: S3 upload failed — keeping local archive, NOT shutting down."
        DO_SHUTDOWN=0
    fi
fi

# ---- Optional shutdown ------------------------------------------------------
if [ "$DO_SHUTDOWN" = 1 ]; then
    echo "[mc_watch] $(date '+%F %T')  powering off in 60 s — abort with: sudo shutdown -c"
    sleep 60
    sudo shutdown -h now
fi
echo "[mc_watch] $(date '+%F %T')  finished."
