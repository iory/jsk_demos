#!/usr/bin/env bash
# Teleop bags -> fine-tuned picking policy -> running it on the right arm.
#
#   kpick.sh check <bag>                 # are the topics the server needs in it?
#   kpick.sh train [-e N] [-s steps] [-n note] [bags or dir ...]
#                                        # default: every bag in ~/k_arm_bags
#   kpick.sh run [result_dir]            # preview: nothing goes to the robot
#   kpick.sh run [result_dir] execute    # really move the arm (teleop on STOP)
#
# train wraps the server's client/kpick_train.sh (copied from the server to
# ~/kpick_train.sh each time) with -a right and config/kpick_right.yaml, and
# leaves the result in ~/kpick_results/<job>/result. run starts
# launch/kpick_right.launch with the newest result unless one is given.
#
# Environment: KPICK_HOST (default supernova), KPICK_REPO (server repo,
# default /mnt/workspace/tree-vla-il), KPICK_BAGS (default ~/k_arm_bags).
set -euo pipefail

HOST="${KPICK_HOST:-supernova}"
REPO="${KPICK_REPO:-/mnt/workspace/tree-vla-il}"
BAGS="${KPICK_BAGS:-$HOME/k_arm_bags}"
RESULTS="$HOME/kpick_results"
HERE="$(cd "$(dirname "$(readlink -f "$0")")/.." && pwd)"
CONFIG="$HERE/config/kpick_right.yaml"
TRAIN="$HOME/kpick_train.sh"

usage() { sed -n '2,12p' "$0"; exit 2; }
[ $# -gt 0 ] || usage
cmd="$1"; shift

case "$cmd" in
  check)
    [ $# -eq 1 ] && [ -f "$1" ] || usage
    tmp="/tmp/kpick_check_$(hostname -s)"
    ssh "$HOST" "mkdir -p '$tmp'"
    rsync -a --partial "$1" "$CONFIG" "$HOST:$tmp/"
    ssh "$HOST" "cd '$REPO' && uv run --extra real python scripts/bag_to_episodes.py --check \
      --config '$tmp/$(basename "$CONFIG")' '$tmp/$(basename "$1")'"
    ;;
  train)
    # the client follows the server's repo: take its current copy every time
    scp -q "$HOST:$REPO/client/kpick_train.sh" "$TRAIN"
    chmod +x "$TRAIN"
    opts=()
    while getopts "e:s:n:" o; do
      case "$o" in e|s|n) opts+=("-$o" "$OPTARG") ;; *) usage ;; esac
    done
    shift $((OPTIND - 1))
    [ $# -gt 0 ] || set -- "$BAGS"
    bags=()
    for b in "$@"; do bags+=("$(readlink -f "$b")"); done
    mkdir -p "$RESULTS"
    # kpick_train.sh leaves its result under ./kpick_results
    cd "$HOME"
    "$TRAIN" -H "$HOST" -r "$REPO" -a right -c "$CONFIG" "${opts[@]}" "${bags[@]}"
    latest="$(ls -td "$RESULTS"/*/result 2>/dev/null | head -1)"
    echo "[kpick.sh] preview with: $(basename "$0") run $latest"
    ;;
  run)
    result=""
    execute=false
    for a in "$@"; do
      if [ "$a" = execute ]; then execute=true; else result="$a"; fi
    done
    if [ -z "$result" ]; then
      result="$(ls -td "$RESULTS"/*/result 2>/dev/null | head -1)"
      [ -n "$result" ] || { echo "no result in $RESULTS; run train first" >&2; exit 1; }
    fi
    [ -d "$result" ] || { echo "not a directory: $result" >&2; exit 1; }
    echo "[kpick.sh] $result (execute=$execute)"
    exec roslaunch k_arm_teleop kpick_right.launch result:="$result" execute:="$execute"
    ;;
  *) usage ;;
esac
