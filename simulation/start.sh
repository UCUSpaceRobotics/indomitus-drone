#!/bin/bash

set -x

SCRIPT=$(readlink -f $0)
SCRIPTDIR="$(dirname "$(realpath "$SCRIPT")")"
cd $SCRIPTDIR

CONFIG="$1"

if [[ -z "$CONFIG" ]]; then
    CONFIG="$SCRIPTDIR/config.cfg"
fi

# Load config
source "$CONFIG"
: "${WORLD:=$SCRIPTDIR/worlds/default.world}"
export WORLD

# Kill previous simulation instances
kill -9 $(pgrep -f 'gz sim') 2>/dev/null

# Start tmuxinator
tmuxinator start -p "$SCRIPTDIR/tmux.yml"