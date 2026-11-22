#!/bin/bash
# Builds the RemoteLoadReceiver firmware once per remote unit, for grid_sim -r.
#
#   build_receivers.sh <RemoteLoadReceiver dir> <out dir> <node ID>...
#
# The RemoteLoadReceiver sources are not in this repository: take them from the 3-phase one
# (PVRouter-3-phase/RemoteLoadReceiver), whose receivers the single-phase router drives too.
# REMOTE_NODE_ID is a compile-time constant of config_rf.h: it is set for each
# build, and config_rf.h is restored afterwards.
set -euo pipefail

receiver=$(realpath "$1")
out=$(realpath -m "$2")
shift 2
config="$receiver/config_rf.h"

mkdir -p "$out"
backup=$(mktemp)
cp "$config" "$backup"
trap 'cp "$backup" "$config"; rm -f "$backup"' EXIT

for node in "$@"; do
  sed -i "s/REMOTE_NODE_ID{ [0-9]* }/REMOTE_NODE_ID{ $node }/" "$config"
  grep -q "REMOTE_NODE_ID{ $node }" "$config" || { echo "cannot set REMOTE_NODE_ID in $config" >&2; exit 1; }
  PLATFORMIO_BUILD_DIR="$receiver/.pio/sim-$node" pio run --silent --project-dir "$receiver" --environment uno
  cp "$receiver/.pio/sim-$node/uno/firmware.elf" "$out/receiver-$node.elf"
  echo "receiver for node $node: $out/receiver-$node.elf"
done
