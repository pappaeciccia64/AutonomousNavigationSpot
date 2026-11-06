#!/usr/bin/env bash
# ONCE: the Python environment of the mission on the CORE I/O (Jetson Xavier NX, aarch64,
# Ubuntu 20.04, Python 3.8, no internet, no pip, no Docker for the user spot).
#   1. on the laptop (internet): the aarch64 / cp38 wheels of the same versions as here, in
#      core_wheels/ (not in git);
#   2. on the CORE: ~/ans/wheels, then the venv ~/ans/venv (system Python 3.8, its own
#      packages: the old numpy 1.17 of the system and conda are not touched) with pip run
#      from its wheel.
# It writes only under ~/ans on the CORE. Run again: it reuses what is there.
#     tools/core_setup.sh
set -euo pipefail
source "$(dirname "$0")/core_common.sh"

PKGS=(numpy==1.24.4 scipy==1.10.1 bosdyn-client==5.1.4 bosdyn-api==5.1.4 bosdyn-core==5.1.4
      protobuf==5.29.6 grpcio==1.70.0 matplotlib==3.7.5)
PIP_WHEEL=pip-25.0.1-py3-none-any.whl
WHEELS="$REPO/core_wheels"

echo "== 1/3 wheels for aarch64 / Python 3.8 in $WHEELS"
mkdir -p "$WHEELS"
python3 -m pip download --only-binary=:all: --platform manylinux2014_aarch64 \
    --platform manylinux_2_17_aarch64 --platform manylinux_2_28_aarch64 \
    --python-version 38 --implementation cp --abi cp38 -d "$WHEELS" pip==25.0.1 "${PKGS[@]}" -q

echo "== 2/3 copy to $DEST:~/$CORE_DIR/wheels"
"${SSH[@]}" "$DEST" "mkdir -p ~/$CORE_DIR/wheels ~/$CORE_DIR/code"
rsync -a --info=stats1 -e "$RSYNC_SSH" "$WHEELS/" "$DEST:$CORE_DIR/wheels/"

echo "== 3/3 venv ~/$CORE_DIR/venv and packages (offline)"
"${SSH[@]}" "$DEST" bash -s -- "$CORE_DIR" "$PIP_WHEEL" "${PKGS[@]}" <<'EOF'
set -euo pipefail
dir="$HOME/$1"; pipw="$2"; shift 2
[ -x "$dir/venv/bin/python" ] || python3 -m venv --without-pip "$dir/venv"
"$dir/venv/bin/python" "$dir/wheels/$pipw/pip" install -q --no-index --find-links "$dir/wheels" pip
"$dir/venv/bin/python" -m pip install -q --no-index --find-links "$dir/wheels" "$@"
"$dir/venv/bin/python" -c "import numpy, scipy, bosdyn.client, importlib.metadata as m; print('OK: numpy', numpy.__version__, '| scipy', scipy.__version__, '| bosdyn', m.version('bosdyn-client'))"
EOF
echo "Done. Next: tools/core_deploy.sh"
