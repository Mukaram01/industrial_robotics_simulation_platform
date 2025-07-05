#!/bin/bash

set -e

WORKSPACE_ROOT="$(pwd)"
SRC_DIR="$WORKSPACE_ROOT/src"

echo "Searching for Python ROS 2 packages in $SRC_DIR..."

for setup_py in $(find "$SRC_DIR" -mindepth 2 -maxdepth 3 -name 'setup.py'); do
    PKG_DIR="$(dirname "$setup_py")"
    PKG_NAME="$(basename "$PKG_DIR")"
    SETUP_CFG="$PKG_DIR/setup.cfg"

    if [[ -f "$SETUP_CFG" ]]; then
        echo "  [$PKG_NAME] setup.cfg already exists, skipping."
        continue
    fi

    echo "  [$PKG_NAME] Creating $SETUP_CFG ..."
    cat <<EOF_INNER > "$SETUP_CFG"
[develop]
script_dir=\$base/lib/$PKG_NAME
[install]
install_scripts=\$base/lib/$PKG_NAME
EOF_INNER

done

echo "All done!"
