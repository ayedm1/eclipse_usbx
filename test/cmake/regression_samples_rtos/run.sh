#!/bin/bash

set -euo pipefail

cd "$(dirname "$0")"

# Checkout externals (idempotent)
EXTERNALS_DIR="../../externals"
THREADX_DIR="$EXTERNALS_DIR/threadx"

mkdir -p "$EXTERNALS_DIR"

if [ ! -d "$THREADX_DIR/.git" ]; then
  git clone --depth 1 https://github.com/eclipse-threadx/threadx.git "$THREADX_DIR"
fi


# Add junit output for ctest generation
BOOTSTRAP_SH="$THREADX_DIR/scripts/cmake_bootstrap.sh"

if ! grep -q "\-\-output\-junit \$1.xml" "$BOOTSTRAP_SH"; then
  sed -i 's/ctest $parallel --timeout 1000 -O $1.txt/& --output-junit $1.xml/g' "$BOOTSTRAP_SH"
fi

[ -f .run.sh ] || ln -sf "$BOOTSTRAP_SH" .run.sh
./.run.sh $*