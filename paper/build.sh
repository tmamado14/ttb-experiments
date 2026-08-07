#!/usr/bin/env bash
# Build the paper. Usage: ./build.sh
set -euo pipefail
cd "$(dirname "$0")"
latexmk -pdf -interaction=nonstopmode -halt-on-error icra_seek.tex
echo "built: $(pwd)/icra_seek.pdf"
