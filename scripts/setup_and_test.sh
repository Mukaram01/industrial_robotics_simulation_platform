#!/usr/bin/env bash

# Install dependencies and run code style checks and tests.

set -euo pipefail

# Determine repository root directory
repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$repo_root"

python3 -m pip install --upgrade pip
python3 -m pip install -r requirements.txt

flake8 src tests
pytest

