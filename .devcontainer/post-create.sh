#!/usr/bin/env bash
# One-time setup for the dev container. Safe to run again.
# The first run takes a few minutes (cargo-deny and cargo-audit compile from source).
# Not included: the Xtensa toolchain for firmware/heltec-v3 (see its README).
set -euo pipefail
cd "$(dirname "$0")/.."

rustup component add rustfmt clippy
for tool in cargo-deny cargo-audit; do
  command -v "$tool" >/dev/null || cargo install --locked "$tool"
done

# Python venv for the T-RF3 array prototype in research/t-rf3-array.
python3 -m venv .venv
.venv/bin/pip install --quiet --upgrade pip
.venv/bin/pip install --quiet -r research/t-rf3-array/requirements.txt

# The oasa.toml tool: python3 .oasa/oasa_toml.py run --markdown
mkdir -p .oasa
if ! curl -fsSL https://raw.githubusercontent.com/joinoasa/website/main/tools/oasa-files/oasa_toml.py -o .oasa/oasa_toml.py; then
  echo "warning: could not download oasa_toml.py; get it later (see https://joinoasa.org/docs/oasa-files/#the-tool)" >&2
fi

cargo fetch --locked
echo "Ready. Run the checks: python3 .oasa/oasa_toml.py run --markdown"
echo "serial-e2e also needs oasa-sec built next to this repo (see oasa.toml)."
