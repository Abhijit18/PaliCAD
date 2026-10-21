#!/usr/bin/env bash
#
# Builds the native `pali_ijo` executable.
#
# This is what makes the kernel reachable from any language: a program in
# Python, Rust, C or a shell script can run this binary and pipe solid JSON
# through it. No FFI, no bindings to keep in step with the Dart API.
#
# Usage: tool/build_cli.sh [output-directory]

set -euo pipefail

cd "$(dirname "$0")/.."

out_dir="${1:-build}"
mkdir -p "$out_dir"

exe="$out_dir/pali_ijo"
# Windows will not run it without the extension.
case "${OS:-}" in
  Windows_NT) exe="$exe.exe" ;;
esac

dart pub get
dart compile exe bin/pali_ijo.dart -o "$exe"

echo
echo "Built $exe"

# A smoke test, because a binary that builds but cannot make a box is not
# worth shipping. Runs the pipeline from the CLI's own help text.
echo "Smoke test:"
"$exe" box --size 10,10,10 </dev/null \
  | "$exe" fillet --radius 1 --edges convex \
  | "$exe" measure
