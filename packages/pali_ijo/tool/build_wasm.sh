#!/usr/bin/env bash
#
# Builds the WasmGC module, for consumers that run in a browser.
#
# The output is a `.wasm` plus the `.mjs` loader Dart generates for it. Load
# it and `globalThis.paliIjo.run(argv, stdin)` becomes available — the same
# command surface as the native executable. See wasm/pali_ijo_wasm.dart.
#
# WasmGC needs a recent browser: Chrome 119+, Firefox 120+, Safari 18.4+.
#
# Usage: tool/build_wasm.sh [output-directory]

set -euo pipefail

cd "$(dirname "$0")/.."

out_dir="${1:-build/wasm}"
mkdir -p "$out_dir"

dart pub get
dart compile wasm wasm/pali_ijo_wasm.dart -o "$out_dir/pali_ijo.wasm"

echo
echo "Built:"
ls -lh "$out_dir" | tail -n +2

cat <<'NOTE'

To use it, serve the output directory and:

    import { instantiate, invoke } from './pali_ijo.mjs';
    const module = await WebAssembly.compileStreaming(fetch('./pali_ijo.wasm'));
    invoke(await instantiate(module, {}));

    const box = await paliIjo.run(['box', '--size', '40,40,20'], null);
    const part = await paliIjo.run(['fillet', '-r', '4'], box.stdout);
    const stl = await paliIjo.run(['export', '--out', 'p.stl'], part.stdout);
    const blob = new Blob([stl.files['p.stl']]);

Note the server must send `application/wasm` for the .wasm file, or
compileStreaming will refuse it.
NOTE
