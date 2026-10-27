import { spawnSync } from 'node:child_process';
import { mkdirSync, rmSync } from 'node:fs';
import { dirname, join, resolve } from 'node:path';
import { fileURLToPath } from 'node:url';

const scriptDirectory = dirname(fileURLToPath(import.meta.url));
const repositoryRoot = resolve(scriptDirectory, '..', '..');
const packageDirectory = join(repositoryRoot, 'packages', 'pali_ijo');
const outputDirectory = join(scriptDirectory, '..', 'public', 'kernel');
const dartExecutable = process.env.DART_SDK
  ? join(process.env.DART_SDK, 'bin', process.platform === 'win32' ? 'dart.exe' : 'dart')
  : process.platform === 'win32'
    ? 'dart.exe'
    : 'dart';

mkdirSync(outputDirectory, { recursive: true });

const result = spawnSync(
  dartExecutable,
  ['compile', 'wasm', '--no-source-maps', 'wasm/pali_ijo_wasm.dart', '-o', join(outputDirectory, 'pali_ijo.wasm')],
  { cwd: packageDirectory, stdio: 'inherit' },
);

if (result.error) {
  console.error(`Could not run Dart: ${result.error.message}`);
  process.exitCode = 1;
} else if (result.status !== 0) {
  process.exitCode = result.status ?? 1;
} else {
  for (const file of ['pali_ijo.unopt.wasm', 'pali_ijo.support.js']) {
    rmSync(join(outputDirectory, file), { force: true });
  }
}
