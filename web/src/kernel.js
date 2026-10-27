let kernelPromise;

export function loadKernel() {
  if (!kernelPromise) kernelPromise = initializeKernel();
  return kernelPromise;
}

async function initializeKernel() {
  const moduleUrl = new URL('/kernel/pali_ijo.mjs', window.location.href);
  const loader = await import(/* @vite-ignore */ moduleUrl.href);
  const wasmUrl = new URL('./pali_ijo.wasm', moduleUrl);
  const compiled = await loader.compileStreaming(fetch(wasmUrl));
  const app = await compiled.instantiate();
  app.invokeMain();

  if (!window.paliIjo?.run) {
    throw new Error('The Pali Ijo geometry kernel did not start.');
  }
  return window.paliIjo;
}

export async function runKernel(args, inputDocument = null, toolDocument = null) {
  const kernel = await loadKernel();
  const result = await kernel.run(args, inputDocument, toolDocument);
  if (result.code !== 0) {
    throw new Error(result.stderr?.trim() || 'The geometry operation failed.');
  }
  return result;
}

export async function runKernelBytes(args, inputBytes, toolDocument = null) {
  const kernel = await loadKernel();
  const result = await kernel.runBytes(args, inputBytes, toolDocument);
  if (result.code !== 0) {
    throw new Error(result.stderr?.trim() || 'The geometry operation failed.');
  }
  return result;
}

export async function importKernelDocuments(inputBytes, name) {
  const kernel = await loadKernel();
  if (!kernel.importAll) {
    throw new Error('The CAD kernel does not support multi-body file import.');
  }
  const documents = JSON.parse(await kernel.importAll(inputBytes, name));
  if (!Array.isArray(documents) || documents.length === 0) {
    throw new Error('The CAD kernel returned no importable bodies.');
  }
  return documents;
}

export async function extrudeKernelProfile(points, height, name) {
  const kernel = await loadKernel();
  if (!kernel.extrudeProfile) {
    throw new Error('The CAD kernel does not support sketch-profile extrusion.');
  }
  return JSON.parse(await kernel.extrudeProfile(JSON.stringify(points), height, name));
}

export async function documentInfo(document) {
  const source = typeof document === 'string' ? document : JSON.stringify(document);
  const meshResult = await runKernel(['tessellate'], source);
  const measureResult = await runKernel(['measure', '--json'], source);
  return {
    displayDocument: JSON.parse(meshResult.stdout),
    measure: JSON.parse(measureResult.stdout),
  };
}
