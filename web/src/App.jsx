import React, { useEffect, useMemo, useRef, useState } from 'react';
import ModelViewport from './ModelViewport.jsx';
import { documentInfo, extrudeKernelProfile, importKernelDocuments, loadKernel, runKernel, runKernelBytes } from './kernel.js';

const colors = ['#9caf9e', '#97a9bd', '#c19e7f', '#b6a1be', '#86aca9'];
const emptyDesign = { name: 'Untitled design', units: 'mm', bodies: [], sketches: [], referenceImage: null };
const numberText = (value) => Number(value).toFixed(2);

function Icon({ name }) {
  const paths = {
    cube: <><path d="m12 3 8 4.5v9L12 21l-8-4.5v-9z" /><path d="m4.2 7.7 7.8 4.5 7.8-4.5M12 12.2V21" /></>,
    cylinder: <><ellipse cx="12" cy="5" rx="8" ry="3" /><path d="M4 5v14c0 1.7 3.6 3 8 3s8-1.3 8-3V5M4 12c0 1.7 3.6 3 8 3s8-1.3 8-3" /></>,
    sphere: <><circle cx="12" cy="12" r="9" /><path d="M4 8h16M3 12h18M4 16h16M12 3c2.4 2.4 3.6 5.4 3.6 9s-1.2 6.6-3.6 9m0-18C9.6 5.4 8.4 8.4 8.4 12s1.2 6.6 3.6 9" /></>,
    union: <><circle cx="9" cy="12" r="7" /><circle cx="15" cy="12" r="7" /></>,
    subtract: <><circle cx="9" cy="12" r="7" /><path d="M12 6.1A7 7 0 0 1 12 18z" /></>,
    intersect: <><circle cx="9" cy="12" r="7" /><circle cx="15" cy="12" r="7" /><path d="M12 5a8 8 0 0 0 0 14 8 8 0 0 0 0-14Z" /></>,
    save: <><path d="M4 3h13l4 4v14H3V3z" /><path d="M7 3v6h10V3M7 21v-8h10v8" /></>,
    download: <><path d="M12 3v12m-5-5 5 5 5-5" /><path d="M4 17v4h16v-4" /></>,
    folder: <><path d="M3 6.5h7l2 2h9v9.8a1.7 1.7 0 0 1-1.7 1.7H4.7A1.7 1.7 0 0 1 3 18.3z" /><path d="M3 9h18" /></>,
    undo: <><path d="M9 14 4 9l5-5" /><path d="M4 9h9a7 7 0 0 1 0 14h-1" /></>,
    redo: <><path d="m15 14 5-5-5-5" /><path d="M20 9h-9a7 7 0 0 0 0 14h1" /></>,
    close: <><path d="m6 6 12 12M18 6 6 18" /></>,
    eye: <><path d="M2.5 12s3.2-5 9.5-5 9.5 5 9.5 5-3.2 5-9.5 5-9.5-5-9.5-5Z" /><circle cx="12" cy="12" r="2.2" /></>,
    plus: <><path d="M12 5v14M5 12h14" /></>,
    move: <><path d="M12 2v20M2 12h20m-10-10-3 3m3-3 3 3m7 7-3-3m3 3 3-3m-10 10-3-3m3 3 3-3m-10-10 3-3m-3 3 3 3" /></>,
    fillet: <><path d="M4 20V7a3 3 0 0 1 3-3h13M4 14h6a4 4 0 0 0 4-4V4" /></>,
    chamfer: <><path d="M4 20V4h16M4 14h6V4" /></>,
    measure: <><path d="M4 18 18 4l3 3L7 21z" /><path d="m14.5 7.5 2 2m-5 0 2 2m-5 0 2 2m-5 0 2 2" /></>,
    rectangle: <><rect x="4" y="5" width="16" height="14" rx="1" /></>,
    sketch: <><path d="M4 18 18 4M6 5h4M5 6v4M18 14v4h-4" /><circle cx="7" cy="17" r="2" /></>,
    polygon: <><path d="m12 3 7.8 5.7-3 9.2H7.2l-3-9.2z" /><path d="m12 3 0 15m7.8-9.3L7.2 17.9M4.2 8.7l11.6 9.2" /></>,
    cone: <><path d="m12 3 8 15H4z" /><ellipse cx="12" cy="18" rx="8" ry="3" /></>,
  };
  return <svg viewBox="0 0 24 24" width="17" height="17" fill="none" stroke="currentColor" strokeWidth="1.7" strokeLinecap="round" strokeLinejoin="round" aria-hidden="true">{paths[name]}</svg>;
}

function IconButton({ icon, label, onClick, disabled = false }) {
  return <button type="button" className="icon-button" aria-label={label} title={label} onClick={onClick} disabled={disabled}><Icon name={icon} /></button>;
}

function DimensionField({ label, value, onChange, min = 0.01, step = 1 }) {
  return (
    <label className="dimension-field">
      <span>{label}</span>
      <div><input type="number" min={min} step={step} value={value} onChange={(event) => onChange(event.target.value)} /><small>mm</small></div>
    </label>
  );
}

function App() {
  const [design, setDesign] = useState(emptyDesign);
  const [activeSketchId, setActiveSketchId] = useState(null);
  const [selectedId, setSelectedId] = useState(null);
  const [tool, setTool] = useState('select');
  const [dimensions, setDimensions] = useState({ z: '8' });
  const [primitiveSides, setPrimitiveSides] = useState('6');
  const [torusTubeRadius, setTorusTubeRadius] = useState('2');
  const [profileDrafts, setProfileDrafts] = useState({});
  const [tracePoints, setTracePoints] = useState([]);
  const [referenceDraft, setReferenceDraft] = useState(null);
  const [bodyDimensions, setBodyDimensions] = useState({ x: '', y: '', z: '' });
  const [translation, setTranslation] = useState({ x: '10', y: '10', z: '0' });
  const [edgeSize, setEdgeSize] = useState('2');
  const [booleanOp, setBooleanOp] = useState('subtract');
  const [toolId, setToolId] = useState('');
  const [exportFormat, setExportFormat] = useState('step');
  const [kernelReady, setKernelReady] = useState(false);
  const [busy, setBusy] = useState(false);
  const [error, setError] = useState('');
  const [message, setMessage] = useState('');
  const [showGrid, setShowGrid] = useState(true);
  const [showAxes, setShowAxes] = useState(true);
  const [isDrawing, setIsDrawing] = useState(false);
  const [past, setPast] = useState([]);
  const [future, setFuture] = useState([]);
  const [projectName, setProjectName] = useState('Untitled design');
  const fileInput = useRef(null);
  const imageInput = useRef(null);
  const selectedBody = design.bodies.find((body) => body.id === selectedId) ?? null;
  const selectedBodyIsMesh = selectedBody?.document?.kind === 'mesh';
  const activeSketch = design.sketches.find((sketch) => sketch.id === activeSketchId) ?? null;
  const referenceImage = design.referenceImage ?? null;
  const tools = useMemo(() => design.bodies.filter((body) => body.id !== selectedId && body.document?.kind !== 'mesh'), [design.bodies, selectedId]);

  useEffect(() => {
    loadKernel().then(() => setKernelReady(true)).catch((cause) => setError(`Could not load the CAD kernel: ${cause.message}`));
  }, []);

  useEffect(() => {
    const bounds = selectedBody?.measure?.bounds;
    if (!bounds) return;
    setBodyDimensions({
      x: numberText(bounds.max[0] - bounds.min[0]),
      y: numberText(bounds.max[1] - bounds.min[1]),
      z: numberText(bounds.max[2] - bounds.min[2]),
    });
  }, [selectedId, selectedBody?.measure]);

  useEffect(() => {
    setReferenceDraft(referenceImage ? {
      widthMm: String(referenceImage.widthMm),
      centerX: String(referenceImage.centerX),
      centerY: String(referenceImage.centerY),
      opacity: String(referenceImage.opacity),
    } : null);
  }, [referenceImage?.id]);

  useEffect(() => {
    const onKeyDown = (event) => {
      if (!(event.ctrlKey || event.metaKey)) return;
      if (event.key.toLowerCase() === 'z') {
        event.preventDefault();
        event.shiftKey ? redo() : undo();
      } else if (event.key.toLowerCase() === 'y') {
        event.preventDefault();
        redo();
      } else if (event.key.toLowerCase() === 's') {
        event.preventDefault();
        saveProject();
      }
    };
    window.addEventListener('keydown', onKeyDown);
    return () => window.removeEventListener('keydown', onKeyDown);
  });

  useEffect(() => {
    if (tool !== 'sketch-trace') return undefined;
    const onTraceKeyDown = (event) => {
      if (event.target instanceof HTMLElement && ['INPUT', 'TEXTAREA', 'SELECT'].includes(event.target.tagName)) return;
      if (event.key === 'Backspace') {
        event.preventDefault();
        setTracePoints((points) => points.slice(0, -1));
      } else if (event.key === 'Enter') {
        event.preventDefault();
        finishTrace();
      } else if (event.key === 'Escape') {
        setTracePoints([]);
        setTool('select');
      }
    };
    window.addEventListener('keydown', onTraceKeyDown);
    return () => window.removeEventListener('keydown', onTraceKeyDown);
  });

  const commitBodies = (bodies, selection = selectedId, name = projectName) => {
    setPast((items) => [...items.slice(-29), { bodies: design.bodies, sketches: design.sketches, referenceImage, activeSketchId, selectedId, projectName }]);
    setFuture([]);
    setDesign({ ...design, name, bodies });
    setSelectedId(selection);
    setProjectName(name);
    setError('');
    setMessage('Model updated');
  };

  const commitSketches = (sketches, selection = selectedId) => {
    setPast((items) => [...items.slice(-29), { bodies: design.bodies, sketches: design.sketches, referenceImage, activeSketchId, selectedId, projectName }]);
    setFuture([]);
    setDesign({ ...design, sketches });
    setSelectedId(selection);
    setError('');
    setMessage('Sketch updated');
  };

  const startSketch = () => {
    const sketch = {
      id: crypto.randomUUID(),
      name: `Sketch ${design.sketches.length + 1}`,
      profiles: [],
      visible: true,
    };
    commitSketches([...design.sketches, sketch], null);
    setActiveSketchId(sketch.id);
    setTool('sketch-rectangle');
  };

  const commitReferenceImage = (next) => {
    setPast((items) => [...items.slice(-29), {
      bodies: design.bodies,
      sketches: design.sketches,
      referenceImage,
      activeSketchId,
      selectedId,
      projectName,
    }]);
    setFuture([]);
    setDesign({ ...design, referenceImage: next });
    setReferenceDraft(next ? {
      widthMm: String(next.widthMm),
      centerX: String(next.centerX),
      centerY: String(next.centerY),
      opacity: String(next.opacity),
    } : null);
    setError('');
  };

  const importReferenceImage = (file) => withBusy(async () => {
    if (!['image/png', 'image/jpeg', 'image/webp'].includes(file.type)) {
      throw new Error('Choose a PNG, JPEG, or WebP reference image.');
    }
    if (file.size > 20 * 1024 * 1024) {
      throw new Error('Reference images must be smaller than 20 MB.');
    }
    const bitmap = await createImageBitmap(file);
    const { width: pixelWidth, height: pixelHeight } = bitmap;
    bitmap.close();
    const dataUrl = await new Promise((resolve, reject) => {
      const reader = new FileReader();
      reader.onload = () => typeof reader.result === 'string'
        ? resolve(reader.result)
        : reject(new Error('The image could not be read.'));
      reader.onerror = () => reject(reader.error || new Error('The image could not be read.'));
      reader.readAsDataURL(file);
    });
    const next = {
      id: crypto.randomUUID(),
      name: file.name,
      dataUrl,
      pixelWidth,
      pixelHeight,
      widthMm: 100,
      centerX: 0,
      centerY: 0,
      opacity: 0.55,
      visible: true,
    };
    commitReferenceImage(next);
    setMessage(`${file.name} added as a sketch reference`);
  });

  const applyReferenceSettings = () => {
    if (!referenceImage || !referenceDraft) return;
    const widthMm = Number(referenceDraft.widthMm);
    const centerX = Number(referenceDraft.centerX);
    const centerY = Number(referenceDraft.centerY);
    const opacity = Number(referenceDraft.opacity);
    if (![widthMm, centerX, centerY, opacity].every(Number.isFinite) || widthMm <= 0 || opacity < 0 || opacity > 1) {
      setError('Image width must be positive, position must be finite, and opacity must be from 0 to 1.');
      return;
    }
    commitReferenceImage({ ...referenceImage, widthMm, centerX, centerY, opacity });
    setMessage('Reference image scale and position updated');
  };

  const startTracing = () => {
    if (!activeSketch) {
      const sketch = {
        id: crypto.randomUUID(),
        name: `Sketch ${design.sketches.length + 1}`,
        profiles: [],
        visible: true,
      };
      commitSketches([...design.sketches, sketch], null);
      setActiveSketchId(sketch.id);
    }
    setTracePoints([]);
    setTool('sketch-trace');
  };

  const finishTrace = () => {
    if (!activeSketch) {
      setError('Start or select a sketch before tracing.');
      return;
    }
    if (tracePoints.length < 3) {
      setError('Trace at least three points to make a closed outline.');
      return;
    }
    const area = tracePoints.reduce((sum, point, index) => {
      const next = tracePoints[(index + 1) % tracePoints.length];
      return sum + point[0] * next[1] - next[0] * point[1];
    }, 0) / 2;
    if (Math.abs(area) < 1e-4) {
      setError('The traced outline is too small or degenerate.');
      return;
    }
    const profile = { id: crypto.randomUUID(), kind: 'polygon', points: tracePoints };
    commitSketches(design.sketches.map((sketch) => sketch.id === activeSketch.id
      ? { ...sketch, profiles: [...sketch.profiles, profile] }
      : sketch));
    setTracePoints([]);
    setError('');
    setMessage('Closed trace added to sketch');
  };

  const addSketchProfile = (drag) => {
    if (!activeSketch) {
      setError('Start a sketch before drawing a profile.');
      return;
    }
    if (drag.kind === 'sketch-rectangle' && (drag.width < 0.08 || drag.depth < 0.08)) {
      setError('Drag both width and depth to draw a sketch rectangle.');
      return;
    }
    if (drag.kind === 'sketch-circle' && drag.radius < 0.08) {
      setError('Drag from the circle centre to set a larger radius.');
      return;
    }
    const profile = drag.kind === 'sketch-rectangle'
      ? { id: crypto.randomUUID(), kind: 'rectangle', x: drag.x, y: drag.y, width: drag.width, depth: drag.depth }
      : { id: crypto.randomUUID(), kind: 'circle', x: drag.x, y: drag.y, radius: drag.radius };
    commitSketches(design.sketches.map((sketch) => sketch.id === activeSketch.id
      ? { ...sketch, profiles: [...sketch.profiles, profile] }
      : sketch));
  };

  const applyProfileDimension = (profile, dimension) => {
    const key = `${profile.id}:${dimension}`;
    const value = Number(profileDrafts[key]);
    if (!Number.isFinite(value) || value <= 0) {
      setError('Sketch dimensions must be positive finite numbers.');
      return;
    }
    const next = dimension === 'diameter'
      ? { radius: value / 2 }
      : { [dimension]: value };
    commitSketches(design.sketches.map((sketch) => sketch.id === activeSketchId
      ? {
        ...sketch,
        profiles: sketch.profiles.map((item) => item.id === profile.id ? { ...item, ...next } : item),
      }
      : sketch));
    setProfileDrafts((drafts) => {
      const { [key]: _saved, ...remaining } = drafts;
      return remaining;
    });
  };

  const profileDimensionInput = (profile, dimension, value, label) => {
    const key = `${profile.id}:${dimension}`;
    return (
      <label className="profile-dimension">
        <span>{label}</span>
        <input
          type="number"
          min="0.01"
          step="1"
          aria-label={`Sketch ${profile.kind} ${label} in millimeters`}
          value={profileDrafts[key] ?? value}
          onChange={(event) => setProfileDrafts((drafts) => ({ ...drafts, [key]: event.target.value }))}
          onBlur={() => {
            if (profileDrafts[key] !== undefined) applyProfileDimension(profile, dimension);
          }}
          onKeyDown={(event) => {
            if (event.key === 'Enter') event.currentTarget.blur();
          }}
        />
      </label>
    );
  };

  const reportFailure = (cause) => {
    const message = cause instanceof Error
      ? cause.message
      : cause?.message || String(cause);
    setError(`${message}${cause?.stack ? `\n${cause.stack}` : ''}`);
    setMessage('');
  };

  const withBusy = async (action) => {
    setBusy(true);
    setError('');
    setMessage('');
    try {
      await action();
    } catch (cause) {
      reportFailure(cause);
    } finally {
      setBusy(false);
    }
  };

  const enrichBody = async (id, name, document, color) => {
    const info = await documentInfo(document);
    return { id, name, document, displayDocument: info.displayDocument, measure: info.measure, color, version: 0 };
  };

  const createFromDrag = (drag) => {
    if (drag.kind.startsWith('sketch-')) {
      addSketchProfile(drag);
      return;
    }
    return withBusy(async () => {
    if (![drag.x, drag.y, drag.width, drag.depth, drag.diagonal, drag.height].every(Number.isFinite)) {
      throw new Error('The mouse drag did not produce valid model dimensions. Draw a larger footprint on the grid.');
    }
    if (drag.kind === 'box' && (drag.width < 0.08 || drag.depth < 0.08)) {
      throw new Error('Drag both width and depth to draw a box footprint.');
    }
    if (drag.diagonal < 0.16) {
      throw new Error('Drag a larger footprint to create this primitive.');
    }
    const sideCount = Number(primitiveSides);
    if (['prism', 'pyramid'].includes(drag.kind) && (!Number.isInteger(sideCount) || sideCount < 3 || sideCount > 128)) {
      throw new Error('Polygon side count must be a whole number from 3 to 128.');
    }
    const minorRadius = Number(torusTubeRadius);
    if (drag.kind === 'torus' && (!Number.isFinite(minorRadius) || minorRadius <= 0 || minorRadius >= drag.diagonal / 2)) {
      throw new Error('Torus tube radius must be positive and smaller than its drawn major radius.');
    }
    const bodyType = drag.kind[0].toUpperCase() + drag.kind.slice(1);
    let number = design.bodies.filter((body) => body.name.startsWith(`${bodyType} `)).length + 1;
    while (design.bodies.some((body) => body.name === `${bodyType} ${number}`)) number += 1;
    const name = `${bodyType} ${number}`;
    const id = crypto.randomUUID();
    const color = colors[design.bodies.length % colors.length];
    let result;
    if (drag.kind === 'box') {
      result = await runKernel(['box', '--size', `${drag.width},${drag.depth},${drag.height}`, '--name', name]);
      const originX = drag.x - drag.width / 2;
      const originY = drag.y - drag.depth / 2;
      if (Math.abs(originX) > 1e-8 || Math.abs(originY) > 1e-8) {
        result = await runKernel(['translate', '--by', `${originX},${originY},0`], result.stdout);
      }
    } else if (drag.kind === 'cylinder') {
      result = await runKernel(['cylinder', '--radius', String(drag.diagonal / 2), '--height', String(drag.height), '--origin', `${drag.x},${drag.y},0`, '--name', name]);
    } else if (drag.kind === 'sphere') {
      const radius = drag.diagonal / 2;
      result = await runKernel(['sphere', '--radius', String(radius), '--center', `${drag.x},${drag.y},${radius}`, '--name', name]);
    } else if (drag.kind === 'cone') {
      result = await runKernel(['cone', '--radius', String(drag.diagonal / 2), '--height', String(drag.height), '--origin', `${drag.x},${drag.y},0`, '--name', name]);
    } else if (drag.kind === 'torus') {
      result = await runKernel(['torus', '--major', String(drag.diagonal / 2), '--minor', String(minorRadius), '--center', `${drag.x},${drag.y},${minorRadius}`, '--name', name]);
    } else {
      result = await runKernel([
        drag.kind,
        '--diameter', String(drag.diagonal),
        '--height', String(drag.height),
        '--sides', String(sideCount),
        '--name', name,
      ]);
      result = await runKernel(['translate', '--by', `${drag.x},${drag.y},0`], result.stdout);
    }
    const body = await enrichBody(id, name, JSON.parse(result.stdout), color);
    commitBodies([...design.bodies, body], id);
    setTool('select');
    setMessage(`${name} drawn and created`);
    });
  };

  const extrudeSketch = () => withBusy(async () => {
    if (!activeSketch) throw new Error('Select or start a sketch before extruding.');
    const rectangles = activeSketch.profiles.filter((profile) => profile.kind === 'rectangle');
    const circles = activeSketch.profiles.filter((profile) => profile.kind === 'circle');
    const polygons = activeSketch.profiles.filter((profile) => profile.kind === 'polygon');
    const height = Number(dimensions.z);
    if (!rectangles.length && !polygons.length) throw new Error('Draw a closed rectangle or traced outline before extruding.');
    if (!Number.isFinite(height) || height <= 0) throw new Error('Extrusion height must be a positive finite number.');
    if (circles.length && !rectangles.length) {
      throw new Error('Circle through-holes currently need to be drawn inside a rectangle profile.');
    }

    const assignedHoles = new Map(rectangles.map((profile) => [profile.id, []]));
    for (const circle of circles) {
      const hosts = rectangles.filter((rectangle) => (
        circle.x - circle.radius > rectangle.x - rectangle.width / 2
        && circle.x + circle.radius < rectangle.x + rectangle.width / 2
        && circle.y - circle.radius > rectangle.y - rectangle.depth / 2
        && circle.y + circle.radius < rectangle.y + rectangle.depth / 2
      ));
      if (hosts.length !== 1) {
        throw new Error('Each circle must fit fully inside exactly one rectangle to create an extruded hole.');
      }
      assignedHoles.get(hosts[0].id).push(circle);
    }

    const created = [];
    for (const [index, rectangle] of rectangles.entries()) {
      const name = rectangles.length === 1 ? `Extrusion ${design.bodies.length + 1}` : `Extrusion ${design.bodies.length + index + 1}`;
      const originX = rectangle.x - rectangle.width / 2;
      const originY = rectangle.y - rectangle.depth / 2;
      let result = await runKernel(['box', '--size', `${rectangle.width},${rectangle.depth},${height}`, '--name', name]);
      if (Math.abs(originX) > 1e-8 || Math.abs(originY) > 1e-8) {
        result = await runKernel(['translate', '--by', `${originX},${originY},0`], result.stdout);
      }
      for (const circle of assignedHoles.get(rectangle.id)) {
        const margin = Math.max(0.01, height * 0.01);
        const cutter = await runKernel([
          'cylinder', '--radius', String(circle.radius), '--height', String(height + margin * 2),
          '--origin', `${circle.x},${circle.y},${-margin}`,
        ]);
        result = await runKernel(
          ['boolean', '--op', 'subtract', '--with', 'tool.json'],
          result.stdout,
          cutter.stdout,
        );
      }
      created.push(await enrichBody(
        crypto.randomUUID(),
        name,
        JSON.parse(result.stdout),
        colors[(design.bodies.length + index) % colors.length],
      ));
    }
    for (const polygon of polygons) {
      const name = `Extrusion ${design.bodies.length + created.length + 1}`;
      const document = await extrudeKernelProfile(polygon.points, height, name);
      created.push(await enrichBody(
        crypto.randomUUID(),
        name,
        document,
        colors[(design.bodies.length + created.length) % colors.length],
      ));
    }
    commitBodies([...design.bodies, ...created], created[created.length - 1].id);
    setTool('inspect');
    setMessage(`${created.length} solid${created.length === 1 ? '' : 's'} extruded from ${activeSketch.name}`);
  });

  const applyBoolean = () => withBusy(async () => {
    if (!selectedBody || !toolId) throw new Error('Select two solids before applying a boolean operation.');
    const operand = design.bodies.find((body) => body.id === toolId);
    if (selectedBody.document?.kind === 'mesh' || operand?.document?.kind === 'mesh') {
      throw new Error('Boolean operations require analytic solid bodies. This imported part is a mesh.');
    }
    const result = await runKernel(
      ['boolean', '--op', booleanOp, '--with', 'tool.json'],
      JSON.stringify(selectedBody.document),
      JSON.stringify(operand.document),
    );
    const next = JSON.parse(result.stdout);
    const updated = await enrichBody(selectedBody.id, selectedBody.name, next, selectedBody.color);
    commitBodies(design.bodies.filter((body) => body.id !== operand.id).map((body) => body.id === updated.id ? updated : body), updated.id);
    setMessage(`${booleanOp[0].toUpperCase()}${booleanOp.slice(1)} complete`);
  });

  const applyTransform = () => withBusy(async () => {
    if (!selectedBody) throw new Error('Select a solid to move.');
    const result = await runKernel(
      ['translate', '--by', `${translation.x},${translation.y},${translation.z}`],
      JSON.stringify(selectedBody.document),
    );
    const updated = await enrichBody(selectedBody.id, selectedBody.name, JSON.parse(result.stdout), selectedBody.color);
    commitBodies(design.bodies.map((body) => body.id === updated.id ? { ...updated, version: (body.version ?? 0) + 1 } : body), updated.id);
    setMessage(`${selectedBody.name} moved`);
  });

  const applyViewportMove = (id, position) => withBusy(async () => {
    const body = design.bodies.find((item) => item.id === id);
    if (!body) return;
    const [x, displayY, displayZ] = position;
    const delta = [x, -displayZ, displayY];
    if (delta.every((value) => Math.abs(value) < 1e-8)) return;
    const result = await runKernel(
      ['translate', '--by', delta.join(',')],
      JSON.stringify(body.document),
    );
    const updated = await enrichBody(body.id, body.name, JSON.parse(result.stdout), body.color);
    commitBodies(
      design.bodies.map((item) => item.id === body.id ? { ...updated, version: (item.version ?? 0) + 1 } : item),
      body.id,
    );
    setMessage(`${body.name} moved`);
  });

  const applyDimensions = () => withBusy(async () => {
    if (!selectedBody) throw new Error('Select a solid to edit its dimensions.');
    const requested = [bodyDimensions.x, bodyDimensions.y, bodyDimensions.z].map(Number);
    if (!requested.every((value) => Number.isFinite(value) && value > 0)) {
      throw new Error('Enter a positive finite dimension for X, Y, and Z.');
    }
    const bounds = selectedBody.measure?.bounds;
    if (!bounds) throw new Error('The kernel did not provide bounds for this solid.');
    const current = bounds.max.map((value, index) => value - bounds.min[index]);
    if (current.some((value) => !Number.isFinite(value) || value <= 0)) {
      throw new Error('The solid has invalid bounds and cannot be resized.');
    }
    const factor = requested.map((value, index) => value / current[index]);
    const result = await runKernel(
      ['scale', '--by', factor.join(','), '--about', bounds.min.join(',')],
      JSON.stringify(selectedBody.document),
    );
    const updated = await enrichBody(selectedBody.id, selectedBody.name, JSON.parse(result.stdout), selectedBody.color);
    commitBodies(
      design.bodies.map((body) => body.id === updated.id ? { ...updated, version: (body.version ?? 0) + 1 } : body),
      updated.id,
    );
    setMessage(`${selectedBody.name} dimensions updated`);
  });

  const applyEdgeTool = (operation) => withBusy(async () => {
    if (!selectedBody) throw new Error(`Select a solid to ${operation}.`);
    if (selectedBody.document?.kind === 'mesh') {
      throw new Error(`${operation} requires an analytic solid. This imported part is a mesh.`);
    }
    const args = operation === 'fillet'
      ? ['fillet', '--radius', edgeSize, '--edges', 'convex']
      : ['chamfer', '--distance', edgeSize, '--edges', 'convex'];
    const result = await runKernel(args, JSON.stringify(selectedBody.document));
    const updated = await enrichBody(selectedBody.id, selectedBody.name, JSON.parse(result.stdout), selectedBody.color);
    commitBodies(design.bodies.map((body) => body.id === updated.id ? updated : body), updated.id);
    setMessage(`${operation[0].toUpperCase()}${operation.slice(1)} complete`);
  });

  const undo = () => {
    if (!past.length || busy) return;
    const previous = past[past.length - 1];
    setFuture((items) => [...items, { bodies: design.bodies, sketches: design.sketches, referenceImage, activeSketchId, selectedId, projectName }]);
    setPast((items) => items.slice(0, -1));
    setDesign({ ...design, bodies: previous.bodies, sketches: previous.sketches ?? [], referenceImage: previous.referenceImage ?? null });
    setReferenceDraft(previous.referenceImage ? {
      widthMm: String(previous.referenceImage.widthMm),
      centerX: String(previous.referenceImage.centerX),
      centerY: String(previous.referenceImage.centerY),
      opacity: String(previous.referenceImage.opacity),
    } : null);
    setActiveSketchId(previous.activeSketchId ?? null);
    setSelectedId(previous.selectedId);
    setProjectName(previous.projectName);
    setMessage('Undo complete');
    setError('');
  };

  const redo = () => {
    if (!future.length || busy) return;
    const next = future[future.length - 1];
    setPast((items) => [...items, { bodies: design.bodies, sketches: design.sketches, referenceImage, activeSketchId, selectedId, projectName }]);
    setFuture((items) => items.slice(0, -1));
    setDesign({ ...design, bodies: next.bodies, sketches: next.sketches ?? [], referenceImage: next.referenceImage ?? null });
    setReferenceDraft(next.referenceImage ? {
      widthMm: String(next.referenceImage.widthMm),
      centerX: String(next.referenceImage.centerX),
      centerY: String(next.referenceImage.centerY),
      opacity: String(next.referenceImage.opacity),
    } : null);
    setActiveSketchId(next.activeSketchId ?? null);
    setSelectedId(next.selectedId);
    setProjectName(next.projectName);
    setMessage('Redo complete');
    setError('');
  };

  const saveProject = () => {
    const project = {
      format: 'palicad-project',
      version: 1,
      name: projectName,
      units: 'mm',
      sketches: design.sketches,
      referenceImage,
      bodies: design.bodies.map(({ id, name, color, document, visible }) => ({ id, name, color, visible, document })),
    };
    const link = document.createElement('a');
    const url = URL.createObjectURL(new Blob([JSON.stringify(project, null, 2)], { type: 'application/json' }));
    link.href = url;
    link.download = `${projectName.toLowerCase().replace(/[^a-z0-9]+/g, '-') || 'design'}.pali`;
    link.click();
    window.setTimeout(() => URL.revokeObjectURL(url), 1000);
    setMessage('Native project saved');
  };

  const openProject = async (file) => withBusy(async () => {
    const extension = file.name.split('.').pop()?.toLowerCase();
    if (!['pali', 'json'].includes(extension)) {
      if (!['step', 'stp', 'p21', 'stl', '3mf', 'obj'].includes(extension)) {
        throw new Error(`Unsupported file type ".${extension || ''}". Open .pali/.json projects or STEP, STL, 3MF and OBJ geometry.`);
      }
      if (['step', 'stp', 'p21'].includes(extension)) {
        const documents = await importKernelDocuments(
          new Uint8Array(await file.arrayBuffer()),
          file.name,
        );
        const importedBodies = [];
        for (const [index, document] of documents.entries()) {
          const fallbackName = documents.length > 1
            ? `${file.name.replace(/\.[^.]+$/, '')} ${index + 1}`
            : file.name.replace(/\.[^.]+$/, '');
          importedBodies.push(await enrichBody(
            crypto.randomUUID(),
            document.name || fallbackName || 'Imported solid',
            document,
            colors[(design.bodies.length + index) % colors.length],
          ));
        }
        commitBodies([...design.bodies, ...importedBodies], importedBodies[0].id);
        setActiveSketchId(null);
        setTool('inspect');
        setMessage(`${importedBodies.length} STEP ${importedBodies.length === 1 ? 'body' : 'bodies'} imported`);
        return;
      }
      const imported = await runKernelBytes(
        ['export', '--out', 'imported.json'],
        new Uint8Array(await file.arrayBuffer()),
      );
      const jsonBytes = imported.files['imported.json'];
      if (!jsonBytes) throw new Error('The kernel did not return imported geometry.');
      const parsed = JSON.parse(new TextDecoder().decode(jsonBytes));
      const name = parsed.name || file.name.replace(/\.[^.]+$/, '') || 'Imported solid';
      const body = await enrichBody(crypto.randomUUID(), name, parsed, colors[0]);
      setPast([]);
      setFuture([]);
      setProjectName(name);
      setDesign({ name, units: 'mm', bodies: [body], sketches: [], referenceImage: null });
      setActiveSketchId(null);
      setSelectedId(body.id);
      setTool('inspect');
      setMessage(`${file.name} imported`);
      return;
    }
    const content = await file.text();
    const parsed = JSON.parse(content);
    if (parsed.pali_ijo != null) {
      const body = await enrichBody(crypto.randomUUID(), file.name.replace(/\.json$/i, '') || 'Imported solid', parsed, colors[0]);
      setPast([]);
      setFuture([]);
      setProjectName(parsed.name || body.name);
      setDesign({ name: parsed.name || body.name, units: 'mm', bodies: [body], sketches: [], referenceImage: null });
      setActiveSketchId(null);
      setSelectedId(body.id);
      setTool('inspect');
      setMessage(`${file.name} opened`);
      return;
    }
    if (parsed.format !== 'palicad-project' || parsed.version !== 1 || !Array.isArray(parsed.bodies)) {
      throw new Error('Unsupported file. Open a .pali project or a Pali Ijo JSON solid.');
    }
    const sketches = parsed.sketches ?? [];
    if (!Array.isArray(sketches) || sketches.some((sketch) => !Array.isArray(sketch.profiles))) {
      throw new Error('The project contains invalid sketch data.');
    }
    const bodies = [];
    for (const [index, raw] of parsed.bodies.entries()) {
      const bodyDoc = raw.document;
      if (!bodyDoc || bodyDoc.pali_ijo == null) throw new Error(`Body ${index + 1} has invalid geometry data.`);
      bodies.push({
        ...await enrichBody(raw.id || crypto.randomUUID(), raw.name || `Body ${index + 1}`, bodyDoc, raw.color || colors[index % colors.length]),
        visible: raw.visible !== false,
      });
    }
    setPast([]);
    setFuture([]);
    setProjectName(parsed.name || 'Untitled design');
    const referenceImage = parsed.referenceImage ?? null;
    if (referenceImage && (
      typeof referenceImage.dataUrl !== 'string'
      || !Number.isFinite(referenceImage.widthMm)
      || !Number.isFinite(referenceImage.pixelWidth)
      || !Number.isFinite(referenceImage.pixelHeight)
      || referenceImage.widthMm <= 0
      || referenceImage.pixelWidth <= 0
      || referenceImage.pixelHeight <= 0
    )) {
      throw new Error('The project contains invalid reference image data.');
    }
    setDesign({ name: parsed.name || 'Untitled design', units: 'mm', bodies, sketches, referenceImage });
    setActiveSketchId(sketches[0]?.id ?? null);
    setSelectedId(bodies[0]?.id ?? null);
    setTool(bodies.length ? 'inspect' : 'select');
    setMessage(`${file.name} opened`);
  });

  const deleteBody = () => {
    if (!selectedBody) return;
    const remaining = design.bodies.filter((body) => body.id !== selectedId);
    commitBodies(remaining, remaining[0]?.id ?? null);
  };

  const toggleBodyVisibility = (id) => {
    commitBodies(design.bodies.map((body) => body.id === id
      ? { ...body, visible: body.visible === false }
      : body));
  };

  const toggleSketchVisibility = (id) => {
    commitSketches(design.sketches.map((sketch) => sketch.id === id
      ? { ...sketch, visible: sketch.visible === false }
      : sketch));
  };

  const exportBody = () => withBusy(async () => {
    if (!selectedBody) throw new Error('Select a solid to export.');
    const extension = exportFormat === 'step' ? 'step' : exportFormat;
    const result = await runKernel(['export', '--out', `${selectedBody.name}.${extension}`], JSON.stringify(selectedBody.document));
    const bytes = result.files[`${selectedBody.name}.${extension}`];
    if (!bytes) throw new Error('The CAD kernel produced no export data.');
    const mime = extension === 'step' ? 'model/step' : extension === '3mf' ? 'model/3mf' : extension === 'obj' ? 'text/plain' : 'application/octet-stream';
    const link = document.createElement('a');
    const url = URL.createObjectURL(new Blob([bytes], { type: mime }));
    link.href = url;
    link.download = `${selectedBody.name.toLowerCase().replace(/[^a-z0-9]+/g, '-')}.${extension}`;
    link.click();
    window.setTimeout(() => URL.revokeObjectURL(url), 1000);
    setMessage(`Exported ${extension.toUpperCase()}`);
  });

  return (
    <main className="cad-app">
      <header className="topbar">
        <div className="brand"><div className="brand-mark">P</div><span className="brand-name">Pali<span>CAD</span></span><i className={`kernel-indicator ${kernelReady ? 'ready' : ''}`} /> <span className="kernel-label">{kernelReady ? 'KERNEL READY' : 'STARTING KERNEL'}</span></div>
        <div className="doc-name"><input aria-label="Project name" value={projectName} onChange={(event) => { setProjectName(event.target.value); setDesign({ ...design, name: event.target.value }); }} /><span>.pali</span></div>
        <div className="top-actions">
          <IconButton icon="undo" label="Undo" onClick={undo} disabled={!past.length || busy} />
          <IconButton icon="redo" label="Redo" onClick={redo} disabled={!future.length || busy} />
          <span className="action-divider" />
          <IconButton icon="folder" label="Open or import model" onClick={() => fileInput.current?.click()} disabled={busy} />
          <input ref={fileInput} className="visually-hidden" type="file" accept=".pali,.json,.step,.stp,.p21,.stl,.3mf,.obj,application/json" onChange={(event) => { const file = event.target.files?.[0]; if (file) openProject(file); event.target.value = ''; }} />
          <input ref={imageInput} className="visually-hidden" type="file" accept="image/png,image/jpeg,image/webp" onChange={(event) => { const file = event.target.files?.[0]; if (file) importReferenceImage(file); event.target.value = ''; }} />
          <button type="button" className="save-button" onClick={saveProject} disabled={busy}><Icon name="save" /> Save project</button>
        </div>
      </header>

      <nav className="toolbar" aria-label="CAD tools">
        <div className="tool-buttons">
          <button type="button" className={tool === 'select' || tool === 'inspect' ? 'active' : ''} onClick={() => setTool('select')}>↖ Select</button>
          <button type="button" className={referenceImage ? 'active' : ''} onClick={() => imageInput.current?.click()} disabled={busy}>Reference image</button>
          <button type="button" className={tool.startsWith('sketch-') ? 'active' : ''} onClick={startSketch}><Icon name="sketch" /> Sketch</button>
          {activeSketch && <>
            <button type="button" className={tool === 'sketch-rectangle' ? 'active' : ''} onClick={() => setTool('sketch-rectangle')}><Icon name="rectangle" /> Rectangle</button>
            <button type="button" className={tool === 'sketch-circle' ? 'active' : ''} onClick={() => setTool('sketch-circle')}><Icon name="sphere" /> Center circle</button>
            {referenceImage && <button type="button" className={tool === 'sketch-trace' ? 'active' : ''} onClick={startTracing}>Trace outline</button>}
            <button type="button" className="extrude-sketch-button" onClick={extrudeSketch} disabled={!activeSketch.profiles.some((profile) => ['rectangle', 'polygon'].includes(profile.kind)) || busy}>Extrude sketch</button>
          </>}
          <span className="toolbar-separator" />
          {['box', 'cylinder', 'sphere', 'cone', 'torus', 'prism', 'pyramid'].map((shape) => (
            <button type="button" key={shape} className={tool === shape ? 'active' : ''} onClick={() => { setSelectedId(null); setActiveSketchId(null); setTool(shape); }}>
              <Icon name={shape === 'prism' || shape === 'pyramid' ? 'polygon' : shape === 'cone' || shape === 'torus' ? 'cone' : shape} />
              {shape[0].toUpperCase() + shape.slice(1)}
            </button>
          ))}
          <span className="toolbar-separator" />
          <button type="button" className={tool === 'boolean' ? 'active' : ''} onClick={() => setTool('boolean')} disabled={!selectedBody || selectedBodyIsMesh} title={selectedBodyIsMesh ? 'Boolean operations require an analytic STEP solid, not a mesh' : undefined}><Icon name="subtract" /> Boolean</button>
          <button type="button" className={tool === 'move' ? 'active' : ''} onClick={() => setTool('move')} disabled={!selectedBody}><Icon name="move" /> Move</button>
          <button type="button" className={tool === 'fillet' ? 'active' : ''} onClick={() => setTool('fillet')} disabled={!selectedBody || selectedBodyIsMesh} title={selectedBodyIsMesh ? 'Fillets require an analytic STEP solid, not a mesh' : undefined}><Icon name="fillet" /> Fillet</button>
          <button type="button" className={tool === 'chamfer' ? 'active' : ''} onClick={() => setTool('chamfer')} disabled={!selectedBody || selectedBodyIsMesh} title={selectedBodyIsMesh ? 'Chamfers require an analytic STEP solid, not a mesh' : undefined}><Icon name="chamfer" /> Chamfer</button>
        </div>
        <span className="toolbar-spacer" />
        <div className="export-controls"><label htmlFor="export-format">EXPORT</label><select id="export-format" value={exportFormat} onChange={(event) => setExportFormat(event.target.value)}><option value="step">STEP · CAD</option><option value="json">Pali Ijo JSON</option><option value="3mf">3MF · Print</option><option value="stl">STL · Mesh</option><option value="obj">OBJ · Mesh</option></select><button type="button" className="export-button" onClick={exportBody} disabled={!selectedBody || busy}><Icon name="download" /> Export</button></div>
      </nav>

      <div className="workspace-grid">
        <aside className="browser">
          <div className="panel-title"><div><small>PROJECT TREE</small><h2>Model browser</h2></div><span className="count-chip">{design.bodies.length + design.sketches.length}</span></div>
          <div className="tree-root"><Icon name="folder" /> <span>{projectName || 'Untitled design'}</span><small>{design.bodies.length} bodies · {design.sketches.length} sketches</small></div>
          <div className="body-list">
            {design.sketches.map((sketch) => (
              <div className={`body-row-wrapper ${activeSketchId === sketch.id ? 'selected' : ''}`} key={sketch.id}>
                <button type="button" className="body-row" onClick={() => { setActiveSketchId(sketch.id); setSelectedId(null); setTool('sketch-rectangle'); }}>
                  <span className="body-color sketch-color" /><Icon name="sketch" /><span className="body-name">{sketch.name}</span>
                  <small className="profile-count">{sketch.profiles.length}</small>
                </button>
                <button type="button" className="visibility-toggle" aria-label={`${sketch.visible === false ? 'Show' : 'Hide'} ${sketch.name}`} title={`${sketch.visible === false ? 'Show' : 'Hide'} ${sketch.name}`} onClick={() => toggleSketchVisibility(sketch.id)}><Icon name="eye" /></button>
              </div>
            ))}
            {design.bodies.map((body) => (
              <div className={`body-row-wrapper ${selectedId === body.id ? 'selected' : ''}`} key={body.id}>
                <button type="button" className="body-row" onClick={() => { setSelectedId(body.id); setActiveSketchId(null); setTool('inspect'); }}>
                  <span className="body-color" style={{ background: body.color }} /><Icon name="cube" /><span className="body-name">{body.name}</span>
                </button>
                <button type="button" className="visibility-toggle" aria-label={`${body.visible === false ? 'Show' : 'Hide'} ${body.name}`} title={`${body.visible === false ? 'Show' : 'Hide'} ${body.name}`} onClick={() => toggleBodyVisibility(body.id)}><Icon name="eye" /></button>
              </div>
            ))}
            {!design.bodies.length && !design.sketches.length && <div className="tree-empty">No model items yet.<br />Start a sketch or draw a solid in the viewport.</div>}
          </div>
          <div className="browser-bottom"><span className={`kernel-indicator ${kernelReady ? 'ready' : ''}`} /> {kernelReady ? 'Geometry kernel online' : 'Loading browser kernel…'}</div>
        </aside>

        <section className="viewport" aria-label="3D CAD viewport">
          <div className="viewport-head"><div className="breadcrumb"><span>Design</span><b>/</b><strong>{selectedBody?.name || 'Empty scene'}</strong></div><div className="viewport-actions"><button type="button" className={showAxes ? 'view-toggle on' : 'view-toggle'} onClick={() => setShowAxes((value) => !value)}>Axes</button><button type="button" className={showGrid ? 'view-toggle on' : 'view-toggle'} onClick={() => setShowGrid((value) => !value)}>Grid</button><span className="unit-badge">MM</span></div></div>
          <div className={`canvas ${isDrawing ? 'is-drawing' : ''}`}>
            <ModelViewport
              bodies={design.bodies}
              sketches={design.sketches}
              selectedId={selectedId}
              onSelect={(id) => { setSelectedId(id); if (id) setTool('inspect'); }}
              showGrid={showGrid}
              showAxes={showAxes}
              activeTool={tool}
              referenceImage={referenceImage}
              tracePoints={tracePoints}
              height={Number(dimensions.z)}
              sides={Number(primitiveSides)}
              tubeRadius={Number(torusTubeRadius)}
              onCreate={createFromDrag}
              onTracePoint={(point) => setTracePoints((points) => [...points, point])}
              onFinishTrace={finishTrace}
              onPointerState={setIsDrawing}
              onTransformEnd={applyViewportMove}
            />
            <div className="scene-tip"><span className="tip-dot" /> {tool === 'sketch-trace' ? 'Click around the reference edge; click the first point or press Enter to close' : isDrawing ? `Release to finish the ${tool.startsWith('sketch-') ? 'sketch profile' : 'solid footprint'}` : ['sketch-rectangle', 'sketch-circle'].includes(tool) ? `Click and drag to draw a ${tool === 'sketch-circle' ? 'circle from its centre' : 'rectangle'}` : ['box', 'cylinder', 'sphere'].includes(tool) ? `Click and drag on the grid to draw a ${tool}` : selectedBody ? 'Drag to orbit · Scroll to zoom · Click a body to inspect' : 'Start a sketch or choose a solid tool and draw in the viewport'}</div>
            <div className="scene-status">{selectedBody ? `${selectedBody.displayDocument?.mesh?.faces?.length ?? 0} mesh faces` : 'No selection'} <span>·</span> Orthographic-style orbit</div>
          </div>
          <div className="viewport-footer"><span><i className={`kernel-indicator ${kernelReady ? 'ready' : ''}`} /> {busy ? 'Computing geometry…' : error ? 'Operation failed' : message || 'Ready'}</span><span>Length <b>mm</b></span></div>
        </section>

        <aside className="inspector">
          <div className="panel-title"><div><small>{['box', 'cylinder', 'sphere', 'cone', 'torus', 'prism', 'pyramid', 'sketch-rectangle', 'sketch-circle', 'sketch-trace'].includes(tool) ? 'DRAW SETTINGS' : tool === 'inspect' ? 'BODY PROPERTIES' : tool === 'select' ? 'SELECTION' : 'MODELING OPERATION'}</small><h2>{['box', 'cylinder', 'sphere', 'cone', 'torus', 'prism', 'pyramid'].includes(tool) ? `Draw ${tool}` : tool === 'sketch-rectangle' ? 'Sketch rectangle' : tool === 'sketch-circle' ? 'Center circle' : tool === 'sketch-trace' ? 'Trace image outline' : tool === 'boolean' ? 'Boolean' : tool === 'move' ? 'Translate body' : tool === 'fillet' ? 'Fillet edges' : tool === 'chamfer' ? 'Chamfer edges' : selectedBody ? 'Properties' : activeSketch ? activeSketch.name : 'Select a tool'}</h2></div></div>
          {tool === 'sketch-trace' && referenceImage && referenceDraft && (
            <div className="inspector-content">
              <p className="panel-description">Set the image scale and position, then click around its outline in the viewport. Close the loop by clicking the first point or pressing Enter.</p>
              <div className="section-caption">REFERENCE IMAGE</div>
              <div className="draw-spec"><span>Image</span><strong>{referenceImage.name}</strong></div>
              <div className="trace-actions">
                <button type="button" className="primary-button" onClick={() => commitReferenceImage({ ...referenceImage, visible: referenceImage.visible === false })}>{referenceImage.visible === false ? 'Show image' : 'Hide image'}</button>
                <button type="button" className="primary-button" onClick={() => { commitReferenceImage(null); setTracePoints([]); setTool('select'); }}>Remove image</button>
              </div>
              <DimensionField label="Width" value={referenceDraft.widthMm} onChange={(widthMm) => setReferenceDraft({ ...referenceDraft, widthMm })} />
              <DimensionField label="Center X" value={referenceDraft.centerX} onChange={(centerX) => setReferenceDraft({ ...referenceDraft, centerX })} min={-10000} />
              <DimensionField label="Center Y" value={referenceDraft.centerY} onChange={(centerY) => setReferenceDraft({ ...referenceDraft, centerY })} min={-10000} />
              <label className="image-opacity-field"><span>Opacity</span><input aria-label="Reference image opacity" type="range" min="0.1" max="1" step="0.05" value={referenceDraft.opacity} onChange={(event) => setReferenceDraft({ ...referenceDraft, opacity: event.target.value })} /></label>
              <button type="button" className="primary-button full-button" onClick={applyReferenceSettings} disabled={busy}>Apply image scale and position</button>
              <div className="draw-spec trace-count"><span>Trace points</span><strong>{tracePoints.length}</strong></div>
              <div className="trace-actions">
                <button type="button" className="primary-button" onClick={finishTrace} disabled={tracePoints.length < 3 || busy}>Close outline</button>
                <button type="button" className="primary-button" onClick={() => setTracePoints((points) => points.slice(0, -1))} disabled={!tracePoints.length || busy}>Undo point</button>
              </div>
              <p className="field-note">Enter closes the outline · Backspace removes the last point · Escape cancels the current trace.</p>
              <div className="draw-spec"><span>Active sketch</span><strong>{activeSketch?.name || 'None'}</strong></div>
              <div className="draw-spec"><span>Closed profiles</span><strong>{activeSketch?.profiles.length ?? 0}</strong></div>
              <div className="sketch-profiles">
                {activeSketch?.profiles.map((profile, index) => (
                  <div className="sketch-profile" key={profile.id}>
                    <span>{index + 1}. {profile.kind === 'polygon' ? `Traced outline · ${profile.points.length} points` : profile.kind === 'circle' ? 'Circle' : 'Rectangle'}</span>
                  </div>
                ))}
              </div>
              <div className="section-caption geometry-caption">EXTRUSION</div>
              <DimensionField label="Length" value={dimensions.z} onChange={(z) => setDimensions({ ...dimensions, z })} />
              <button type="button" className="primary-button full-button" onClick={extrudeSketch} disabled={!activeSketch?.profiles.some((profile) => ['rectangle', 'polygon'].includes(profile.kind)) || busy}>{busy ? 'Computing…' : 'Extrude closed profiles'}</button>
              <p className="field-note">Tracing is manual; each closed outline extrudes as a separate solid. Inner traced loops are not cutouts yet.</p>
            </div>
          )}
          {['sketch-rectangle', 'sketch-circle'].includes(tool) && (
            <div className="inspector-content">
              {tool !== 'sketch-trace' && <><p className="panel-description">Draw closed profiles on the XY work plane. The sketch is saved separately from the solid you extrude from it.</p><div className="draw-instructions"><span className="mouse-icon"><i /><b /></span><div><strong>{tool === 'sketch-circle' ? 'Set circle centre and radius' : 'Set opposite corners'}</strong><p>{tool === 'sketch-circle' ? 'Click the centre, then drag outward to define the radius.' : 'Click and drag across the grid to create a closed rectangle.'}</p></div></div></>}
              <div className="draw-spec"><span>Active sketch</span><strong>{activeSketch?.name || 'None'}</strong></div>
              <div className="draw-spec"><span>Closed profiles</span><strong>{activeSketch?.profiles.length ?? 0}</strong></div>
              <div className="sketch-profiles">
                {activeSketch?.profiles.map((profile, index) => (
                  <div className="sketch-profile" key={profile.id}>
                    <span>{index + 1}. {profile.kind === 'circle' ? 'Circle' : profile.kind === 'polygon' ? `Traced outline · ${profile.points.length} points` : 'Rectangle'}</span>
                    <div className="profile-dimensions">
                      {profile.kind === 'circle'
                        ? profileDimensionInput(profile, 'diameter', numberText(profile.radius * 2), 'Ø')
                        : profile.kind === 'rectangle' ? <>
                          {profileDimensionInput(profile, 'width', numberText(profile.width), 'W')}
                          {profileDimensionInput(profile, 'depth', numberText(profile.depth), 'D')}
                        </> : null}
                    </div>
                  </div>
                ))}
              </div>
              <div className="section-caption geometry-caption">EXTRUSION</div>
              <DimensionField label="Length" value={dimensions.z} onChange={(z) => setDimensions({ ...dimensions, z })} />
              <button type="button" className="primary-button full-button" onClick={extrudeSketch} disabled={!activeSketch?.profiles.some((profile) => ['rectangle', 'polygon'].includes(profile.kind)) || busy}>{busy ? 'Computing…' : 'Extrude closed profiles'}</button>
              <p className="field-note">Rectangle and traced outlines create solids; circles inside a rectangle become through-holes. Open or invalid contours are rejected.</p>
            </div>
          )}
          {['box', 'cylinder', 'sphere', 'cone', 'torus', 'prism', 'pyramid'].includes(tool) && (
            <div className="inspector-content">
              {tool !== 'sphere' && tool !== 'torus' && <><div className="section-caption">HEIGHT <span>mm</span></div><DimensionField label="Height Z" value={dimensions.z} onChange={(z) => setDimensions({ ...dimensions, z })} /></>}
              {tool === 'torus' && <><div className="section-caption">TUBE SIZE <span>mm</span></div><DimensionField label="Tube radius" value={torusTubeRadius} onChange={setTorusTubeRadius} /></>}
              {['prism', 'pyramid'].includes(tool) && <><div className="section-caption">POLYGON</div><label className="dimension-field"><span>Side count</span><div><input type="number" min="3" max="128" step="1" value={primitiveSides} onChange={(event) => setPrimitiveSides(event.target.value)} /><small>sides</small></div></label></>}
              <div className="draw-instructions"><span className="mouse-icon"><i /><b /></span><div><strong>Draw in the viewport</strong><p>Click and drag on the ground grid. Your pointer movement sets the footprint dimensions in millimeters.</p></div></div>
              <div className="draw-spec"><span>Footprint</span><strong>{tool === 'box' ? 'Drag width × depth' : tool === 'torus' ? 'Drag major radius' : tool === 'sphere' ? 'Drag sphere diameter' : 'Drag diameter'}</strong></div>
              {tool !== 'sphere' && tool !== 'torus' && <div className="draw-spec"><span>Height</span><strong>{numberText(Number(dimensions.z) || 0)} mm</strong></div>}
              {!kernelReady && <p className="field-note">Loading the geometry kernel…</p>}
            </div>
          )}
          {tool === 'boolean' && (
            <div className="inspector-content">
              <p className="panel-description">Combine two analytic solids. The selected body is the target; the tool body is consumed by the operation.</p>
              <div className="section-caption">OPERATION</div>
              <div className="boolean-options">{[['union', 'Union', 'union'], ['subtract', 'Subtract', 'subtract'], ['intersect', 'Intersect', 'intersect']].map(([value, label, icon]) => <button key={value} type="button" className={booleanOp === value ? 'chosen' : ''} onClick={() => setBooleanOp(value)}><Icon name={icon} /><span>{label}</span></button>)}</div>
              <div className="section-caption">TARGET BODY</div><div className="read-only-field">{selectedBody?.name || 'Select a body in the browser'}</div>
              <label className="select-field">TOOL BODY<select value={toolId} onChange={(event) => setToolId(event.target.value)}><option value="">Choose a tool body</option>{tools.map((body) => <option key={body.id} value={body.id}>{body.name}</option>)}</select></label>
              <button type="button" className="primary-button full-button" onClick={applyBoolean} disabled={!kernelReady || busy || !toolId || selectedBodyIsMesh}>{busy ? 'Computing…' : `Apply ${booleanOp}`}</button>
              <p className="field-note">Boolean support depends on the shapes and intersections; kernel errors are shown without changing the model.</p>
            </div>
          )}
          {tool === 'move' && (
            <div className="inspector-content"><p className="panel-description">Drag the viewport gizmo to move the selected body. Use exact offsets when needed.</p><div className="section-caption">PRECISE OFFSET</div><DimensionField label="X" value={translation.x} onChange={(x) => setTranslation({ ...translation, x })} min={-10000} /><DimensionField label="Y" value={translation.y} onChange={(y) => setTranslation({ ...translation, y })} min={-10000} /><DimensionField label="Z" value={translation.z} onChange={(z) => setTranslation({ ...translation, z })} min={-10000} /><button type="button" className="primary-button full-button" onClick={applyTransform} disabled={!selectedBody || busy}>{busy ? 'Computing…' : 'Apply precise offset'}</button></div>
          )}
          {(tool === 'fillet' || tool === 'chamfer') && (
            <div className="inspector-content"><p className="panel-description">{tool === 'fillet' ? 'Round all convex edges of the selected solid.' : 'Cut all convex edges of the selected solid at 45°.'}</p><div className="section-caption">{tool === 'fillet' ? 'RADIUS' : 'DISTANCE'}</div><DimensionField label={tool === 'fillet' ? 'Fillet radius' : 'Chamfer distance'} value={edgeSize} onChange={setEdgeSize} /><div className="read-only-field">Edge selection · All convex</div><button type="button" className="primary-button full-button" onClick={() => applyEdgeTool(tool)} disabled={!selectedBody || busy || selectedBodyIsMesh}>{busy ? 'Computing…' : `Apply ${tool}`}</button><p className="field-note">Large fillets and complex intersections may not be supported by the geometry kernel.</p></div>
          )}
          {(tool === 'inspect' || tool === 'select') && selectedBody && (
            <div className="inspector-content">
              <div className="selected-body"><span className="body-color" style={{ background: selectedBody.color }} /><div><strong>{selectedBody.name}</strong><small>{selectedBodyIsMesh ? 'Faceted mesh body' : 'Analytic BREP solid'}</small></div></div>
              {selectedBodyIsMesh && <p className="field-note">This STEP part was imported as a mesh because its CAD surfaces were unsupported. You can move, resize, inspect and export it, but face-based booleans, fillets and chamfers require an analytic solid.</p>}
              <div className="section-caption">MASS PROPERTIES <span>{selectedBody.measure?.exact ? 'EXACT' : 'KERNEL'}</span></div>
              <div className="metric"><span>Volume</span><strong>{numberText(selectedBody.measure?.volume ?? 0)} <small>mm³</small></strong></div>
              <div className="metric"><span>Surface area</span><strong>{numberText(selectedBody.measure?.surfaceArea ?? 0)} <small>mm²</small></strong></div>
              {selectedBody.measure?.bounds && <>{['X', 'Y', 'Z'].map((axis, index) => <div className="metric" key={axis}><span>Bounds {axis}</span><strong>{numberText(selectedBody.measure.bounds.max[index] - selectedBody.measure.bounds.min[index])} <small>mm</small></strong></div>)}</>}
              <div className="section-caption geometry-caption">EDIT DIMENSIONS <span>mm</span></div>
              <DimensionField label="X" value={bodyDimensions.x} onChange={(x) => setBodyDimensions({ ...bodyDimensions, x })} />
              <DimensionField label="Y" value={bodyDimensions.y} onChange={(y) => setBodyDimensions({ ...bodyDimensions, y })} />
              <DimensionField label="Z" value={bodyDimensions.z} onChange={(z) => setBodyDimensions({ ...bodyDimensions, z })} />
              <button type="button" className="primary-button full-button" onClick={applyDimensions} disabled={busy}>{busy ? 'Computing…' : 'Apply dimensions'}</button>
              <div className="section-caption geometry-caption">GEOMETRY</div><div className="metric"><span>Mesh faces</span><strong>{selectedBody.displayDocument?.mesh?.faces?.length ?? '—'}</strong></div>
              <div className="inspector-actions"><button type="button" className="danger-button" onClick={deleteBody} disabled={busy}>Delete body</button></div>
            </div>
          )}
          {tool === 'select' && !selectedBody && <div className="inspector-content"><div className="selection-help"><strong>No selection</strong><p>Choose a primitive tool and draw directly on the viewport grid. Click a body to inspect its true dimensions and mass properties.</p><span>BOX · SPHERE · CYLINDER · CONE · TORUS · PRISM · PYRAMID</span></div></div>}
          {error && <div className="error-banner" role="alert"><strong>Operation failed</strong><span>{error}</span></div>}
          <div className="inspector-footer"><span>GEOMETRY ENGINE</span><strong>Pali Ijo · Dart BREP</strong></div>
        </aside>
      </div>
    </main>
  );
}

export default App;
