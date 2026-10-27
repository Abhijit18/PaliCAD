import { Suspense, useEffect, useMemo, useRef, useState } from 'react';
import { Canvas, useLoader, useThree } from '@react-three/fiber';
import { Bounds, Grid, OrbitControls, TransformControls } from '@react-three/drei';
import * as THREE from 'three';

function InteractiveSolid({ body, selected, movable, drawing, onSelect, onTransformEnd }) {
  const group = useRef(null);
  const bounds = body.measure?.bounds;
  const pivot = bounds
    ? [
      (bounds.min[0] + bounds.max[0]) / 2,
      (bounds.min[2] + bounds.max[2]) / 2,
      -(bounds.min[1] + bounds.max[1]) / 2,
    ]
    : [0, 0, 0];
  const geometry = useMemo(() => {
    const mesh = body.displayDocument?.mesh;
    if (!mesh) return null;
    const positions = [];
    for (const face of mesh.faces) {
      for (let index = 1; index < face.length - 1; index += 1) {
        for (const vertexIndex of [face[0], face[index], face[index + 1]]) {
          const vertex = mesh.vertices[vertexIndex];
          positions.push(vertex[0] - pivot[0], vertex[2] - pivot[1], -vertex[1] - pivot[2]);
        }
      }
    }
    const buffer = new THREE.BufferGeometry();
    buffer.setAttribute('position', new THREE.Float32BufferAttribute(positions, 3));
    buffer.computeVertexNormals();
    return buffer;
  }, [body.displayDocument, pivot[0], pivot[1], pivot[2]]);

  if (!geometry || body.visible === false) return null;
  const color = selected ? '#dba47b' : body.color;
  const solid = (
    <group ref={group} position={pivot} onClick={drawing ? undefined : (event) => { event.stopPropagation(); onSelect(body.id); }}>
      <mesh geometry={geometry}>
        <meshStandardMaterial color={color} roughness={0.62} metalness={0.12} side={THREE.DoubleSide} />
      </mesh>
      <lineSegments geometry={new THREE.EdgesGeometry(geometry, 26)}>
        <lineBasicMaterial color={selected ? '#f3c096' : '#27342b'} transparent opacity={selected ? 0.8 : 0.24} />
      </lineSegments>
    </group>
  );
  return movable ? (
    <TransformControls
      mode="translate"
      onMouseUp={() => {
        if (group.current) {
          onTransformEnd(body.id, group.current.position.toArray().map((value, index) => value - pivot[index]));
        }
      }}
    >
      {solid}
    </TransformControls>
  ) : solid;
}

function SketchOutline({ profile, preview = false }) {
  const points = useMemo(() => {
    if (profile.kind === 'polygon') {
      return profile.points.map(([x, y]) => [x, 0.025, -y]);
    }
    if (profile.kind === 'circle' || profile.kind === 'sketch-circle') {
      const radius = profile.radius;
      return Array.from({ length: 64 }, (_, index) => {
        const angle = (index / 64) * Math.PI * 2;
        return [profile.x + Math.cos(angle) * radius, 0.025, -(profile.y + Math.sin(angle) * radius)];
      });
    }
    const halfWidth = profile.width / 2;
    const halfDepth = profile.depth / 2;
    return [
      [profile.x - halfWidth, 0.025, -(profile.y - halfDepth)],
      [profile.x + halfWidth, 0.025, -(profile.y - halfDepth)],
      [profile.x + halfWidth, 0.025, -(profile.y + halfDepth)],
      [profile.x - halfWidth, 0.025, -(profile.y + halfDepth)],
    ];
  }, [profile]);
  const geometry = useMemo(() => new THREE.BufferGeometry().setFromPoints(
    points.map(([x, y, z]) => new THREE.Vector3(x, y, z)),
  ), [points]);

  return (
    <lineLoop geometry={geometry} raycast={() => null}>
      <lineBasicMaterial color={preview ? '#f0b68a' : '#81b8df'} transparent opacity={preview ? 0.9 : 0.85} />
    </lineLoop>
  );
}

function ReferenceImage({ image }) {
  const texture = useLoader(THREE.TextureLoader, image.dataUrl);
  const height = image.widthMm * image.pixelHeight / image.pixelWidth;
  texture.colorSpace = THREE.SRGBColorSpace;
  if (image.visible === false) return null;
  return (
    <mesh
      position={[image.centerX, 0.005, -image.centerY]}
      rotation={[-Math.PI / 2, 0, 0]}
      raycast={() => null}
    >
      <planeGeometry args={[image.widthMm, height]} />
      <meshBasicMaterial map={texture} transparent opacity={image.opacity} depthWrite={false} side={THREE.DoubleSide} />
    </mesh>
  );
}

function TracePreview({ points, cursor }) {
  const geometry = useMemo(() => {
    const linePoints = points.map(([x, y]) => new THREE.Vector3(x, 0.04, -y));
    if (cursor) linePoints.push(new THREE.Vector3(cursor[0], 0.04, -cursor[1]));
    return new THREE.BufferGeometry().setFromPoints(linePoints);
  }, [points, cursor]);
  if (!points.length) return null;
  return <line geometry={geometry} raycast={() => null}><lineBasicMaterial color="#efac79" /></line>;
}

function SketchCamera({ active }) {
  const { camera, controls } = useThree();
  const saved = useRef(null);

  useEffect(() => {
    if (!controls) return;
    if (active && !saved.current) {
      saved.current = {
        position: camera.position.toArray(),
        up: camera.up.toArray(),
        target: controls.target.toArray(),
      };
      camera.position.set(0, 200, 0);
      camera.up.set(0, 0, -1);
      controls.target.set(0, 0, 0);
      controls.update();
    } else if (!active && saved.current) {
      camera.position.fromArray(saved.current.position);
      camera.up.fromArray(saved.current.up);
      controls.target.fromArray(saved.current.target);
      controls.update();
      saved.current = null;
    }
  }, [active, camera, controls]);

  return null;
}

function DrawingPlane({ tool, height, sides, tubeRadius, tracePoints, onCreate, onTracePoint, onFinishTrace, onPointerState }) {
  const [preview, setPreview] = useState(null);
  const [traceCursor, setTraceCursor] = useState(null);
  const start = useRef(null);
  const { gl } = useThree();

  const updatePreview = (point) => {
    if (!start.current || !point) return;
    setPreview({ start: start.current.clone(), end: point.clone() });
  };

  const finishDrawing = (event) => {
    if (!start.current) return;
    const end = event.point.clone();
    const origin = start.current;
    start.current = null;
    setPreview(null);
    onPointerState(false);
    gl.domElement.style.cursor = 'default';
    const dx = end.x - origin.x;
    const dz = end.z - origin.z;
    if (Math.hypot(dx, dz) < 0.08) return;
    const diagonal = Math.hypot(dx, dz);
    onCreate({
      kind: tool,
      x: tool === 'sketch-circle' ? origin.x : (origin.x + end.x) / 2,
      y: tool === 'sketch-circle' ? -origin.z : -(origin.z + end.z) / 2,
      width: Math.abs(dx),
      depth: Math.abs(dz),
      diagonal,
      radius: diagonal,
      height,
    });
  };

  const onDown = (event) => {
    if (!tool || event.button !== 0) return;
    event.stopPropagation();
    if (tool === 'sketch-trace') {
      const point = [event.point.x, -event.point.z];
      const first = tracePoints[0];
      if (tracePoints.length >= 3 && first && Math.hypot(point[0] - first[0], point[1] - first[1]) <= 0.8) {
        onFinishTrace();
      } else {
        onTracePoint(point);
      }
      return;
    }
    start.current = event.point.clone();
    setPreview({ start: event.point.clone(), end: event.point.clone() });
    onPointerState(true);
    gl.domElement.style.cursor = 'crosshair';
    event.target.setPointerCapture?.(event.pointerId);
  };

  const onMove = (event) => {
    if (tool === 'sketch-trace') {
      setTraceCursor([event.point.x, -event.point.z]);
      return;
    }
    if (!start.current) return;
    event.stopPropagation();
    updatePreview(event.point);
  };

  const onUp = (event) => {
    if (!start.current) return;
    event.stopPropagation();
    finishDrawing(event);
    event.target.releasePointerCapture?.(event.pointerId);
  };

  const boxPreview = preview && ['box', 'sketch-rectangle'].includes(tool) ? {
    x: (preview.start.x + preview.end.x) / 2,
    y: height / 2,
    z: (preview.start.z + preview.end.z) / 2,
    width: Math.abs(preview.end.x - preview.start.x),
    depth: Math.abs(preview.end.z - preview.start.z),
  } : null;
  const radialPreview = preview && tool !== 'box' ? {
    x: (preview.start.x + preview.end.x) / 2,
    y: tool === 'sphere' ? Math.hypot(preview.end.x - preview.start.x, preview.end.z - preview.start.z) / 2 : tool === 'torus' ? tubeRadius : height / 2,
    z: (preview.start.z + preview.end.z) / 2,
    radius: Math.hypot(preview.end.x - preview.start.x, preview.end.z - preview.start.z) / 2,
  } : null;
  const sketchPreview = preview && tool?.startsWith('sketch-')
    ? {
      kind: tool,
      x: tool === 'sketch-circle' ? preview.start.x : (preview.start.x + preview.end.x) / 2,
      y: tool === 'sketch-circle' ? -preview.start.z : -(preview.start.z + preview.end.z) / 2,
      width: Math.abs(preview.end.x - preview.start.x),
      depth: Math.abs(preview.end.z - preview.start.z),
      radius: Math.hypot(preview.end.x - preview.start.x, preview.end.z - preview.start.z),
    }
    : null;

  return (
    <>
      <mesh
        rotation={[-Math.PI / 2, 0, 0]}
        position={[0, -0.035, 0]}
        onPointerDown={tool ? onDown : undefined}
        onPointerMove={tool ? onMove : undefined}
        onPointerUp={tool ? onUp : undefined}
        onPointerCancel={() => { start.current = null; setPreview(null); onPointerState(false); gl.domElement.style.cursor = 'default'; }}
        visible
      >
        <planeGeometry args={[10000, 10000]} />
        <meshBasicMaterial transparent opacity={0} depthWrite={false} side={THREE.DoubleSide} />
      </mesh>
      {tool === 'sketch-trace' && <TracePreview points={tracePoints} cursor={traceCursor} />}
      {boxPreview && boxPreview.width > 0.01 && boxPreview.depth > 0.01 && (
        tool === 'sketch-rectangle'
          ? <SketchOutline profile={{ ...sketchPreview, kind: 'rectangle' }} preview />
          : <mesh position={[boxPreview.x, boxPreview.y, boxPreview.z]}>
            <boxGeometry args={[boxPreview.width, height, boxPreview.depth]} />
            <meshBasicMaterial color="#e2aa7e" transparent opacity={0.25} wireframe />
          </mesh>
      )}
      {sketchPreview?.kind === 'sketch-circle' && sketchPreview.radius > 0.01 && (
        <SketchOutline profile={{ ...sketchPreview, kind: 'circle' }} preview />
      )}
      {!tool?.startsWith('sketch-') && radialPreview && radialPreview.radius > 0.01 && (
        <mesh position={[radialPreview.x, radialPreview.y, radialPreview.z]}>
          {tool === 'sphere'
            ? <sphereGeometry args={[radialPreview.radius, 24, 16]} />
            : tool === 'torus'
              ? <torusGeometry args={[radialPreview.radius, tubeRadius, 12, 48]} />
              : tool === 'cone'
                ? <coneGeometry args={[radialPreview.radius, height, 32]} />
                : tool === 'pyramid'
                  ? <coneGeometry args={[radialPreview.radius, height, sides]} />
                  : <cylinderGeometry args={[radialPreview.radius, radialPreview.radius, height, tool === 'prism' ? sides : 32]} />}
          <meshBasicMaterial color="#e2aa7e" transparent opacity={0.32} wireframe />
        </mesh>
      )}
    </>
  );
}

export default function ModelViewport({ bodies, sketches, selectedId, onSelect, showGrid, showAxes, activeTool, referenceImage, tracePoints, height, sides, tubeRadius, onCreate, onTracePoint, onFinishTrace, onPointerState, onTransformEnd }) {
  const isDrawing = ['box', 'cylinder', 'sphere', 'cone', 'torus', 'prism', 'pyramid', 'sketch-rectangle', 'sketch-circle', 'sketch-trace'].includes(activeTool);
  return (
    <Canvas
      camera={{ position: [120, 115, 150], fov: 40, near: 0.1, far: 10000 }}
      dpr={[1, 1.75]}
      gl={{ antialias: true, preserveDrawingBuffer: true }}
      onPointerMissed={() => onSelect(null)}
    >
      <color attach="background" args={['#111612']} />
      <ambientLight intensity={1.4} />
      <directionalLight position={[120, 160, 90]} intensity={2.2} />
      <directionalLight position={[-90, 50, -120]} intensity={0.75} />
      <Suspense fallback={null}>
        {showGrid && (
          <Grid
            args={[1200, 1200]}
            cellSize={10}
            cellThickness={0.55}
            cellColor="#354038"
            sectionSize={100}
            sectionThickness={1}
            sectionColor="#586b5b"
            fadeDistance={1200}
            fadeStrength={1.5}
            infiniteGrid
            position={[0, -0.02, 0]}
          />
        )}
        {showAxes && <axesHelper args={[70]} rotation={[-Math.PI / 2, 0, 0]} />}
        {referenceImage && <ReferenceImage image={referenceImage} />}
        {sketches.filter((sketch) => sketch.visible !== false).flatMap((sketch) => sketch.profiles.map((profile) => (
          <SketchOutline key={`${sketch.id}-${profile.id}`} profile={profile} />
        )))}
        <DrawingPlane tool={isDrawing ? activeTool : null} height={height} sides={sides} tubeRadius={tubeRadius} tracePoints={tracePoints} onCreate={onCreate} onTracePoint={onTracePoint} onFinishTrace={onFinishTrace} onPointerState={onPointerState} />
        {bodies.length > 0 && (
          <Bounds fit clip observe margin={1.45}>
            <group>
              {bodies.map((body) => (
                <InteractiveSolid
                  key={`${body.id}-${body.version ?? 0}`}
                  body={body}
                  selected={body.id === selectedId}
                  movable={body.id === selectedId && activeTool === 'move'}
                  drawing={isDrawing}
                  onSelect={onSelect}
                  onTransformEnd={onTransformEnd}
                />
              ))}
            </group>
          </Bounds>
        )}
      </Suspense>
      <OrbitControls makeDefault enabled={!isDrawing} enableDamping dampingFactor={0.08} />
      <SketchCamera active={['sketch-rectangle', 'sketch-circle', 'sketch-trace'].includes(activeTool)} />
    </Canvas>
  );
}
