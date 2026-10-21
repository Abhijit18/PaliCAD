import 'dart:convert';
import 'dart:io';
import 'package:pali_ijo/pali_ijo.dart';
import 'package:pali_ijo/src/brep/boundary_boolean.dart';
import 'package:test/test.dart';
import 'package:vector_math/vector_math_64.dart';

void main() {
  for (final angle in [-2.0, -1.0, 0.0, .63]) {
  test('longitudinal bore crossing existing circle and polygon holes rotation=$angle', () {
    var base = BrepSolid.fromJson(jsonDecode(File(
      'test/fixtures/crossing_bore_base.json',
    ).readAsStringSync()) as Map<String, dynamic>);
    var tool = BrepXform.map(
      BrepPrimitives.cylinder(origin: Vector3.zero(), axis: Vector3(0,0,1), radius: 4.125367939445581, height: 62.73),
      (p) => Vector3(p.z, p.x + 9.943785302249786, p.y + 9.78327754683141),
    );
    if(angle==-1) tool=BrepPrimitives.cylinder(origin:Vector3(63.129999999999995,9.943785302249784,9.783277546831409),axis:Vector3(-1,0,0),radius:4.125367939445578,height:63.529999999999994);
    final q=Quaternion.axisAngle(Vector3(1,2,3).normalized(),angle);
    Vector3 map(Vector3 p)=>angle==-1?p:q.rotated(p)+Vector3(31,-17,9);
    base=BrepXform.map(base,map);tool=BrepXform.map(tool,map);
    if(angle==-2) {
      final operands=jsonDecode(File('test/fixtures/crossing_bore_app_operands.json').readAsStringSync()) as List;
      base=BrepSolid.fromJson(operands[0] as Map<String,dynamic>);
      tool=BrepSolid.fromJson(operands[1] as Map<String,dynamic>);
    }
    final result = BrepBoundaryBoolean.subtract(base, tool);
    for(final f in result.faces) {
      for(final l in f.loops) {for(final co in l.coedges) {
        final c=result.edges[co.edge].curve;
        for(var i=0;i<=8;i++) {
          final p=c.evaluate(c.t0+(c.t1-c.t0)*i/8);
          expect((f.surface.closestPoint(p)-p).length,lessThan(1e-6));
        }
      }}
    }
    File('/tmp/crossing-boundary-$angle.json').writeAsStringSync(jsonEncode(result.toJson()));
    print('out faces ${result.faces.length} edges ${result.edges.length}');
    final uses=<int,List<int>>{};
    for(var i=0;i<result.faces.length;i++) { for(final l in result.faces[i].loops) {for(final c in l.coedges) {(uses[c.edge]??=[]).add(i);}}}
    for(final e in uses.entries.where((e)=>e.value.length!=2)) {print('unpaired ${e.key} ${e.value} ${result.edges[e.key].curve.toJson()}');}
    expect(BrepHeal.topologyIsClosed(result), isTrue);
    final mesh=BrepTessellate.run(result);
    print('mesh ${mesh.faces.length} volume ${MeshKit.signedVolume(mesh)}');
    final stats=<int,List<double>>{};
    final usesM=<String,List<int>>{};
    for(var i=0;i<mesh.faces.length;i++) {
      final t=mesh.faces[i],fi=mesh.faceIds![i];
      final stat=stats[fi]??=[0,0];stat[0]++;
      stat[1]+=mesh.vertices[t[0]].dot(mesh.vertices[t[1]].cross(mesh.vertices[t[2]]))/6;
      for(var j=0;j<3;j++) {final a=t[j],b=t[(j+1)%3];final k=a<b?'$a:$b':'$b:$a';(usesM[k]??=[]).add(fi);}
    }
    final open=usesM.values.where((v)=>v.length!=2).expand((v)=>v).toList();
    File('/tmp/stats-$angle.json').writeAsStringSync(jsonEncode({'stats':stats.map((k,v)=>MapEntry('$k',v)),'open':open}));
    expect(MeshKit.isClosedSolid(mesh), isTrue);
    expect(MeshKit.signedVolume(mesh),closeTo(13360.55,15));
    final restored=BrepSolid.fromJson(jsonDecode(jsonEncode(result.toJson())) as Map<String,dynamic>);
    for(var i=0;i<result.edges.length;i++) {
      final c=result.edges[i].curve, d=restored.edges[i].curve;
      for(final t in [c.t0,(c.t0+c.t1)/2,c.t1]) {expect((c.evaluate(t)-d.evaluate(t)).length,lessThan(1e-7));}
    }
    expect(IjoOps.splitBrep(result), hasLength(1));
  });
  }
}
