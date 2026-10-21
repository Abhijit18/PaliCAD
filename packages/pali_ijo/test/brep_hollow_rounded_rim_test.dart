import 'dart:convert';
import 'dart:math' as math;
import 'package:pali_ijo/pali_ijo.dart';
import 'package:test/test.dart';
import 'package:vector_math/vector_math_64.dart';

BrepSolid cylinder(bool both) {
  final s = BrepPrimitives.cylinder(origin: Vector3.zero(),
    axis: Vector3(0,0,1), radius:20, height:50);
  return IjoOps.filletSolid(s, [for(var i=0;i<s.edges.length;i++)
    if(s.edges[i].curve is BrepCircle &&
      (both || (s.edges[i].curve as BrepCircle).center.z > 49)) i],
    radius:5, inverse:false);
}

void checkShell(BrepSolid solid) {
  expect(BrepHeal.topologyIsClosed(solid), isTrue);
  for (final face in solid.faces) {
    for(final loop in face.loops) for(final co in loop.coedges) {
      final c=solid.edges[co.edge].curve;
      for(final fraction in [0.0,.25,.5,.75,1.0]) {
        final p=c.evaluate(c.t0+(c.t1-c.t0)*fraction);
        expect((face.surface.closestPoint(p)-p).length,lessThan(1e-6),
          reason:'Every trim must lie on its supporting surface');
      }
    }
  }
  final mesh=BrepTessellate.run(solid);
  expect(MeshKit.isClosedSolid(mesh),isTrue,
    reason:'Raw tessellation must be manifold and coherently oriented without repair');
  expect(MeshKit.signedVolume(mesh),greaterThan(0));
  for(var i=0;i<mesh.faces.length;i++) {
    final f=mesh.faces[i], face=solid.faces[mesh.faceIds![i]];
    final a=mesh.vertices[f[0]],b=mesh.vertices[f[1]],c=mesh.vertices[f[2]];
    final uv=face.surface.parameters((a+b+c)/3)!;
    expect((b-a).cross(c-a).dot(face.outwardNormal(uv.$1,uv.$2)),greaterThan(0));
  }
  expect(MeshKit.isClosedSolid(IjoOps.tessellateKeep(solid)),isTrue);
}

void main() {
  for(final both in [false,true]) for(final bottom in [false,true])
  for(final thickness in [.1,1.0]) for(final rotated in [false,true]) {
    test('rounded hollow both=$both bottom=$bottom wall=$thickness rotated=$rotated',(){
      final q=Quaternion.axisAngle(Vector3(1,2,3).normalized(),rotated ? .63 : 0);
      Vector3 place(Vector3 p)=>q.rotated(p)+Vector3(123,-90,42);
      final original=BrepXform.map(cylinder(both),place);
      final n=q.rotated(Vector3(0,0,bottom?-1:1));
      final opening=original.faces.indexWhere((f)=>f.surface is BrepPlane && f.outwardNormal(0,0).dot(n)>.99);
      final before=jsonEncode(original.toJson());
      final shell=IjoOps.hollowSolid(original,[opening],n,wallThicknessMm:thickness);
      expect(jsonEncode(original.toJson()),before);
      checkShell(shell);
      checkShell(BrepSolid.fromJson(jsonDecode(jsonEncode(shell.toJson()))));
      if(both) {
        double band(double r)=>225*r+30*r*r*math.pi/4+2*r*r*r/3;
        final expected=math.pi*(400*40+2*band(5)-
          ((20-thickness)*(20-thickness)*40+2*band(5-thickness))-225*thickness);
        expect(IjoMeasure.volumeSolid(shell),closeTo(expected,1e-5));
      }
    });
  }
  for(final stages in [1,2]) for(final bottom in [false,true]) {
    test('hollow after $stages cap extrusions bottom=$bottom',(){
      var s=cylinder(true);
      var z=50.0;
      for(final (r,h) in [(15.0,5.0),(17.5,8.0)].take(stages)) {
        s=IjoOps.booleanSolid(s,BrepPrimitives.cylinder(origin:Vector3(0,0,z),axis:Vector3(0,0,1),radius:r,height:h),IjoOps.unionOp);
        z+=h;
      }
      final n=Vector3(0,0,bottom?-1:1);
      final opening=s.faces.indexWhere((f)=>f.surface is BrepPlane && f.outwardNormal(0,0).dot(n)>.99 && ((f.surface as BrepPlane).origin.z-(bottom?0:z)).abs()<1e-6);
      checkShell(IjoOps.hollowSolid(s,[opening],n,wallThicknessMm:1));
    });
  }
}
