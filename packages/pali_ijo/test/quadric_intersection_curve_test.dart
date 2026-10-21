import 'dart:convert';
import 'package:pali_ijo/pali_ijo.dart';
import 'package:test/test.dart';
import 'package:vector_math/vector_math_64.dart';

void main() {
  final carrier=BrepCylinder(origin:Vector3.zero(),axis:Vector3(0,0,1),radius:1);
  final targets=<String,BrepSurface>{
    'unequal crossing cylinder':BrepCylinder(origin:Vector3(.1,.2,3),axis:Vector3(1,0,0),radius:2),
    'skew cylinder':BrepCylinder(origin:Vector3(.1,.2,3),axis:Vector3(1,2,3),radius:2),
    'tiny near-tangent loops':BrepCylinder(origin:Vector3(0,1.999999,0),axis:Vector3(1,0,0),radius:1),
    'off-axis sphere':BrepSphere(center:Vector3(.3,.5,4),radius:2),
  };
  for(final target in targets.entries) {
    test('${target.key}: exact shared curve and native roundtrip',(){
      final curves=BrepIntersect.quadricBranches(carrier,target.value);
      expect(curves,isNotEmpty);
      for(final c in curves) {
        final restored=BrepCurve.fromJson(jsonDecode(jsonEncode(c.toJson())) as Map<String,dynamic>);
        for(var i=0;i<=80;i++) {
          final t=c.t0+(c.t1-c.t0)*i/80,p=c.evaluate(t);
          expect((carrier.closestPoint(p)-p).length,lessThan(1e-7));
          expect((target.value.closestPoint(p)-p).length,lessThan(1e-7));
          expect((restored.evaluate(t)-p).length,lessThan(1e-7));
          if(i>0&&i<80) {
            final tangent=c.tangent(t);
            final uv=target.value.parameters(p)!;
            expect(tangent.dot(carrier.normalAt(t,0)).abs(),lessThan(1e-7));
            expect(tangent.dot(target.value.normalAt(uv.$1,uv.$2)).abs(),lessThan(1e-7));
          }
        }
      }
    });
  }
  test('disjoint quadrics produce no real branch',(){
    expect(BrepIntersect.quadricBranches(carrier,
      BrepCylinder(origin:Vector3(0,3,0),axis:Vector3(1,0,0),radius:1)),isEmpty);
  });
  test('intersection curves survive rotation, uniform scaling and reflection',(){
    final c=BrepIntersect.quadricBranches(carrier,targets.values.first).first;
    final s=BrepSolid(vertices:[c.evaluate(c.t0),c.evaluate(c.t1)],edges:[BrepEdge(v0:0,v1:1,curve:c)],faces:[]);
    final q=Quaternion.axisAngle(Vector3(1,2,3).normalized(),.73);
    Vector3 map(Vector3 p)=>q.rotated(p)*3+Vector3(25,-13,7);
    final transformed=BrepXform.map(s,map).edges.single.curve;
    final mirrored=BrepXform.mirror(s,Vector3.zero(),Vector3(1,0,0)).edges.single.curve;
    for(var i=0;i<=20;i++) {
      final t=c.t0+(c.t1-c.t0)*i/20,p=c.evaluate(t);
      expect((transformed.evaluate(t)-map(p)).length,lessThan(1e-7));
      expect((mirrored.evaluate(t)-Vector3(-p.x,p.y,p.z)).length,lessThan(1e-7));
    }
  });
}
