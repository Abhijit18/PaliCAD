import 'package:pali_ijo/pali_ijo.dart';
import 'package:test/test.dart';
import 'package:vector_math/vector_math_64.dart';
void main(){test('rim probe',(){
var s=BrepPrimitives.cylinder(origin:Vector3.zero(),axis:Vector3(0,0,1),radius:20,height:50);
s=IjoOps.filletSolid(s,[for(var i=0;i<s.edges.length;i++) if(s.edges[i].curve is BrepCircle)i],radius:5,inverse:false);
final top=s.faces.indexWhere((f)=>f.surface is BrepPlane && f.outwardNormal(0,0).z>.9);
final h=IjoOps.hollowSolid(s,[top],Vector3(0,0,1),wallThicknessMm:1);
for(var i=0;i<h.faces.length;i++){final f=h.faces[i];double err=0; for(final l in f.loops)for(final co in l.coedges){final c=h.edges[co.edge].curve; final p=c.evaluate((c.t0+c.t1)/2);final d=(p-f.surface.closestPoint(p)).length;if(d>err)err=d;} print('$i ${f.surface.runtimeType} reversed=${f.reversed} trimError=$err');}
final m=BrepTessellate.run(h); print('raw closed=${MeshKit.isClosedSolid(m)} faces=${m.faces.length}');expect(MeshKit.isClosedSolid(m),isTrue);
});}
