import 'package:pali_ijo/pali_ijo.dart';
import 'package:test/test.dart';
import 'package:vector_math/vector_math_64.dart';
void main(){
  MeshData cube()=>BrepTessellate.run(BrepPrimitives.box(10,10,10));
  test('rejects a flipped face even with no open edges',(){
    final m=cube(); m.faces[0]=m.faces[0].reversed.toList();
    expect(MeshKit.openBoundaryEdgeCount(m),0);
    expect(MeshKit.isClosedSolid(m),isFalse);
  });
  test('rejects overlapping nonmanifold triangles',(){
    final m=cube();m.faces.add(List.of(m.faces.first));
    expect(MeshKit.openBoundaryEdgeCount(m),0);
    expect(MeshKit.isClosedSolid(m),isFalse);
  });
  test('rejects nonfinite and invalid indices without throwing',(){
    final m=cube();m.vertices[0].x=double.nan;
    expect(MeshKit.isClosedSolid(m),isFalse);
    final n=cube();n.faces[0][0]=n.vertices.length;
    expect(MeshKit.isClosedSolid(n),isFalse);
  });
  test('accepts a closed cavity with inward inner walls',(){
    final s=IjoOps.hollowSolid(BrepPrimitives.box(10,10,10),[],Vector3(0,0,1),wallThicknessMm:1);
    expect(MeshKit.isClosedSolid(IjoOps.tessellateClosed(s)),isTrue);
  });
  test('closed BREP topology cannot authorize capping missing faces',(){
    final s=BrepPrimitives.cylinder(origin:Vector3.zero(),axis:Vector3(0,0,1),radius:10,height:20);
    final i=s.faces.indexWhere((f)=>f.surface is BrepPlane && f.outwardNormal(0,0).z>.9);
    // Topological edge references remain closed, but the cap is on the wrong plane.
    s.faces[i].surface=BrepPlane(origin:Vector3(0,0,19),normal:Vector3(0,0,1));
    expect(BrepHeal.topologyIsClosed(s),isTrue);
    expect(()=>IjoOps.tessellateKeep(s),throwsA(isA<BrepUnsupported>()));
  });
}
