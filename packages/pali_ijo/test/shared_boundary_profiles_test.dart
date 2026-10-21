import 'package:pali_ijo/pali_ijo.dart';
import 'package:test/test.dart';
import 'package:vector_math/vector_math_64.dart';

void main() {
  for(final profile in ['circle','rounded','chamfered','concave']) {
    test('crossing an existing bore with a $profile profile',(){
      final box=BrepPrimitives.box(20,20,20);
      final old=BrepPrimitives.cylinder(origin:Vector3(8,-5,10),axis:Vector3(0,1,0),radius:3,height:30);
      final base=IjoOps.booleanSolid(box,old,IjoOps.subtractOp);
      BrepSolid tool;
      if(profile=='circle') {
        tool=BrepPrimitives.cylinder(origin:Vector3(-5,10,10),axis:Vector3(1,0,0),radius:4,height:30);
      } else if(profile=='rounded') {
        var prism=BrepPrimitives.box(8,8,30);
        final edges=[for(var i=0;i<prism.edges.length;i++)
          if((prism.vertices[prism.edges[i].v0].z-prism.vertices[prism.edges[i].v1].z).abs()>29) i];
        prism=IjoOps.filletSolid(prism,edges,radius:1,inverse:false);
        tool=BrepXform.map(prism,(p)=>Vector3(p.z-5,p.x+6,p.y+6));
      } else {
        final xy=profile=='chamfered'?<Vector2>[
          Vector2(7,6),Vector2(13,6),Vector2(14,7),Vector2(14,13),Vector2(13,14),Vector2(7,14),Vector2(6,13),Vector2(6,7)
        ]:<Vector2>[Vector2(6,6),Vector2(14,6),Vector2(14,9),Vector2(10,9),Vector2(10,14),Vector2(6,14)];
        tool=BrepExtrude.polygon([for(final p in xy) Vector3(-5,p.x,p.y)],Vector3(30,0,0));
      }
      final result=IjoOps.booleanSolid(base,tool,IjoOps.subtractOp);
      if(!BrepHeal.topologyIsClosed(result)) {
        final uses=<int,List<int>>{};
        for(var i=0;i<result.faces.length;i++) {for(final l in result.faces[i].loops) {for(final co in l.coedges) {(uses[co.edge]??=[]).add(i);}}}
        for(final e in uses.entries.where((e)=>e.value.length!=2)) {print('bad ${e.value} ${result.edges[e.key].curve.toJson()}');}
      }
      expect(BrepHeal.topologyIsClosed(result),isTrue);
      final mesh=BrepTessellate.run(result);
      expect(MeshKit.isClosedSolid(mesh),isTrue);
      expect(IjoOps.splitBrep(result),hasLength(1));
      expect(MeshKit.signedVolume(mesh),lessThan(MeshKit.signedVolume(BrepTessellate.run(base))));
    });
  }
}
