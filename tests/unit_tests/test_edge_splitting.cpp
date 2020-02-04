//
// Created by frongere on 18/05/18.
//

#include "frydom/mesh/FrMeshClipper.h"
#include "frydom/mesh/FrMesh.h"

using namespace frydom::mesh;
using namespace frydom;


inline bool IsHalfEdgeCrossing(const FrMesh &mesh, const FrMesh::HalfedgeHandle &heh, bool down = true) {

  auto from_vh = mesh.from_vertex_handle(heh);
  auto to_vh = mesh.to_vertex_handle(heh);

  auto z_from = mesh.point(from_vh)[2];
  auto z_to = mesh.point(to_vh)[2];

  if (down) {
    return (z_from > 0 && z_to < 0);
  } else {
    return (z_from < 0 && z_to > 0);
  }

}

inline FrMesh::HalfedgeHandle GetCrossingHalfEdge(const FrMesh &mesh, const FrMesh::FaceHandle &fh, bool down = true) {
  FrMesh::HalfedgeHandle heh = mesh.halfedge_handle(fh);

  while (!IsHalfEdgeCrossing(mesh, heh, true)) {
    heh = mesh.next_halfedge_handle(heh);
  }
  return heh;
}

FrMesh::VertexHandle AddIntersectionPoint(FrMesh &mesh,
                                          const FrMesh::HalfedgeHandle &heh) {  // TODO : definir une classe de plan (ou de surface de maniere plus generale)
  auto p0 = mesh.point(mesh.from_vertex_handle(heh));
  auto p1 = mesh.point(mesh.to_vertex_handle(heh));

  double t = (p0[2] - 0.) / (p0[2] - p1[2]);

  return mesh.add_vertex(p0 * (1 - t) + p1 * t);
}


int main() {

  FrMesh mesh;

  typedef FrMesh::Point Point;
  typedef FrMesh::VertexHandle VertexHandle;

  VertexHandle vh[3];
  vh[0] = mesh.add_vertex(Point(0, 0, 1));
  vh[1] = mesh.add_vertex(Point(0, 0, -1));
  vh[2] = mesh.add_vertex(Point(1, 0, -1));
//    vh[3] = mesh.add_vertex(Point(-1,  1,  1));
//    vh[4] = mesh.add_vertex(Point(-1, -1, -1));
//    vh[5] = mesh.add_vertex(Point( 1, -1, -1));
//    vh[6] = mesh.add_vertex(Point( 1,  1, -1));
//    vh[7] = mesh.add_vertex(Point(-1,  1, -1));


  std::vector<VertexHandle> face_vhandles;

  face_vhandles.clear();
  face_vhandles.push_back(vh[0]);
  face_vhandles.push_back(vh[1]);
  face_vhandles.push_back(vh[2]);

  auto fh = mesh.add_face(face_vhandles);

  face_vhandles.clear();


  // Splitting try

  // Getting edge that cross the border
  auto heh = GetCrossingHalfEdge(mesh, fh, true);

  auto vh_new = AddIntersectionPoint(mesh, heh);

  auto eh = mesh.edge_handle(heh);

  mesh.split(eh, vh_new);
//    auto aa = mesh.is_valid_handle(heh);

  // Qui sont les points de heh ?
  std::cout << mesh.point(mesh.from_vertex_handle(heh)) << std::endl;
  std::cout << mesh.point(mesh.to_vertex_handle(heh)) << std::endl;





//    heh = GetCrossingHalfEdge(mesh, fh, false);
//    vh_new = InsertIntersectionVertex(mesh, heh);
//    eh = mesh.edge_handle(heh);
//    mesh.split(eh, vh_new);




  mesh.garbage_collection();
  mesh.Write("triangle.obj");


  return 0;
}