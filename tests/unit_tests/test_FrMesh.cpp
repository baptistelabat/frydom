//
// Created by lletourn on 19/07/19.
//

#include "frydom/frydom.h"

#include "gtest/gtest.h"

using namespace frydom;
using namespace geom;


class TestMesh : public ::testing::Test, public mesh::FrMesh {

 protected:

  FRAME_CONVENTION fc = NWU;

  std::shared_ptr<geom::FrPlane> m_plane;

 protected:

  void SetUp() override;

  void CreateCustomMesh(mesh::FrMesh &mesh, double Lx, double Ly, double Lz);

  void SetClippingPlane(const Position &planeOrigin, const Direction &planeNormal);

  void Clip();

  void TestIntegrals(double volume, const Position &COG);

};

void TestMesh::CreateCustomMesh(mesh::FrMesh &mesh, double Lx, double Ly, double Lz) {

  // generate vertices
  mesh::FrMesh::VertexHandle vhandle[8];
  vhandle[0] = mesh.add_vertex(mesh::FrMesh::Point(-Lx, -Ly, Lz) * .5);
//    vhandle[1] = mesh.add_vertex(mesh::FrMesh::Point( Lx, -Ly,  Lz)*.5);
//    vhandle[2] = mesh.add_vertex(mesh::FrMesh::Point( Lx,  Ly,  Lz)*.5);
  vhandle[1] = mesh.add_vertex(mesh::FrMesh::Point(-Lx, Ly, Lz) * .5);
  vhandle[2] = mesh.add_vertex(mesh::FrMesh::Point(-Lx, -Ly, -Lz) * .5);
  vhandle[3] = mesh.add_vertex(mesh::FrMesh::Point(Lx, -Ly, -Lz) * .5);
  vhandle[4] = mesh.add_vertex(mesh::FrMesh::Point(Lx, Ly, -Lz) * .5);
  vhandle[5] = mesh.add_vertex(mesh::FrMesh::Point(-Lx, Ly, -Lz) * .5);

  // generate (triangular) faces
  std::vector<mesh::FrMesh::VertexHandle> face_vhandles;

  face_vhandles.clear();
  face_vhandles.push_back(vhandle[0]);
  face_vhandles.push_back(vhandle[1]);
  face_vhandles.push_back(vhandle[2]);
  mesh.add_face(face_vhandles);

  face_vhandles.clear();
  face_vhandles.push_back(vhandle[1]);
  face_vhandles.push_back(vhandle[5]);
  face_vhandles.push_back(vhandle[2]);
  mesh.add_face(face_vhandles);

  face_vhandles.clear();
  face_vhandles.push_back(vhandle[1]);
  face_vhandles.push_back(vhandle[4]);
  face_vhandles.push_back(vhandle[5]);
  mesh.add_face(face_vhandles);

  face_vhandles.clear();
  face_vhandles.push_back(vhandle[0]);
  face_vhandles.push_back(vhandle[2]);
  face_vhandles.push_back(vhandle[3]);
  mesh.add_face(face_vhandles);

  face_vhandles.clear();
  face_vhandles.push_back(vhandle[4]);
  face_vhandles.push_back(vhandle[3]);
  face_vhandles.push_back(vhandle[2]);
  mesh.add_face(face_vhandles);

  face_vhandles.clear();
  face_vhandles.push_back(vhandle[5]);
  face_vhandles.push_back(vhandle[4]);
  face_vhandles.push_back(vhandle[2]);
  mesh.add_face(face_vhandles);

  face_vhandles.clear();
  face_vhandles.push_back(vhandle[0]);
  face_vhandles.push_back(vhandle[3]);
  face_vhandles.push_back(vhandle[4]);
  mesh.add_face(face_vhandles);

  face_vhandles.clear();
  face_vhandles.push_back(vhandle[4]);
  face_vhandles.push_back(vhandle[1]);
  face_vhandles.push_back(vhandle[0]);
  mesh.add_face(face_vhandles);

  mesh.UpdateAllProperties();

}

void TestMesh::SetUp() {

  CreateBox(10, 10, 10);

}

void TestMesh::SetClippingPlane(const Position &planeOrigin, const Direction &planeNormal) {
  m_plane = std::make_shared<geom::FrPlane>(planeOrigin, planeNormal, fc);
}

void TestMesh::Clip() {

//    Position origin(5.,5.,5.);
//
//    Direction normal(1.,0.,1.);
//    normal.normalize();
//
//    auto plane = std::make_shared<geom::FrPlane>(origin, normal, fc);

  auto clippingSurface = std::make_shared<mesh::FrClippingPlane>(m_plane);

  mesh::FrMeshClipper clipper;
  clipper.SetClippingSurface(clippingSurface);

  clipper.Apply(this);

}


void TestMesh::TestIntegrals(double volume, const Position &COG) {

  auto normal = m_plane->GetNormal(fc);
  auto origin = m_plane->GetOrigin(fc);

  // Test on polygon
  auto polygonSet = GetBoundaryPolygonSet();

  EXPECT_TRUE(!polygonSet.empty());

  auto polygon = polygonSet[0];

  auto planeTest = polygon.GetPlane();
  Direction testNormal = normal - planeTest.GetNormal(fc);
  EXPECT_NEAR(testNormal.norm(), 0, 1E-8);

  EXPECT_NEAR(planeTest.GetDistanceToPoint(origin, fc), 0, 1E-8);

  // Test on integrals
//    std::cout<<"Volume:"<<GetVolume()<<std::endl;
  EXPECT_NEAR(std::abs(GetVolume() - volume) / std::abs(volume), 0, 1E-5);

//    std::cout<<"G : ("<<GetCOG().GetX()<<","<<GetCOG().GetY()<<","<<GetCOG().GetZ()<<")"<<std::endl;

//    double h = 5./3.;
//    Position testCOG = origin + Position(-h,0.,-h) - GetCOG();
//    Position testCOG = Position(2*h,5.,2*h) - mesh.GetCOG();
  Position testCOG = origin + COG - GetCOG();
  EXPECT_NEAR(testCOG.norm(), 0, 1E-5);

};

TEST_F(TestMesh, X) {

  SetClippingPlane(Position(), Direction(1., 0., 0.));
  Clip();
  TestIntegrals(500, Position(-2.5, 0., 0.));

}

TEST_F(TestMesh, Y) {

  SetClippingPlane(Position(), Direction(0., 1., 0.));
  Clip();
  TestIntegrals(500, Position(0., -2.5, 0.));

}

TEST_F(TestMesh, Z) {

  SetClippingPlane(Position(), Direction(0., 0., 1.));
  Clip();
  TestIntegrals(500, Position(0., 0., -2.5));

}

TEST_F(TestMesh, XZ) {

  Direction normal(1., 0., 1.);
  normal.normalize();

  SetClippingPlane(Position(), normal);
  Clip();
  double h = 5. / 3.;
  TestIntegrals(500, Position(-h, 0., -h));

}


TEST_F(TestMesh, Xtrans) {

  Translate({5., 5., 5.});
  SetClippingPlane(Position(5., 5., 5.), Direction(1., 0., 0.));
  Clip();
  TestIntegrals(500, Position(-2.5, 0., 0.));

}

TEST_F(TestMesh, Ytrans) {

  Translate({5., 5., 5.});
  SetClippingPlane(Position(5., 5., 5.), Direction(0., 1., 0.));
  Clip();
  TestIntegrals(500, Position(0., -2.5, 0.));

}

TEST_F(TestMesh, Ztrans) {

  Translate({5., 5., 5.});
  SetClippingPlane(Position(5., 5., 5.), Direction(0., 0., 1.));
  Clip();
  TestIntegrals(500, Position(0., 0., -2.5));

}

TEST_F(TestMesh, XZtrans) {

  Translate({5., 5., 5.});
  Direction normal(1., 0., 1.);
  normal.normalize();

  SetClippingPlane(Position(5., 5., 5.), normal);
  Clip();
//    Write("XZtrans.obj");
  double h = 5. / 3.;
  TestIntegrals(500, Position(-h, 0., -h));

}


TEST_F(TestMesh, Xrot) {

  Rotate(MU_PI_4, 0., 0.);
  SetClippingPlane(Position(), Direction(1., 0., 0.));
  Clip();
//    Write("Xrot.obj");
  TestIntegrals(500, Position(-2.5, 0., 0.));

}


TEST_F(TestMesh, XrotTrans) {

  Rotate(MU_PI_4, 0., 0.);
  Translate({5., 5., 5.});
  SetClippingPlane(Position(5., 5., 5.), Direction(1., 0., 0.));
  Clip();
//    Write("XrotTrans.obj");
  TestIntegrals(500, Position(-2.5, 0., 0.));

}
