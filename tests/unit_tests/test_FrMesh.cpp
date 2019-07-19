//
// Created by lletourn on 19/07/19.
//

#include "frydom/frydom.h"

#include "gtest/gtest.h"

using namespace frydom;
using namespace geom;

void CreateCustomMesh(mesh::FrMesh& mesh, double Lx, double Ly, double Lz) {

    // generate vertices
    mesh::FrMesh::VertexHandle vhandle[8];
    vhandle[0] = mesh.add_vertex(mesh::FrMesh::Point(-Lx, -Ly,  Lz)*.5);
//    vhandle[1] = mesh.add_vertex(mesh::FrMesh::Point( Lx, -Ly,  Lz)*.5);
//    vhandle[2] = mesh.add_vertex(mesh::FrMesh::Point( Lx,  Ly,  Lz)*.5);
    vhandle[1] = mesh.add_vertex(mesh::FrMesh::Point(-Lx,  Ly,  Lz)*.5);
    vhandle[2] = mesh.add_vertex(mesh::FrMesh::Point(-Lx, -Ly, -Lz)*.5);
    vhandle[3] = mesh.add_vertex(mesh::FrMesh::Point( Lx, -Ly, -Lz)*.5);
    vhandle[4] = mesh.add_vertex(mesh::FrMesh::Point( Lx,  Ly, -Lz)*.5);
    vhandle[5] = mesh.add_vertex(mesh::FrMesh::Point(-Lx,  Ly, -Lz)*.5);

    // generate (triangular) faces
    std::vector<mesh::FrMesh::VertexHandle>  face_vhandles;

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


TEST(FrMesh,Polygon) {

    FRAME_CONVENTION fc = NWU;

    mesh::FrMesh mesh;

    mesh.CreateBox(10,10,10);

    mesh.Translate({5.,5.,5.});

//    mesh.Write("testPolygon.obj");

    Position origin(5.,5.,5.);
//    Position origin(0.,0.,0.);
//    origin.setRandom();

    Direction normal(1.,0.,1.);
    normal.normalize();

    auto plane = std::make_shared<geom::FrPlane>(origin, normal, fc);

    auto clippingSurface = std::make_shared<mesh::FrClippingPlane>(plane);

    mesh::FrMeshClipper clipper;
    clipper.SetClippingSurface(clippingSurface);

    clipper.Apply(&mesh);

//    mesh.Write("testPolygon.obj");

    // Test on polygon
    auto polygonSet = mesh.GetBoundaryPolygonSet();

    EXPECT_TRUE(!polygonSet.empty());

    auto polygon = polygonSet[0];

    auto planeTest = polygon.GetPlane();
    Direction testNormal = normal - planeTest.GetNormal(fc);
    EXPECT_NEAR(testNormal.norm(),0,1E-8);

    EXPECT_NEAR(planeTest.GetDistanceToPoint(origin,fc), 0, 1E-8);

    // Test on integrals
    EXPECT_NEAR(mesh.GetVolume(), 500, 1E-8);

    std::cout<<"G : ("<<mesh.GetCOG().GetX()<<","<<mesh.GetCOG().GetY()<<","<<mesh.GetCOG().GetZ()<<")"<<std::endl;

    double h = 5./3.;
    Position testCOG = origin + Position(-h,0.,-h) - mesh.GetCOG();
//    Position testCOG = Position(2*h,5.,2*h) - mesh.GetCOG();
    EXPECT_NEAR(testCOG.norm(), 0, 1E-8);

    //

//    mesh::FrMesh customMesh;
//
//    CreateCustomMesh(customMesh, 10, 10, 10);
//
//    customMesh.Translate({5.,5.,5.});
//
//    customMesh.Write("customMesh.obj");
//
//
//    std::cout<<"G2 : ("<<customMesh.GetCOG().GetX()<<","<<customMesh.GetCOG().GetY()<<","<<customMesh.GetCOG().GetZ()<<")"<<std::endl;



}