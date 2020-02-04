#include "frydom/frydom.h"

#include "gtest/gtest.h"

using namespace frydom;

TEST(FrShape, FrBoxShape) {
  FrOffshoreSystem system("test_FrShape");

  auto body = system.NewBody("body");
  body->AddBoxShape(1, 2, 3);

  ASSERT_EQ(body->GetBoxShapes().size(), 1);
  ASSERT_EQ(body->GetCylinderShapes().size(), 0);
  ASSERT_EQ(body->GetSphereShapes().size(), 0);
  ASSERT_EQ(body->GetMeshAssets().size(), 0);

  auto shape = body->GetBoxShapes()[0];
  ASSERT_EQ(shape->xSize(), 1);
  ASSERT_EQ(shape->ySize(), 2);
  ASSERT_EQ(shape->zSize(), 3);
}

TEST(FrShape, FrCylinderShape) {
  FrOffshoreSystem system("test_FrShape");

  auto body = system.NewBody("body");
  body->AddCylinderShape(1, 10);

  ASSERT_EQ(body->GetBoxShapes().size(), 0);
  ASSERT_EQ(body->GetCylinderShapes().size(), 1);
  ASSERT_EQ(body->GetSphereShapes().size(), 0);
  ASSERT_EQ(body->GetMeshAssets().size(), 0);

  auto shape = body->GetCylinderShapes()[0];
  ASSERT_EQ(shape->radius(), 1);
  ASSERT_EQ(shape->height(), 10);
}

TEST(FrShape, FrSphereShape) {
  FrOffshoreSystem system("test_FrShape");

  auto body = system.NewBody("body");
  body->AddSphereShape(1);

  ASSERT_EQ(body->GetBoxShapes().size(), 0);
  ASSERT_EQ(body->GetCylinderShapes().size(), 0);
  ASSERT_EQ(body->GetSphereShapes().size(), 1);
  ASSERT_EQ(body->GetMeshAssets().size(), 0);

  auto shape = body->GetSphereShapes()[0];
  ASSERT_EQ(shape->radius(), 1);
}

TEST(FrShape, FrTriangleMeshShape) {
  // export a mesh file
  std::string mesh_content("\
v 1.000000 -1.000000 -1.000000\n\
v 1.000000 -1.000000 1.000000\n\
v -1.000000 -1.000000 1.000000\n\
v -1.000000 -1.000000 -1.000000\n\
v 1.000000 1.000000 -1.000000\n\
v 1.000000 1.000000 1.000001\n\
v -1.000000 1.000000 1.000000\n\
v -1.000000 1.000000 -1.000000\n\
vt 0.748573 0.750412\n\
vt 0.749279 0.501284\n\
vt 0.999110 0.501077\n\
vt 0.999455 0.750380\n\
vt 0.250471 0.500702\n\
vt 0.249682 0.749677\n\
vt 0.001085 0.750380\n\
vt 0.001517 0.499994\n\
vt 0.499422 0.500239\n\
vt 0.500149 0.750166\n\
vt 0.748355 0.998230\n\
vt 0.500193 0.998728\n\
vt 0.498993 0.250415\n\
vt 0.748953 0.250920\n\
vn 0.000000 0.000000 -1.000000\n\
vn -1.000000 -0.000000 -0.000000\n\
vn -0.000000 -0.000000 1.000000\n\
vn -0.000001 0.000000 1.000000\n\
vn 1.000000 -0.000000 0.000000\n\
vn 1.000000 0.000000 0.000001\n\
vn 0.000000 1.000000 -0.000000\n\
vn -0.000000 -1.000000 0.000000\n\
s off\n\
f 5/1/1 1/2/1 4/3/1\n\
f 5/1/1 4/3/1 8/4/1\n\
f 3/5/2 7/6/2 8/7/2\n\
f 3/5/2 8/7/2 4/8/2\n\
f 2/9/3 6/10/3 3/5/3\n\
f 6/10/4 7/6/4 3/5/4\n\
f 1/2/5 5/1/5 2/9/5\n\
f 5/1/6 6/10/6 2/9/6\n\
f 5/1/7 8/11/7 6/10/7\n\
f 8/11/7 7/12/7 6/10/7\n\
f 1/2/8 2/9/8 3/13/8\n\
f 1/2/8 3/13/8 4/14/8");

  std::ofstream f;
  f.open("/tmp/test.mesh");
  f << mesh_content;
  f.close();

  // load the mesh file
  FrOffshoreSystem system("test_FrShape");

  auto body = system.NewBody("body");
  body->AddMeshAsset("/tmp/test.mesh");

  ASSERT_EQ(body->GetBoxShapes().size(), 0);
  ASSERT_EQ(body->GetCylinderShapes().size(), 0);
  ASSERT_EQ(body->GetSphereShapes().size(), 0);
  ASSERT_EQ(body->GetMeshAssets().size(), 1);

  // assert a couple of values
  auto mesh = body->GetMeshAssets()[0];

  ASSERT_EQ(mesh->vertices().size(), 8);
  ASSERT_EQ(mesh->vertices()[0][0], 1.0);
  ASSERT_EQ(mesh->vertices()[3][1], -1.0);

  ASSERT_EQ(mesh->normals().size(), 8);
  ASSERT_EQ(mesh->normals()[4][0], 1.0);
  ASSERT_EQ(mesh->normals()[4][1], 0.0);
  ASSERT_EQ(mesh->normals()[4][2], 0.0);

  ASSERT_EQ(mesh->faceVertexIndices().size(), 12);
  ASSERT_EQ(mesh->faceVertexIndices()[2][0], 3 - 1);
  ASSERT_EQ(mesh->faceVertexIndices()[2][1], 7 - 1);
  ASSERT_EQ(mesh->faceVertexIndices()[2][2], 8 - 1);

  ASSERT_EQ(mesh->faceNormalIndices().size(), 12);
  ASSERT_EQ(mesh->faceNormalIndices()[4][0], 3 - 1);
  ASSERT_EQ(mesh->faceNormalIndices()[4][1], 3 - 1);
  ASSERT_EQ(mesh->faceNormalIndices()[4][2], 3 - 1);
}
