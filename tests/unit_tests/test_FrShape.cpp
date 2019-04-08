#include "frydom/frydom.h"

#include "gtest/gtest.h"

using namespace frydom;

TEST(FrShape, FrBoxShape) {
  FrBody body;
  body.AddBoxShape(1, 1, 1);

  ASSERT_EQ(body.GetBoxShapes().size(), 1);
  ASSERT_EQ(body.GetCylinderShapes().size(), 0);
  ASSERT_EQ(body.GetSphereShapes().size(), 0);
  ASSERT_EQ(body.GetMeshAssets().size(), 0);
}

TEST(FrShape, FrCylinderShape) {
  FrBody body;
  body.AddCylinderShape(1, 10);

  ASSERT_EQ(body.GetBoxShapes().size(), 0);
  ASSERT_EQ(body.GetCylinderShapes().size(), 1);
  ASSERT_EQ(body.GetSphereShapes().size(), 0);
  ASSERT_EQ(body.GetMeshAssets().size(), 0);
}

TEST(FrShape, FrSphereShape) {
  FrBody body;
  body.AddSphereShape(1);

  ASSERT_EQ(body.GetBoxShapes().size(), 0);
  ASSERT_EQ(body.GetCylinderShapes().size(), 0);
  ASSERT_EQ(body.GetSphereShapes().size(), 1);
  ASSERT_EQ(body.GetMeshAssets().size(), 0);
}

TEST(FrShape, FrTriangleMeshShape) {
  FrBody body;
  body.AddMeshAsset("/tmp/test.msh");

  ASSERT_EQ(body.GetBoxShapes().size(), 0);
  ASSERT_EQ(body.GetCylinderShapes().size(), 0);
  ASSERT_EQ(body.GetSphereShapes().size(), 0);
  ASSERT_EQ(body.GetMeshAssets().size(), 1);
}
