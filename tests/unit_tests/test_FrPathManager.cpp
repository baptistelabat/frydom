//
// Created by frongere on 30/09/19.
//

#include "frydom/frydom.h"
#include "frydom/logging/FrPathManager.h"

#include "frydom/utils/FrFileSystem.h"

#include "gtest/gtest.h"


using namespace frydom;

TEST(FrPathManager, path) {

//int main() {

  FrOffshoreSystem system("TestPathBuilding");

  auto path_manager = system.GetPathManager();


  auto body1 = system.NewBody("myBody1");

  auto node1 = body1->NewNode("myNode1");
  auto node2 = body1->NewNode("myNode2");


  auto body2 = system.NewBody("myBody2");
  auto node3 = body2->NewNode("myNode3");

  auto force1 = make_manoeuvring_model("man_model", body1); // TODO : changer en maneuvring force...

  auto revolute_link = make_revolute_link("revolute_link", &system, node1, node3);


  ASSERT_TRUE(path_manager->GetPath(system) == "FRYDOM_TestPathBuilding/");
  ASSERT_TRUE(path_manager->GetPath(body1.get()) == "FRYDOM_TestPathBuilding/BODY/BODY_myBody1/");
  ASSERT_TRUE(path_manager->GetPath(node1.get()) == "FRYDOM_TestPathBuilding/BODY/BODY_myBody1/NODE/NODE_myNode1/");
  ASSERT_TRUE(path_manager->GetPath(node2.get()) == "FRYDOM_TestPathBuilding/BODY/BODY_myBody1/NODE/NODE_myNode2/");
  ASSERT_TRUE(path_manager->GetPath(body2.get()) == "FRYDOM_TestPathBuilding/BODY/BODY_myBody2/");
  ASSERT_TRUE(path_manager->GetPath(node3.get()) == "FRYDOM_TestPathBuilding/BODY/BODY_myBody2/NODE/NODE_myNode3/");
  ASSERT_TRUE(path_manager->GetPath(revolute_link.get()) == "FRYDOM_TestPathBuilding/LINK/LINK_revolute_link/");

}
