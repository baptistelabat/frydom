//
// Created by frongere on 30/09/19.
//

#include <type_traits>

#include "frydom/frydom.h"
#include "frydom/logging/FrPathManager.h"

#include "frydom/utils/FrFileSystem.h"

#include "gtest/gtest.h"


using namespace frydom;

TEST(FrPathManager, path) {

  FrOffshoreSystem system("test_FrPathManager");

  auto path_manager = system.GetPathManager();


  auto body1 = system.NewBody("myBody1");

  auto node1 = body1->NewNode("myNode1");
  auto node2 = body1->NewNode("myNode2");


  auto body2 = system.NewBody("myBody2");
  auto node3 = body2->NewNode("myNode3");

  auto force1 = make_manoeuvring_model("man_model", body1); // TODO : changer en manoeuvring force...

  auto revolute_link = make_revolute_link("revolute_link", &system, node1, node3);
  auto motor = revolute_link->Motorize("motor", ACTUATOR_CONTROL::POSITION);

  auto plane2 = std::make_shared<FrCPlane>(node2);
  auto plane3 = std::make_shared<FrCPlane>(node3);
  auto planeConstraint = make_constraint_plane_on_plane("plane_on_plane", &system, plane2, plane3);

  system.Initialize();

  EXPECT_TRUE(path_manager->GetPath(&system) == "FRYDOM_test_FrPathManager/");
  EXPECT_TRUE(path_manager->GetPath(body1.get()) == "FRYDOM_test_FrPathManager/BODIES/BODY_myBody1/");
  EXPECT_TRUE(
      path_manager->GetPath(node1.get()) == "FRYDOM_test_FrPathManager/BODIES/BODY_myBody1/NODES/NODE_myNode1/");
  EXPECT_TRUE(
      path_manager->GetPath(node2.get()) == "FRYDOM_test_FrPathManager/BODIES/BODY_myBody1/NODES/NODE_myNode2/");
  EXPECT_TRUE(path_manager->GetPath(body2.get()) == "FRYDOM_test_FrPathManager/BODIES/BODY_myBody2/");

  EXPECT_TRUE(
      path_manager->GetPath(node3.get()) == "FRYDOM_test_FrPathManager/BODIES/BODY_myBody2/NODES/NODE_myNode3/");
  EXPECT_TRUE(path_manager->GetPath(std::dynamic_pointer_cast<FrLink>(revolute_link).get()) ==
              "FRYDOM_test_FrPathManager/LINKS/LINK_revolute_link/");
  std::cout << path_manager->GetPath(dynamic_cast<FrActuator *>(motor)) << std::endl;
  EXPECT_TRUE(path_manager->GetPath(dynamic_cast<FrActuator *>(motor)) ==
              "FRYDOM_test_FrPathManager/LINKS/LINK_revolute_link/ACTUATORS/ACTUATOR_motor/");

  EXPECT_TRUE(path_manager->GetPath(std::dynamic_pointer_cast<FrConstraint>(planeConstraint).get()) ==
              "FRYDOM_test_FrPathManager/CONSTRAINTS/CONSTRAINT_plane_on_plane/");
}
