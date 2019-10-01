//
// Created by frongere on 30/09/19.
//

#include "frydom/frydom.h"
#include "frydom/logging/FrPathManager.h"

#include "gtest/gtest.h"


using namespace frydom;

//TEST(FrPathManager, FrOffshoreSystem) {

int main() {

  FrOffshoreSystem system;

  auto path_manager = system.GetPathManager();


  auto body1 = system.NewBody("myBody1");

  auto node1 = body1->NewNode("myNode1");
  auto node2 = body1->NewNode("myNode2");


  auto body2 = system.NewBody("myBody2");


  auto force1 = make_manoeuvring_model("man_model_body2", body2);


  auto system_path = path_manager->GetPath(&system);
  auto body1_path = path_manager->GetPath(body1.get());


  return 0;
}
