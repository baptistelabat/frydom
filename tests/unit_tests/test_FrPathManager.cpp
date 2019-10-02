//
// Created by frongere on 30/09/19.
//

#include "frydom/frydom.h"
#include "frydom/logging/FrPathManager.h"

#include "gtest/gtest.h"


using namespace frydom;

//TEST(FrPathManager, FrOffshoreSystem) {

int main() {

  FrOffshoreSystem system("TestPathBuilding");

  auto path_manager = system.GetPathManager();


  auto body1 = system.NewBody("myBody1");

  auto node1 = body1->NewNode("myNode1");
  auto node2 = body1->NewNode("myNode2");


  auto body2 = system.NewBody("myBody2");

  auto force1 = make_manoeuvring_model("man_model", body2); // TODO : changer en maneuvring force...



  std::cout << path_manager->GetPath(&system) << std::endl;
  std::cout << path_manager->GetPath(body1.get()) << std::endl;
  std::cout << path_manager->GetPath(force1.get()) << std::endl;
  std::cout << path_manager->GetPath(node2.get()) << std::endl;


  return 0;
}
