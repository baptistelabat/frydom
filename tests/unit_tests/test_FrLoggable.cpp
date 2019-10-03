//
// Created by frongere on 03/10/19.
//

#include "frydom/frydom.h"

using namespace frydom;

int main() {

  FrOffshoreSystem system("test_FrLoggable");

//  auto path_manager = system.GetPathManager();

  auto body1 = system.NewBody("myBody1");

  auto node1 = body1->NewNode("myNode1");
//  auto node2 = body1->NewNode("myNode2");


//  auto body2 = system.NewBody("myBody2");
//  auto node3 = body2->NewNode("myNode3");

  auto force1 = make_manoeuvring_model("man_model", body1);








  return 0;
}
