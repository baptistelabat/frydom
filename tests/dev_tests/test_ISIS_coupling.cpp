//
// Created by frongere on 17/10/19.
//

#include "frydom/frydom.h"

using namespace frydom;

int main() {

  FrOffshoreSystem system;

  auto body = system.NewBody();
//  makeItSphere(body, 5, 1000);

  double time = 0.;
  double dt = 0.1;
  while(1) {
    system.AdvanceOneStep(dt);
    time += dt;
    std::cout << time << std::endl;
    std::cout << body->GetPosition(NWU) << std::endl;
  }


  return 0;
}
