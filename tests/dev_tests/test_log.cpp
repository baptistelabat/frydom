//
// Created by frongere on 27/09/19.
//

#include "frydom/frydom.h"

#include "frydom/logging/FrLogManager.h"

using namespace frydom;

int main() {

  FrOffshoreSystem system;

  auto body = system.NewBody("body1");


  auto body2 = std::make_shared<FrBody>("body2");

  system.AddBody(body2);

  system.Initialize();

  auto log = system.GetLogManager();

  log->Initialize();


  return 0;
}
