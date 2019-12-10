//
// Created by frongere on 03/10/19.
//

#include "frydom/frydom.h"
#include "frydom/logging/FrLogManager.h"
#include "gtest/gtest.h"

using namespace frydom;

TEST(FrLogManager, registering) {

  FrOffshoreSystem system("test_FrLoggable");

  auto body1 = system.NewBody("myBody1");

  auto node1 = body1->NewNode("myNode1");

  auto force1 = make_manoeuvring_model("man_model", body1);

  ASSERT_TRUE(system.GetLogManager()->GetNumberOfLoggables() == 5); // The 3 loggables created above + world body + system

  system.Remove(body1);

  ASSERT_TRUE(system.GetLogManager()->GetNumberOfLoggables() == 2); // It only remains the world body + system

  system.GetLogManager()->Initialize();


}
