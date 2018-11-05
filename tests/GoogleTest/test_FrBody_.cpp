//
// Created by Lucas Letournel on 05/11/18.
//

#include "frydom/frydom.h"

#include "gtest/gtest.h"

using namespace frydom;


class test_FrBody_ :public FrBody_ {
public:

    int Test_Smthg(){return 0;};
    int Test_SmthgElse(){return 100;};
};

TEST(FrBody_Test,test_SMTHG){
//    FrOffshoreSystem_ system;
    // Defining the ship
    auto body = std::make_shared<test_FrBody_>();
//    body->SetSmoothContact();
//    system.AddBody(body);
    // Testeing something
    EXPECT_EQ(body->Test_Smthg(),0);
};


TEST(FrBody_Test,test_SMTHGELSE){
//    FrOffshoreSystem_ system;
    // Defining the ship
    auto body = std::make_shared<test_FrBody_>();
//    body->SetSmoothContact();
//    system.AddBody(body);
    // Testeing something
    EXPECT_EQ(body->Test_SmthgElse(),100);
};