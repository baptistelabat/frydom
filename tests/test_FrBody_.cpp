//
// Created by Lucas Letournel on 05/11/18.
//

#include "frydom/frydom.h"

namespace frydom{

    class test_FrBody_ :public FrBody_ {
    public:

        int Test_Smthg(){};
    };


    int main(int argc, char* argv[]) {
        FrOffshoreSystem_ system;
        // Defining the ship
        auto body = std::make_shared<test_FrBody_>();
        body->SetSmoothContact();
        system.Add(body);

        body->Test_Smthg();

    }
}