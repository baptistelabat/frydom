//
// Created by frongere on 07/06/17.
//

#ifndef FRYDOM_FRTRYALFORCE_H
#define FRYDOM_FRTRYALFORCE_H

#include <iostream>
#include <fstream>

#include "../core/FrForce.h"

namespace frydom {


    class FrTryalForce : public FrForce {

      private:
        bool done;
        std::ofstream myfile;

      public:

        FrTryalForce();
        ~FrTryalForce() {
            myfile.close();
            std::cout << "fichier ferme" << std::endl;
        }

        void UpdateState() override;

    };

}  //end namespace frydom

#endif //FRYDOM_FRTRYALFORCE_H
