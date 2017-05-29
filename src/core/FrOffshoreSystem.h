//
// Created by frongere on 29/05/17.
//

#ifndef FRYDOM_FROFFSHORESYSTEM_H
#define FRYDOM_FROFFSHORESYSTEM_H

#include "../environment/waves/FrFreeSurface.h"
//#include "../environment/waves/FrFlatFreeSurface.h"
#include "chrono/physics/ChSystemNSC.h"

namespace frydom {

    // TODO: voir aussi a deriver de ChSystemSMC pour comparer les 2 ? Avoir une classe de base virtuelle derivant de ChSystem ???
    class FrOffshoreSystem : public chrono::ChSystemNSC {

    public:

    private:
        std::shared_ptr<environment::FrFreeSurface> fs;

    };

} // end namespace frydom

#endif //FRYDOM_FROFFSHORESYSTEM_H
