//
// Created by frongere on 21/06/17.
//

#ifndef FRYDOM_FRSHIP_H
#define FRYDOM_FRSHIP_H

#include "FrHydroBody.h"

// Forward declaration chrono
namespace chrono {
    class ChLinkMatePlane;
}

namespace frydom {

    // Forward declaration
    class FrPropeller;

    class FrShip : public FrHydroBody {

    private:
        // Special attributes for ships
        std::vector<std::shared_ptr<FrPropeller>> propellerlist;  // FIXME pourquoi avoir des propeller shared ???

        bool is3DOF = false;
        std::shared_ptr<chrono::ChLinkMatePlane> constraint3DOF;

    public:
        FrShip() : is3DOF(false),
                   FrHydroBody() {}

        ~FrShip() {}

        void AddPropeller(std::shared_ptr<FrPropeller> propeller); // FIXME: shared ?

        void RemovePropeller(std::shared_ptr<FrPropeller> propeller); // FIXME: shared ?

        int GetNbPropellers() { return int(propellerlist.size()); }


        // TODO: deplacer la plupart de ces methodes dans hydrobody !!
        bool Get3DOF() const { return is3DOF; };
        void Set3DOF(const bool flag);
        void Set3DOF_ON();
        void Set3DOF_OFF();


    };

}  // end namespace frydom

#endif //FRYDOM_FRSHIP_H
