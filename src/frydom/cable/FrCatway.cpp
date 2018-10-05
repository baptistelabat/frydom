//
// Created by frongere on 05/10/18.
//

#include "FrCatway.h"
#include "catenary/Catenary.h"




namespace frydom {

    frydom::FrCatway::FrCatway(double youngModulus, double diameter, double linearDensity, double length,
                                   unsigned int nbElt, std::shared_ptr<FrNode_> node1, std::shared_ptr<FrNode_> node2) {

        auto props = catenary::make_properties(youngModulus, MU_PI * pow(0.5*diameter, 2), linearDensity, true);

        auto catNode1 = catenary::make_node();
        auto catNode2 = catenary::make_node();

        m_catLine = std::make_unique<catenary::CatenaryLine>(props, catNode1, catNode2, length);




    }

}  // end namespace frydom
