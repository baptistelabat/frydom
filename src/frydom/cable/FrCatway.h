//
// Created by frongere on 05/10/18.
//

#ifndef FRYDOM_FRCATWAY_H
#define FRYDOM_FRCATWAY_H

#include "catenary/CatenaryNode.h"

#include "FrCable.h"


//using namespace catenary;

namespace catenary {
    class CatenaryLine;
//    class CatenaryNode;
}


namespace frydom {


    class FrNode_;



    namespace internal {

        class CatNode : public catenary::CatenaryNode {

            std::shared_ptr<FrNode_> m_node1;
            std::shared_ptr<FrNode_> m_node2;

        };

    }


    class FrCatway : public FrCable {

    private:

        std::unique_ptr<catenary::CatenaryLine> m_catLine;


        std::shared_ptr<FrNode> m_startingNode;
        std::shared_ptr<FrNode> m_endNode;

    public:

        FrCatway(double youngModulus, double diameter, double linearDensity, double length,
                         unsigned int nbElt, std::shared_ptr<FrNode_> node1, std::shared_ptr<FrNode_> node2);










    };


}  // end namespace frydom

#endif //FRYDOM_FRCATWAY_H
