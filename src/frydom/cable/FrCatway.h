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
}


namespace frydom {


    class FrNode_;



    namespace internal {

        struct CatNode : public catenary::CatenaryNode {

            std::shared_ptr<FrNode_> m_node;

            explicit CatNode(std::shared_ptr<FrNode_> node);

        };

    }


    class FrCatway : public FrCable_ {

    private:

        std::unique_ptr<catenary::CatenaryLine> m_catLine;


        std::shared_ptr<FrNode_> m_startingNode;
        std::shared_ptr<FrNode_> m_endNode;

    public:

        FrCatway(double youngModulus, double diameter, double linearDensity, double length,
                         unsigned int nbElt, std::shared_ptr<FrNode_> node1, std::shared_ptr<FrNode_> node2);

        ~FrCatway();

        Force GetTension(const double s) const override;

        Position GetAbsPosition(const double s) const override;

        void Update();

        void Initialize() override;

        void StepFinalize() override;






    };


}  // end namespace frydom

#endif //FRYDOM_FRCATWAY_H
