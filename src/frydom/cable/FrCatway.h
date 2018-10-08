//
// Created by frongere on 05/10/18.
//

#ifndef FRYDOM_FRCATWAY_H
#define FRYDOM_FRCATWAY_H

//#include "catenary/CatenaryNode.h"

#include "FrCable.h"

#include "frydom/core/FrForce.h"




//using namespace catenary;

namespace catenary {
    class CatenaryLine;
    class CatenaryNode;
}


namespace frydom {





    class FrCatForce;


    class FrCatway : public FrCable_ {

    private:

        std::unique_ptr<catenary::CatenaryLine> m_catLine;

        std::shared_ptr<catenary::CatenaryNode> m_startCatNode;
        std::shared_ptr<catenary::CatenaryNode> m_endCatNode;

        std::shared_ptr<FrCatForce> m_startForce;
        std::shared_ptr<FrCatForce> m_endForce;


    public:

        FrCatway(double youngModulus, double diameter, double linearDensity, double length,
                         unsigned int nbElt, std::shared_ptr<FrNode_> startNode, std::shared_ptr<FrNode_> endNode);

        ~FrCatway();

        Force GetTension(double s) const override;

        Force GetStartNodeTension() const;

        Force GetEndNodeTension() const;

        Position GetAbsPosition(double s) const override;

        void Update();

        void Initialize() override;

        void StepFinalize() override;


    };


    class FrCatForce : public FrForce_ { // Doit etre mis en std::shared_ptr<FrForce_> dans FrBody_...

        enum SIDE {
            START,
            END
        };

        FrCatway* m_catenaryLine;

        SIDE m_side;

        friend class FrCatway;

    public:

        FrCatForce(FrCatway* catenaryLine, std::shared_ptr<FrNode_> node);

        void Update(double time) override;

        void Initialize() override;

        void StepFinalize() override;

    private:

        void SetAbsTension(const Force& tension);

    };





}  // end namespace frydom

#endif //FRYDOM_FRCATWAY_H
