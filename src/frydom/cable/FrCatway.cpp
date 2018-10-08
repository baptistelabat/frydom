//
// Created by frongere on 05/10/18.
//

#include "FrCatway.h"
#include "catenary/Catenary.h"

#include "frydom/core/FrNode.h"


namespace frydom {


    internal::CatNode::CatNode(std::shared_ptr<FrNode_> node) : m_node(node){}


    FrCatway::FrCatway(double youngModulus, double diameter, double linearDensity, double length,
                       unsigned int nbElt, std::shared_ptr<FrNode_> node1, std::shared_ptr<FrNode_> node2) {

        auto props = catenary::make_properties(youngModulus, MU_PI * pow(0.5*diameter, 2), linearDensity, true);

        auto catNode1 = std::make_shared<internal::CatNode>(node1);
        auto catNode2 = std::make_shared<internal::CatNode>(node2);

        m_catLine = std::make_unique<catenary::CatenaryLine>(props, catNode1, catNode2, length);

        m_catLine->Discretize(nbElt);

    }

    FrCatway::~FrCatway() {}

    void FrCatway::Update() {

    }

    void FrCatway::Initialize() {

    }

    void FrCatway::StepFinalize() {

    }

    Force FrCatway::GetTension(const double s) const {


    }

    Position FrCatway::GetAbsPosition(const double s) const {

    }


}  // end namespace frydom
