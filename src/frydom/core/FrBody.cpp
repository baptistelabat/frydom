//
// Created by frongere on 21/06/17.
//

#include "FrBody.h"
#include "FrNode.h"

namespace frydom {

    void FrBody::AddNode(std::shared_ptr<frydom::FrNode> node) {
        // Adding the node as a marker to the body
        AddMarker(node);
        // TODO: PUT Force related stuff here


    }

    std::shared_ptr<frydom::FrNode> FrBody::CreateNode() {
        auto node = std::make_shared<FrNode>();
        node->UpdateState();
        return node;
    }


    std::shared_ptr<frydom::FrNode> FrBody::CreateNode(const chrono::ChVector<double> relpos) {
        auto node = std::make_shared<FrNode>();
        AddNode(node);

        node->SetPos(relpos);
        node->UpdateState();  // To keep the absolute coordinates updated

        return node;
    }

}  // end namespace frydom