//
// Created by Lucas Letournel on 20/07/18.
//

#include "chrono/physics/ChSystem.h"
#include "chrono/physics/ChLinkLock.h"
#include "FrKinematicJoint.h"

namespace frydom {

    void make_rotoid(const std::shared_ptr<FrNode> &Node1, const std::shared_ptr<FrNode> Node2) {
        try {
            if (Node1->GetBody()== nullptr) throw std::string("make_rotoid : Node1 does not belong to a body.");
            auto body1 = Node1->GetBody();
            if (Node2->GetBody()== nullptr) throw std::string("make_rotoid : Node2 does not belong to a body.");
            auto body2 = Node2->GetBody();
            if (body1 == body2) throw std::string("make_rotoid : Node1 and Node2 belong to the same body.");

            auto revoluteJoint = std::make_shared<chrono::ChLinkLockRevolute>();
            revoluteJoint->Initialize(Node1, Node2);

            if (body1->GetSystem()== nullptr) throw std::string("make_rotoid : body1 does not belong to a system.");
            auto my_system = body1->GetSystem();
            my_system->AddLink(revoluteJoint);
        }
        catch(std::string const& error_msg)
        {
            fmt::print(error_msg);
        }
    }


}