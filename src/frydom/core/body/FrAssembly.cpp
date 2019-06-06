//
// Created by lletourn on 06/06/19.
//

#include "FrAssembly.h"
#include "FrInertiaTensor.h"
#include "FrBody.h"

namespace frydom {

    void FrAssembly::AddToAssembly(const std::shared_ptr<frydom::FrBody> &body) {
        m_bodyList.push_back(body);
    }

    void FrAssembly::RemoveFromAssembly(const std::shared_ptr<FrBody> &body) {
        // trying to remove objects not previously added?
        assert(std::find<std::vector<std::shared_ptr<FrBody>>::iterator>(m_bodyList.begin(), m_bodyList.end(), body) !=
               m_bodyList.end());

        // warning! linear time search
        m_bodyList.erase(
                std::find<std::vector<std::shared_ptr<FrBody>>::iterator>(m_bodyList.begin(), m_bodyList.end(), body));
    }

    FrInertiaTensor FrAssembly::GetInertiaTensor(FRAME_CONVENTION fc) const {

        for (const auto &body: m_bodyList) {

            auto Inertia = body->GetInertiaTensor(fc);

//            Inertia.GetPointMassInertiaMatrix()


        }


        return nullptr;
    }

} // end namespace frydom