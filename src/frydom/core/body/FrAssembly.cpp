//
// Created by lletourn on 06/06/19.
//

#include "FrAssembly.h"
#include "FrInertiaTensor.h"
#include "FrBody.h"

namespace frydom {

    FrAssembly::FrAssembly(const std::shared_ptr<FrBody> &masterBody) : m_masterBody(masterBody) {}

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

    FrInertiaTensor FrAssembly::GetInertiaTensor() const {

        if (m_masterBody == nullptr) {
            throw std::runtime_error("no master body set for the assembly");
        }

        // frame of the master body at COG
        auto frameMaster = m_masterBody->GetFrame();

        auto tensor = m_masterBody->GetInertiaTensor();

        for (const auto &body: m_bodyList) {

            auto frameBodyToMaster = frameMaster.GetOtherFrameRelativeTransform_WRT_ThisFrame(body->GetFrame());

            tensor.Add(body->GetInertiaTensor(), frameBodyToMaster);

        }

        return tensor;
    }

    void FrAssembly::DoAssembly() {

        m_masterBody->SetFixedInWorld(true);

        auto system = m_masterBody->GetSystem();

        system->Initialize();
        system->DoAssembly();

        m_masterBody->SetFixedInWorld(false);

        std::cout<<GetInertiaTensor()<<std::endl;

    }

  std::shared_ptr<FrBody> FrAssembly::GetMasterBody() {
        return m_masterBody;
    }

  std::vector<std::shared_ptr<FrBody>> FrAssembly::GetBodyList() {
        return m_bodyList;
    }

  std::shared_ptr<FrBody> FrAssembly::GetBody(int iBody) {
        return m_bodyList[iBody];
    }

} // end namespace frydom