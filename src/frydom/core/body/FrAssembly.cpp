//
// Created by lletourn on 06/06/19.
//

#include "FrAssembly.h"
#include "FrInertiaTensor.h"
#include "FrBody.h"

namespace frydom {


    void FrAssembly::SetMasterBody(const std::shared_ptr<FrBody>& body) {
        m_masterBody = body;
    }

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

        try {
            if (m_masterBody == nullptr) {
                throw std::string("no master body set for the assembly");
            }
//            if (m_bodyList.empty()) {
//                throw std::string("no bodies added to the assembly");
//            }

    //        FrInertiaTensor tensor(0.,0.,0.,0.,0.,0.,0.,Position(),NWU);

            // frame of the master body at COG
            auto frameMaster = m_masterBody->GetFrame();

            auto tensor = m_masterBody->GetInertiaTensor();

            for (const auto &body: m_bodyList) {

                auto frameBodyToMaster = frameMaster.GetOtherFrameRelativeTransform_WRT_ThisFrame(body->GetFrame());
    //            auto frameBodyToMaster = frameMaster.GetThisFrameRelativeTransform_WRT_OtherFrame(body->GetFrame());

                tensor.Add(body->GetInertiaTensor(), frameBodyToMaster);

            }

            return tensor;

        }
        catch(std::string const& error_msg)
        {
            fmt::print(error_msg);
        }
    }

    void FrAssembly::DoAssembly() {

        m_masterBody->SetFixedInWorld(true);

        auto system = m_masterBody->GetSystem();

        system->Initialize();
        system->DoAssembly();

        m_masterBody->SetFixedInWorld(false);

        std::cout<<GetInertiaTensor()<<std::endl;

    }

} // end namespace frydom