//
// Created by camille on 29/01/19.
//

#include "frydom/hydrodynamics/seakeeping/linear/radiation/FrAddedMassBase.h"
#include "frydom/hydrodynamics/seakeeping/linear/hdb/FrHydroDB.h"
#include "frydom/core/body/FrBody.h"

namespace frydom {

    namespace internal {

        FrAddedMassBase::FrAddedMassBase(FrHydroDB_* HDB) : m_HDB(HDB) { }

        void FrAddedMassBase::SetupInitial() {

        }

        void FrAddedMassBase::IntLoadResidual_Mv(const unsigned int off, chrono::ChVectorDynamic<> &R,
                                                 const chrono::ChVectorDynamic<> &w, const double c) {

            for (auto BEMBody = m_HDB->begin(); BEMBody!=m_HDB->end(); BEMBody++) {

                auto chronoBody = m_HDB->GetBody(BEMBody->get())->GetChronoBody();
                auto residualOffset = off + chronoBody->GetOffset_w();

                for (auto BEMBodyMotion = m_HDB->begin(); BEMBodyMotion!=m_HDB->end(); BEMBodyMotion++) {

                    auto chronoBodyMotion = m_HDB->GetBody(BEMBodyMotion->get())->GetChronoBody();
                    auto bodyOffset = off + chronoBodyMotion->GetOffset_w();

                    auto infiniteAddedMass = BEMBody->get()->GetInfiniteAddedMass(BEMBodyMotion->get());

                    Eigen::VectorXd q(6);
                    for (int i = 0; i < 6; i++) { q(i) = w(bodyOffset + i); }

                    Eigen::VectorXd Mv = c * infiniteAddedMass * q;
                    auto Mw = chrono::ChVector<>(Mv(0), Mv(1), Mv(2));
                    auto Iw = chrono::ChVector<>(Mv(3), Mv(4), Mv(6));

                    R.PasteSumVector(Mw, residualOffset, 0);
                    R.PasteSumVector(Iw, residualOffset + 3, 0);
                }
            }
        }

        void FrAddedMassBase::InjectVariables(chrono::ChSystemDescriptor &mdescriptor) {
            m_variables->SetDisabled(!this->IsActive());
            mdescriptor.InsertVariables(m_variables.get());
        }

    }
}