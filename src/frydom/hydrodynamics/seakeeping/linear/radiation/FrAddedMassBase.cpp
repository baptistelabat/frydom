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

                auto residualOffset = off + GetBodyOffset(BEMBody->get());

                //for (auto BEMBodyMotion = m_HDB->begin(); BEMBodyMotion!=m_HDB->end(); BEMBodyMotion++) {
                    auto BEMBodyMotion = BEMBody;

                    auto bodyOffset = off + GetBodyOffset(BEMBodyMotion->get());

                    auto infiniteAddedMass = BEMBody->get()->GetInfiniteAddedMass(BEMBodyMotion->get());

                    Eigen::VectorXd q(6);
                    for (int i = 0; i < 6; i++) { q(i) = w(bodyOffset + i); }

                    Eigen::VectorXd Mv = c * infiniteAddedMass * q;
                    auto Mw = chrono::ChVector<>(Mv(0), Mv(1), Mv(2));
                    auto Iw = chrono::ChVector<>(Mv(3), Mv(4), Mv(6));

                    R.PasteSumVector(Mw, residualOffset, 0);
                    R.PasteSumVector(Iw, residualOffset + 3, 0);
                //}
            }
        }

        void FrAddedMassBase::IntToDescriptor(const unsigned int off_v, const chrono::ChStateDelta& v,
                                              const chrono::ChVectorDynamic<>& R, const unsigned int off_L,
                                              const chrono::ChVectorDynamic<>& L, const chrono::ChVectorDynamic<>& Qc) {

            for (auto BEMBody = m_HDB->begin(); BEMBody!=m_HDB->end(); BEMBody++) {

                auto bodyOffset = GetBodyOffset(BEMBody->get());

                m_variables->Get_qb().PasteClippedMatrix(v, bodyOffset, 0, 6, 1, bodyOffset, 0);
            }

        }

        void FrAddedMassBase::IntFromDescriptor(const unsigned int off_v, chrono::ChStateDelta& v,
                                                const unsigned int off_L, chrono::ChVectorDynamic<>& L) {

            for (auto BEMBody = m_HDB->begin(); BEMBody!=m_HDB->end(); BEMBody++) {

                auto bodyOffset = GetBodyOffset(BEMBody->get());

                v.PasteClippedMatrix(m_variables->Get_qb(), bodyOffset, 0, 6, 1, bodyOffset, 0);
            }
        }

        void FrAddedMassBase::InjectVariables(chrono::ChSystemDescriptor &mdescriptor) {
            mdescriptor.InsertVariables(m_variables.get());
        }

        void FrAddedMassBase::VariablesFbReset() {
            m_variables->Get_fb().FillElem(0.0);
        }

        void FrAddedMassBase::VariablesFbIncrementMq() {
            m_variables->Compute_inc_Mb_v(m_variables->Get_fb(), m_variables->Get_qb());
        }

        int FrAddedMassBase::GetBodyOffset(FrBEMBody_* BEMBody) {
            auto chronoBody = m_HDB->GetBody(BEMBody)->GetChronoBody();
            return chronoBody->GetOffset_w();
        }

    }
}