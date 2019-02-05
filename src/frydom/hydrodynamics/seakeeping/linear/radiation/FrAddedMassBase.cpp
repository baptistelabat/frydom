//
// Created by camille on 29/01/19.
//

#include "FrAddedMassBase.h"

#include "frydom/hydrodynamics/seakeeping/linear/hdb/FrHydroDB.h"
#include "frydom/core/body/FrBody.h"

#include "FrVariablesAddedMassBase.h"
#include "frydom/hydrodynamics/seakeeping/linear/radiation/FrRadiationModel.h"

namespace frydom {

    namespace internal {

        FrAddedMassBase::FrAddedMassBase(FrRadiationModel_* radiationModel) :
                m_frydomRadiationModel(radiationModel), _FrPhysicsItemBase(radiationModel) {
            auto nBodies = radiationModel->GetHydroDB()->GetNbBodies();
            int nDof = 6*nBodies;
            m_variables = std::make_shared<FrVariablesAddedMassBase>(this, nDof);
        }

        void FrAddedMassBase::SetupInitial() {
            //m_frydomRadiationModel->Initialize();
            m_variables->Initialize();
        }

        void FrAddedMassBase::Update(bool update_assets) {
            this->Update(ChTime, update_assets);
        }

        void FrAddedMassBase::Update(double time, bool update_assets) {
            m_frydomRadiationModel->Update(time);
            ChPhysicsItem::Update(time, update_assets);
        }

        void FrAddedMassBase::IntLoadResidual_Mv(const unsigned int off, chrono::ChVectorDynamic<> &R,
                                                 const chrono::ChVectorDynamic<> &w, const double c) {

            auto HDB = m_frydomRadiationModel->GetHydroDB();

            for (auto BEMBody = HDB->begin(); BEMBody!=HDB->end(); BEMBody++) {

                auto residualOffset = GetBodyOffset( HDB->GetBody(BEMBody->get()) ); //+off

                //for (auto BEMBodyMotion = m_HDB->begin(); BEMBodyMotion!=m_HDB->end(); BEMBodyMotion++) {
                    auto BEMBodyMotion = BEMBody;

                    auto bodyOffset = GetBodyOffset( HDB->GetBody(BEMBodyMotion->get()) ); //+off

                    auto infiniteAddedMass = BEMBody->get()->GetInfiniteAddedMass(BEMBodyMotion->get());

                    Eigen::VectorXd q(6);
                    for (int i = 0; i < 6; i++) { q(i) = w(bodyOffset + i); }

                    Eigen::VectorXd Mv = c * infiniteAddedMass * q;
                    auto Mw = chrono::ChVector<>(Mv(0), Mv(1), Mv(2));
                    auto Iw = chrono::ChVector<>(Mv(3), Mv(4), Mv(5));

                    R.PasteSumVector(Mw, residualOffset, 0);
                    R.PasteSumVector(Iw, residualOffset + 3, 0);
                //}
            }
        }

        void FrAddedMassBase::IntToDescriptor(const unsigned int off_v, const chrono::ChStateDelta& v,
                                              const chrono::ChVectorDynamic<>& R, const unsigned int off_L,
                                              const chrono::ChVectorDynamic<>& L, const chrono::ChVectorDynamic<>& Qc) {

            auto HDB = m_frydomRadiationModel->GetHydroDB();

            for (auto BEMBody = HDB->begin(); BEMBody!=HDB->end(); BEMBody++) {

                auto bodyOffset = off_v - offset_w + GetBodyOffset( HDB->GetBody(BEMBody->get()) );

                m_variables->Get_qb().PasteClippedMatrix(v, bodyOffset, 0, 6, 1, bodyOffset, 0);
                m_variables->Get_fb().PasteClippedMatrix(R, bodyOffset, 0, 6, 1, bodyOffset, 0);
            }
        }

        void FrAddedMassBase::IntFromDescriptor(const unsigned int off_v, chrono::ChStateDelta& v,
                                                const unsigned int off_L, chrono::ChVectorDynamic<>& L) {

            auto HDB = m_frydomRadiationModel->GetHydroDB();

            for (auto BEMBody = HDB->begin(); BEMBody!=HDB->end(); BEMBody++) {

                auto bodyOffset = off_v - offset_w + GetBodyOffset( HDB->GetBody(BEMBody->get()) );

                //v.PasteClippedMatrix(m_variables->Get_qb(), bodyOffset, 0, 6, 1, bodyOffset, 0);  // FIXME

                //SetVariables(HDB->GetBody(BEMBody->get()), m_variables->Get_qb(), bodyOffset);    // FIXME
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

        int FrAddedMassBase::GetBodyOffset(FrBody_* body) const {
            auto chronoBody = body->GetChronoBody();
            return chronoBody->GetOffset_w();
        }

        void FrAddedMassBase::SetVariables(FrBody_* body, chrono::ChMatrix<double>& qb, int offset) const {
            auto chronoBody = body->GetChronoBody();
            chronoBody->GetVariables1()->Get_qb().PasteClippedMatrix(qb, offset, 0, 6, 1, 0, 0);
        }
    }
}