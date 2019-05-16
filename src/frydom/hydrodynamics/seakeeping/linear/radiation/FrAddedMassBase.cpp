// ==========================================================================
// FRyDoM - frydom-ce.org
//
// Copyright (c) Ecole Centrale de Nantes (LHEEA lab.) and D-ICE Engineering.
// All rights reserved.
//
// Use of this source code is governed by a GPLv3 license that can be found
// in the LICENSE file of FRyDoM.
//
// ==========================================================================


#include "FrAddedMassBase.h"

#include "frydom/core/body/FrBody.h"
#include "FrVariablesAddedMassBase.h"
#include "frydom/hydrodynamics/seakeeping/linear/radiation/FrRadiationModel.h"
#include "frydom/hydrodynamics/seakeeping/linear/hdb/FrLinearHDBInc.h"



namespace frydom {

    namespace internal {

        FrAddedMassBase::FrAddedMassBase(FrRadiationModel* radiationModel) :
                m_frydomRadiationModel(radiationModel), FrPhysicsItemBase(radiationModel) {


            auto nBodies = radiationModel->GetHydroDB()->GetNbBodies();
            int nDof = 6*nBodies;

            // Creation of a FrVariablesAddedMassBase class.
            m_variables = std::make_shared<FrVariablesAddedMassBase>(this, nDof);
        }

        void FrAddedMassBase::SetupInitial() {
            m_variables->Initialize();
            BuildGeneralizedMass();
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

                auto residualOffset = GetBodyOffset( HDB->GetBody(BEMBody->first) ); //+off

                for (auto BEMBodyMotion = HDB->begin(); BEMBodyMotion!=HDB->end(); BEMBodyMotion++) {
                    //auto BEMBodyMotion = BEMBody;
                    auto bodyOffset = GetBodyOffset( HDB->GetBody(BEMBodyMotion->first) ); //+off

                    auto infiniteAddedMass = BEMBody->first->GetInfiniteAddedMass(BEMBodyMotion->first);

                    Eigen::VectorXd q(6);
                    for (int i = 0; i < 6; i++) { q(i) = w(bodyOffset + i); }

                    Eigen::VectorXd Mv = c * infiniteAddedMass * q;
                    auto Mw = chrono::ChVector<>(Mv(0), Mv(1), Mv(2));
                    auto Iw = chrono::ChVector<>(Mv(3), Mv(4), Mv(5));

                    R.PasteSumVector(Mw, residualOffset, 0);
                    R.PasteSumVector(Iw, residualOffset + 3, 0);
                }
            }
        }

        void FrAddedMassBase::IntToDescriptor(const unsigned int off_v, const chrono::ChStateDelta& v,
                                              const chrono::ChVectorDynamic<>& R, const unsigned int off_L,
                                              const chrono::ChVectorDynamic<>& L, const chrono::ChVectorDynamic<>& Qc) {

            // FIXME : Nothing to do since added mass variables encapsulate the body variables
            /*
            auto HDB = m_frydomRadiationModel->GetHydroDB();

            for (auto BEMBody = HDB->begin(); BEMBody!=HDB->end(); BEMBody++) {

                auto bodyOffset = off_v - offset_w + GetBodyOffset( HDB->GetBody(BEMBody->get()) );

                m_variables->Get_qb().PasteClippedMatrix(v, bodyOffset, 0, 6, 1, 0, 0);
                m_variables->Get_fb().PasteClippedMatrix(R, bodyOffset, 0, 6, 1, 0, 0);
            }
            */
        }

        void FrAddedMassBase::IntFromDescriptor(const unsigned int off_v, chrono::ChStateDelta& v,
                                                const unsigned int off_L, chrono::ChVectorDynamic<>& L) {

            // Nothing to do since added mass variables encapsulate the body variables
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

        int FrAddedMassBase::GetBodyOffset(FrBody* body) const {
            auto chronoBody = body->GetChronoBody();
            return chronoBody->GetOffset_w();
        }

        void FrAddedMassBase::SetVariables(FrBody* body, chrono::ChMatrix<double>& qb, int offset) const {
            auto chronoBody = body->GetChronoBody();
            chronoBody->GetVariables1()->Get_qb().PasteClippedMatrix(qb, offset, 0, 6, 1, 0, 0);
        }

        void FrAddedMassBase::BuildGeneralizedMass() {

            auto HDB = GetRadiationModel()->GetHydroDB();

            auto nBody = HDB->GetMapper()->GetNbMappings();

            mathutils::MatrixMN<double> massMatrix(6*nBody, 6*nBody);

            int iBody = 0;

            // Loop over the bodies subject to hydrodynamic loads
            for (auto body = HDB->begin(); body!=HDB->end(); body++) {

                int jBody = 0;

                // Loop over the bodies subject to motion
                for (auto bodyMotion = HDB->begin(); bodyMotion!=HDB->end(); bodyMotion++) {

                    mathutils::Matrix66<double> subMatrix = body->first->GetInfiniteAddedMass(bodyMotion->first);

                    if (bodyMotion->first == body->first) {
                        subMatrix += body->second->GetInertiaTensor(NWU).GetMatrix();
                    }

                    for (int i=0; i<6; i++) {
                        for (int j=0; j<6; j++) {
                            massMatrix(6*iBody + i, 6*jBody + j) = subMatrix(i, j);
                        }
                    }
                    jBody += 1;
                }
                iBody += 1;
            }

            // Inverse the mass matrix
            massMatrix.Inverse();

            // Save the inverse of mass matrix in map
            mathutils::Matrix66<double> invMassMatrix;
            iBody = 0;
            for (auto body = HDB->begin(); body!=HDB->end(); body++) {
                int jBody = 0;
                for (auto bodyMotion = HDB->begin(); bodyMotion!=HDB->end(); bodyMotion++) {

                    for (int i=0; i>6; i++) {
                        for (int j=0; j>6; j++) {
                            invMassMatrix(i, j) = massMatrix(6*iBody + i, 6*jBody + j);
                        }
                    }
                    m_invGeneralizedMass[std::make_pair(body->first, bodyMotion->first)] = invMassMatrix;
                    jBody += 1;
                }
                iBody += 1;
            }
        }


        mathutils::Matrix66<double>
                FrAddedMassBase::GetInverseGeneralizedMass(FrBEMBody* BEMBody, FrBEMBody* BEMBodyMotion) const {
            return m_invGeneralizedMass.at(std::pair<FrBEMBody*, FrBEMBody*>(BEMBody, BEMBodyMotion));
        }

        mathutils::Matrix66<double>
                FrAddedMassBase::GetGeneralizedMass(FrBEMBody* BEMBody, FrBEMBody* BEMBodyMotion) const {
            auto generalizedMass = BEMBody->GetInfiniteAddedMass(BEMBodyMotion);

            if (BEMBody == BEMBodyMotion) {
                auto body = GetRadiationModel()->GetHydroDB()->GetMapper()->GetBody(BEMBody);
                generalizedMass += body->GetInertiaTensor(NWU).GetMatrix();
            }

            return generalizedMass;
        }

    }  // end namespace frydom::internal

}  // end namespace frydom
