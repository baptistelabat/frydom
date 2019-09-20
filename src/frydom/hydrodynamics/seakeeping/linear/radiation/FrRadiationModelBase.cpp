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


#include "FrRadiationModelBase.h"

#include "frydom/core/body/FrBody.h"
#include "frydom/hydrodynamics/seakeeping/linear/radiation/FrRadiationModel.h"
#include "frydom/hydrodynamics/seakeeping/linear/hdb/FrLinearHDBInc.h"


namespace frydom {

    namespace internal {

        template<typename OffshoreSystemType>
        FrRadiationModelBase<OffshoreSystemType>::FrRadiationModelBase(FrRadiationModel<OffshoreSystemType> *radiationModel) :
            m_frydomRadiationModel(radiationModel), FrPhysicsItemBase<OffshoreSystemType>(radiationModel) {

        }

        template<typename OffshoreSystemType>
        void FrRadiationModelBase<OffshoreSystemType>::SetupInitial() {
          InjectVariablesToBody();
          BuildGeneralizedMass();
        }

        template<typename OffshoreSystemType>
        void FrRadiationModelBase<OffshoreSystemType>::Update(bool update_assets) {
          this->Update(this->ChTime, update_assets);
        }

        template<typename OffshoreSystemType>
        void FrRadiationModelBase<OffshoreSystemType>::Update(double time, bool update_assets) {
          m_frydomRadiationModel->Update(time);
          chrono::ChPhysicsItem::Update(time, update_assets);
        }

        template<typename OffshoreSystemType>
        void FrRadiationModelBase<OffshoreSystemType>::IntLoadResidual_Mv(const unsigned int off, chrono::ChVectorDynamic<> &R,
                                                      const chrono::ChVectorDynamic<> &w, const double c) {

          auto HDB = m_frydomRadiationModel->GetHydroDB();

          for (auto BEMBody = HDB->begin(); BEMBody != HDB->end(); BEMBody++) {

            if (BEMBody->second->IsActive()) {

              auto residualOffset = GetBodyOffset(HDB->GetBody(BEMBody->first)); //+off

              for (auto BEMBodyMotion = HDB->begin(); BEMBodyMotion != HDB->end(); BEMBodyMotion++) {

                if (BEMBodyMotion->second->IsActive()) {

                  auto bodyOffset = GetBodyOffset(HDB->GetBody(BEMBodyMotion->first)); //+off

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
          }
        }

        template<typename OffshoreSystemType>
        void FrRadiationModelBase<OffshoreSystemType>::IntToDescriptor(const unsigned int off_v, const chrono::ChStateDelta &v,
                                                   const chrono::ChVectorDynamic<> &R, const unsigned int off_L,
                                                   const chrono::ChVectorDynamic<> &L,
                                                   const chrono::ChVectorDynamic<> &Qc) {

          // Nothing to do since added mass variables encapsulate the body variables
        }

        template<typename OffshoreSystemType>
        void FrRadiationModelBase<OffshoreSystemType>::IntFromDescriptor(const unsigned int off_v, chrono::ChStateDelta &v,
                                                     const unsigned int off_L, chrono::ChVectorDynamic<> &L) {

          // Nothing to do since added mass variables encapsulate the body variables
        }

        template<typename OffshoreSystemType>
        int FrRadiationModelBase<OffshoreSystemType>::GetBodyOffset(FrBody<OffshoreSystemType> *body) const {
          auto chronoBody = body->GetChronoBody();
          return chronoBody->GetOffset_w();
        }

        template<typename OffshoreSystemType>
        void FrRadiationModelBase<OffshoreSystemType>::InjectVariablesToBody() {

          auto HDB = GetRadiationModel()->GetHydroDB();

          for (auto body = HDB->begin(); body != HDB->end(); body++) {
            auto chronoBody = body->second->GetChronoBody();
            auto variable = std::make_shared<FrVariablesBEMBodyBase>(this, body->first, &chronoBody->VariablesBody());
            chronoBody->SetVariables(variable);
          }

        }

        template<typename OffshoreSystemType>
        void FrRadiationModelBase<OffshoreSystemType>::BuildGeneralizedMass() {

          auto HDB = GetRadiationModel()->GetHydroDB();

          auto nBody = HDB->GetMapper()->GetNbMappings();

          mathutils::MatrixMN<double> massMatrix(6 * nBody, 6 * nBody);

          int iBody = 0;

          // Loop over the bodies subject to hydrodynamic loads
          for (auto body = HDB->begin(); body != HDB->end(); body++) {

            int jBody = 0;

            // Loop over the bodies subject to motion
            for (auto bodyMotion = HDB->begin(); bodyMotion != HDB->end(); bodyMotion++) {

              mathutils::Matrix66<double> subMatrix = body->first->GetInfiniteAddedMass(bodyMotion->first);

              if (bodyMotion->first == body->first) {
                subMatrix += body->second->GetInertiaTensor().GetMassMatrixAtCOG();
              }

              for (int i = 0; i < 6; i++) {
                for (int j = 0; j < 6; j++) {
                  massMatrix(6 * iBody + i, 6 * jBody + j) = subMatrix(i, j);
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
          for (auto body = HDB->begin(); body != HDB->end(); body++) {
            int jBody = 0;
            for (auto bodyMotion = HDB->begin(); bodyMotion != HDB->end(); bodyMotion++) {

              for (int i = 0; i < 6; i++) {
                for (int j = 0; j < 6; j++) {
                  invMassMatrix(i, j) = massMatrix(6 * iBody + i, 6 * jBody + j);
                }
              }
              m_invGeneralizedMass[std::make_pair(body->first, bodyMotion->first)] = invMassMatrix;
              jBody += 1;
            }
            iBody += 1;
          }
        }

        template<typename OffshoreSystemType>
        mathutils::Matrix66<double>
        FrRadiationModelBase<OffshoreSystemType>::GetInverseGeneralizedMass(FrBEMBody<OffshoreSystemType> *BEMBody, FrBEMBody<OffshoreSystemType> *BEMBodyMotion) const {
          return m_invGeneralizedMass.at(std::pair<FrBEMBody<OffshoreSystemType> *, FrBEMBody<OffshoreSystemType> *>(BEMBody, BEMBodyMotion));
        }

        template<typename OffshoreSystemType>
        mathutils::Matrix66<double>
        FrRadiationModelBase<OffshoreSystemType>::GetGeneralizedMass(FrBEMBody<OffshoreSystemType> *BEMBody, FrBEMBody<OffshoreSystemType> *BEMBodyMotion) const {
          auto generalizedMass = BEMBody->GetInfiniteAddedMass(BEMBodyMotion);

          if (BEMBody == BEMBodyMotion) {
            auto body = GetRadiationModel()->GetHydroDB()->GetMapper()->GetBody(BEMBody);
            generalizedMass += body->GetInertiaTensor().GetMassMatrixAtCOG();
          }

          return generalizedMass;
        }

    }  // end namespace frydom::internal

}  // end namespace frydom
