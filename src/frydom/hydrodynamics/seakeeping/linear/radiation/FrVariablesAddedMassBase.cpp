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


#include "FrLinearRadiationInc.h"

#include "frydom/hydrodynamics/seakeeping/linear/hdb/FrLinearHDBInc.h"
#include "frydom/core/body/FrBody.h"


namespace frydom {

    namespace internal {

        /// Constructor of the class.
        FrVariablesAddedMassBase::FrVariablesAddedMassBase(FrAddedMassBase* addedMassBase, int ndof)
            : ChVariables(ndof), m_addedMassBase(addedMassBase) { }

        void FrVariablesAddedMassBase::Initialize() {

            // TODO : void s'il est possible d'éviter l'override des variables de FrBody

            // HDB (of all bodies).
            auto HDB = m_addedMassBase->GetRadiationModel()->GetHydroDB();

            // Loop over the bodies subject to hydrodynamique loads.
            for (auto BEMBody = HDB->begin(); BEMBody!=HDB->end(); BEMBody++) {

                auto body = HDB->GetBody(BEMBody->get());

                // Mass.
                mathutils::Matrix66<double> generalizedMass = body->GetInertiaTensor(NWU).GetMatrix();
                //mathutils::Matrix66<double> invGeneralizedMass = generalizedMass;
                //invGeneralizedMass.Inverse();

                // Added mass for infinite frequency.
                mathutils::Matrix66<double> infiniteAddedMass = BEMBody->get()->GetSelfInfiniteAddedMass();
                //mathutils::Matrix66<double> invInfiniteAddedMass = infiniteAddedMass.inverse();

                // Sum of all masses.
                mathutils::Matrix66<double> sumMatrixMass = generalizedMass + infiniteAddedMass;

                // Inversion of the sum of all masses.
                mathutils::Matrix66<double> invSumMatrixMass = sumMatrixMass;
                invSumMatrixMass.Inverse();

                //m_invAddedMassCorrection[BEMBody->get()] = - invGeneralizedMass * infiniteAddedMass * invSumMatrixMass;
                m_invAddedMassCorrection[BEMBody->get()] = invSumMatrixMass;
            }
        }

        void FrVariablesAddedMassBase::Compute_invMb_v(chrono::ChMatrix<double>& result,
                                                        const chrono::ChMatrix<double>& vect) const {

            auto HDB = m_addedMassBase->GetRadiationModel()->GetHydroDB();

            for (auto BEMBody = HDB->begin(); BEMBody!=HDB->end(); BEMBody++) {

                auto body = HDB->GetBody(BEMBody->get());
                auto resultOffset = GetBodyOffset(body);

                //for (auto BEMBodyMotion = HDB->begin(); BEMBodyMotion!=HDB->end(); BEMBodyMotion++) {

                    auto BEMBodyMotion = BEMBody;

                    auto bodyOffset = GetBodyOffset(body);

                    auto fb = GetVariablesFb(body);
                    auto qb = GetVariablesQb(body);

                    auto invAddedMassCorrection = m_invAddedMassCorrection.at(BEMBody->get());

                    for (int i=0; i<6; i++) {
                        qb(i) = 0.;
                        for (int j=0; j<6; j++) {
                            //result(resultOffset + i) += invAddedMassCorrection(i, j) * vect(bodyOffset + j);
                           qb(i) += invAddedMassCorrection(i, j) * fb(j);
                        }
                    };
                //}
                this->SetVariables(body, qb, 0);
            }
        }

        void FrVariablesAddedMassBase::Compute_inc_invMb_v(chrono::ChMatrix<double>& result,
                                                            const chrono::ChMatrix<double>& vect) const {

            auto HDB = m_addedMassBase->GetRadiationModel()->GetHydroDB();

            for (auto BEMBody = HDB->begin(); BEMBody!=HDB->end(); BEMBody++) {

                auto body = HDB->GetBody(BEMBody->get());

                auto resultOffset = GetBodyOffset(body);

                //for (auto BEMBodyMotion = HDB->begin(); BEMBodyMotion!=HDB->end(); BEMBodyMotion++) {
                    auto BEMBodyMotion = BEMBody;

                    auto bodyOffset = GetBodyOffset( HDB->GetBody(BEMBodyMotion->get()) );

                    auto fb = GetVariablesFb(body);
                    auto qb = GetVariablesQb(body);

                    auto invAddedMassCorrection = m_invAddedMassCorrection.at(BEMBody->get());

                    for (int i=0; i<6; i++) {
                        for (int j=0; j<6; j++) {
                            //result(resultOffset + i) += invAddedMassCorrection(i, j) * vect(bodyOffset + j);
                            qb(i) += invAddedMassCorrection(i, j) * fb(j);
                        }
                    };
                //}
                this->SetVariables(body, qb, 0);
            }
        }

        void FrVariablesAddedMassBase::Compute_inc_Mb_v(chrono::ChMatrix<double>& result,
                                                         const chrono::ChMatrix<double>& vect) const {
            auto HDB = m_addedMassBase->GetRadiationModel()->GetHydroDB();

            for (auto BEMBody = HDB->begin(); BEMBody!=HDB->end(); BEMBody++) {

                auto body = HDB->GetBody(BEMBody->get());

                auto resultOffset = GetBodyOffset(body);

                //for (auto BEMBodyMotion = HDB->begin(); BEMBodyMotion!=HDB->end(); BEMBodyMotion++) {
                    auto BEMBodyMotion = BEMBody;

                    auto fb = GetVariablesFb(body);
                    auto qb = GetVariablesQb(body);

                    auto bodyOffset = GetBodyOffset( HDB->GetBody(BEMBodyMotion->get()) );

                    auto generalizedMass = BEMBody->get()->GetInfiniteAddedMass(BEMBodyMotion->get());

                    for (int i=0; i<6; i++) {
                        for (int j=0; j<6; j++) {
                            //result(resultOffset + i) += generalizedMass(i, j) * vect(bodyOffset + j);
                            qb(i) += generalizedMass(i, j) * fb(j);
                        }
                    };
                // };
                this->SetVariables(body, qb, 0);
            }
        }

        void FrVariablesAddedMassBase::MultiplyAndAdd(chrono::ChMatrix<double> &result,
                                                      const chrono::ChMatrix<double> &vect, const double c_a) const {

            auto HDB = m_addedMassBase->GetRadiationModel()->GetHydroDB();

            for (auto BEMBody = HDB->begin(); BEMBody!=HDB->end(); BEMBody++) {

                auto body = HDB->GetBody(BEMBody->get());
                auto bodyOffset = GetBodyOffset(body);

                auto fb = this->GetVariablesFb(body);
                auto qb = this->GetVariablesQb(body);

                auto generalizedMass = BEMBody->get()->GetInfiniteAddedMass(BEMBody->get());

                for (int i=0; i<6; i++) {
                    for (int j=0; j<6; j++) {
                        //result(bodyOffset + i) += c_a * generalizedMass(i, j) * vect(bodyOffset + j);
                        qb(i) += c_a * generalizedMass(i, j) * fb(j);
                    }
                }
                this->SetVariables(body, qb, 0);
            }
        }

        void FrVariablesAddedMassBase::DiagonalAdd(chrono::ChMatrix<double> &result, const double c_a) const {

            auto HDB = m_addedMassBase->GetRadiationModel()->GetHydroDB();

            for (auto BEMBody = HDB->begin(); BEMBody!=HDB->end(); BEMBody++) {

                auto bodyOffset = GetBodyOffset( HDB->GetBody(BEMBody->get()) );

                auto infiniteAddedMass = BEMBody->get()->GetInfiniteAddedMass(BEMBody->get());

                result(bodyOffset + 0) += c_a * infiniteAddedMass(0, 0);
                result(bodyOffset + 1) += c_a * infiniteAddedMass(1, 1);
                result(bodyOffset + 2) += c_a * infiniteAddedMass(2, 2);
                result(bodyOffset + 3) += c_a * infiniteAddedMass(3, 3);
                result(bodyOffset + 4) += c_a * infiniteAddedMass(4, 4);
                result(bodyOffset + 5) += c_a * infiniteAddedMass(5, 5);

                this->SetVariables(HDB->GetBody(BEMBody->get()), result, bodyOffset);
            }
        }

        void FrVariablesAddedMassBase::Build_M(chrono::ChSparseMatrix &storage, int insrow, int inscol,
                                               const double c_a) {

            auto HDB = m_addedMassBase->GetRadiationModel()->GetHydroDB();

            for (auto BEMBody = HDB->begin(); BEMBody!=HDB->end(); BEMBody++) {

                auto bodyOffset = GetBodyOffset( HDB->GetBody(BEMBody->get()) );

                auto infiniteAddedMass = BEMBody->get()->GetInfiniteAddedMass(BEMBody->get());

                for (int i=0; i<6; i++) {
                    for (int j=0; j<6; j++) {
                        storage.SetElement(insrow + i, inscol + j, c_a * infiniteAddedMass(i, j));
                    }
                }
            }
        }

        int internal::FrVariablesAddedMassBase::GetBodyOffset(FrBody* body) const {
            auto chronoBody = body->GetChronoBody();
            return chronoBody->GetOffset_w();
        }

        void internal::FrVariablesAddedMassBase::SetVariables(FrBody* body, chrono::ChMatrix<double> &result,
                                                              int offset) const {
            auto chronoBody = body->GetChronoBody();
            chronoBody->GetVariables1()->Get_qb().PasteClippedMatrix(result, offset, 0, 6, 1, 0, 0);
        }

        chrono::ChMatrix<double> internal::FrVariablesAddedMassBase::GetVariablesFb(FrBody* body) const {
            auto chronoBody = body->GetChronoBody();
            return chronoBody->GetVariables1()->Get_fb();
        }

        chrono::ChMatrix<double> internal::FrVariablesAddedMassBase::GetVariablesQb(FrBody* body) const {
            auto chronoBody = body->GetChronoBody();
            return chronoBody->GetVariables1()->Get_qb();
        }

    }   // end namespace frydom::internal

} // end namespace frydom
