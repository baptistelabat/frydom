//
// Created by camille on 29/01/19.
//



#include "frydom/hydrodynamics/seakeeping/linear/hdb/FrHydroDB.h"
#include "frydom/core/body/FrBody.h"
#include "frydom/hydrodynamics/seakeeping/linear/radiation/FrRadiationModel.h"

#include "frydom/hydrodynamics/seakeeping/linear/radiation/FrVariablesAddedMassBase.h"
#include "FrAddedMassBase.h"

namespace frydom {

    namespace internal {

        FrVariablesAddedMassBase::FrVariablesAddedMassBase(FrAddedMassBase* addedMassBase)
            : m_addedMassBase(addedMassBase) { }

        void FrVariablesAddedMassBase::Initialize() {

            auto HDB = m_addedMassBase->GetRadiationModel()->GetHydroDB();

            for (auto BEMBody = HDB->begin(); BEMBody!=HDB->end(); BEMBody++) {

                auto body = HDB->GetBody(BEMBody->get());

                auto generalizedMass = body->GetInertiaTensor(NWU).GetMatrix();
                auto invGeneralizedMass = generalizedMass.inverse();

                auto infiniteAddedMass = BEMBody->get()->GetSelfInfiniteAddedMass();
                auto invInfiniteAddedMass = infiniteAddedMass.inverse();

                auto sumMatrixMass = generalizedMass + infiniteAddedMass;
                auto invSumMatrixMax = sumMatrixMass.inverse();

                m_invAddedMassCorrection[BEMBody->get()] = - invGeneralizedMass * infiniteAddedMass * invSumMatrixMax;
            }
        }

        void FrVariablesAddedMassBase::Compute_invMb_v(chrono::ChMatrix<double>& result,
                                                        const chrono::ChMatrix<double>& vect) const {

            auto HDB = m_addedMassBase->GetRadiationModel()->GetHydroDB();

            for (auto BEMBody = HDB->begin(); BEMBody!=HDB->end(); BEMBody++) {

                auto resultOffset = GetBodyOffset( HDB->GetBody(BEMBody->get()) );

                //for (auto BEMBodyMotion = HDB->begin(); BEMBodyMotion!=HDB->end(); BEMBodyMotion++) {
                    auto BEMBodyMotion = BEMBody;

                    auto bodyOffset = GetBodyOffset( HDB->GetBody(BEMBodyMotion->get()) );

                    auto invAddedMassCorrection = m_invAddedMassCorrection.at(BEMBody->get());

                    for (int i=0; i<6; i++) {
                        result(resultOffset + i) = 0.;
                        for (int j=0; j<6; j++) {
                            result(resultOffset + i) += invAddedMassCorrection(i, j) * vect(bodyOffset + j);
                        }
                    };
                //}
            }
        }

        void FrVariablesAddedMassBase::Compute_inc_invMb_v(chrono::ChMatrix<double>& result,
                                                            const chrono::ChMatrix<double>& vect) const {

            auto HDB = m_addedMassBase->GetRadiationModel()->GetHydroDB();

            for (auto BEMBody = HDB->begin(); BEMBody!=HDB->end(); BEMBody++) {

                auto resultOffset = GetBodyOffset( HDB->GetBody(BEMBody->get()) );

                //for (auto BEMBodyMotion = HDB->begin(); BEMBodyMotion!=HDB->end(); BEMBodyMotion++) {
                    auto BEMBodyMotion = BEMBody;

                    auto bodyOffset = GetBodyOffset( HDB->GetBody(BEMBodyMotion->get()) );

                    auto invAddedMassCorrection = m_invAddedMassCorrection.at(BEMBody->get());

                    for (int i=0; i<6; i++) {
                        for (int j=0; j<6; j++) {
                            result(resultOffset + i) += invAddedMassCorrection(i, j) * vect(bodyOffset + j);
                        }
                    };
                //}
            }
        }

        void FrVariablesAddedMassBase::Compute_inc_Mb_v(chrono::ChMatrix<double>& result,
                                                         const chrono::ChMatrix<double>& vect) const {
            auto HDB = m_addedMassBase->GetRadiationModel()->GetHydroDB();

            for (auto BEMBody = HDB->begin(); BEMBody!=HDB->end(); BEMBody++) {

                auto resultOffset = GetBodyOffset(HDB->GetBody(BEMBody->get()) );

                //for (auto BEMBodyMotion = HDB->begin(); BEMBodyMotion!=HDB->end(); BEMBodyMotion++) {
                    auto BEMBodyMotion = BEMBody;

                    auto bodyOffset = GetBodyOffset( HDB->GetBody(BEMBodyMotion->get()) );

                    auto generalizedMass = BEMBody->get()->GetInfiniteAddedMass(BEMBodyMotion->get());

                    for (int i=0; i<6; i++) {
                        for (int j=0; j<6; j++) {
                            result(resultOffset + i) += generalizedMass(i, j) * vect(bodyOffset + j);
                        }
                    };
                //}
            }
        }

        void FrVariablesAddedMassBase::MultiplyAndAdd(chrono::ChMatrix<double> &result,
                                                      const chrono::ChMatrix<double> &vect, const double c_a) const {

            auto HDB = m_addedMassBase->GetRadiationModel()->GetHydroDB();

            for (auto BEMBody = HDB->begin(); BEMBody!=HDB->end(); BEMBody++) {

                auto bodyOffset = GetBodyOffset( HDB->GetBody(BEMBody->get()) );

                auto generalizedMass = BEMBody->get()->GetInfiniteAddedMass(BEMBody->get());

                for (int i=0; i<6; i++) {
                    for (int j=0; j<6; j++) {
                        result(bodyOffset + i) += c_a * generalizedMass(i, j) * vect(bodyOffset + j);
                    }
                }
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

        int internal::FrVariablesAddedMassBase::GetBodyOffset(FrBody_* body) const {
            //auto chronoBody = body->GetChronoBody();
            //return chronoBody->GetOffset_w();
            return 0;
        }

    }   // end namespace internal

} // end namespace frydom
