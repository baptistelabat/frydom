//
// Created by camille on 29/01/19.
//


#include "frydom/hydrodynamics/seakeeping/linear/radiation/FrVariablesAddedMassBase.h"
#include "FrAddedMassBase.h"
#include "frydom/hydrodynamics/seakeeping/linear/hdb/FrHydroDB.h"

namespace frydom {

    namespace internal {

        FrVariablesAddedMassBase::FrVariablesAddedMassBase() {

        }

        void FrVariablesAddedMassBase::Initialize() {

        }

        void FrVariablesAddedMassBase::Compute_invMb_v(chrono::ChMatrix<double>& result,
                                                        const chrono::ChMatrix<double>& vect) const {

            auto HDB = m_addedMassBase->GetHDB();

            for (auto BEMBody = HDB->begin(); BEMBody!=HDB->end(); BEMBody++) {

                auto resultOffset = GetBodyOffset(BEMBody->get());

                //for (auto BEMBodyMotion = HDB->begin(); BEMBodyMotion!=HDB->end(); BEMBodyMotion++) {
                    auto BEMBodyMotion = BEMBody;

                    auto bodyOffset = GetBodyOffset(BEMBodyMotion->get());

                    auto invGeneralizedMass = GetInverseGeneralizedMass(BEMBody, BEMBodyMotion);

                    for (int i=0; i<6; i++) {
                        result(resultOffset + i) = 0.;
                        for (int j=0; j<6; j++) {
                            result(resultOffset + i) += invGeneralizedMass(i, j) * vect(bodyOffset + j);
                        }
                    };
                //}
            }
        }

        void FrVariablesAddedMassBase::Compute_inc_invMb_v(chrono::ChMatrix<double>& result,
                                                            const chrono::ChMatrix<double>& vect) const {

            auto HDB = m_addedMassBase->GetHDB();

            for (auto BEMBody = HDB->begin(); BEMBody!=HDB->end(); BEMBody++) {

                auto resultOffset = GetBodyOffset(BEMBody->get());

                //for (auto BEMBodyMotion = HDB->begin(); BEMBodyMotion!=HDB->end(); BEMBodyMotion++) {
                    auto BEMBodyMotion = BEMBody;

                    auto bodyOffset = GetBodyOffset(BEMBodyMotion->get());

                    auto invGeneralizedMass = GetInverseGeneralizedMass(BEMBody, BEMBodyMotion);

                    for (int i=0; i<6; i++) {
                        for (int j=0; j<6; j++) {
                            result(resultOffset + i) += invGeneralizedMass(i, j) * vect(bodyOffset + j);
                        }
                    };
                //}
            }
        }

        void FrVariablesAddedMassBase::Compute_inc_Mb_v(chrono::ChMatrix<double>& result,
                                                         const chrono::ChMatrix<double>& vect) const {
            auto HDB = m_addedMassBase->GetHDB();

            for (auto BEMBody = HDB->begin(); BEMBody!=HDB->end(); BEMBody++) {

                auto resultOffset = GetBodyOffset(BEMBody->get());

                //for (auto BEMBodyMotion = HDB->begin(); BEMBodyMotion!=HDB->end(); BEMBodyMotion++) {
                    auto BEMBodyMotion = BEMBody;

                    auto bodyOffset = GetBodyOffset(BEMBodyMotion->get());

                    auto generalizedMass = GetGeneralizedMass(BEMBody->get(), BEMBodyMotion->get());

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

            auto HDB = m_addedMassBase->GetHDB();

            for (auto BEMBody = HDB->begin(); BEMBody!=HDB->end(); BEMBody++) {

                auto bodyOffset = GetBodyOffset(BEMBody->get());

                auto generalizedMass = GetGeneralizedMass(BEMBody->get(), BEMBody->get());

                for (int i=0; i<6; i++) {
                    for (int j=0; j<6; j++) {
                        result(bodyOffset + i) += c_a * generalizedMass(i, j) * vect(bodyOffset + j);
                    }
                }
            }
        }

        void FrVariablesAddedMassBase::DiagonalAdd(chrono::ChMatrix<double> &result, const double c_a) const {

            auto HDB = m_addedMassBase->GetHDB();

            for (auto BEMBody = HDB->begin(); BEMBody!=HDB->end(); BEMBody++) {

                auto bodyOffset = GetBodyOffset(BEMBody->get());

                auto generalizedMass = GetGeneralizedMass(BEMBody->get(), BEMBody->get());

                result(bodyOffset + 0) += c_a * generalizedMass(0, 0);
                result(bodyOffset + 1) += c_a * generalizedMass(1, 1);
                result(bodyOffset + 2) += c_a * generalizedMass(2, 2);
                result(bodyOffset + 3) += c_a * generalizedMass(3, 3);
                result(bodyOffset + 4) += c_a * generalizedMass(4, 4);
                result(bodyOffset + 5) += c_a * generalizedMass(5, 5);
            }
        }

        void FrVariablesAddedMassBase::Build_M(chrono::ChSparseMatrix &storage, int insrow, int inscol,
                                               const double c_a) {

            auto HDB = m_addedMassBase->GetHDB();

            for (auto BEMBody = HDB->begin(); BEMBody!=HDB->end(); BEMBody++) {

                auto bodyOffset = GetBodyOffset(BEMBody->get());

                auto generalizedMass = GetGeneralizedMass(BEMBody->get(), BEMBody->get());

                for (int i=0; i<6; i++) {
                    for (int j=0; j<6; j++) {
                        storage.SetElement(insrow + i, inscol + j, c_a * generalizedMass(i, j));
                    }
                }
            }
        }

        int internal::FrVariablesAddedMassBase::GetBodyOffset(FrBEMBody_* BEMBody) const {
            auto chronoBody = m_addedMassBase->GetHDB()->GetBody(BEMBody)->GetChronoBody();
            return chronoBody->GetOffset_w();
        }

    }   // end namespace internal

} // end namespace frydom
