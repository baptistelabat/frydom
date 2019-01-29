//
// Created by camille on 29/01/19.
//

#include "frydom/hydrodynamics/seakeeping/linear/radiation/FrVariablesAddedMassBase.h"

namespace frydom {

    namespace internal {

        FrVariablesAddedMassBase::FrVariablesAddedMassBase() {

        }

        void FrVariablesAddedMassBase::SetInfiniteAddedMass(const Eigen::MatrixXd infiniteAddedMass) {
            m_infiniteAddedMass = infiniteAddedMass;
        }

        void FrVariablesAddedMassBase::Compute_invMb_v(chrono::ChMatrix<double>& result,
                                                        const chrono::ChMatrix<double>& vect) const {

            auto HDB = this->GetHDB();

            for (auto BEMBody = HDB->begin(); BEMBody!=HDB->end(); BEMBody++) {

                auto chronoBody = HDB->GetBody(BEMBody->get())->GetChronoBody();
                auto resultOffset = chronoBody->GetOffset_w();

                for (auto BEMBodyMotion = HDB->begin(); BEMBodyMotion!=HDB->end(); BEMBodyMotion++) {

                    auto chronoBodyMotion = HDB->GetBody(BEMBodyMotion->get())->GetChronoBody();
                    auto bodyOffset = chronoBodyMotion->GetOffset_w();

                    auto invGeneralizedMass = GetInverseGeneralizedMass(BEMBody, BEMBodyMotion);

                    for (int i=0; i<6; i++) {
                        result(resultOffset + i) = 0.;
                        for (int j=0; j<6; j++) {
                            result(resultOffset + i) += invGeneralizedMass(i, j) * vect(bodyOffset + j);
                        }
                    };
                }
            }
        }

        void FrVariablesAddedMassBase::Compute_inc_invMb_v(chrono::ChMatrix<double>& result,
                                                            const chrono::ChMatrix<double>& vect) const {

            auto HDB = this->GetHDB();

            for (auto BEMBody = HDB->begin(); BEMBody!=HDB->end(); BEMBody++) {

                auto chronoBody = HDB->GetBody(BEMBody->get())->GetChronoBody();
                auto resultOffset = chronoBody->GetOffset_w();

                for (auto BEMBodyMotion = HDB->begin(); BEMBodyMotion!=HDB->end(); BEMBodyMotion++) {

                    auto chronoBodyMotion = HDB->GetBody(BEMBodyMotion->get())->GetChronoBody();
                    auto bodyOffset = chronoBodyMotion->GetOffset_w();

                    auto invGeneralizedMass = GetInverseGeneralizedMass(BEMBody, BEMBodyMotion);

                    for (int i=0; i<6; i++) {
                        for (int j=0; j<6; j++) {
                            result(resultOffset + i) += invGeneralizedMass(i, j) * vect(bodyOffset + j);
                        }
                    };
                }
            }

        }

        void FrVariablesAddedMassBase::Compute_inc_Mb_v(chrono::ChMatrix<double>& result,
                                                         const chrono::ChMatrix<double>& vect) const {
            auto HDB = this->GetHDB();

            for (auto BEMBody = HDB->begin(); BEMBody!=HDB->end(); BEMBody++) {

                auto chronoBody = HDB->GetBody(BEMBody->get())->GetChronoBody();
                auto resultOffset = chronoBody->GetOffset_w();

                for (auto BEMBodyMotion = HDB->begin(); BEMBodyMotion!=HDB->end(); BEMBodyMotion++) {

                    auto chronoBodyMotion = HDB->GetBody(BEMBodyMotion->get())->GetChronoBody();
                    auto bodyOffset = chronoBodyMotion->GetOffset_w();

                    auto generalizedMass = GetGeneralizedMass(BEMBody, BEMBodyMotion);

                    for (int i=0; i<6; i++) {
                        for (int j=0; j<6; j++) {
                            result(resultOffset + i) += generalizedMass(i, j) * vect(bodyOffset + j);
                        }
                    };
                }
            }
        }



    }
}