//
// Created by camille on 15/05/19.
//

#include "FrVariablesBEMBodyBase.h"

#include "frydom/core/body/FrBody.h"
#include "frydom/hydrodynamics/seakeeping/linear/hdb/FrLinearHDBInc.h"
#include "frydom/hydrodynamics/seakeeping/linear/radiation/FrRadiationModel.h"

namespace frydom {

    namespace internal {

        FrVariablesBEMBodyBase::FrVariablesBEMBodyBase(frydom::internal::FrRadiationModelBase *radiationModelBase,
                frydom::FrBEMBody* BEMBody)
            : m_radiationModelBase(radiationModelBase), m_BEMBody(BEMBody) {}



        void FrVariablesBEMBodyBase::Compute_invMb_v(chrono::ChMatrix<double>& result,
                                                     const chrono::ChMatrix<double>& vect) const {

            result.Reset();

            auto HDB = m_radiationModelBase->GetRadiationModel()->GetHydroDB();

            for (auto bodyMotion = HDB->begin(); bodyMotion!=HDB->end(); bodyMotion++) {

                auto fb = GetVariablesFb(bodyMotion->second);
                auto invGeneralizedMass = m_radiationModelBase->GetInverseGeneralizedMass(m_BEMBody, bodyMotion->first);

                for (int i=0; i<6; i++) {
                    for (int j=0; j<6; j++) {
                        result(i) +=  invGeneralizedMass(i, j) * fb(j);
                    }
                }
            }

        }

        void FrVariablesBEMBodyBase::Compute_inc_invMb_v(chrono::ChMatrix<double>& result,
                                                         const chrono::ChMatrix<double>& vector) const {

            auto HDB = m_radiationModelBase->GetRadiationModel()->GetHydroDB();

            for (auto bodyMotion = HDB->begin(); bodyMotion!=HDB->end(); bodyMotion++) {

                auto fb = GetVariablesFb(bodyMotion->second);
                auto invGeneralizedMass = m_radiationModelBase->GetInverseGeneralizedMass(m_BEMBody, bodyMotion->first);

                for (int i=0; i<6; i++) {
                    for (int j=0; j<6; j++) {
                        result(i) += invGeneralizedMass(i, j) * fb(j);
                    }
                }
            }
        }

        void FrVariablesBEMBodyBase::Compute_inc_Mb_v(chrono::ChMatrix<double>& result,
                                                      const chrono::ChMatrix<double>& vect) const {

            auto HDB = m_radiationModelBase->GetRadiationModel()->GetHydroDB();

            for (auto bodyMotion = HDB->begin(); bodyMotion!=HDB->end(); bodyMotion++) {

                auto fb = GetVariablesFb(bodyMotion->second);
                auto generalizedMass = m_radiationModelBase->GetGeneralizedMass(m_BEMBody, bodyMotion->first);

                for (int i=0; i<6; i++) {
                    for (int j=0; j<6; j++) {
                        result(i) += generalizedMass(i, j) * fb(j);
                    }
                }
            }
        }

        void FrVariablesBEMBodyBase::MultiplyAndAdd(chrono::ChMatrix<double> &result,
                                                    const chrono::ChMatrix<double> &vect, const double c_a) const {

            auto HDB = m_radiationModelBase->GetRadiationModel()->GetHydroDB();

            for (auto bodyMotion = HDB->begin(); bodyMotion!=HDB->end(); bodyMotion++) {

                auto fb = GetVariablesFb(bodyMotion->second);
                auto generalizedMass = m_radiationModelBase->GetGeneralizedMass(m_BEMBody, bodyMotion->first);

                for (int i=0; i<6; i++) {
                    for (int j=0; j<6; j++) {
                        result(this->offset + i) += c_a * generalizedMass(i, j) * fb(j);
                    }
                }
            }
        }

        void FrVariablesBEMBodyBase::DiagonalAdd(chrono::ChMatrix<double>& result, const double c_a) const {

            auto generalizedMass = m_radiationModelBase->GetGeneralizedMass(m_BEMBody, m_BEMBody);

            for (int i=0; i<6; i++) {
                result(this->offset + i) += c_a * generalizedMass(i, i);
            }

        }

        void FrVariablesBEMBodyBase::Build_M(chrono::ChSparseMatrix& storage, int insrow, int inscol, const double c_a) {

            auto generalizedMass = m_radiationModelBase->GetGeneralizedMass(m_BEMBody, m_BEMBody);

            for (int i=0; i<6; i++) {
                for (int j=0; j<6; j++) {
                    storage.SetElement(insrow + i, inscol + j, c_a * generalizedMass(i, j));
                }
            }
        }

        chrono::ChMatrix<double> FrVariablesBEMBodyBase::GetVariablesFb(frydom::FrBody *body) const {
            auto chronoBody = body->GetChronoBody();
            return chronoBody->GetVariables1()->Get_fb();
        }

    } // end namespace internal

} // end namespace frydom