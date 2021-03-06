//
// Created by camille on 15/05/19.
//

#include "FrVariablesBEMBodyBase.h"

#include "frydom/core/body/FrBody.h"
#include "frydom/hydrodynamics/seakeeping/linear/hdb/FrHydroDB.h"
#include "frydom/hydrodynamics/seakeeping/linear/hdb/FrBEMBody.h"
#include "frydom/hydrodynamics/seakeeping/linear/radiation/FrRadiationModel.h"

namespace frydom {

  namespace internal {

    FrVariablesBEMBodyBase::FrVariablesBEMBodyBase(FrRadiationModelBase *radiationModelBase,
                                                   FrBEMBody *BEMBody,
                                                   chrono::ChVariablesBodyOwnMass *variables)
        : chrono::ChVariablesBody(*variables), m_radiationModelBase(radiationModelBase), m_BEMBody(BEMBody) {
      m_variablesBodyOwnMass = variables;
    }

    void FrVariablesBEMBodyBase::SetBEMBody(FrBEMBody *BEMBody) {
      m_BEMBody = BEMBody;
    }

    void FrVariablesBEMBodyBase::SetRadiationModelBase(FrRadiationModelBase *radiationModelBase) {
      m_radiationModelBase = radiationModelBase;
    }

    void FrVariablesBEMBodyBase::SetVariablesBodyOwnMass(chrono::ChVariablesBodyOwnMass *variables) {
      m_variablesBodyOwnMass = variables;
    }

    void FrVariablesBEMBodyBase::Compute_invMb_v(chrono::ChMatrix<double> &result,
                                                 const chrono::ChMatrix<double> &vect) const {

      result.Reset();

      auto HDB = m_radiationModelBase->GetRadiationModel()->GetHydroDB();

      if (vect.Equals(GetVariablesFb(HDB->GetBody(m_BEMBody)))) {

        for (auto bodyMotion = HDB->begin(); bodyMotion != HDB->end(); bodyMotion++) {

          if (bodyMotion->second->IsActive()) {

            auto fb = GetVariablesFb(bodyMotion->second);
            auto invGeneralizedMass = m_radiationModelBase->GetInverseGeneralizedMass(m_BEMBody, bodyMotion->first);

            for (int i = 0; i < 6; i++) {
              for (int j = 0; j < 6; j++) {
                result(i) += invGeneralizedMass(i, j) * fb(j);
              }
            }
          }
        }

      } else {

        auto invGeneralizedMass = m_radiationModelBase->GetInverseGeneralizedMass(m_BEMBody, m_BEMBody);
        for (int i = 0; i < 6; i++) {
          for (int j = 0; j < 6; j++) {
            result(i) += invGeneralizedMass(i, j) * vect(j);
          }
        }

      }

    }

    void FrVariablesBEMBodyBase::Compute_inc_invMb_v(chrono::ChMatrix<double> &result,
                                                     const chrono::ChMatrix<double> &vect) const {

      auto HDB = m_radiationModelBase->GetRadiationModel()->GetHydroDB();

      if (vect.Equals(GetVariablesFb(HDB->GetBody(m_BEMBody)))) {

        for (auto bodyMotion = HDB->begin(); bodyMotion != HDB->end(); bodyMotion++) {

          if (bodyMotion->second->IsActive()) {

            auto fb = GetVariablesFb(bodyMotion->second);
            auto invGeneralizedMass = m_radiationModelBase->GetInverseGeneralizedMass(m_BEMBody, bodyMotion->first);

            for (int i = 0; i < 6; i++) {
              for (int j = 0; j < 6; j++) {
                result(i) += invGeneralizedMass(i, j) * fb(j);
              }
            }
          }
        }

      } else {

        auto invGeneralizedMass = m_radiationModelBase->GetInverseGeneralizedMass(m_BEMBody, m_BEMBody);

        for (int i = 0; i < 6; i++) {
          for (int j = 0; j < 6; j++) {
            result(i) += invGeneralizedMass(i, j) * vect(j);
          }
        }
      }
    }

    void FrVariablesBEMBodyBase::Compute_inc_invMb_v(chrono::ChMatrix<double> &result,
                                                     const chrono::ChMatrix<double> &vect,
                                                     chrono::ChVariables *variable) const {

      auto BEMBody2 = dynamic_cast<FrVariablesBEMBodyBase *>(variable)->m_BEMBody;
      auto invGeneralizedMass = m_radiationModelBase->GetInverseGeneralizedMass(m_BEMBody, BEMBody2);

      for (int i = 0; i < 6; i++) {
        for (int j = 0; j < 6; j++) {
          result(i) += invGeneralizedMass(i, j) * vect(j);
        }
      }
    }

    void FrVariablesBEMBodyBase::Compute_inc_Mb_v(chrono::ChMatrix<double> &result,
                                                  const chrono::ChMatrix<double> &vect) const {

      auto HDB = m_radiationModelBase->GetRadiationModel()->GetHydroDB();

      if (vect.Equals(GetVariablesQb(HDB->GetBody(m_BEMBody)))) {

        for (auto bodyMotion = HDB->begin(); bodyMotion != HDB->end(); bodyMotion++) {
          if (bodyMotion->second->IsActive()) {
            auto qb = GetVariablesQb(bodyMotion->second);
            auto generalizedMass = m_radiationModelBase->GetGeneralizedMass(m_BEMBody, bodyMotion->first);

            for (int i = 0; i < 6; i++) {
              for (int j = 0; j < 6; j++) {
                result(i) += generalizedMass(i, j) * qb(j);
              }
            }
          }
        }

      } else {

        auto generalizedMass = m_radiationModelBase->GetGeneralizedMass(m_BEMBody, m_BEMBody);

        for (int i = 0; i < 6; i++) {
          for (int j = 0; j < 6; j++) {
            result(i) += generalizedMass(i, j) * vect(j);
          }
        }
      }
    }

    void FrVariablesBEMBodyBase::MultiplyAndAdd(chrono::ChMatrix<double> &result,
                                                const chrono::ChMatrix<double> &vect, const double c_a) const {

      auto HDB = m_radiationModelBase->GetRadiationModel()->GetHydroDB();

      auto generalizedMass = m_radiationModelBase->GetGeneralizedMass(m_BEMBody, m_BEMBody);

      for (int i = 0; i < 6; i++) {
        for (int j = 0; j < 6; j++) {
          result(this->offset + i) += c_a * generalizedMass(i, j) * vect(j);
        }
      }
    }

    void FrVariablesBEMBodyBase::DiagonalAdd(chrono::ChMatrix<double> &result, const double c_a) const {

      auto generalizedMass = m_radiationModelBase->GetGeneralizedMass(m_BEMBody, m_BEMBody);

      for (int i = 0; i < 6; i++) {
        result(this->offset + i) += c_a * generalizedMass(i, i);
      }

    }

    void FrVariablesBEMBodyBase::Build_M(chrono::ChSparseMatrix &storage, int insrow, int inscol, const double c_a) {

      auto generalizedMass = m_radiationModelBase->GetGeneralizedMass(m_BEMBody, m_BEMBody);

      for (int i = 0; i < 6; i++) {
        for (int j = 0; j < 6; j++) {
          storage.SetElement(insrow + i, inscol + j, c_a * generalizedMass(i, j));
        }
      }
    }

    chrono::ChMatrix<double> FrVariablesBEMBodyBase::GetVariablesFb(frydom::FrBody *body) const {
      auto chronoBody = body->GetChronoBody();
      return chronoBody->Variables().Get_fb();
    }

    chrono::ChMatrix<double> FrVariablesBEMBodyBase::GetVariablesQb(frydom::FrBody *body) const {
      auto chronoBody = body->GetChronoBody();
      return chronoBody->Variables().Get_qb();
    }

  } // end namespace internal

} // end namespace frydom
