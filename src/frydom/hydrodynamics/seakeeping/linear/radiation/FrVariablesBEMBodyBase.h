//
// Created by camille on 15/05/19.
//

#ifndef FRYDOM_FRVARIABLESBEMBODYBASE_H
#define FRYDOM_FRVARIABLESBEMBODYBASE_H

#include "chrono/solver/ChVariablesBody.h"
#include "chrono/solver/ChVariablesBodyOwnMass.h"

#include "MathUtils/Matrix66.h"

namespace frydom {

    // Forward declaration
    class FrBody;
    class FrBEMBody;

    namespace internal {

        // Forward declaration
        class FrRadiationModelBase;

        class FrVariablesBEMBodyBase : public chrono::ChVariablesBody {

        private:

            FrRadiationModelBase* m_radiationModelBase;
            FrBEMBody* m_BEMBody;
            chrono::ChVariablesBodyOwnMass* m_variablesBodyOwnMass;

        public:

            FrVariablesBEMBodyBase() : ChVariablesBody() {}

            explicit FrVariablesBEMBodyBase(FrRadiationModelBase* radiationModelBase,
                                            FrBEMBody* BEMBody,
                                            chrono::ChVariablesBodyOwnMass* variables);

            void SetBEMBody(FrBEMBody* BEMBody);

            FrBEMBody* GetBEMBody() const { return m_BEMBody; }

            void SetRadiationModelBase(FrRadiationModelBase* radiationModelBase);

            void SetVariablesBodyOwnMass(chrono::ChVariablesBodyOwnMass* variables);

            void Compute_invMb_v(chrono::ChMatrix<double>& result, const chrono::ChMatrix<double>& vect) const override;

            void Compute_inc_invMb_v(chrono::ChMatrix<double>& result, const chrono::ChMatrix<double>& vect) const override;

            void Compute_inc_invMb_v(chrono::ChMatrix<double>& result, const chrono::ChMatrix<double>& vect,
                    chrono::ChVariables* variable) const;

            void Compute_inc_Mb_v(chrono::ChMatrix<double>& result, const chrono::ChMatrix<double>& vect) const override;

            void MultiplyAndAdd(chrono::ChMatrix<double>& result, const chrono::ChMatrix<double>& vect,
                    const double c_a) const override;

            void DiagonalAdd(chrono::ChMatrix<double>& result, const double c_a) const override;

            void Build_M(chrono::ChSparseMatrix& storage, int insrow, int inscol, const double c_a) override;

            chrono::ChMatrix<double> GetVariablesFb(FrBody* body) const;

            chrono::ChMatrix<double> GetVariablesQb(FrBody* body) const;

            //
            // VIRTUAL FUNCTION
            //


            double GetBodyMass() const override { return m_variablesBodyOwnMass->GetBodyMass(); }

            chrono::ChMatrix33<>& GetBodyInertia() override { return m_variablesBodyOwnMass->GetBodyInertia(); }

            const chrono::ChMatrix33<>& GetBodyInertia() const override { return m_variablesBodyOwnMass->GetBodyInertia(); }

            chrono::ChMatrix33<>& GetBodyInvInertia() override { return m_variablesBodyOwnMass->GetBodyInvInertia(); }

            const chrono::ChMatrix33<>& GetBodyInvInertia() const override { return m_variablesBodyOwnMass->GetBodyInvInertia(); }

        };

    } // end namespace internal

} // end namespace frydom

#endif //FRYDOM_FRVARIABLESBEMBODYBASE_H
