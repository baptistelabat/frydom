//
// Created by camille on 15/05/19.
//

#ifndef FRYDOM_FRVARIABLESBEMBODYBASE_H
#define FRYDOM_FRVARIABLESBEMBODYBASE_H

#include "chrono/solver/ChVariables.h"
#include "chrono/solver/ChVariablesBodyOwnMass.h"

#include "MathUtils/Matrix66.h"

namespace frydom {

    // Forward declaration
    class FrBody;
    class FrBEMBody;

    namespace internal {

        // Forward declaration
        class FrRadiationModelBase;

        class FrVariablesBEMBodyBase : public chrono::ChVariables {

        private:

            FrRadiationModelBase* m_radiationModelBase;
            FrBEMBody* m_BEMBody;
            chrono::ChVariablesBodyOwnMass* m_variablesBodyOwnMass;

        public:

            FrVariablesBEMBodyBase() : ChVariables(6) {}

            explicit FrVariablesBEMBodyBase(FrRadiationModelBase* radiationModelBase, FrBEMBody* BEMBody);

            void SetBEMBody(FrBEMBody* BEMBody);

            void SetRadiationModelBase(FrRadiationModelBase* radiationModelBase);

            void SetVariablesBodyOwnMass(chrono::ChVariablesBodyOwnMass* variables);

            void Compute_invMb_v(chrono::ChMatrix<double>& result, const chrono::ChMatrix<double>& vect) const override;

            void Compute_inc_invMb_v(chrono::ChMatrix<double>& result, const chrono::ChMatrix<double>& vect) const override;

            void Compute_inc_Mb_v(chrono::ChMatrix<double>& result, const chrono::ChMatrix<double>& vect) const override;

            void MultiplyAndAdd(chrono::ChMatrix<double>& result, const chrono::ChMatrix<double>& vect,
                    const double c_a) const override;

            void DiagonalAdd(chrono::ChMatrix<double>& result, const double c_a) const override;

            void Build_M(chrono::ChSparseMatrix& storage, int insrow, int inscol, const double c_a) override;

            chrono::ChMatrix<double> GetVariablesFb(FrBody* body) const;


        };

    } // end namespace internal

} // end namespace frydom

#endif //FRYDOM_FRVARIABLESBEMBODYBASE_H
