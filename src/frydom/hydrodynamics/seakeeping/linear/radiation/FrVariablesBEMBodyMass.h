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


#ifndef FRYDOM_FRVARIABLESBEMBODYMASS_H
#define FRYDOM_FRVARIABLESBEMBODYMASS_H

//#include "frydom/utils/FrEigen.h"
#include "chrono/solver/ChVariablesBodyOwnMass.h"

#include "MathUtils/MathUtils.h"

namespace frydom {

    enum VARIABLES_BODY {
        HYDRO_VAR,
        STANDARD_VAR
    };

    /**
    * \class FrVariablesBEMBodyMass
    * \brief Class for dealing with added masses.
    */
    class FrVariablesBEMBodyMass : public chrono::ChVariablesBodyOwnMass {

    private:

        Eigen::MatrixXd m_InfiniteAddedMass;        ///< Infinite added mass matrix
        Eigen::MatrixXd m_GeneralizedMass;          ///< Sum of the infinite added mass and mass and inertia
        Eigen::MatrixXd m_invGeneralizedMass;       ///< Inverse of the generalized matrix

    public:

        FrVariablesBEMBodyMass();
        virtual ~FrVariablesBEMBodyMass() {};

        /// Assignment operator : copy from other object
        //FrVariablesBEMBodyMass& operator=(const FrVariablesBEMBodyMass& other);

        void Initialize(const ChVariablesBodyOwnMass& variables);

        /// Set the infinite added mass matrix
        void SetInfiniteAddedMass(const Eigen::MatrixXd InfiniteAddedMass);

        /// Computes the product of the inverse mass matrix by a
        /// vector, and set in result: result = [invMb]*vect
        void Compute_invMb_v(chrono::ChMatrix<double>& result, const chrono::ChMatrix<double>& vect) const override;

        /// Computes the product of the inverse mass matrix by a
        /// vector, and increment result: result += [invMb]*vect
        void Compute_inc_invMb_v(chrono::ChMatrix<double>& result, const chrono::ChMatrix<double>& vect) const override;

        /// Computes the product of the mass matrix by a
        /// vector, and set in result: result = [Mb]*vect
        void Compute_inc_Mb_v(chrono::ChMatrix<double>& result, const chrono::ChMatrix<double>& vect) const override;

        /// Computes the product of the corresponding block in the
        /// system matrix (ie. the mass matrix) by 'vect', scale by c_a, and add to 'result'.
        /// NOTE: the 'vect' and 'result' vectors must already have
        /// the size of the total variables&constraints in the system; the procedure
        /// will use the ChVariable offsets (that must be already updated) to know the
        /// indexes in result and vect.
        void MultiplyAndAdd(chrono::ChMatrix<double>& result,
                                    const chrono::ChMatrix<double>& vect,
                                    const double c_a) const override;

        /// Add the diagonal of the mass matrix scaled by c_a, to 'result'.
        /// NOTE: the 'result' vector must already have the size of system unknowns, ie
        /// the size of the total variables&constraints in the system; the procedure
        /// will use the ChVariable offset (that must be already updated) as index.
        void DiagonalAdd(chrono::ChMatrix<double>& result, const double c_a) const override;

        /// Build the mass matrix (for these variables) scaled by c_a, storing
        /// it in 'storage' sparse matrix, at given column/row offset.
        /// Note, most iterative solvers don't need to know mass matrix explicitly.
        /// Optimized: doesn't fill unneeded elements except mass and 3x3 inertia.
        void Build_M(chrono::ChSparseMatrix& storage, int insrow, int inscol, const double c_a) override;

        void SetGeneralizedMass(const Eigen::MatrixXd& GeneralizedMass) { m_GeneralizedMass = GeneralizedMass; }
        Eigen::MatrixXd GetGeneralizedMass() const { return m_GeneralizedMass; }

        void SetInverseGeneralizedMass( const Eigen::MatrixXd& invGeneralizedMass) { m_invGeneralizedMass = invGeneralizedMass; }
        Eigen::MatrixXd GetInverseGeneralizedMass() const { return m_invGeneralizedMass; }

    protected:

        /// Computes the sum of the infinite added mass matrix and the mass and inertia of the body.
        /// Computes the inverse of the generalized mass matrix
        void SetGeneralizedMass();

    };

}  // end namespace frydom


#endif //FRYDOM_FRVARIABLESBEMBODYMASS_H
