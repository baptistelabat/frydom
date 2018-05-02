//
// Created by camille on 09/04/18.
//

#ifndef FRYDOM_FRVARIABLESBEMBODYMASS_H
#define FRYDOM_FRVARIABLESBEMBODYMASS_H

#include "frydom/utils/FrEigen.h"
#include "chrono/solver/ChVariablesBodyOwnMass.h"

namespace frydom {

    enum FrVariablesBody {
        variablesHydro,
        variablesStandard
    };

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

        void SetGeneralizedMass(const Eigen::MatrixXd& GeneralizedMass) { m_GeneralizedMass = GeneralizedMass; }
        Eigen::MatrixXd GetGeneralizedMass() const { return m_GeneralizedMass; }

        void SetInverseGeneralizedMass( const Eigen::MatrixXd& invGeneralizedMass) { m_invGeneralizedMass = invGeneralizedMass; }
        Eigen::MatrixXd GetInverseGeneralizedMass() const { return m_invGeneralizedMass; }

    protected:

        /// Computes the sum of the infinite added mass matrix and the mass and inertia of the body.
        /// Computes the inverse of the generalized mass matrix
        void SetGeneralizedMass();


    };

}


#endif //FRYDOM_FRVARIABLESBEMBODYMASS_H
