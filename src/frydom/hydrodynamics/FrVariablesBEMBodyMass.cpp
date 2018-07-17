//
// Created by camille on 09/04/18.
//

#include "frydom/utils/FrEigen.h"
#include "FrVariablesBEMBodyMass.h"

namespace frydom {

    FrVariablesBEMBodyMass::FrVariablesBEMBodyMass() {
        m_InfiniteAddedMass = Eigen::MatrixXd::Zero(6,6);
        m_GeneralizedMass = Eigen::MatrixXd::Zero(6,6);
        m_GeneralizedMass(0,0) = 1.;
        m_GeneralizedMass(1,1) = 1.;
        m_GeneralizedMass(2,2) = 1.;
        m_GeneralizedMass(3,3) = 1.;
        m_GeneralizedMass(4,4) = 1.;
        m_GeneralizedMass(5,5) = 1.;
        m_invGeneralizedMass = m_GeneralizedMass.inverse();
    }

    void FrVariablesBEMBodyMass::Initialize(const ChVariablesBodyOwnMass& variables) {

        ChVariablesBodyOwnMass::operator=(variables);

        m_InfiniteAddedMass = Eigen::MatrixXd::Zero(6,6);
        SetGeneralizedMass();
    }

    void FrVariablesBEMBodyMass::SetInfiniteAddedMass(const Eigen::MatrixXd InfiniteAddedMass) {
        m_InfiniteAddedMass = InfiniteAddedMass;
        SetGeneralizedMass();
    }

    void FrVariablesBEMBodyMass::SetGeneralizedMass() {
        m_GeneralizedMass = m_InfiniteAddedMass;
        m_GeneralizedMass(0,0) += GetBodyMass();
        m_GeneralizedMass(1,1) += GetBodyMass();
        m_GeneralizedMass(2,2) += GetBodyMass();
        m_GeneralizedMass.block<3,3>(3,3) += ChEig(GetBodyInertia());
        m_invGeneralizedMass = m_GeneralizedMass.inverse();

//        std::cout << m_invGeneralizedMass(0,0) << ";" << m_invGeneralizedMass(1,1) << ";" << m_invGeneralizedMass(2,2) << std::endl;
//        std::cout << m_invGeneralizedMass(0,1) << ";" << m_invGeneralizedMass(0,2) << ";" << m_invGeneralizedMass(1,2) << std::endl;

    }

    void FrVariablesBEMBodyMass::Compute_invMb_v(chrono::ChMatrix<double>& result, const chrono::ChMatrix<double>& vect) const {
        assert(vect.GetRows() == Get_ndof());
        assert(result.GetRows() == Get_ndof());
        //
        for (int i=0; i<result.GetRows(); i++) {
            result(i) = 0;
            for (int j=0; j<vect.GetRows(); j++) {
                result(i) += m_invGeneralizedMass(i, j) * vect(j);
            }
        }
    }

    void FrVariablesBEMBodyMass::Compute_inc_invMb_v(chrono::ChMatrix<double>& result, const chrono::ChMatrix<double>& vect) const {
        assert(vect.GetRows() == Get_ndof());
        assert(result.GetRows() == Get_ndof());
        //
        for (int i=0; i<result.GetRows(); i++) {
            for (int j=0; j<vect.GetRows(); j++) {
                result(i) += m_invGeneralizedMass(i, j) * vect(j);
            }
        }
    }

    void FrVariablesBEMBodyMass::Compute_inc_Mb_v(chrono::ChMatrix<double>& result, const chrono::ChMatrix<double>& vect) const {
        assert(vect.GetRows() == Get_ndof());
        assert(result.GetRows() == Get_ndof());
        //
        for (int i=0; i<result.GetRows(); i++) {
            for (int j=0; j<vect.GetRows(); j++) {
                result(i) += m_GeneralizedMass(i, j) * vect(j);
            }
        }

    }

}
