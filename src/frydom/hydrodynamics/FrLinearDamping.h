//
// Created by frongere on 11/09/17.
//

#ifndef FRYDOM_FRLINEARDAMPING_H
#define FRYDOM_FRLINEARDAMPING_H


#include "frydom/utils/FrEigen.h"
#include "frydom/core/FrForce.h"

namespace frydom {

    class FrLinearDamping : public FrForce {

    private:
        Eigen::MatrixXd m_dampings;
    public:

        FrLinearDamping() {};

        void SetDampingMatrix(Eigen::MatrixXd dampingMatrix) {
            m_dampings = dampingMatrix;
        }

        void SetDiagonalDamping(double Du, double Dv, double Dw, double Dp, double Dq, double Dr){
            SetDiagonalTranslationDamping(Du, Dv, Dw);
            SetDiagonalRotationDamping(Dp, Dq, Dr);
        }

        void SetDiagonalTranslationDamping(double Du, double Dv, double Dw) {
            m_dampings(0,0) = Du;
            m_dampings(1,1) = Dv;
            m_dampings(2,2) = Dw;
        }
        void SetDiagonalRotationDamping(double Dp, double Dq, double Dr) {
            m_dampings(3,3) = Dp;
            m_dampings(4,4) = Dq;
            m_dampings(5,5) = Dr;
        }

        void SetNonDiagonalDamping(int row, int column, double Dnd) {
            assert(row!=column);
            assert(row<6);
            assert(column<6);
            m_dampings(row,column) = Dnd;
        }

        void SetNonDiagonalRowDamping(int row,const double Dnd[5]){
            assert(row<6);
            int j=0;
            for (int i=1;i<6;i++) {
                if (i==row) continue;
                m_dampings(row,j) = Dnd[i];
                j++;
            }
        }

        void SetNonDiagonalColumnDamping(int column,const double Dnd[5]){
            assert(column<6);
            int j=0;
            for (int i=1;i<6;i++) {
                if (i==column) continue;
                m_dampings(j,column) = Dnd[i];
                j++;
            }
        }

        void SetLogPrefix(std::string prefix_name) override {
            if (prefix_name=="") {
                m_logPrefix = "FlinDamp_" + FrForce::m_logPrefix;
            } else {
                m_logPrefix = prefix_name + "_" + FrForce::m_logPrefix;
            }
        }

        void UpdateState() override;

    };


};  // end namespace frydom
#endif //FRYDOM_FRLINEARDAMPING_H