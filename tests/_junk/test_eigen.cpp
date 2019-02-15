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

#include "Eigen/Dense"
#include "frydom/frydom.h"


//#include "Eigen/Dense"

using namespace std;
using namespace frydom;

using namespace chrono;

using namespace Eigen;  // TODO : a retirer

void test_conversions() {

    // chrono::ChVector <--> Eigen
    auto vect = ChVector<double>(1, 2, 3);
//    cout << vect;

    auto vect_eigen = ChEig(vect);
    cout << vect_eigen << endl;

    Eigen::Vector3d vect3d_eigen;
    vect3d_eigen << 4, 5, 6;

    auto vect3d_chrono = ChEig(vect3d_eigen);


    // chrono::ChMatrix33 <--> Eigen
    auto mat33 = ChMatrix33<double>();
    for (int i=0; i<9; ++i) mat33.SetElementN(i, i);
//    cout << mat33;

    auto mat_eigen = ChEig(mat33);
    cout << "\n" << mat_eigen << endl;
//    cout << ChEig(mat_eigen);

    // chrono::ChMatrixNM <--> Eigen
    auto matNM = ChMatrixNM<double, 5, 10>();
    int k(0);
    for (int i=0; i<5; ++i) {
        for (int j=0; j<10; ++j) {
            ++k;
            matNM.SetElement(i, j, k);
        }
    }
//    print(matNM);
//    cout << matNM;

    auto matNM_eigen = ChEig(matNM);

    cout << "\n" << matNM_eigen << endl;

//    cout << ChEig(matNM_eigen);

    // chrono::ChMatrixDynamic <--> Eigen
    ChMatrixDynamic<double> mat_dyn;
    mat_dyn.Resize(4, 8);
    k = 0;
    for (int i=0; i<4; ++i) {
        for (int j=0; j<8; ++j) {
            ++k;
            mat_dyn.SetElement(i, j, double(k));
        }
    }
//    cout << mat_dyn;

    auto mat_dyn_eigen = ChEig(mat_dyn);

    cout << mat_dyn_eigen;

    cout << ChEig(mat_dyn_eigen);
}

void test_QR() {
    Eigen::MatrixXd A(5, 3);
    A.setRandom();

    auto QR = QR_decomposition<double>(A);

    // Computing the infinite norm
    auto norm_inf = (A - QR.GetA()).eval().cwiseAbs().rowwise().sum().maxCoeff();

    assert(norm_inf < 1e-12);

}

void test_LU() {
    Eigen::MatrixXd A(5, 5);
    A.setRandom();

    auto LU = LU_decomposition<double>(A);

    // Computing the infinite norm
    auto norm_inf = (A - LU.GetA()).eval().cwiseAbs().rowwise().sum().maxCoeff();

    assert(norm_inf < 1e-12);
}

void test_SVD() {
    Eigen::MatrixXd A(5, 3);
    A.setRandom();

    auto SVD = SVD_decomposition<double>(A);

    auto norm_inf = (A - SVD.GetA()).eval().cwiseAbs().rowwise().sum().maxCoeff();

    assert(norm_inf < 1e-12);

}

void test_Cholesky() {

    Eigen::MatrixXd A(5, 5);
    A.setRandom();
    A = A * A.transpose();  // Making a symmetric matrix

    auto chol = Cholesky_decomposition<double>(A);

    auto norm_inf = (A - chol.GetA()).eval().cwiseAbs().rowwise().sum().maxCoeff();

    assert(norm_inf < 1e-12);
}

void test_pinv() {
    Eigen::MatrixXd A(10, 5);
    A.setRandom();

    auto Apinv = pinv(A);

    auto norm_inf = (A * Apinv * A - A).cwiseAbs().rowwise().sum().maxCoeff();

    assert(norm_inf < 1e-12);

}

void test_lsqec() {
    // TODO
}

void test_linear_algebra() {

    test_QR();
    test_LU();
    test_SVD();
    test_Cholesky();
    test_pinv();
    test_lsqec();


}

int main(int argc, char* argv[]) {
    srand((unsigned int) time(0)); // seeding the standard library random number generator

    test_conversions();
    test_linear_algebra();


    return 0;
}