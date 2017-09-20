//
// Created by frongere on 18/09/17.
//


#include <iostream>
#include "frydom/misc/FrEigen.h"

#include "Eigen/QR"

using namespace std;
using namespace frydom;

using namespace Eigen;  // TODO : a retirer

void test_conversions() {

    // chrono::ChVector <--> Eigen
    auto vect = chrono::ChVector<double>(1, 2, 3);
    cout << vect;

    auto vect_eigen = ChEig(vect);
    cout << vect_eigen << endl;

    Eigen::Vector3d vect3d_eigen;
    vect3d_eigen << 4, 5, 6;

    auto vect3d_chrono = ChEig(vect3d_eigen);


    // chrono::ChMatrix33 <--> Eigen
    auto mat33 = chrono::ChMatrix33<double>();
    for (int i=0; i<9; ++i) mat33.SetElementN(i, i);
    cout << mat33;

    auto mat_eigen = ChEig(mat33);
    cout << "\n" << mat_eigen << endl;
    cout << ChEig(mat_eigen);

    // chrono::ChMatrixNM <--> Eigen
    auto matNM = chrono::ChMatrixNM<double, 5, 10>();
    int k(0);
    for (int i=0; i<5; ++i) {
        for (int j=0; j<10; ++j) {
            ++k;
            matNM.SetElement(i, j, k);
        }
    }
//    print(matNM);
    cout << matNM;

    auto matNM_eigen = ChEig(matNM);

    cout << "\n" << matNM_eigen << endl;

    cout << ChEig(matNM_eigen);

    // chrono::ChMatrixDynamic <--> Eigen
    chrono::ChMatrixDynamic<double> mat_dyn;
    mat_dyn.Resize(4, 8);
    k = 0;
    for (int i=0; i<4; ++i) {
        for (int j=0; j<8; ++j) {
            ++k;
            mat_dyn.SetElement(i, j, double(k));
        }
    }
    cout << mat_dyn;

    auto mat_dyn_eigen = ChEig(mat_dyn);

    cout << mat_dyn_eigen;

    cout << ChEig(mat_dyn_eigen);
}


//#define nbrows 4
//#define nbcols 10

void test_QR() {

//    auto A = Eigen::Matrix<double, 5, 3>();
//    A.setRandom();
//
//    auto Q = Eigen::Matrix<double, 5, 3>();
//    auto R = Eigen::Matrix<double, 3, 3>();
//
//    QR_decomposition(A, Q, R);
//
//    cout << Q;


//    auto QR = QR_decomp<Eigen::Matrix<double, 5, 3>>(A);


//    auto QR = QR_decomp<Eigen::Matrix<double, 5, 3>>(A);


//    QR_decomposition();


    // A partir d'ici ca fonctionne

    const int m = 10;
    const int n = 3;

//    MatrixXd A(m,n), Q, R, thinQ(MatrixXd::Identity(m, n)), coucou2;
    MatrixXd A(m,n);
    A.setRandom();

//    print(A);

    auto qr = A.householderQr();
//    auto q = qr.householderQ();

    Eigen::MatrixXd Q = qr.householderQ();

    Eigen::MatrixXd R = qr.matrixQR();

    Eigen::MatrixXd tp = qr.matrixQR().template triangularView<Eigen::Upper>();

    cout << tp.block(0,0,3,3);

//    cout << q;




//    cout << "cout";

//    std::cout << "A = \n" << A << "\n\n";
//    HouseholderQR<Eigen::MatrixXd> qr(A);
//
//    Q = qr.householderQ();
////    cout << Q << endl;
////    cout << Q*thinQ << endl;
//    R = qr.matrixQR();
//
//    coucou2 = R.triangularView<Upper>();
//    auto coucou3 = coucou2.block(0, 0, n, n);
//    cout << coucou3 << endl;
//
//
////    cout << R << endl;
//
//    std::cout << Q << "\n\n" << R << "\n\n" << Q*thinQ * coucou3 - A << "\n";
//
//    Eigen::MatrixXd Q2;
//    Eigen::MatrixXd R2;

//    QR_decomposition(A, Q2, R2);



////    int nbrows = 4;
////    int nbcols = 10;
//
////    Eigen::Matrix<double, nbrows, nbcols> mat;
//
//    Eigen::MatrixXd mat(nbrows, nbcols), Q, R;
////    Eigen::Matrix<double, nbrows, nbcols> R;
////    auto mat = Eigen::Matrix<double, 3, 3>();
//
//    mat.setRandom();
//
//    Eigen::HouseholderQR<Eigen::MatrixXd> qr(mat);
//    Q = qr.householderQ();
//    R = qr.matrixQR().triangularView();
//
//    cout << mat << endl << endl;
//    std::cout << Q << "\n\n" << R << "\n\n" << Q * R - mat << "\n";
////    cout << R;
////    auto qr = Eigen::ColPivHouseholderQR();





}

void test_LU() {
    // TODO
}

void test_SVD() {
    // TODO
}

void test_Cholesky() {
    // TODO
}

void test_pinv() {
    // TODO
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





template <class Scalar>
class qr_dec {

private:
    Eigen::HouseholderQR<Eigen::Matrix<Scalar, -1, -1>> QR;

public:
    explicit qr_dec(const Eigen::Matrix<Scalar, -1, -1>& A) {
        QR = A.householderQr();
    }

    Eigen::Matrix<Scalar, -1, -1> GetQ(const bool thin=true) const {
        Eigen::Matrix<Scalar, -1, -1> Q;
        Q = QR.householderQ();
        if (thin) {
            auto thinQ = Eigen::Matrix<Scalar, -1, -1>::Identity(QR.rows(), QR.cols());
            Q *= thinQ;
        }
        return Q;
    }

    Eigen::Matrix<Scalar, -1, -1> GetR(const bool thin=true) const {
        Eigen::Matrix<Scalar, -1, -1> R;
        R = QR.matrixQR().template triangularView<Eigen::Upper>();
//        cout << R;
        if (thin) {
            return R.block(0, 0, QR.cols(), QR.cols());
        } else {
            return R;
        }

    };

};
template <typename Derived, typename OtherDerived>
void decompose(const Eigen::MatrixBase<Derived>& A, const Eigen::MatrixBase<Derived>& Q) {
    cout << A;
}



int main(int argc, char* argv[]) {
//    srand((unsigned int) time(0)); // seeding the standard library random number generator

//    test_conversions();
//    test_linear_algebra();

    Eigen::MatrixXd A(5, 3);
    A.setRandom();
    cout << A << "\n\n";

    auto QR = qr_dec<double>(A);
//    cout << QR.GetQ() << endl << endl;
//    cout << QR.GetR();

//    cout << A - QR.GetQ(true) * QR.GetR(true) << "\n\n";
//    auto diff = A - QR.GetQ(false) * QR.GetR(false);
    auto A2 = QR.GetQ() * QR.GetR();
    cout << A2;
//    cout << QR.GetQ() * QR.GetR();
//    cout << diff;
//    decompose<Eigen::Matrix<double, -1, -1>, Eigen::Matrix<double, -1, -1>>(A, Q, R);


//    Eigen::MatrixXd qr = A.householderQr();

//    auto qr = qr_dec<Eigen::MatrixXd>(A);



    return 0;
}