//
// Created by frongere on 18/09/17.
//


#include <iostream>
#include "chrono/core/ChVector.h"
#include "Eigen/Dense"

using namespace std;
using namespace Eigen;

int main(int argc, char* argv[]) {

    // Trying to get a ChVector map to eigen vectors
    auto vect = chrono::ChVector<double>(1, 2, 3);

    



//
//    chrono::ChVector<double>* vect_ptr;
//    vect_ptr = &vect;

//    int array[8];
//    for(int i = 0; i < 8; ++i) array[i] = i;
//    cout << "Column-major:\n" << Map<Matrix<int,2,4> >(array) << endl;
//    cout << "Row-major:\n" << Map<Matrix<int,2,4,RowMajor> >(array) << endl;
//    cout << "Row-major using stride:\n" <<
//         Map<Matrix<int,2,4>, Unaligned, Stride<1,4> >(array) << endl;

    MatrixXd m(2, 2);
    m(0, 0) = 3;
    m(1, 0) = 2.5;
    m(0, 1) = -1;
    m(1, 1) = m(1, 0) + m(0, 1);

    std::cout << m << std::endl;

    return 0;
}