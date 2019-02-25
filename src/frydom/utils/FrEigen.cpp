//// ==========================================================================
//// FRyDoM - frydom-ce.org
////
//// Copyright (c) Ecole Centrale de Nantes (LHEEA lab.) and D-ICE Engineering.
//// All rights reserved.
////
//// Use of this source code is governed by a GPLv3 license that can be found
//// in the LICENSE file of FRyDoM.
////
//// ==========================================================================
//
//
//
//#include "Eigen/Dense"
//#include "FrEigen.h"
//
//#include "chrono/core/ChVector.h"
//#include <chrono/core/ChMatrix33.h>
//
//namespace frydom {
//
////    Eigen::Vector3d Ch2Ei(const chrono::ChVector<double> vect) {
////        Eigen::Vector3d out;
////        out << vect.x(), vect.y(), vect.z();
////        return out;
////    }
////
////    chrono::ChVector<double> Ch2Ei(const Eigen::Vector3d vect) {
////        chrono::ChVector<double> out(vect.x(), vect.y(), vect.z());
////        return out;
////    }
////
////    Eigen::Matrix3d Ch2Ei(const chrono::ChMatrix33<double> mat) {
////        Eigen::Matrix3d out;
////        out << mat.Get33Element(0, 0), mat.Get33Element(0, 1), mat.Get33Element(0, 2),
////               mat.Get33Element(1, 0), mat.Get33Element(1, 1), mat.Get33Element(1, 2),
////               mat.Get33Element(2, 0), mat.Get33Element(2, 1), mat.Get33Element(2, 2);
////        return out;
////    }
////
////    chrono::ChMatrix33<double> Ch2Ei(const Eigen::Matrix3d mat) {
////        chrono::ChMatrix33<double> out;
////        out.Set33Element(0, 0, mat(0, 0));
////        out.Set33Element(0, 1, mat(0, 1));
////        out.Set33Element(0, 2, mat(0, 2));
////        out.Set33Element(1, 0, mat(1, 0));
////        out.Set33Element(1, 1, mat(1, 1));
////        out.Set33Element(1, 2, mat(1, 2));
////        out.Set33Element(2, 0, mat(2, 0));
////        out.Set33Element(2, 1, mat(2, 1));
////        out.Set33Element(2, 2, mat(2, 2));
////        return out;
////    }
//
//
//}
