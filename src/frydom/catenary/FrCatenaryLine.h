//
// Created by frongere on 28/07/17.
//

#ifndef FRYDOM_FRCATENARYLINE_H
#define FRYDOM_FRCATENARYLINE_H

#include <chrono/physics/ChMarker.h>
#include <chrono/physics/ChNlsolver.h>


namespace frydom {

    void CatenaryEquations() {};


    void CatenaryResidualFcn(chrono::ChMatrix<>* mx, chrono::ChMatrix<>* res, FrCatenaryLine* line) {
        // mx sont les X
        // res sont les residus
        // line est la ligne qu'on resout
    }

    void CatenaryJacobianCompute(chrono::ChMatrix<> mx, chrono::ChMatrix<>* mJ, FrCatenaryLine* line) {
        // Calcule la matrice jacobienne en mx
    }



    class FrCatenaryLine {

    private:
        chrono::ChMarker m_marker1;  // TODO: Voir si on ne cree pas de classe derivee frydom
        chrono::ChMarker m_marker2;
        double A;                       ///> diameter
        double E;                       ///> stiffness of the medium
        double Lu = 0.;                 ///> unstretched length
        double EA = 0.;                 ///> axial stiffness
        double Cb = 0.;                 ///> cable-seabed friction coefficient
        double w = 0.;                  ///>
        double rho_line;                 ///> density of the line (kg/m**3)

        chrono::ChMatrix33<double> rotmat;  ///> rotation matrice of the plane inwhich the element lies
        double psi;                     ///> azimuthal angle of the vertical plane in which the line element lies

        double L = 0.; ///> current line length
        double HA;
        double VA;
        double l;
        double h;

    public:



        void Solve() {

            int maxiter = 100;
            double tolerance = 1e-8;

            chrono::ChMatrix<>* mx;  // solutions a initialiser avec un initial guess...
            chrono::ChMatrix<>* res; // residus


            chrono::ChNonlinearSolver::NewtonRaphson(&CatenaryResidualFcn,
                                                     &CatenaryJacobianCompute,
                                                     mx,
                                                     this,
                                                     maxiter,
                                                     tolerance);

        }; // utiliser chrono/physics/ChNLSolver.h (Newton Raphson)





    };





}// end namespace frydom

#endif //FRYDOM_FRCATENARYLINE_H
