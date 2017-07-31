//
// Created by frongere on 28/07/17.
//

#ifndef FRYDOM_FRCATENARYLINE_H
#define FRYDOM_FRCATENARYLINE_H

#include <chrono/physics/ChMarker.h>
//#include <chrono/physics/ChNlsolver.h>
#include "chrono/core/ChLinearAlgebra.h"

namespace frydom {


    class FrCatenaryLine {

    private:
        chrono::ChMarker m_marker1;  // TODO: Voir si on ne cree pas de classe derivee frydom
        chrono::ChMarker m_marker2;
        double Lu = 0.;                 ///> unstretched length
        double EA = 0.;                 ///> axial stiffness
        double Cb = 0.;                 ///> cable-seabed friction coefficient
        double w = 0.;                  ///>
        double rho_line = 0.;                 ///> density of the line (kg/m**3)

        chrono::ChMatrix33<double> rotmat;  ///> rotation matrice of the plane inwhich the element lies
        double psi = 0.;                     ///> azimuthal angle of the vertical plane in which the line element lies

        // Data updates
        double L = 0.; ///> current line length
        double HA = 0.;
        double VA = 0.;
        double l = 0.;
        double h = 0.;

        // Data for Newton-Raphson solver
        chrono::ChMatrixNM<double, 2, 1> solution;
        chrono::ChMatrixNM<double, 2, 1> residual;
        chrono::ChMatrixNM<double, 2, 2> jacobian;
        chrono::ChMatrixNM<double, 2, 1> delta;
        double tolerance = 1e-6;
        unsigned int maxiter = 100;

    public:

        /// Set the line axial stiffness from material stiffness and diameter
        void SetAxialStiffness(double E, double A) { EA = E*A; }

        /// Set the line axial stiffness from section stiffness
        void SetAxialStiffness(double p_EA) {EA = p_EA; }

        /// Set the unstretched length of the line element
        void SetUnstretchedLength(double length) { Lu = length; }

        /// Set the seabed-line friction coefficient
        void SetFrictionCoeff(double p_Cb) { Cb = p_Cb; }

        /// Set the line density (kg/m**3)
        void SetDensity(double rho_medium) {};

        /// Compute l and h
        void GetRelativePositions() {};

        /// Compute x(s) and z(s)
        void ComputePosition() {};

        /// Compute tension at s curvilinear abscissa
        double ComputeTension() {};

        inline bool CheckCurvilinearAbscissa() {};

        /// Solve the catenary equations for forces at boundaries
        void Solve() {

            // 1- Initializing solution with initial guess based on precedent solution for HA, VA


            solution.FillElem(0.);  // FIXME: remplir avec le resultat precedent pour HA et VA

            unsigned int iter = 0;
            while (true) {

                if (iter >= maxiter) {

                    break;
                }

                // Compute residuals
                ComputeResiduals();

                // Convergence criteria
                if (residual.NormInf() <= tolerance) {

                    break;
                }

                //Compute jacobian
                ComputeJacobian();

                // Solve LU for delta
                chrono::ChLinearAlgebra::Solve_LinSys(jacobian, &solution, &delta);
                delta.MatrNeg();
                solution.MatrInc(delta);

                iter++;

            }  // end Newton-Raphson iterations

        }; // utiliser chrono/physics/ChNLSolver.h (Newton Raphson)

    private:

        void ComputeResiduals() {
            // 1- Computing h_cur and l_cur at s = L

            // 2- Computing the residual
            residual.FillElem(0.);  // Retirer et renseigner les valeurs
        }

        void ComputeJacobian() {};





    };





}// end namespace frydom

#endif //FRYDOM_FRCATENARYLINE_H
