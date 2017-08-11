//
// Created by frongere on 28/07/17.
//

#ifndef FRYDOM_FRCATENARYLINE_H
#define FRYDOM_FRCATENARYLINE_H

#include <chrono/physics/ChMarker.h>
#include "chrono/core/ChLinearAlgebra.h"
#include "chrono/core/ChMatrix33.h"
#include "FrCatenaryNode.h"

#include <limits>

#define SQRT_EPS sqrt(std::numeric_limits<double>::epsilon())


namespace frydom {


    class FrCatenaryLine {

    private:

        std::shared_ptr<FrCatenaryNode> m_starting_node;
        std::shared_ptr<FrCatenaryNode> m_ending_node;

        bool m_elastic = true;

        double m_Lu = 0.;                 ///> unstretched length
        double m_E = 1e12;                ///> young_modulus (default is near from infinite)
        double m_A = 0.15;                ///> line section area
        double c_EA = 0.15e12;

        chrono::ChVector<double> m_t0 = {1, 1, 1};
        chrono::ChVector<double> c_tL;

        double m_q = 50;
        chrono::ChVector<double> m_u;
        chrono::ChVector<double> c_qvec;

        chrono::ChMatrix33<double> c_Umat;


        // Data for Newton-Raphson solver
        double Lmin = 1e-10;
        double m_tolerance = 1e-6;
        unsigned int m_itermax = 100;
        double relax = 0.1;

    public:

//        FrCatenaryLine() {
//
//        }


        FrCatenaryLine(std::shared_ptr<FrCatenaryNode>& starting_node,
                       std::shared_ptr<FrCatenaryNode>& ending_node,
                       bool elastic,
                       double EA,
                       double L,
                       double q,
                       chrono::ChVector<double> u = {0, 0, -1}
        )
                : m_starting_node(starting_node), m_ending_node(ending_node),
                  m_elastic(elastic),
                  c_EA(EA),
                  m_Lu(L),
                  m_q(q),
                  m_u(u),
                  c_qvec(q*u)
        {

            // Initializing U matrix
            c_Umat.Set33Identity();
            c_Umat -= chrono::TensorProduct(u, u);

            //
            c_tL = m_t0 - c_qvec * m_Lu;

        };

        double _rho(const double s) const {
            auto t0_qS = m_t0 - c_qvec*s;
            return t0_qS.Length() - chrono::Vdot(m_u, t0_qS);
        }

        chrono::ChVector<double> get_unstrained_chord(const double s) const {
            auto rho_0 = _rho(0.);
            auto rho_s = _rho(s);
            return (c_Umat.Matr_x_Vect(m_t0) / m_q) * log(rho_s/rho_0)
                   - (m_u/m_q) * ( (m_t0-c_qvec*s).Length() - m_t0.Length() );

        }

        chrono::ChVector<double> get_elastic_increment(const double s) const {
            if (m_elastic) {
                return m_q * s * (m_t0 / m_q - 0.5 * m_u * s) / c_EA;
            } else {
                return chrono::VNULL;
            }
        }

        chrono::ChVector<double> get_position(const double s) const {
            auto pos = chrono::VNULL;
            pos += m_starting_node->GetPos();
            pos += get_unstrained_chord(s);
            pos += get_elastic_increment(s);
            return pos;
        }

        chrono::ChVector<double> get_residual() const {
            return get_position(m_Lu) - m_ending_node->GetPos();
        }

        chrono::ChMatrix33<double> numerical_jacobian() const {
            auto jac = chrono::ChMatrix33<double>();
            // TODO
            return jac;
        }

        chrono::ChMatrix33<double> analytical_jacobian() const {
            auto t0n = m_t0.Length();
            auto rho_0 = _rho(0.);
            auto tL = m_t0 - c_qvec * m_Lu;
            auto tLn = tL.Length();
            auto rho_L = _rho(m_Lu);
            auto ln_q = log(rho_L/rho_0) / m_q;

            auto jac = chrono::ChMatrix33<double>();

            double L_EA = 0.;
            if (m_elastic) L_EA = m_Lu / c_EA;


            chrono::ChVector<double> Ui;
            double Uit0;
            double jac_ij;
            double diff_ln;

            for (uint i=0; i<3; ++i) {
                Ui.x() = c_Umat(i, 0);
                Ui.y() = c_Umat(i, 1);
                Ui.z() = c_Umat(i, 2);

                Uit0 = chrono::Vdot(Ui, m_t0) / m_q;

                for (uint j=i; j<3; ++j) {
                    jac_ij = c_Umat.Get33Element(i, j) * ln_q;

                    diff_ln = (tL[j]/tLn - m_u[j]) / rho_L - (m_t0[j]/t0n - m_u[j]) / rho_0;
                    jac_ij += Uit0 * diff_ln;

                    jac_ij -= (tL[j]/tLn - m_t0[j]/t0n) * m_u[i] / m_q;

                    if (i==j) {  // elasticity
                        jac_ij += L_EA;
                        jac.SetElement(i, j, jac_ij);
                    } else {
                        jac.SetElement(i, j, jac_ij);
                        jac.SetElement(j, i, jac_ij);
                    }
                } // end for j
            } // end for i

            return jac;
        }

        void solve() {
            auto res = get_residual();
            auto jac = analytical_jacobian();

            chrono::ChVector<double> delta_t0;
            chrono::ChMatrixNM<double, 3, 1> b;
            chrono::ChMatrixNM<double, 3, 1> sol;

            b.SetElement(0, 0, -res.x());
            b.SetElement(1, 0, -res.y());
            b.SetElement(2, 0, -res.z());

            // Solving system
            chrono::ChLinearAlgebra::Solve_LinSys(jac, &b, &sol);

            // Updating the tension
            delta_t0.x() = sol.GetElement(0, 0);
            delta_t0.y() = sol.GetElement(1, 0);
            delta_t0.z() = sol.GetElement(2, 0);

            m_t0 += relax * delta_t0;

            res = get_residual();
            double err = res.LengthInf();

            chrono::ChVector<double> delta_t0_temp;
            uint iter = 1;
            while ((err > m_tolerance) && (iter < m_itermax)) {

                iter++;

                res = get_residual();
                jac = analytical_jacobian();

                b.SetElement(0, 0, -res.x());
                b.SetElement(1, 0, -res.y());
                b.SetElement(2, 0, -res.z());

                chrono::ChLinearAlgebra::Solve_LinSys(jac, &b, &sol);

                delta_t0_temp.x() = sol.GetElement(0, 0);
                delta_t0_temp.y() = sol.GetElement(1, 0);
                delta_t0_temp.z() = sol.GetElement(2, 0);

                while (delta_t0.LengthInf() < (relax*delta_t0_temp).LengthInf()) {
                    relax *= 0.5;
                    if (relax < Lmin) {
                        std::cout << "DAMPING TOO STRONG. NO CATENARY CONVERGENCE." << std::endl;
                    }
                }

                delta_t0 = delta_t0_temp;
                m_t0 += relax * delta_t0;

                relax = chrono::ChMin(1., relax*2.);

                res = get_residual();
                err = res.LengthInf();
            }  // end while

            if (iter < m_itermax) {
                std::cout << "Convergence in " << iter << " iterations." << std::endl;
            } else {
                std::cout << "NO CONVERGENCE AFTER " << m_itermax << " iterations" << std::endl;
            }
        }

        double get_cable_length(double n=1000) const {
            double cl = 0.;

            double ds = m_Lu / (n-1);
            auto pos_prev = get_position(0.);
            chrono::ChVector<double> pos;
            double s;

            for (uint i=0; i<n; ++i) {
                s = i*ds;
                pos = get_position(s);
                cl += (pos - pos_prev).Length();
                pos_prev = pos;
            }
            return cl;
        }

    };





}// end namespace frydom

#endif //FRYDOM_FRCATENARYLINE_H
