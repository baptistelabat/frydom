//
// Created by frongere on 28/07/17.
//

#include "FrCatenaryLine.h"


namespace frydom {


    FrCatenaryLine::FrCatenaryLine(std::shared_ptr<FrNode> &starting_node, std::shared_ptr<FrNode> &ending_node,
                                   bool elastic, double EA, double L, double q, chrono::ChVector<double> u)
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

        // First guess for the tension
        guess_tension();
        solve();

        // B uilding the catenary forces and adding them to bodies
        m_starting_force = std::make_shared<FrCatenaryForce>(this, LINE_START);
        auto starting_body = m_starting_node->GetBody();
        starting_body->AddForce(m_starting_force);


        m_ending_force = std::make_shared<FrCatenaryForce>(this, LINE_END);
        auto ending_body = m_ending_node->GetBody();
        ending_body->AddForce(m_ending_force);

    }

    void FrCatenaryLine::guess_tension() {

        auto p0pL = GetPosEndingNode() - GetPosStartingNode();
        auto lx = p0pL[0];
        auto ly = p0pL[1];
        auto lz = p0pL[2];

        auto chord_length = p0pL.Length();
        auto v = m_u.Cross(p0pL/chord_length).Cross(m_u);

        double lambda = 0;
        if (m_Lu <= chord_length) {
            lambda = 0.2;
        } else if ( (m_u.Cross(p0pL)).Length() < 1e-4 ) {
            lambda = 1e6;
        } else {
            lambda = sqrt(3. * (m_Lu*m_Lu - lz*lz) / (lx*lx + ly*ly));
        }

        auto fu = - 0.5 * m_q * (lz / tanh(lambda) - m_Lu);
        auto fv = 0.5 * m_q * sqrt(lx*lx + ly*ly) / lambda;

        m_t0 = fu * m_u + fv * v;
    }

    void FrCatenaryLine::SetEA(const double E, const double A) {
        m_E = E;
        m_A = A;
        c_EA = E*A;
    }

    chrono::ChVector<double> FrCatenaryLine::GetTension(const double s) const {
        return m_t0 - c_qvec * s;
    }

    chrono::ChVector<double> FrCatenaryLine::GetEndingNodeTension() const {
        return m_t0 - c_qvec * m_Lu;
    }

    chrono::ChVector<double> FrCatenaryLine::GetUnstrainedChord(const double s) const {

        chrono::ChVector<double> pc;
        pc = - (m_u/m_q) * ( (GetTension(s)).Length() - m_t0.Length() );
        auto rho_0 = _rho(0.);  // TODO: calculer directement
        if (rho_0 > 0.) {
            auto rho_s = _rho(s);  // TODO: calculer directement
            if (rho_s > 0.) {
                pc += (c_Umat.Matr_x_Vect(m_t0) / m_q) * log(rho_s / rho_0);
            }
        }
        return pc;

    }

    chrono::ChVector<double> FrCatenaryLine::GetElasticIncrement(const double s) const {

        if (m_elastic) {
            return m_q * s * (m_t0 / m_q - 0.5 * m_u * s) / c_EA;
        } else {
            return chrono::VNULL;
        }

    }

    chrono::ChVector<double> FrCatenaryLine::GetPos(const double s) const {

        auto pos = chrono::VNULL;
        pos += GetPosStartingNode();
        pos += GetUnstrainedChord(s);
        pos += GetElasticIncrement(s);
        return pos;

    }

    chrono::ChVector<double> FrCatenaryLine::get_residual() const {
        return GetPos(m_Lu) - GetPosEndingNode();
    }

    chrono::ChMatrix33<double> FrCatenaryLine::analytical_jacobian() const {
        auto t0n = m_t0.Length();

        auto tL = m_t0 - c_qvec * m_Lu;
        auto tLn = tL.Length();

        auto rho_0 = _rho(0.);  // TODO: calculer directement
        double ln_q = 0.;
        double rho_L = 0.;
        if (rho_0 > 0.) {
            rho_L = _rho(m_Lu);  // TODO: calculer directement
            ln_q = log(rho_L/rho_0) / m_q;
        }

        double L_EA = 0.;
        if (m_elastic) L_EA = m_Lu / c_EA;


        chrono::ChVector<double> Ui;
        double Uit0;
        double jac_ij;
        double diff_ln;
        auto jac = chrono::ChMatrix33<double>();
        for (uint i=0; i<3; ++i) {
            Ui.x() = c_Umat(i, 0);
            Ui.y() = c_Umat(i, 1);
            Ui.z() = c_Umat(i, 2);

            Uit0 = chrono::Vdot(Ui, m_t0) / m_q;

            for (uint j=i; j<3; ++j) {

                jac_ij = - (tL[j]/tLn - m_t0[j]/t0n) * m_u[i] / m_q;

                if (rho_0 > 0.) {
                    jac_ij += c_Umat.Get33Element(i, j) * ln_q;
                    diff_ln = (tL[j]/tLn - m_u[j]) / rho_L - (m_t0[j]/t0n - m_u[j]) / rho_0;
                    jac_ij += Uit0 * diff_ln;
                }

                if ( i==j ) {  // elasticity
                    jac_ij += L_EA;  // L_EA is null if no elasticity
                    jac.SetElement(i, j, jac_ij);
                } else {
                    jac.SetElement(i, j, jac_ij);
                    jac.SetElement(j, i, jac_ij);
                }
            } // end for j
        } // end for i

        return jac;
    }

    void FrCatenaryLine::solve() {

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
        unsigned int iter = 1;
        while ((err > m_tolerance) && (iter < m_itermax)) {

            iter++;

            res = get_residual();
            jac = analytical_jacobian();

            b.SetElement(0, 0, -res.x());
            b.SetElement(1, 0, -res.y());
            b.SetElement(2, 0, -res.z());

            // TODO: voir a remplacer par une resolution via Eigen
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

//        if (iter < m_itermax) {
//            std::cout << "Catenary convergence reached in " << iter << " iterations." << std::endl;
//        } else {
//            std::cout << "NO CONVERGENCE AFTER " << m_itermax << " iterations" << std::endl;
//        }
    }

    double FrCatenaryLine::get_cable_length(const double n) const {
        double cl = 0.;

        double ds = m_Lu / (n-1);
        auto pos_prev = GetPos(0.);
        chrono::ChVector<double> pos;
        double s;

        for (uint i=0; i<n; ++i) {
            s = i*ds;
            pos = GetPos(s);
            cl += (pos - pos_prev).Length();
            pos_prev = pos;
        }
        return cl;
    }


}// end namespace frydom