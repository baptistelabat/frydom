//
// Created by frongere on 28/07/17.
//

#include "FrCatenaryLine.h"


namespace frydom {


    FrCatenaryLine::FrCatenaryLine(const std::shared_ptr<FrNode> &startingNode, const std::shared_ptr<FrNode> &endingNode,
                                   bool elastic, const double youngModulus, const double sectionArea,
                                   const double cableLength,
                                   const double q, const chrono::ChVector<double> u)
            : m_elastic(elastic),
              m_q(q),
              m_u(u),
              c_qvec(q*u),
              FrCable(startingNode, endingNode, cableLength, youngModulus, sectionArea)
    {

        // Initializing U matrix
        c_Umat.Set33Identity();
        c_Umat -= chrono::TensorProduct(u, u);

        // First guess for the tension
        guess_tension();
        solve();

        // Building the catenary forces and adding them to bodies
        m_startingForce = std::make_shared<FrCatenaryForce>(this, LINE_START);
        auto starting_body = m_startingNode->GetBody();
        starting_body->AddForce(m_startingForce);


        m_endingForce = std::make_shared<FrCatenaryForce>(this, LINE_END);
        auto ending_body = m_endingNode->GetBody();
        ending_body->AddForce(m_endingForce);

        // Completing the ChLink required parameters
        Body1 = m_startingNode->GetBody();
        Body2 = m_endingNode  ->GetBody();

    }

    void FrCatenaryLine::guess_tension() {

        auto p0pL = GetPosEndingNode() - GetPosStartingNode();
        auto lx = p0pL[0];
        auto ly = p0pL[1];
        auto lz = p0pL[2];

        auto chord_length = p0pL.Length();
        auto v = m_u.Cross(p0pL/chord_length).Cross(m_u);

        double lambda = 0;
        if (m_cableLength <= chord_length) {
            lambda = 0.2;
        } else if ( (m_u.Cross(p0pL)).Length() < 1e-4 ) {
            lambda = 1e6;
        } else {
            lambda = sqrt(3. * (m_cableLength*m_cableLength - lz*lz) / (lx*lx + ly*ly));
        }

        auto fu = - 0.5 * m_q * (lz / tanh(lambda) - m_cableLength);
        auto fv = 0.5 * m_q * sqrt(lx*lx + ly*ly) / lambda;

        m_t0 = fu * m_u + fv * v;
    }

    chrono::ChVector<double> FrCatenaryLine::GetTension(const double s) const {
        return m_t0 - c_qvec * s;
    }

    chrono::ChVector<double> FrCatenaryLine::GetEndingNodeTension() const {
        return m_t0 - c_qvec * m_cableLength;
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
            return m_q * s * (m_t0 / m_q - 0.5 * m_u * s) / GetEA();
        } else {
            return chrono::VNULL;
        }

    }

    chrono::ChVector<double> FrCatenaryLine::GetAbsPosition(const double s) const {

        auto pos = chrono::VNULL;
        pos += GetPosStartingNode();
        pos += GetUnstrainedChord(s);
        pos += GetElasticIncrement(s);
        return pos;

    }

    chrono::ChVector<double> FrCatenaryLine::get_residual() const {
        return GetAbsPosition(m_cableLength) - GetPosEndingNode();
    }

    chrono::ChMatrix33<double> FrCatenaryLine::analytical_jacobian() const {
        auto t0n = m_t0.Length();

        auto tL = m_t0 - c_qvec * m_cableLength;
        auto tLn = tL.Length();

        auto rho_0 = _rho(0.);  // TODO: calculer directement
        double ln_q = 0.;
        double rho_L = 0.;
        if (rho_0 > 0.) {
            rho_L = _rho(m_cableLength);  // TODO: calculer directement
            ln_q = log(rho_L/rho_0) / m_q;
        }

        double L_EA = 0.;
        if (m_elastic) L_EA = m_cableLength / GetEA();

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

        m_t0 += m_relax * delta_t0;

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

            while (delta_t0.LengthInf() < (m_relax*delta_t0_temp).LengthInf()) {
                m_relax *= 0.5;
                if (m_relax < Lmin) {
                    std::cout << "DAMPING TOO STRONG. NO CATENARY CONVERGENCE." << std::endl;
                }
            }

            delta_t0 = delta_t0_temp;
            m_t0 += m_relax * delta_t0;

            m_relax = chrono::ChMin(1., m_relax*2.);

            res = get_residual();
            err = res.LengthInf();
        }  // end while

//        if (iter < m_itermax) {
//            std::cout << "Catenary convergence reached in " << iter << " iterations." << std::endl;
//        } else {
//            std::cout << "NO CONVERGENCE AFTER " << m_itermax << " iterations" << std::endl;
//        }
    }

    double FrCatenaryLine::GetCableLength(const double n) const {
        double cl = 0.;

        double ds = m_cableLength / (n-1);
        auto pos_prev = GetAbsPosition(0.);
        chrono::ChVector<double> pos;
        double s;

        for (uint i=0; i<n; ++i) {
            s = i*ds;
            pos = GetAbsPosition(s);
            cl += (pos - pos_prev).Length();
            pos_prev = pos;
        }
        return cl;
    }

    ////////////////////////////////////////////////
    // Visu
    ////////////////////////////////////////////////

    void FrCatenaryLine::GenerateAssets() {
        // Assets for the cable visualisation
        if (m_drawCableElements) {
            double ds = m_cableLength/m_nbDrawnElements;
            auto Pos0 = Body2->TransformPointParentToLocal(GetAbsPosition(0));
            for (int i=1; i<m_nbDrawnElements; i++){
                auto Pos1 = Body2->TransformPointParentToLocal(GetAbsPosition(ds*i));
                auto newLine = std::make_shared<chrono::geometry::ChLineSegment>(Pos0,Pos1);
                auto newElement = std::make_shared<chrono::ChLineShape>();
                auto myColor = chrono::ChColor::ComputeFalseColor(GetTension(ds*i).Length(),0,m_maxTension,true);
                newElement->SetColor(myColor);
                newElement->SetLineGeometry(newLine);
                m_cableElements.push_back(newElement);
                AddAsset(newElement);
                Pos0 = Pos1;
            }
        }
    }

    void FrCatenaryLine::UpdateAsset() {
        if (m_drawCableElements) {
            double ds = m_cableLength/m_nbDrawnElements;
            auto Pos0 = Body2->TransformPointParentToLocal(GetAbsPosition(0));
            for (int i=1; i<m_nbDrawnElements; i++){
                auto Pos1 = Body2->TransformPointParentToLocal(GetAbsPosition(i*ds));
                auto newLine = std::make_shared<chrono::geometry::ChLineSegment>(Pos0,Pos1);
                m_cableElements[i-1]->SetLineGeometry(newLine);
                auto myColor = chrono::ChColor::ComputeFalseColor(GetTension(ds*i).Length(),0,m_maxTension,true);
                m_cableElements[i-1]->SetColor(myColor);
                Pos0 = Pos1;
            }
        }
    }

    void FrCatenaryLine::InitRangeTensionColor() {
        double ds = m_cableLength/m_nbDrawnElements;
        double max = GetTension(0).Length();
        for (int i=1; i<m_nbDrawnElements; i++){
            auto LocalTension = GetTension(i*ds).Length();
            if (LocalTension > max) max = LocalTension;
        }
        m_maxTension = 1.25*max;
    }


}// end namespace frydom
