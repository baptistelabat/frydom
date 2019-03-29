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


#include "FrCatenaryLine.h"

#include "frydom/core/common/FrNode.h"
#include "FrCatenaryForce.h"
#include "frydom/core/body/FrBody.h"
#include "frydom/environment/FrEnvironment.h"


namespace frydom {

    FrCatenaryLine::FrCatenaryLine(const std::shared_ptr<FrNode> &startingNode,
                                     const std::shared_ptr<FrNode> &endingNode, bool elastic, double youngModulus,
                                     double sectionArea, double unstretchedLength, double linearDensity,
                                     FLUID_TYPE fluid) :
                                     m_elastic(elastic), m_q(linearDensity), c_fluid(fluid), m_u(0.,0.,-1.),
                                     FrCable(startingNode, endingNode, unstretchedLength, youngModulus, sectionArea, linearDensity) {}

//    FrCatenaryLineAsset *FrCatenaryLine::GetLineAsset() const {
//        return m_lineAsset.get();
//    }

    void FrCatenaryLine::guess_tension() {

        Position p0pL = GetEndingNode()->GetPositionInWorld(NWU) - GetStartingNode()->GetPositionInWorld(NWU);
        auto lx = p0pL[0];
        auto ly = p0pL[1];
        auto lz = p0pL[2];

        auto chord_length = p0pL.norm();
        auto v = m_u.cross(p0pL/chord_length).cross(m_u);

        double lambda = 0;
        if (m_cableLength <= chord_length) {
            lambda = 0.2;
        } else if ( (m_u.cross(p0pL)).norm() < 1e-4 ) {
            lambda = 1e6;
        } else {
            lambda = sqrt(3. * (m_cableLength*m_cableLength - lz*lz) / (lx*lx + ly*ly));
        }

        auto fu = - 0.5 * m_q * (lz / tanh(lambda) - m_cableLength);
        auto fv = 0.5 * m_q * sqrt(lx*lx + ly*ly) / lambda;

        m_t0 = fu * m_u + fv * v;
    }

    Force FrCatenaryLine::GetTension(double s, FRAME_CONVENTION fc) const {
        Force tension = m_t0 - c_qvec * s;
        if(IsNED(fc)) {internal::SwapFrameConvention(tension);}
        return tension;
    }

    std::shared_ptr<FrCatenaryForce> FrCatenaryLine::GetStartingForce() {
        return m_startingForce;
    }

    std::shared_ptr<FrCatenaryForce> FrCatenaryLine::GetEndingForce() {
        return m_endingForce;
    }

    Force FrCatenaryLine::GetStartingNodeTension(FRAME_CONVENTION fc) const {
        Force t0 = m_t0;
        if(IsNED(fc)) {internal::SwapFrameConvention(t0);}
        return t0;
    }

    Force FrCatenaryLine::GetEndingNodeTension(FRAME_CONVENTION fc) const {
        Force t0 = m_t0 - c_qvec * m_cableLength;
        if(IsNED(fc)) {internal::SwapFrameConvention(t0);}
        return -t0;
    }

    double FrCatenaryLine::_rho(double s) const {
        auto t0_qS = GetTension(s, NWU);
        return t0_qS.norm() - m_u.dot(t0_qS);
    }

    Position FrCatenaryLine::GetUnstrainedChord(double s, FRAME_CONVENTION fc) const {

        Position pc = - (m_u/m_q) * ( (GetTension(s, NWU)).norm() - m_t0.norm() );
        auto rho_0 = _rho(0.);  // TODO: calculer directement

        if (rho_0 > 0.) {
            auto rho_s = _rho(s);  // TODO: calculer directement
            if (rho_s > 0.) {
                pc += (c_Umat*m_t0 / m_q) * log(rho_s / rho_0);
            }
        }
        if (IsNED(fc)) {internal::SwapFrameConvention(pc);}
        return pc;

    }

    Position FrCatenaryLine::GetElasticIncrement(double s, FRAME_CONVENTION fc) const {
        Position Inc(0.,0.,0.);
        if (m_elastic) {Inc = m_q * s * (m_t0 / m_q - 0.5 * m_u * s) / GetEA();}
        if (IsNED(fc)) {internal::SwapFrameConvention(Inc);}
        return Inc;
    }

    Position FrCatenaryLine::GetAbsPosition(double s, FRAME_CONVENTION fc) const {

        Position pos;
        pos += GetStartingNode()->GetPositionInWorld(fc);
        pos += GetUnstrainedChord(s, fc);
        pos += GetElasticIncrement(s, fc);
        return pos;

    }

    double FrCatenaryLine::GetStretchedLength() const {
        double cl = 0.;
        int n = 1000;

        double ds = m_cableLength / (n-1);
        auto pos_prev = GetAbsPosition(0., NWU);

        for (uint i=0; i<n; ++i) {
            auto s = i*ds;
            auto pos = GetAbsPosition(s, NWU);
            cl += (pos - pos_prev).norm();
            pos_prev = pos;
        }
        return cl;
    }

    Position FrCatenaryLine::get_residual(FRAME_CONVENTION fc) const {
        return GetAbsPosition(m_cableLength, fc) - GetEndingNode()->GetPositionInWorld(fc);
    }

    mathutils::Matrix33<double> FrCatenaryLine::analytical_jacobian() const {
        auto t0n = m_t0.norm();

        auto tL = m_t0 - c_qvec * m_cableLength;
        auto tLn = tL.norm();

        auto rho_0 = _rho(0.);  // TODO: calculer directement
        double ln_q = 0.;
        double rho_L = 0.;
        if (rho_0 > 0.) {
            rho_L = _rho(m_cableLength);  // TODO: calculer directement
            ln_q = log(rho_L/rho_0) / m_q;
        }

        double L_EA = 0.;
        if (m_elastic) L_EA = m_cableLength / GetEA();

        mathutils::Vector3d<double> Ui;
        double Uit0;
        double jac_ij;
        double diff_ln;
        auto jac = mathutils::Matrix33<double>();
        for (uint i=0; i<3; ++i) {
            Ui.x() = c_Umat(i, 0);
            Ui.y() = c_Umat(i, 1);
            Ui.z() = c_Umat(i, 2);

            Uit0 = Ui.dot(m_t0) / m_q;

            for (uint j=i; j<3; ++j) {

                jac_ij = - (tL[j]/tLn - m_t0[j]/t0n) * m_u[i] / m_q;

                if (rho_0 > 0.) {
                    jac_ij += c_Umat.at(i, j) * ln_q;
                    diff_ln = (tL[j]/tLn - m_u[j]) / rho_L - (m_t0[j]/t0n - m_u[j]) / rho_0;
                    jac_ij += Uit0 * diff_ln;
                }

                if ( i==j ) {  // elasticity
                    jac_ij += L_EA;  // L_EA is null if no elasticity
                    jac(i, j)= jac_ij;
                } else {
                    jac(i, j)= jac_ij;
                    jac(j, i)= jac_ij;
                }
            } // end for j
        } // end for i

        return jac;
    }

    void FrCatenaryLine::SetSolverTolerance(double tol) {
        m_tolerance = tol;
    }

    void FrCatenaryLine::SetSolverMaxIter(unsigned int maxiter) {
        m_itermax = maxiter;
    }

    void FrCatenaryLine::SetSolverInitialRelaxFactor(double relax) {
        m_relax = relax;
    }

    void FrCatenaryLine::solve() {

        auto res = get_residual(NWU);
        auto jac = analytical_jacobian();

        jac.Inverse();
        mathutils::Vector3d<double> delta_t0 = jac*(-res);

        m_t0 += m_relax * delta_t0;

        res = get_residual(NWU);
        double err = res.infNorm();

        unsigned int iter = 1;
        while ((err > m_tolerance) && (iter < m_itermax)) {
            iter++;

            res = get_residual(NWU);
            jac = analytical_jacobian();

            jac.Inverse();
            mathutils::Vector3d<double> delta_t0_temp = jac*(-res);

            while (delta_t0.infNorm() < m_relax*delta_t0_temp.infNorm()) {
                m_relax *= 0.5;
                if (m_relax < Lmin) {
                    std::cout << "DAMPING TOO STRONG. NO CATENARY CONVERGENCE." << std::endl;
                }
            }

            delta_t0 = delta_t0_temp;
            m_t0 += m_relax * delta_t0;

            m_relax = std::min(1., m_relax*2.);

            res = get_residual(NWU);
            err = res.infNorm();
        }  // end while

    }

    void FrCatenaryLine::Initialize() {

        m_q = GetLinearDensity() - m_sectionArea * GetSystem()->GetEnvironment()->GetFluidDensity(c_fluid);
        c_qvec = m_q*m_u;

        // Initializing U matrix
        c_Umat.SetIdentity();
        c_Umat -= m_u*(m_u.transpose().eval());

        // First guess for the tension
        // FIXME: supprimer ces initialize de node et mettre en place la séparation des SetupInitial des FrPhysicsItems en fonction des Pre, Mid et Post.
        m_startNode->Initialize();
        m_endNode->Initialize();
        guess_tension();
        solve();

        // Building the catenary forces and adding them to bodies
        if (!m_startingForce) {
            m_startingForce = std::make_shared<FrCatenaryForce>(this, LINE_START);
            auto starting_body = m_startNode->GetBody();
            starting_body->AddExternalForce(m_startingForce);
        }

        if (!m_endingForce) {
            m_endingForce = std::make_shared<FrCatenaryForce>(this, LINE_END);
            auto ending_body = m_endNode->GetBody();
            ending_body->AddExternalForce(m_endingForce);
        }

        // Generate assets for the cable
        if (is_lineAsset) {

            auto lineAsset = std::make_shared<FrCatenaryLineAsset>(this);
            lineAsset->Initialize();
            AddAsset(lineAsset);
//            m_lineAsset = std::make_unique<FrCatenaryLineAsset>(this);
//            m_lineAsset->Initialize();
        }
    }

    void FrCatenaryLine::Compute(double time) {
        UpdateTime(time);
        UpdateState();
//        if (is_lineAsset) {
//            m_lineAsset->Update();
//        }
    }

    void FrCatenaryLine::UpdateTime(double time) {
        m_time_step = time - m_time;
        m_time = time;
    }

    void FrCatenaryLine::UpdateState() {
        if (std::abs(m_unrollingSpeed) > DBL_EPSILON and std::abs(m_time_step) > DBL_EPSILON) {
            m_cableLength += m_unrollingSpeed * m_time_step;
        }
        solve();
    }

    void FrCatenaryLine::SetNbElements(unsigned int n) {
        m_nbDrawnElements = n;
    }

    unsigned int FrCatenaryLine::GetNbElements() {
        return m_nbDrawnElements;
    }

    void FrCatenaryLine::InitializeLog() {

        if (IsLogged()) {

            // Build the path to the catenary line log
            auto logPath = m_system->GetPathManager()->BuildPath(this, fmt::format("{}_{}.csv",GetTypeName(),GetShortenUUID()));

            // Add the fields to be logged here
            m_message->AddField<double>("time", "s", "Current time of the simulation",
                                        [this]() { return GetTime(); });

            m_message->AddField<double>("Stretched Length", "m", "Stretched length of the catenary line",
                                        [this]() { return GetStretchedLength(); });

            m_message->AddField<Eigen::Matrix<double, 3, 1>>
            ("Starting Node Tension","N", fmt::format("Starting node tension in world reference frame in {}",c_logFrameConvention),
                    [this]() {return GetStartingNodeTension(c_logFrameConvention);});

            m_message->AddField<Eigen::Matrix<double, 3, 1>>
            ("Ending Node Tension","N", fmt::format("Ending node tension in world reference frame in {}",c_logFrameConvention),
                    [this]() {return GetEndingNodeTension(c_logFrameConvention);});

            //TODO : logger la position de la ligne pour un ensemble d'abscisses curvilignes?

            // Initialize the message
            FrObject::InitializeLog(logPath);

        }

    }

    void FrCatenaryLine::StepFinalize() {

        FrPhysicsItem::StepFinalize();

        // Serialize and send the log message
        FrObject::SendLog();

    }

    std::shared_ptr<FrCatenaryLine>
    make_catenary_line(const std::shared_ptr<FrNode> &startingNode, const std::shared_ptr<FrNode> &endingNode,
                       FrOffshoreSystem *system, bool elastic, double youngModulus, double sectionArea,
                       double unstretchedLength, double linearDensity, FLUID_TYPE fluid){
        auto CatenaryLine = std::make_shared<FrCatenaryLine>(startingNode, endingNode, elastic, youngModulus,
                                                              sectionArea, unstretchedLength, linearDensity, fluid);
        system->Add(CatenaryLine);
        return CatenaryLine;

    }

}  // end namespace frydom
