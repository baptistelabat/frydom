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
#include "frydom/asset/FrCatenaryLineAsset.h"


namespace frydom {

    template<typename OffshoreSystemType>
    FrCatenaryLine<OffshoreSystemType>::FrCatenaryLine(const std::shared_ptr<FrNode<OffshoreSystemType>> &startingNode,
                                                       const std::shared_ptr<FrNode<OffshoreSystemType>> &endingNode,
                                                       const std::shared_ptr<FrCableProperties> &properties,
                                                       bool elastic,
                                                       double unstrainedLength,
                                                       FLUID_TYPE fluid) :
        m_elastic(elastic),
        c_fluid(fluid),
        FrCable<OffshoreSystemType>(startingNode, endingNode, properties, unstrainedLength),
        FrPrePhysicsItem<OffshoreSystemType>() {

      m_q = properties->GetLinearDensity();
      this->SetLogged(true);
    };

    template<typename OffshoreSystemType>
    void FrCatenaryLine<OffshoreSystemType>::SetSolverTolerance(double tol) {
      m_tolerance = tol;
    }

    template<typename OffshoreSystemType>
    void FrCatenaryLine<OffshoreSystemType>::SetSolverMaxIter(unsigned int maxiter) {
      m_itermax = maxiter;
    }

    template<typename OffshoreSystemType>
    void FrCatenaryLine<OffshoreSystemType>::SetSolverInitialRelaxFactor(double relax) {
      m_relax = relax;
    }

    template<typename OffshoreSystemType>
    void FrCatenaryLine<OffshoreSystemType>::guess_tension() {

      Position p0pL = this->GetEndingNode()->GetPositionInWorld(NWU) - this->GetStartingNode()->GetPositionInWorld(NWU);
      auto lx = p0pL[0];
      auto ly = p0pL[1];
      auto lz = p0pL[2];

      auto chord_length = p0pL.norm();
      auto v = m_u.cross(p0pL / chord_length).cross(m_u);

      double lambda = 0;
      if (this->m_unstrainedLength <= chord_length) {
        lambda = 0.2;
      } else if ((m_u.cross(p0pL)).norm() < 1e-4) {
        lambda = 1e6;
      } else {
        lambda = sqrt(3. * (this->m_unstrainedLength * this->m_unstrainedLength - lz * lz) / (lx * lx + ly * ly));
      }

      auto fu = -0.5 * m_q * (lz / tanh(lambda) - this->m_unstrainedLength);
      auto fv = 0.5 * m_q * sqrt(lx * lx + ly * ly) / lambda;

      m_t0 = fu * m_u + fv * v;
    }

    template<typename OffshoreSystemType>
    Force FrCatenaryLine<OffshoreSystemType>::GetTension(double s, FRAME_CONVENTION fc) const {
      Force tension = m_t0 - c_qvec * s;
      if (IsNED(fc)) { internal::SwapFrameConvention(tension); }
      return tension;
    }

    template<typename OffshoreSystemType>
    std::shared_ptr<FrCatenaryForce<OffshoreSystemType>> FrCatenaryLine<OffshoreSystemType>::GetStartingForce() {
      return m_startingForce;
    }

    template<typename OffshoreSystemType>
    std::shared_ptr<FrCatenaryForce<OffshoreSystemType>> FrCatenaryLine<OffshoreSystemType>::GetEndingForce() {
      return m_endingForce;
    }

    template<typename OffshoreSystemType>
    Force FrCatenaryLine<OffshoreSystemType>::GetStartingNodeTension(FRAME_CONVENTION fc) const {
      return GetTension(0., fc);
    }

    template<typename OffshoreSystemType>
    Force FrCatenaryLine<OffshoreSystemType>::GetEndingNodeTension(FRAME_CONVENTION fc) const {
      return -GetTension(GetUnstrainedLength(), fc);
    }

    template<typename OffshoreSystemType>
    double FrCatenaryLine<OffshoreSystemType>::_rho(double s) const {
      auto t0_qS = GetTension(s, NWU);
      return t0_qS.norm() - m_u.dot(t0_qS);
    }

    template<typename OffshoreSystemType>
    double FrCatenaryLine<OffshoreSystemType>::GetUnstrainedLength() const {
      return this->m_unstrainedLength;
    }

    template<typename OffshoreSystemType>
    Position FrCatenaryLine<OffshoreSystemType>::GetUnstrainedChord(double s, FRAME_CONVENTION fc) const {

      Position pc = -(m_u / m_q) * ((GetTension(s, NWU)).norm() - m_t0.norm());
      auto rho_0 = _rho(0.);  // TODO: calculer directement

      if (rho_0 > 0.) {
        auto rho_s = _rho(s);  // TODO: calculer directement
        if (rho_s > 0.) {
          pc += (c_Umat * m_t0 / m_q) * log(rho_s / rho_0);
        }
      }
      if (IsNED(fc)) { internal::SwapFrameConvention(pc); }
      return pc;

    }

    template<typename OffshoreSystemType>
    Position FrCatenaryLine<OffshoreSystemType>::GetElasticIncrement(double s, FRAME_CONVENTION fc) const {
      Position Inc(0., 0., 0.);
      if (m_elastic) { Inc = m_q * s * (m_t0 / m_q - 0.5 * m_u * s) / this->m_properties->GetEA(); }
      if (IsNED(fc)) { internal::SwapFrameConvention(Inc); }
      return Inc;
    }

    template<typename OffshoreSystemType>
    Position FrCatenaryLine<OffshoreSystemType>::GetNodePositionInWorld(double s, FRAME_CONVENTION fc) const {

      Position pos;
      pos += this->GetStartingNode()->GetPositionInWorld(fc);
      pos += GetUnstrainedChord(s, fc);
      pos += GetElasticIncrement(s, fc);
      return pos;

    }

    template<typename OffshoreSystemType>
    Position FrCatenaryLine<OffshoreSystemType>::get_residual(FRAME_CONVENTION fc) const {
      return GetNodePositionInWorld(this->m_unstrainedLength, fc) - this->GetEndingNode()->GetPositionInWorld(fc);
    }

    template<typename OffshoreSystemType>
    mathutils::Matrix33<double> FrCatenaryLine<OffshoreSystemType>::analytical_jacobian() const {
      auto t0n = m_t0.norm();

      auto tL = m_t0 - c_qvec * this->m_unstrainedLength;
      auto tLn = tL.norm();

      auto rho_0 = _rho(0.);  // TODO: calculer directement
      double ln_q = 0.;
      double rho_L = 0.;
      if (rho_0 > 0.) {
        rho_L = _rho(this->m_unstrainedLength);  // TODO: calculer directement
        ln_q = log(rho_L / rho_0) / m_q;
      }

      double L_EA = 0.;
      if (m_elastic) L_EA = this->m_unstrainedLength / this->m_properties->GetEA();

      mathutils::Vector3d<double> Ui;
      double Uit0;
      double jac_ij;
      double diff_ln;
      auto jac = mathutils::Matrix33<double>();
      for (uint i = 0; i < 3; ++i) {
        Ui.x() = c_Umat(i, 0);
        Ui.y() = c_Umat(i, 1);
        Ui.z() = c_Umat(i, 2);

        Uit0 = Ui.dot(m_t0) / m_q;

        for (uint j = i; j < 3; ++j) {

          jac_ij = -(tL[j] / tLn - m_t0[j] / t0n) * m_u[i] / m_q;

          if (rho_0 > 0.) {
            jac_ij += c_Umat.at(i, j) * ln_q;
            diff_ln = (tL[j] / tLn - m_u[j]) / rho_L - (m_t0[j] / t0n - m_u[j]) / rho_0;
            jac_ij += Uit0 * diff_ln;
          }

          if (i == j) {  // elasticity
            jac_ij += L_EA;  // L_EA is null if no elasticity
            jac(i, j) = jac_ij;
          } else {
            jac(i, j) = jac_ij;
            jac(j, i) = jac_ij;
          }
        } // end for j
      } // end for i

      return jac;
    }

    template<typename OffshoreSystemType>
    void FrCatenaryLine<OffshoreSystemType>::solve() {

      auto res = get_residual(NWU);
      auto jac = analytical_jacobian();

      jac.Inverse();
      mathutils::Vector3d<double> delta_t0 = jac * (-res);

      m_t0 += m_relax * delta_t0;

      res = get_residual(NWU);
      double err = res.infNorm();

      unsigned int iter = 1;
      while ((err > m_tolerance) && (iter < m_itermax)) {
        iter++;

        res = get_residual(NWU);
        jac = analytical_jacobian();

        jac.Inverse();
        mathutils::Vector3d<double> delta_t0_temp = jac * (-res);

        while (delta_t0.infNorm() < m_relax * delta_t0_temp.infNorm()) {
          m_relax *= 0.5;
          if (m_relax < Lmin) {
            std::cout << "DAMPING TOO STRONG. NO CATENARY CONVERGENCE." << std::endl;
          }
        }

        delta_t0 = delta_t0_temp;
        m_t0 += m_relax * delta_t0;

        m_relax = std::min(1., m_relax * 2.);

        res = get_residual(NWU);
        err = res.infNorm();
      }  // end while

    }

    template<typename OffshoreSystemType>
    void FrCatenaryLine<OffshoreSystemType>::Initialize() {

      m_q = this->m_properties->GetLinearDensity() -
          this->m_properties->GetSectionArea() * this->GetSystem()->GetEnvironment()->GetFluidDensity(c_fluid);
      m_q *= this->GetSystem()->GetGravityAcceleration();
      c_qvec = m_q * m_u;

      // Initializing U matrix
      c_Umat.SetIdentity();
      c_Umat -= m_u * (m_u.transpose().eval());

      // First guess for the tension
      // FIXME: supprimer ces initialize de node et mettre en place la séparation des SetupInitial des FrPhysicsItems en fonction des Pre, Mid et Post.
      this->m_startingNode->Initialize();
      this->m_endingNode->Initialize();
      guess_tension();
      solve();

      // Building the catenary forces and adding them to bodies
      if (!m_startingForce) {
        m_startingForce = std::make_shared<FrCatenaryForce>(this, LINE_START);
        auto starting_body = this->m_startingNode->GetBody();
        starting_body->AddExternalForce(m_startingForce);
      }

      if (!m_endingForce) {
        m_endingForce = std::make_shared<FrCatenaryForce>(this, LINE_END);
        auto ending_body = this->m_endingNode->GetBody();
        ending_body->AddExternalForce(m_endingForce);
      }

      FrCatenaryAssetOwner::Initialize();
    }

    template<typename OffshoreSystemType>
    void FrCatenaryLine<OffshoreSystemType>::Compute(double time) {

      this->UpdateTime(time);
      UpdateState();

    }

    template<typename OffshoreSystemType>
    void FrCatenaryLine<OffshoreSystemType>::UpdateState() {

      FrCable<OffshoreSystemType>::UpdateState();
      solve();

    }

    template<typename OffshoreSystemType>
    void FrCatenaryLine<OffshoreSystemType>::AddFields() {

      if (this->IsLogged()) {

        // Add the fields to be logged here
        this->m_message->template AddField<double>("time", "s", "Current time of the simulation",
                                    [this]() { return this->m_system->GetTime(); });

        this->m_message->template AddField<double>("StrainedLength", "m", "Strained length of the catenary line",
                                    [this]() { return this->GetStrainedLength(); });

        this->m_message->template AddField<Eigen::Matrix<double, 3, 1>>
            ("StartingNodeTension", "N",
             fmt::format("Starting node tension in world reference frame in {}", this->GetLogFrameConvention()),
             [this]() { return GetStartingNodeTension(this->GetLogFrameConvention()); });

        this->m_message->template AddField<Eigen::Matrix<double, 3, 1>>
            ("EndingNodeTension", "N",
             fmt::format("Ending node tension in world reference frame in {}", this->GetLogFrameConvention()),
             [this]() { return GetEndingNodeTension(this->GetLogFrameConvention()); });

        //TODO : logger la position de la ligne pour un ensemble d'abscisses curvilignes?

      }

    }

    template<typename OffshoreSystemType>
    void FrCatenaryLine<OffshoreSystemType>::StepFinalize() {

      FrAssetOwner::UpdateAsset();

      FrPhysicsItem<OffshoreSystemType>::StepFinalize();

      // Serialize and send the log message
      FrObject<OffshoreSystemType>::StepFinalize();

    }

    template<typename OffshoreSystemType>
    internal::FrPhysicsItemBase<OffshoreSystemType> *
    FrCatenaryLine<OffshoreSystemType>::GetChronoItem_ptr() const { return this->m_chronoPhysicsItem.get(); }

    template<typename OffshoreSystemType>
    std::shared_ptr<FrCatenaryLine<OffshoreSystemType>>
    make_catenary_line(const std::shared_ptr<FrNode<OffshoreSystemType>> &startingNode,
                       const std::shared_ptr<FrNode<OffshoreSystemType>> &endingNode,
                       FrOffshoreSystem<OffshoreSystemType> *system,
                       const std::shared_ptr<FrCableProperties> &properties,
                       bool elastic,
                       double unstrainedLength,
                       FLUID_TYPE fluid) {

      auto CatenaryLine = std::make_shared<FrCatenaryLine>(startingNode, endingNode, properties, elastic,
                                                           unstrainedLength, fluid);
      system->Add(CatenaryLine);
      return CatenaryLine;

    }

}  // end namespace frydom
