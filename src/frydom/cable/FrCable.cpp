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


#include "FrCable.h"


namespace frydom {

    FrCable::FrCable() = default;

    FrCable::~FrCable() = default;

    FrCable::FrCable(const std::shared_ptr<FrNode> startingNode, const std::shared_ptr<FrNode> endingNode,
                       double cableLength, double youngModulus, double sectionArea,
                       double linearDensity)
            : m_startNode(startingNode),
              m_endNode(endingNode),
              m_cableLength(cableLength),
              m_unrollingSpeed(0.),
              m_youngModulus(youngModulus),
              m_sectionArea(sectionArea),
              m_linearDensity(linearDensity){}

    void FrCable::SetYoungModulus(double E) {
        m_youngModulus = E;
    }

    double FrCable::GetYoungModulus() const {
        return m_youngModulus;
    }

    void FrCable::SetSectionArea(double A) {
        m_sectionArea = A;
    }

    double FrCable::GetSectionArea() const {
        return m_sectionArea;
    }

    void FrCable::SetUnstretchedLength(double L) {
        m_cableLength = L;
    }

    double FrCable::GetUnstretchedLength() const {
        return m_cableLength;
    }

    void FrCable::SetDiameter(double d) {
        m_sectionArea = M_PI * pow(d*0.5, 2);
    }

    double FrCable::GetDiameter() const {
        return sqrt(4. * m_sectionArea / M_PI);
    }

    double FrCable::GetEA() const {
        return m_youngModulus * m_sectionArea;
    }

    void FrCable::SetLinearDensity(double lambda) {
        m_linearDensity = lambda;
    }

    double FrCable::GetLinearDensity() const {
        return m_linearDensity;
    }

    void FrCable::SetDensity(double rho) {
        m_linearDensity = rho * m_sectionArea;
    }

    double FrCable::GetDensity() const {
        return m_linearDensity / m_sectionArea;
    }

    void FrCable::SetStartingNode(std::shared_ptr<FrNode> startingNode) {
        // TODO: permettre de re-attacher le cable a un autre noeud si elle etait deja attachee a un noeud
        m_startNode = startingNode;
    }

    std::shared_ptr<FrNode> FrCable::GetStartingNode() const {
        return m_startNode;
    }

    void FrCable::SetEndingNode(std::shared_ptr<FrNode> endingNode) {
        // TODO: permettre de re-attacher le cable a un autre noeud si elle etait deja attachee a un noeud
        m_endNode = endingNode;
    }

    std::shared_ptr<FrNode> FrCable::GetEndingNode() const {
        return m_endNode;
    }

    void FrCable::SetBreakingTension(double tension) {
        m_breakingTension = tension;
    }

    double FrCable::GetBreakingTension() const {
        return m_breakingTension;
    }

    void FrCable::SetUnrollingSpeed(double unrollingSpeed) {
        m_unrollingSpeed = unrollingSpeed;
    }

    double FrCable::GetUnrollingSpeed() const {
        return m_unrollingSpeed;
    }

    void FrCable::UpdateTime(const double time) {
        m_time_step = time - m_time;
        m_time = time;
    }

    void FrCable::UpdateState() {
        if (std::abs(m_unrollingSpeed) > DBL_EPSILON and std::abs(m_time_step) > DBL_EPSILON) {
            m_cableLength += m_unrollingSpeed * m_time_step;
        }
    }

}  // end namespace frydom
