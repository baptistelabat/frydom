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

//    void FrCable::SetYoungModulus(const double E) { m_youngModulus = E; }
//
//    double FrCable::GetYoungModulus() const { return m_youngModulus; }
//
//    void FrCable::SetSectionArea(const double A) { m_sectionArea = A; }
//
//    double FrCable::GetSectionArea() const { return m_sectionArea; }
//
//    void FrCable::SetCableLength(const double L) { m_cableLength = L; }
//
//    double FrCable::GetCableLength() const { return m_cableLength; }
//
//    void FrCable::SetUnrollingSpeed(const double unrollingSpeed) { m_unrollingSpeed = unrollingSpeed; }
//
//    double FrCable::GetUnrollingSpeed() const { return m_unrollingSpeed; }
//
//    void FrCable::SetDiameter(const double d) {
//        m_sectionArea = M_PI * std::pow(d*0.5, 2);
//    }
//
//    double FrCable::GetDiameter() const {
//        return std::sqrt(4. * m_sectionArea / M_PI);
//    }
//
//    double FrCable::GetEA() const {
//        return m_youngModulus * m_sectionArea;
//    }
//
//    void FrCable::SetLinearDensity(const double lambda) { m_linearDensity = lambda; }
//
//    double FrCable::GetLinearDensity() const { return m_linearDensity; }
//
//    void FrCable::SetDensity(const double rho) {
//        m_linearDensity = rho * m_sectionArea;
//    }
//
//    double FrCable::GetDensity() const {
//        return m_linearDensity / m_sectionArea;
//    }
//
//    void FrCable::SetStartingNode(std::shared_ptr<FrNode> startingNode) {
//        // TODO: permettre de re-attacher le cable a un autre noeud si elle etait deja attachee a un noeud
//        m_startingNode = startingNode;
//    }
//
//    std::shared_ptr<FrNode> FrCable::GetStartingNode() const {
//        return m_startingNode;
//    }
//
//
//
//
//
//
//
//
//
//    ///////// REFACTO



    FrCable_::FrCable_() = default;

    FrCable_::~FrCable_() = default;

    FrCable_::FrCable_(const std::shared_ptr<FrNode_> startingNode, const std::shared_ptr<FrNode_> endingNode,
                       double cableLength, double youngModulus, double sectionArea,
                       double linearDensity)
            : m_startNode(startingNode),
              m_endNode(endingNode),
              m_cableLength(cableLength),
              m_unrollingSpeed(0.),
              m_youngModulus(youngModulus),
              m_sectionArea(sectionArea),
              m_linearDensity(linearDensity),
              FrMidPhysicsItem_(){}

    void FrCable_::SetYoungModulus(double E) {
        m_youngModulus = E;
    }

    double FrCable_::GetYoungModulus() const {
        return m_youngModulus;
    }

    void FrCable_::SetSectionArea(double A) {
        m_sectionArea = A;
    }

    double FrCable_::GetSectionArea() const {
        return m_sectionArea;
    }

    void FrCable_::SetUnstretchedLength(double L) {
        m_cableLength = L;
    }

    double FrCable_::GetUnstretchedLength() const {
        return m_cableLength;
    }

    void FrCable_::SetDiameter(double d) {
        m_sectionArea = M_PI * pow(d*0.5, 2);
    }

    double FrCable_::GetDiameter() const {
        return sqrt(4. * m_sectionArea / M_PI);
    }

    double FrCable_::GetEA() const {
        return m_youngModulus * m_sectionArea;
    }

    void FrCable_::SetLinearDensity(double lambda) {
        m_linearDensity = lambda;
    }

    double FrCable_::GetLinearDensity() const {
        return m_linearDensity;
    }

    void FrCable_::SetDensity(double rho) {
        m_linearDensity = rho * m_sectionArea;
    }

    double FrCable_::GetDensity() const {
        return m_linearDensity / m_sectionArea;
    }

    void FrCable_::SetStartingNode(std::shared_ptr<FrNode_> startingNode) {
        // TODO: permettre de re-attacher le cable a un autre noeud si elle etait deja attachee a un noeud
        m_startNode = startingNode;
    }

    std::shared_ptr<FrNode_> FrCable_::GetStartingNode() const {
        return m_startNode;
    }

    void FrCable_::SetEndingNode(std::shared_ptr<FrNode_> endingNode) {
        // TODO: permettre de re-attacher le cable a un autre noeud si elle etait deja attachee a un noeud
        m_endNode = endingNode;
    }

    std::shared_ptr<FrNode_> FrCable_::GetEndingNode() const {
        return m_endNode;
    }

    void FrCable_::SetBreakingTension(double tension) {
        m_breakingTension = tension;
    }

    double FrCable_::GetBreakingTension() const {
        return m_breakingTension;
    }

    void FrCable_::SetUnrollingSpeed(double unrollingSpeed) {
        m_unrollingSpeed = unrollingSpeed;
    }

    double FrCable_::GetUnrollingSpeed() const {
        return m_unrollingSpeed;
    }



}
