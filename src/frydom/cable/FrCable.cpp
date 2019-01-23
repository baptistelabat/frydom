//
// Created by frongere on 10/10/17.
//

#include "FrCable.h"

namespace frydom {

    FrCable_::FrCable_() = default;

    FrCable_::FrCable_(const std::shared_ptr<FrNode_> startingNode, const std::shared_ptr<FrNode_> endingNode,
                       double cableLength, double youngModulus, double sectionArea,
                       double linearDensity)
            : m_startNode(startingNode),
              m_endNode(endingNode),
              m_cableLength(cableLength),
              m_unrollingSpeed(0.),
              m_youngModulus(youngModulus),
              m_sectionArea(sectionArea),
              m_linearDensity(linearDensity) {}

    void FrCable_::SetYoungModulus(double E) { m_youngModulus = E; }

    double FrCable_::GetYoungModulus() const { return m_youngModulus; }

    void FrCable_::SetSectionArea(double A) { m_sectionArea = A; }

    double FrCable_::GetSectionArea() const { return m_sectionArea; }

    void FrCable_::SetCableLength(double L) { m_cableLength = L; }

    double FrCable_::GetCableLength() const { return m_cableLength; }

    void FrCable_::SetDiameter(double d) {
        m_sectionArea = M_PI * pow(d*0.5, 2);
    }

    double FrCable_::GetDiameter() const {
        return sqrt(4. * m_sectionArea / M_PI);
    }

    double FrCable_::GetEA() const {
        return m_youngModulus * m_sectionArea;
    }

    void FrCable_::SetLinearDensity(double lambda) { m_linearDensity = lambda; }

    double FrCable_::GetLinearDensity() const { return m_linearDensity; }

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


    FrCable_::~FrCable_() = default;

}