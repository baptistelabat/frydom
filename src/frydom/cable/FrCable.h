//
// Created by frongere on 10/10/17.
//

#ifndef FRYDOM_FRCABLE_H
#define FRYDOM_FRCABLE_H

#include "frydom/core/FrObject.h"
#include "frydom/core/FrNode.h"
#include "chrono/physics/ChLink.h"

namespace frydom {

    /// Abstract base class for cables
    class FrCable : public FrObject,
                    public chrono::ChLink {

    protected:

        double m_time = 0.;
        double m_time_step = 0.;

        std::shared_ptr<FrNode> m_startingNode;
        std::shared_ptr<FrNode> m_endingNode;

        double m_youngModulus; // FIXME: mettre des valeurs par defaut non verolees !!!
        double m_sectionArea;
        double m_cableLength;
        double m_unrollingSpeed;                ///< linear unrolling speed of the cable in m/s

        double m_linearDensity; // in kg/m

        bool m_initialized = false;

    public:

        FrCable() = default;

        FrCable(const std::shared_ptr<FrNode> startingNode,
                const std::shared_ptr<FrNode> endingNode,
                const double cableLength,
                const double youngModulus,
                const double sectionArea)
                : m_startingNode(startingNode),
                  m_endingNode(endingNode),
                  m_cableLength(cableLength),
                  m_unrollingSpeed(0.),
                  m_youngModulus(youngModulus),
                  m_sectionArea(sectionArea) {}

        void SetYoungModulus(const double E) { m_youngModulus = E; }

        double GetYoungModulus() const { return m_youngModulus; }

        void SetSectionArea(const double A) { m_sectionArea = A; }

        double GetSectionArea() const { return m_sectionArea; }

        void SetCableLength(const double L) { m_cableLength = L; }

        double GetCableLength() const { return m_cableLength; }

        /// Definition of the linear unrolling speed of the cable in m/s
        void SetUnrollingSpeed(const double unrollingSpeed) { m_unrollingSpeed = unrollingSpeed; }

        /// Return the linear unrolling speed of the cable in m/s
        double GetUnrollingSpeed() const { return m_unrollingSpeed; }

        void SetDiameter(const double d) {
            m_sectionArea = M_PI * pow(d*0.5, 2);
        }

        double GetDiameter() const {
            return sqrt(4. * m_sectionArea / M_PI);
        }

        double GetEA() const {
            return m_youngModulus * m_sectionArea;
        }

        void SetLinearDensity(const double lambda) { m_linearDensity = lambda; }

        double GetLinearDensity() const { return m_linearDensity; }

        void SetDensity(const double rho) {
            m_linearDensity = rho * m_sectionArea;
        }

        double GetDensity() const {
            return m_linearDensity / m_sectionArea;
        }

        void SetStartingNode(std::shared_ptr<FrNode> startingNode) {
            // TODO: permettre de re-attacher le cable a un autre noeud si elle etait deja attachee a un noeud
            m_startingNode = startingNode;
        }

        std::shared_ptr<FrNode> GetStartingNode() const {
            return m_startingNode;
        }

        void SetEndingNode(std::shared_ptr<FrNode> endingNode) {
            // TODO: permettre de re-attacher le cable a un autre noeud si elle etait deja attachee a un noeud
            m_endingNode = endingNode;
        }

        std::shared_ptr<FrNode> GetEndingNode() const {
            return m_endingNode;
        }

        virtual chrono::ChVector<double> GetTension(const double s) const = 0;

        virtual chrono::ChVector<double> GetAbsPosition(const double s) const = 0;

        virtual void Initialize() override {};

        virtual void StepFinalize() override {}

        /// Update time and state of the cable
        virtual void Update(const double time, bool update_asserts = true) override {
            UpdateTime(time);
            UpdateState();
        }
//
        /// Update internal time and time step for dynamic behaviour of the cable
        virtual void UpdateTime(const double time) override {
            m_time_step = time - m_time;
            m_time = time;
        };
//
        /// Update the length of the cable if unrolling speed is defined.
        virtual void UpdateState() {
            if (std::abs(m_unrollingSpeed) > DBL_EPSILON and std::abs(m_time_step) > DBL_EPSILON) {
                m_cableLength += m_unrollingSpeed * m_time_step;
            }
        }

    };

}  // end namespace frydom


#endif //FRYDOM_FRCABLE_H
