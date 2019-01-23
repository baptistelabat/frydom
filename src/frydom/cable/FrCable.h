//
// Created by frongere on 10/10/17.
//

#ifndef FRYDOM_FRCABLE_H
#define FRYDOM_FRCABLE_H

#include "frydom/core/common/FrObject.h"
#include "frydom/core/common/FrNode.h"
#include "frydom/core/math/FrVector.h"

#include "frydom/core/FrOffshoreSystem.h"

namespace frydom {

    /// Abstract base class for cables
    class FrCable : public FrObject {

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



    };










    ///  REFACTORING ------------------>>>>>>>>>>>>>>>>>>>>>>



    /// Abstract base class for cables
    class FrCable_ : public FrObject {

    protected:

        FrOffshoreSystem_* m_system;

        double m_time = 0.;
        double m_time_step = 0.;

        std::shared_ptr<FrNode_> m_startNode;
        std::shared_ptr<FrNode_> m_endNode;

        double m_youngModulus; // FIXME: mettre des valeurs par defaut non verolees !!!
        double m_sectionArea;
        double m_cableLength;
        double m_unrollingSpeed;                ///< linear unrolling speed of the cable in m/s

        double m_linearDensity; // in kg/m


    public:

        FrCable_();

        FrCable_(const std::shared_ptr<FrNode_> startingNode,
                 const std::shared_ptr<FrNode_> endingNode,
                 const double cableLength,
                 const double youngModulus,
                 const double sectionArea,
                 const double linearDensity);

        ~FrCable_();

        FrOffshoreSystem_* GetSystem();

        void SetYoungModulus(const double E) { m_youngModulus = E; }

        double GetYoungModulus() const { return m_youngModulus; }

        void SetSectionArea(const double A) { m_sectionArea = A; }

        double GetSectionArea() const { return m_sectionArea; }

        void SetCableLength(const double L) { m_cableLength = L; }

        double GetCableLength() const { return m_cableLength; }

//        /// Definition of the linear unrolling speed of the cable in m/s
//        void SetUnrollingSpeed(const double unrollingSpeed) { m_unrollingSpeed = unrollingSpeed; }
//
//        /// Return the linear unrolling speed of the cable in m/s
//        double GetUnrollingSpeed() const { return m_unrollingSpeed; }

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

        void SetStartingNode(std::shared_ptr<FrNode_> startingNode) {
            // TODO: permettre de re-attacher le cable a un autre noeud si elle etait deja attachee a un noeud
            m_startNode = startingNode;
        }

        std::shared_ptr<FrNode_> GetStartingNode() const {
            return m_startNode;
        }

        void SetEndingNode(std::shared_ptr<FrNode_> endingNode) {
            // TODO: permettre de re-attacher le cable a un autre noeud si elle etait deja attachee a un noeud
            m_endNode = endingNode;
        }

        std::shared_ptr<FrNode_> GetEndingNode() const {
            return m_endNode;
        }

        virtual Force GetTension(const double s) const = 0;

        virtual Position GetAbsPosition(const double s) const = 0;


    private:
        virtual std::shared_ptr<chrono::ChPhysicsItem> GetChronoPhysicsItem() = 0;

        friend void FrOffshoreSystem_::AddCable(std::shared_ptr<FrCable_>);



    };

}  // end namespace frydom


#endif //FRYDOM_FRCABLE_H
