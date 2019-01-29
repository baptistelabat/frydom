//
// Created by frongere on 10/10/17.
//

#ifndef FRYDOM_FRCABLE_H
#define FRYDOM_FRCABLE_H

#include "frydom/core/FrObject.h"
#include "frydom/core/FrNode.h"
//#include "chrono/physics/ChLink.h"

#include "frydom/core/FrPhysicsItem.h"
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










    //  REFACTORING ------------------>>>>>>>>>>>>>>>>>>>>>>


    /**
     * \class FrCable_ FrCable.h
     * \brief Abstract base class for cables, subclass of FrMidPhysicsItem_, superclass of FrCatenaryLine_ and
     * FrDynamicCable_ .
     * This means cables are updated between bodies and links.
     * A cable is connected to two nodes : a starting node and an ending node. Nodes are contained by at
     * least one body, and used to connect bodies to other components (cables, links,etc.)
     * \see FrMidPhysicsItem_, FrCatenaryLine_, FrDynamicCable_, FrNode_
     *
     */
    class FrCable_ : public FrMidPhysicsItem_ {

    protected:

        //--------------------------------------------------------------------------------------------------------------
        // time cached values
        double m_time = 0.;                         ///< cached value of the simulation time
        double m_time_step = 0.;                    ///< cached value of the simulation time step

        //--------------------------------------------------------------------------------------------------------------
        // Nodes
        std::shared_ptr<FrNode_> m_startNode;       ///< starting node
        std::shared_ptr<FrNode_> m_endNode;         ///< ending node

        //--------------------------------------------------------------------------------------------------------------
        // Cable properties
        // FIXME: mettre des valeurs par defaut non verolees !!!
        double m_youngModulus = 3.1416E10;          ///< Yound modulus of the cable in Pa
        double m_sectionArea = 0.05;                ///< Section area of the cable in m²
        double m_cableLength = 100;                 ///< Unstretched length of the cable in m
        double m_unrollingSpeed = 0;                ///< linear unrolling speed of the cable in m/s
        double m_linearDensity = 616.538;           ///< Linear density of the cable in kg/m
        double m_breakingTension = 0;               ///< breaking tension in N (for visualization purpose for now)

    public:

        //--------------------------------------------------------------------------------------------------------------
        // Constructor - destructor
        /// Default constructor
        FrCable_();

        /// FrCable_ constructor, using two nodes and cable properties
        /// \param startingNode starting node
        /// \param endingNode ending node
        /// \param cableLength unstretched length
        /// \param youngModulus Young modulus
        /// \param sectionArea section area
        /// \param linearDensity linear density
        FrCable_(const std::shared_ptr<FrNode_> startingNode,
                 const std::shared_ptr<FrNode_> endingNode,
                 double cableLength,
                 double youngModulus,
                 double sectionArea,
                 double linearDensity);

        /// Default destructor
        ~FrCable_();

        //--------------------------------------------------------------------------------------------------------------
        // cable properties accessors
        ///Set the Young modulus of the cable
        /// \param E Young modulus
        void SetYoungModulus(double E);

        /// Get the Young modulus of the cable
        /// \return Young modulus
        double GetYoungModulus() const;

        /// Set the section area of the cable
        /// \param A section area
        void SetSectionArea(double A);

        /// Get the section area of the cable
        /// \return section area
        double GetSectionArea() const;

        /// Set the section area through the diameter of the cable
        /// \param d diameter
        void SetDiameter(double d);

        /// Get the diameter of the cable
        /// \return diameter
        double GetDiameter() const;

        /// Get the product of the Young modulus and the section area
        /// \return product of the Young modulus and the section area
        double GetEA() const;

        /// Set the linear density of the cable
        /// \param lambda linear density
        void SetLinearDensity(double lambda);

        /// Get the linear density of the cable
        /// \return linear density
        double GetLinearDensity() const;

        /// Set the density of the cable (lambda = A.rho)
        /// \param rho denisty
        void SetDensity(double rho);

        /// Get the density of the cable
        /// \return denisty
        double GetDensity() const;

        /// Set the unstretched length of the cable
        /// \param L unstretched length
        void SetUnstretchedLength(double L);

        /// Get the unstretched length of the cable
        /// \return unstretched length
        double GetUnstretchedLength() const;

        /// Set the linear unrolling speed of the cable in m/s
        void SetUnrollingSpeed(double unrollingSpeed);

        /// Get the linear unrolling speed of the cable in m/s
        double GetUnrollingSpeed() const;

        //--------------------------------------------------------------------------------------------------------------
        // Node accessors
        /// Set the starting node of the cable
        /// \param startingNode starting node
        void SetStartingNode(std::shared_ptr<FrNode_> startingNode);

        /// Get the starting node of the cable
        /// \return starting node
        std::shared_ptr<FrNode_> GetStartingNode() const;

        /// Set the ending node of the cable
        /// \param endingNode ending node
        void SetEndingNode(std::shared_ptr<FrNode_> endingNode);

        /// Get the ending node of the cable
        /// \return ending node
        std::shared_ptr<FrNode_> GetEndingNode() const;

        //--------------------------------------------------------------------------------------------------------------
        // pure virtual methods
        /// Get the inside line tension at the lagrangian coordinate s, from the starting node to the ending node
        /// \param s lagrangian coordinate
        /// \param fc frame convention (NED/NWU)
        /// \return inside line tension
        virtual Force GetTension(double s, FRAME_CONVENTION fc) const = 0;

        /// Get the line position at lagrangian coordinate s
        /// \param s lagrangian coordinate
        /// \param fc frame convention (NED/NWU)
        /// \return line position
        virtual Position GetAbsPosition(double s, FRAME_CONVENTION fc) const = 0;

        /// Get the stretched length of the cable
        /// \return stretched length
        virtual double GetStretchedLength() const = 0;

        //--------------------------------------------------------------------------------------------------------------
        /// Set the breaking tension of the cable (for visualization purpose only for now)
        /// \param tension breaking tension
        void SetBreakingTension(double tension);

        /// Get the breaking tension of the cable
        /// \return breaking tension
        double GetBreakingTension() const;

    };

}  // end namespace frydom


#endif //FRYDOM_FRCABLE_H
