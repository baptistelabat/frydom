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


#ifndef FRYDOM_FRCABLE_H
#define FRYDOM_FRCABLE_H


#include "frydom/core/common/FrPhysicsItem.h"
#include "frydom/core/math/FrVector.h"
#include "frydom/core/common/FrConvention.h"


namespace frydom {

    // Forward declaration
    class FrNode;


    /**
     * \class FrCable FrCable.h
     * \brief Abstract base class for cables, superclass of FrCatenaryLine and
     * FrDynamicCable .
     * This means cables are updated between bodies and links.
     * A cable is connected to two nodes : a starting node and an ending node. Nodes are contained by at
     * least one body, and used to connect bodies to other components (cables, links,etc.)
     * \see FrCatenaryLine, FrDynamicCable, FrNode
     *
     */
    class FrCable {

    protected:

        //--------------------------------------------------------------------------------------------------------------
        // time cached values
        double m_time = 0.;                         ///< cached value of the simulation time
        double m_time_step = 0.;                    ///< cached value of the simulation time step

        //--------------------------------------------------------------------------------------------------------------
        // Nodes
        std::shared_ptr<FrNode> m_startNode;       ///< starting node
        std::shared_ptr<FrNode> m_endNode;         ///< ending node

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
        FrCable();

        /// FrCable constructor, using two nodes and cable properties
        /// \param startingNode starting node
        /// \param endingNode ending node
        /// \param cableLength unstretched length
        /// \param youngModulus Young modulus
        /// \param sectionArea section area
        /// \param linearDensity linear density
        FrCable(const std::shared_ptr<FrNode> startingNode,
                 const std::shared_ptr<FrNode> endingNode,
                 double cableLength,
                 double youngModulus,
                 double sectionArea,
                 double linearDensity);

        /// Default destructor
        ~FrCable();

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
        void SetStartingNode(std::shared_ptr<FrNode> startingNode);

        /// Get the starting node of the cable
        /// \return starting node
        std::shared_ptr<FrNode> GetStartingNode() const;

        /// Set the ending node of the cable
        /// \param endingNode ending node
        void SetEndingNode(std::shared_ptr<FrNode> endingNode);

        /// Get the ending node of the cable
        /// \return ending node
        std::shared_ptr<FrNode> GetEndingNode() const;

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

        /// Update internal time and time step for dynamic behaviour of the cable
        /// \param time time of the simulation
        virtual void UpdateTime(const double time);

        /// Update the length of the cable if unrolling speed is defined.
        virtual void UpdateState();

    };

}  // end namespace frydom


#endif //FRYDOM_FRCABLE_H
