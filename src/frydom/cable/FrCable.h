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


#include "frydom/core/math/FrVector.h"
#include "frydom/core/common/FrConvention.h"
#include "frydom/logging/FrLoggable.h"


namespace frydom {

    // Forward declaration
    class FrNode;

    class FrCableProperties {

     private:

      // Cable properties
      // FIXME: mettre des valeurs par defaut non verolees !!!
      double m_section = 0.05;                    ///< Section area of the cable, in m²
      double m_youngModulus = 3.1416E10;          ///< Young modulus of the cable, in Pa
      double m_linearDensity = 616.538;           ///< Linear density of the cable, in kg/m
      double m_rayleighDamping = 0;               ///< Rayleigh damping of the cable (for dynamic cable only)
//        double m_breakingTension = 0;               ///< breaking tension, in N (for visualization purpose for now)

     public:

      /// Default constructor
      FrCableProperties() = default;

      /// Cable properties constructor from Young modulus, diameter and linear density
      /// \param diameter diameter of the cable, in m
      /// \param linearDensity Linear density of the cable, in kg/m
      /// \param youngModulus Young modulus of the cable, in Pa
      /// \param elastic Is the cable elastic (for catenary lines only)
      FrCableProperties(double diameter, double linearDensity, double youngModulus);

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

      void SetEA(double EA);

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
      /// \param rho density
      void SetDensity(double rho);

      /// Get the density of the cable
      /// \return density
      double GetDensity() const;

    };

    std::shared_ptr<FrCableProperties> make_cable_properties();

    std::shared_ptr<FrCableProperties>
    make_cable_properties(double diameter, double linearDensity, double youngModulus);


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
    class FrCable : public FrLoggable<FrOffshoreSystem> {

     protected:

      //--------------------------------------------------------------------------------------------------------------
      // time cached values
      double m_time = 0.;                         ///< cached value of the simulation time
      double m_time_step = 0.;                    ///< cached value of the simulation time step

      //--------------------------------------------------------------------------------------------------------------
      // Nodes
      std::shared_ptr<FrNode> m_startingNode;       ///< starting node
      std::shared_ptr<FrNode> m_endingNode;         ///< ending node

      //--------------------------------------------------------------------------------------------------------------
      // Cable properties
      std::shared_ptr<FrCableProperties> m_properties;    ///< Cable properties (section, Young modulus, linear density, etc.)

      double m_unstrainedLength = 100;                    ///< Unstrained length of the cable in m
      double m_unrollingSpeed = 0;                        ///< linear unrolling speed of the cable in m/s
//        double m_breakingTension = 0;                       ///< breaking tension in N (for visualization purpose for now)

     public:

      //--------------------------------------------------------------------------------------------------------------
      // Constructor - destructor
      /// Default constructor
      explicit FrCable(const std::string& name);

      /// FrCable constructor, using two nodes
      /// \param startingNode starting node
      /// \param endingNode ending node
      FrCable(const std::string& name,
              const std::shared_ptr<FrNode> &startingNode,
              const std::shared_ptr<FrNode> &endingNode);

      /// FrCable constructor, using two nodes and cable properties
      /// \param startingNode starting node
      /// \param endingNode ending node
      /// \param properties cable properties
      /// \param unstrainedLength unstrained length, in m
      FrCable(const std::string& name,
              const std::shared_ptr<FrNode> &startingNode,
              const std::shared_ptr<FrNode> &endingNode,
              const std::shared_ptr<FrCableProperties> &properties,
              double unstrainedLength);

      /// Default destructor
      ~FrCable();

      //--------------------------------------------------------------------------------------------------------------

      /// Set the cable properties (section, Young modulus, linear density, etc.)
      /// \param prop Cable properties
      void SetCableProperties(const std::shared_ptr<FrCableProperties> prop);

      /// Get the cable properties (section, Young modulus, linear density, etc.)
      /// \return prop Cable properties
      std::shared_ptr<FrCableProperties> GetCableProperties() const;

      /// Set the unstrained length of the cable
      /// \param L unstrained length
      void SetUnstrainedLength(double L);

      /// Get the unstrained length of the cable
      /// \return unstrained length
      double GetUnstrainedLength() const;

      /// Set the linear unrolling speed of the cable in m/s
      void SetUnrollingSpeed(double unrollingSpeed);

      /// Get the linear unrolling speed of the cable in m/s
      double GetUnrollingSpeed() const;

      //--------------------------------------------------------------------------------------------------------------
      // Node accessors
      /// Set the starting node of the cable
      /// \param startingNode starting node
      void SetStartingNode(const std::shared_ptr<FrNode> startingNode);

      /// Get the starting node of the cable
      /// \return starting node
      std::shared_ptr<FrNode> GetStartingNode() const;

      /// Set the ending node of the cable
      /// \param endingNode ending node
      void SetEndingNode(const std::shared_ptr<FrNode> endingNode);

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
      virtual Position GetNodePositionInWorld(double s, FRAME_CONVENTION fc) const = 0;

      /// Get the strained length of the cable
      /// \return strained length
      virtual double GetStrainedLength() const;

//    protected:
//
//        virtual void InitBreakingTension();
//
//    public:
//
//        //--------------------------------------------------------------------------------------------------------------
//        /// Set the breaking tension of the cable (for visualization purpose only for now)
//        /// \param tension breaking tension
//        void SetBreakingTension(double tension);
//
//        /// Get the breaking tension of the cable
//        /// \return breaking tension
//        double GetBreakingTension() const;

      /// Update internal time and time step for dynamic behaviour of the cable
      /// \param time time of the simulation
      virtual void UpdateTime(double time);

      /// Update the length of the cable if unrolling speed is defined.
      virtual void UpdateState();

    };


}  // end namespace frydom


#endif //FRYDOM_FRCABLE_H
