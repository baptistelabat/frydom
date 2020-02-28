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


#ifndef FRYDOM_FRCATENARYLINE_H
#define FRYDOM_FRCATENARYLINE_H


#include "FrCable.h"
#include "frydom/core/common/FrPhysicsItem.h"
#include "frydom/core/math/FrVector.h"
#include "frydom/environment/FrFluidType.h"

#include "frydom/core/force/FrForce.h"

#include "frydom/asset/FrCatenaryLineAsset.h"


// TODO: prevoir une discretisation automatique pour laquelle on precise la taille cible d'un element
// Servira dans la visu et pour l'application de forces comme Morrison.


namespace frydom {

  // Forward declarations:
  class FrCatenaryForce;

  class FrNode;

  /**
   * \class FrCatenaryLine FrCatenaryLine.h
   * \brief Class for catenary line objects, subclass of FrCable and FrMidPhysicsItem
   * The catenary line can be specified elastic or not. However be careful not to strain the line if it has been
   * defined as non elastic. Only an elastic line can be strained !
   * The model for the catenary line is a quasi-static approach, based on uniform distributed load. In water, the
   * uniform load consists of the linear density of the cable and the hydrostatic restoring force per length of cable.
   *
   * Greco, L., "A procedure for the static analysis of cables structures following elastic catenary theory",
   * International Journal of Solids and Structures,pp 1521-1533, 2014
   */
  //TODO: check that the chrono_objects are deleted correctly, when the frydom objects are deleted (assets included)
  class FrCatenaryLine
      : public FrLoggable<FrOffshoreSystem>,
        public FrCable,
        public FrPrePhysicsItem,
        public FrCatenaryAssetOwner {

   public:

    enum LINE_SIDE {
      LINE_START,
      LINE_END
    };

   protected:

    // Catenary line properties
    bool m_elastic = true;              ///< Is the catenary line elastic
    Force m_t0;   ///< Tension vector at the starting node
    double m_q;                         ///< Uniform distributed load, in N/m : (linear density + hydrostatic)*g
    Direction m_u = {0., 0., -1.};        ///< Uniform distributed load direction

    // Cached values
    FLUID_TYPE c_fluid;                 ///< cached value of the fluid type in which the catenary line is mostly in.
    mathutils::Vector3d<double> c_qvec; ///< cached value of the uniform distributed load : qvec = u.q
    mathutils::Matrix33<double> c_Umat; ///< cached value of the jacobian matrix

    // Data for Newton-Raphson solver
    //TODO: Complete the missing doc (FR)
    const double Lmin = 1e-10;
    double m_tolerance = 1e-6;
    unsigned int m_itermax = 100;
    double m_relax = 0.1;

    // Forces to apply to bodies
    std::shared_ptr<FrCatenaryForce> m_startingForce;   ///< Force applied by the catenary line to the body at the
    ///< starting node
    std::shared_ptr<FrCatenaryForce> m_endingForce;     ///< Force applied by the catenary line to the body at the
    ///< ending node

    bool m_is_for_shape_initialization;

   public:

    /// Catenary line constructor, using two nodes and catenary line properties
    /// \param startingNode starting node of the catenary line
    /// \param endingNode ending node of the catenary line
    /// \param properties cable properties
    /// \param elastic true if the catenary line is elastic (remember only an elastic line can be strained !)
    /// \param unstretchedLength Unstrained length of the catenary line
    /// \param fluid fluid type in which the catenary line is mostly in
    FrCatenaryLine(const std::string &name,
                   const std::shared_ptr<FrNode> &startingNode,
                   const std::shared_ptr<FrNode> &endingNode,
                   const std::shared_ptr<FrCableProperties> &properties,
                   bool elastic,
                   double unstretchedLength,
                   FLUID_TYPE fluid  // FIXME : on ne devrait pas specifier le fluide !!! on doit le recuperer de l'environnement...
    );

    /// Catenary line constructor, based on a generic FrCable pointer. It is used by shape initialization functions
    /// To get a catenary shape that fit the data of a given uninitialized more complicated cable.
    /// Internal use only
    explicit FrCatenaryLine(const std::string &name,
                            FrCable *cable,
                            bool elastic,
                            FLUID_TYPE fluid_type);


    /// Get the FrOffshoreSystem
    inline FrOffshoreSystem *GetSystem() const {
      return GetParent();
    }

    //--------------------------------------------------------------------------------------------------------------
    // Accessors related to the embedded Newton-Raphson solver

    /// Set the Newton-Raphson solver tolerance
    /// \param tol solver tolerance
    void SetSolverTolerance(double tol);

    /// Set the Newton-Raphson solver maximum number of iterations
    /// \param maxiter maximum number of iterations
    void SetSolverMaxIter(unsigned int maxiter);

    /// Set the Newton-Raphson initial relaxation factor
    /// \param relax initial relaxation factor
    void SetSolverInitialRelaxFactor(double relax);

    /// Tells the line it is only for shape initialization and not for a real cable usage so that
    /// no force is added to boundary bodies
    void UseForShapeInitialization();

    //--------------------------------------------------------------------------------------------------------------
    // TODO: avoir une methode pour detacher d'un noeud ou d'un corps. Dans ce cas, un nouveau noeud fixe est cree a
    // la position courante de la ligne.

    //--------------------------------------------------------------------------------------------------------------
    // Force accessors
    /// Get the starting force of the line
    /// \return the starting force of the line
    std::shared_ptr<FrCatenaryForce> GetStartingForce();

    /// Get the ending force of the line
    /// \return the ending force of the line
    std::shared_ptr<FrCatenaryForce> GetEndingForce();

    // TODO: accessors pour le champ de force distribue
    /// Get the inside line tension at the lagrangian coordinate s, from the starting node to the ending node
    /// \param s lagrangian coordinate
    /// \param fc frame convention (NED/NWU)
    /// \return inside line tension
    Force GetTension(double s, FRAME_CONVENTION fc) const override;

    /// Returns the cartesian tension at the start of the line.
    /// This tension is applied by the line on its node
    /// \param fc frame convention (NED/NWU)
    /// \return tension applied by the line on the starting node
    Force GetStartingNodeTension(FRAME_CONVENTION fc) const;

    /// Returns the cartesian tension at the end of the line.
    /// This tension is applied by the line on its node
    /// \param fc frame convention (NED/NWU)
    /// \return tension applied by the line on the ending node
    Force GetEndingNodeTension(FRAME_CONVENTION fc) const;

    /// Get the lagrangian coordinate and the position of the lowest point on the line by bisection algorithm
    void GetLowestPoint(Position &position,
                        double &s,
                        FRAME_CONVENTION fc,
                        const double tol,
                        const unsigned int maxIter) const;

    //--------------------------------------------------------------------------------------------------------------
    // Positions accessors
    /// Get the line position at lagrangian coordinate s
    /// \param s lagrangian coordinate
    /// \param fc frame convention (NED/NWU)
    /// \return line position
    Position GetPositionInWorld(double s, FRAME_CONVENTION fc) const override;

    double GetUnstretchedLength() const override;

    /// Get the current chord at lagrangian coordinate s
    /// This is the position of the line if there is no elasticity.
    /// This is given by the catenary equation
    /// \param s lagrangian coordinate
    /// \param fc frame convention (NED/NWU)
    /// \return current unstrained chord
    Position GetUnstretchedChord(double s, FRAME_CONVENTION fc) const;

    /// Get the current elastic increment at lagrangian coordinate s
    /// \param s lagrangian coordinate
    /// \param fc frame convention (NED/NWU)
    /// \return current elastic increment
    Position GetElasticIncrement(double s, FRAME_CONVENTION fc) const;

    /// Get the position residual.
    /// This is the difference between the end line position calculated using catenary equation and the effective
    /// geometrical position (position of the ending node)
    /// \param fc frame convention (NED/NWU)
    /// \return position residual
    Position get_residual(FRAME_CONVENTION fc) const;

    //--------------------------------------------------------------------------------------------------------------
    /// Returns true if the catenary line has seabed interactions
    bool HasSeabedInteraction() const;

    //--------------------------------------------------------------------------------------------------------------
    // solving methods
    /// Solve the nonlinear catenary equation for line tension using a Relaxed Newton-Raphson solver
    virtual void solve();

    /// Guess the line tension from line boundary positions
    /// Used to initialize the Newton-Raphson algorithm.
    /// \see FrCatenaryLine_::solve()
    void guess_tension();

    //--------------------------------------------------------------------------------------------------------------
    // Initialize - Update - Finalize methods
    /// Catenary line initialization method
    void Initialize() override;

//      /// Initialize the log
//      void AddFields() override;

    /// Update the length of the cable if unrolling speed is defined.
    void UpdateState() override;

    /// Method called at the send of a time step. Logging may be used here
    void StepFinalize() override;

    //--------------------------------------------------------------------------------------------------------------

   protected:

    /// Get the pointer to the chrono related physics item
    /// \return Chrono related physics item
    internal::FrPhysicsItemBase *GetChronoItem_ptr() const override;

    void DefineLogMessages() override;

    /// Compute the jacobian matrix with respect to tension using its analytical expression
    /// \return jacobian matrix
    mathutils::Matrix33<double> analytical_jacobian() const;

    /// Cached function to compute ||t(s)|| - u.t(s)
    /// \param s lagrangian coordinate
    /// \return rho function value
    double _rho(double s) const;

   private :

    /// Get the tangent to the line at s
    Direction GetTangent(const double s, FRAME_CONVENTION fc) const;

    /// Catenary line update method
    /// \param time time of the simulation
    void Compute(double time) override;




    friend void FrCatenaryLineAsset::Initialize();

  };

  std::shared_ptr<FrCatenaryLine>
  make_catenary_line(const std::string &name,
                     const std::shared_ptr<FrNode> &startingNode,
                     const std::shared_ptr<FrNode> &endingNode,
                     const std::shared_ptr<FrCableProperties> &properties,
                     bool elastic,
                     double unstretchedLength,
                     FLUID_TYPE fluid);


  /**
* \class FrCatenaryForce FrCatenaryForce.h
* \brief Class for getting the tension from the catenary line, subclass of FrForce.
* This class get the tension computed by the catenary line class, to the body on which the force is applied.
* A differenciation is done on which side of the cable (starting or ending), the force is applied.
* \see FrCatenaryLine_, FrForce
*/
  class FrCatenaryForce : public FrForce {

   private:

    FrCatenaryLine *m_line; ///< The parent line
    FrCatenaryLine::LINE_SIDE m_line_side;   ///< The side of the line where the tension is applied

   public:

    /// FrCatenaryForce constructor, from a catenary line, and the description of the side of this line
    /// \param line catenary line applying a tension
    /// \param side side of the line (starting or ending)
    FrCatenaryForce(const std::string &name,
                    FrBody *body,
                    FrCatenaryLine *line,
                    FrCatenaryLine::LINE_SIDE side);;

    /// Return true if the force is included in the static analysis
    bool IncludedInStaticAnalysis() const override;

   private:

    /// Update the catenary force : get the tension applied by the line on the corresponding node
    /// \param time time of the simulation
    void Compute(double time) override;

  };


}  // end namespace frydom

#endif //FRYDOM_FRCATENARYLINE_H
