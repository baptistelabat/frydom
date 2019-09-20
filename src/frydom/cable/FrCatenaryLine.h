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

#include "frydom/asset/FrCatenaryLineAsset.h"


// TODO: prevoir une discretisation automatique pour laquelle on precise la taille cible d'un element
// Servira dans la visu et pour l'application de forces comme Morrison.


namespace frydom {


    // Forward declarations:
    template <typename OffshoreSystemType>
    class FrCatenaryForce;

    template <typename OffshoreSystemType>
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
    template <typename OffshoreSystemType>
    class FrCatenaryLine : public FrCable<OffshoreSystemType>, public FrPrePhysicsItem<OffshoreSystemType>, public FrCatenaryAssetOwner {

    public:

        enum LINE_SIDE {
            LINE_START,
            LINE_END
        };

    private:

        //--------------------------------------------------------------------------------------------------------------
        // Catenary line properties
        bool m_elastic = true;              ///< Is the catenary line elastic
        mathutils::Vector3d<double> m_t0;   ///< Tension vector at the starting node
        double m_q;                         ///< Uniform distributed load, in N/m : (linear density + hydrostatic)*g
        Direction m_u = {0.,0.,-1.};        ///< Uniform distributed load direction
        //--------------------------------------------------------------------------------------------------------------

        //--------------------------------------------------------------------------------------------------------------
        // Cached values
        FLUID_TYPE c_fluid;                 ///< cached value of the fluid type in which the catenary line is mostly in.
        mathutils::Vector3d<double> c_qvec; ///< cached value of the uniform distributed load : qvec = u.q
        mathutils::Matrix33<double> c_Umat; ///< cached value of the jacobian matrix
        //--------------------------------------------------------------------------------------------------------------

        //--------------------------------------------------------------------------------------------------------------
        // Data for Newton-Raphson solver
        //TODO: Complete the missing doc (FR)
        const double Lmin      = 1e-10;     ///<
        double m_tolerance     = 1e-6;      ///<
        unsigned int m_itermax = 100;       ///<
        double m_relax         = 0.1;       ///<
        //--------------------------------------------------------------------------------------------------------------

        //--------------------------------------------------------------------------------------------------------------
        // Forces to apply to bodies
        std::shared_ptr<FrCatenaryForce<OffshoreSystemType>> m_startingForce;   ///< Force applied by the catenary line to the body at the
                                                            ///< starting node
        std::shared_ptr<FrCatenaryForce<OffshoreSystemType>> m_endingForce;     ///< Force applied by the catenary line to the body at the
                                                            ///< ending node
        //--------------------------------------------------------------------------------------------------------------


    public:

        /// Catenary line constructor, using two nodes and catenary line properties
        /// \param startingNode starting node of the catenary line
        /// \param endingNode ending node of the catenary line
        /// \param properties cable properties
        /// \param elastic true if the catenary line is elastic (remember only an elastic line can be strained !)
        /// \param unstrainedLength Unstrained length of the catenary line
        /// \param fluid fluid type in which the catenary line is mostly in
        FrCatenaryLine(const std::shared_ptr<FrNode<OffshoreSystemType>>& startingNode,
                       const std::shared_ptr<FrNode<OffshoreSystemType>>& endingNode,
                       const std::shared_ptr<FrCableProperties>& properties,
                       bool elastic,
                       double unstrainedLength,
                       FLUID_TYPE fluid
        );

        /// Get the type name of this object
        /// \return type name of this object
        std::string GetTypeName() const override { return "CatenaryLine"; }

    public:

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

        //--------------------------------------------------------------------------------------------------------------
        // TODO: avoir une methode pour detacher d'un noeud ou d'un corps. Dans ce cas, un nouveau noeud fixe est cree a
        // la position courante de la ligne.

        //--------------------------------------------------------------------------------------------------------------
        // Force accessors
        /// Get the starting force of the line
        /// \return the starting force of the line
        std::shared_ptr<FrCatenaryForce<OffshoreSystemType>> GetStartingForce();

        /// Get the ending force of the line
        /// \return the ending force of the line
        std::shared_ptr<FrCatenaryForce<OffshoreSystemType>> GetEndingForce();

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

        //--------------------------------------------------------------------------------------------------------------
        // Positions accessors
        /// Get the line position at lagrangian coordinate s
        /// \param s lagrangian coordinate
        /// \param fc frame convention (NED/NWU)
        /// \return line position
        Position GetNodePositionInWorld(double s, FRAME_CONVENTION fc) const override;

        double GetUnstrainedLength() const override;

        /// Get the current chord at lagrangian coordinate s
        /// This is the position of the line if there is no elasticity.
        /// This is given by the catenary equation
        /// \param s lagrangian coordinate
        /// \param fc frame convention (NED/NWU)
        /// \return current unstrained chord
        Position GetUnstrainedChord(double s, FRAME_CONVENTION fc) const;

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
        // solving methods
        /// Solve the nonlinear catenary equation for line tension using a Relaxed Newton-Raphson solver
        void solve();

        /// Guess the line tension from line boundary positions
        /// Used to initialize the Newton-Raphson algorithm.
        /// \see FrCatenaryLine_::solve()
        void guess_tension();

        //--------------------------------------------------------------------------------------------------------------
        // Initialize - Update - Finalize methods
        /// Catenary line initialization method
        void Initialize() override;

        /// Initialize the log
        void AddFields() override;

        /// Update the length of the cable if unrolling speed is defined.
        void UpdateState() override;

        /// Method called at the send of a time step. Logging may be used here
        void StepFinalize() override;

        //--------------------------------------------------------------------------------------------------------------

    protected:

        /// Get the pointer to the chrono related physics item
        /// \return Chrono related physics item
        internal::FrPhysicsItemBase<OffshoreSystemType>* GetChronoItem_ptr() const override;


    private :

        /// Catenary line update method
        /// \param time time of the simulation
        void Compute(double time) override;

        /// Cached function to compute ||t(s)|| - u.t(s)
        /// \param s lagrangian coordinate
        /// \return rho function value
        double _rho(double s) const;

        /// Compute the jacobian matrix with respect to tension using its analytical expression
        /// \return jacobian matrix
        mathutils::Matrix33<double> analytical_jacobian() const;


//        friend void FrCatenaryLineAsset::Initialize();

    };

    template <typename OffshoreSystemType>
    std::shared_ptr<FrCatenaryLine<OffshoreSystemType>>
    make_catenary_line(const std::shared_ptr<FrNode<OffshoreSystemType>>& startingNode,
                       const std::shared_ptr<FrNode<OffshoreSystemType>>& endingNode,
                       FrOffshoreSystem<OffshoreSystemType>* system,
                       const std::shared_ptr<FrCableProperties>& properties,
                       bool elastic,
                       double unstrainedLength,
                       FLUID_TYPE fluid);

}// end namespace frydom

#endif //FRYDOM_FRCATENARYLINE_H
