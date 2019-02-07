//
// Created by frongere on 28/07/17.
//

#ifndef FRYDOM_FRCATENARYLINE_H
#define FRYDOM_FRCATENARYLINE_H

#include "chrono/physics/ChMarker.h"
#include "chrono/core/ChLinearAlgebra.h"
#include "chrono/core/ChMatrix33.h"
#include "chrono/assets/ChPointPointDrawing.h"
#include "FrCatenaryForce.h"
#include "frydom/core/common/FrNode.h"
#include "FrCable.h"
#include "frydom/asset/FrAssetComponent.h"
#include "frydom/asset/FrAssetBuoy.h"

#include "frydom/asset/FrCatenaryLineAsset_.h"
#include "frydom/environment/FrFluidType.h"


// TODO: prevoir une discretisation automatique pour laquelle on precise la taille cible d'un element
// Servira dans la visu et pour l'application de forces comme Morrison.


namespace frydom {


    class FrCatenaryLine : public FrCable,
                           public chrono::ChLink {

    private:

        bool m_elastic = true;

        chrono::ChVector<double> m_t0;

        double m_q;
        chrono::ChVector<double> m_u;
        chrono::ChVector<double> c_qvec;

        chrono::ChMatrix33<double> c_Umat;

        // Forces to apply to bodies
        std::shared_ptr<FrCatenaryForce> m_startingForce;
        std::shared_ptr<FrCatenaryForce> m_endingForce;

        // Data for Newton-Raphson solver
        const double Lmin            = 1e-10;
        double m_tolerance     = 1e-6;
        unsigned int m_itermax = 100;
        double m_relax           = 0.1;

        //-------------------------------------
        // Asset parameters
        bool m_drawCableElements = true;
        int m_nbDrawnElements = 21;
        std::vector<std::shared_ptr<chrono::ChLineShape>> m_cableElements;
        double m_maxTension = 0;
        double m_breakingTension = 0;
        //-------------------------------------


    public:

        FrCatenaryLine(const std::shared_ptr<FrNode>& startingNode,
                       const std::shared_ptr<FrNode>& endingNode,
                       bool elastic,
                       const double youngModulus,
                       const double sectionArea,
                       const double cableLength,
                       const double q,  // TODO: pour q, lier a la densite du cable !!!
                       const chrono::ChVector<double> u
        );

        // TODO: avoir une methode pour detacher d'un noeud ou d'un corps. Dans ce cas, un nouveau oeud fixe est cree a
        // la position courante de la ligne.

        /// Get the starting force of the line
        std::shared_ptr<FrCatenaryForce> GetStartingForce() { return m_startingForce; }

        /// Get the ending force of the line
        std::shared_ptr<FrCatenaryForce> GetEndingForce() { return m_endingForce; }

        /// Get the current absolute position of the line's starting node
        chrono::ChVector<> GetPosStartingNode() const { return m_startingNode->GetAbsPos(); }

        /// Get the current absolute position of the line's ending node
        chrono::ChVector<> GetPosEndingNode() const { return m_endingNode->GetAbsPos(); }

        /// Guess the line tension from line boundary positions
        /// Used to initialize the Newton-Raphson algorithm (see solve method) TODO: mettre un see a facon doxygen...
        void guess_tension();

        // TODO: accessors pour le champ de force distribue

        chrono::ChVector<double> GetTension(const double s) const;

        /// Returns the cartesian tension at the start of the line.
        /// This tension is applied by the line on its node
        chrono::ChVector<double> getStartingNodeTension() const { return m_t0; }

        /// Returns the cartensian tension at the end of the line.
        /// This tension is applied by the end node's parent body ON the line
        chrono::ChVector<double> GetEndingNodeTension() const;


        // TODO: supprimer a terme cette methode et coder en dur a chaque fois qu'on en a besoin
        double _rho(const double s) const {
            // FIXME: cette fonction calcule le tension en s mais generalement, cette derniere doit etre accessible ailleurs...
            auto t0_qS = GetTension(s);
            return t0_qS.Length() - chrono::Vdot(m_u, t0_qS);
        }

        /// Get the current chord at lagrangian coordinate s
        /// This is the position of the line if there is no elasticity.
        /// This is given by the catenary equation
        chrono::ChVector<double> GetUnstrainedChord(const double s) const;

        /// Get the current elastic increment at lagrangian coordinate s
        chrono::ChVector<double> GetElasticIncrement(const double s) const;

        /// Get the line position at lagrangian coordinate s
        chrono::ChVector<double> GetAbsPosition(const double s) const override;

        /// Get the position residual.
        /// This is the difference between the end line position calculated using catenary equation and the effective
        /// geometrical position (position of the ending node)
        chrono::ChVector<double> get_residual() const;

//        /// Compute the jacobian matrix with respect to tension using finite difference method
//        chrono::ChMatrix33<double> numerical_jacobian() const {
//            auto jac = chrono::ChMatrix33<double>();
//            // TODO
//            return jac;
//        }

        /// Compute the jacobian matrix with respect to tension using its analytical expression
        chrono::ChMatrix33<double> analytical_jacobian() const;

        /// Accessor relative to the embedded Newton-Raphson solver

        void SetSolverTolerance(const double tol) { m_tolerance = tol; }
        void SetSolverMaxIter(unsigned int maxiter) { m_itermax = maxiter; }
        void SetSolverInitialRelaxFactor(const double relax) { m_relax = relax; }

        /// Solve the nonlinear catenary equation for line tension using a Relaxed Newton-Raphson solver
        void solve();

        void Initialize() override {
            solve();
            // Generate assets for the cable
            GenerateAssets();
            InitRangeTensionColor();
        }

        /// Returns the current cable length by line discretization
        double GetCableLength(const double n = 1000) const;

        //void UpdateTime(const double time) {
        //    m_time = time;
        //}

        /// Update time and state of the cable
        virtual void Update(const double time, bool update_assets = true) override {
            UpdateTime(time);
            UpdateState();
            UpdateAsset();
            ChPhysicsItem::Update(time, update_assets);
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
            solve();
        }

        ////////////////////////////////////////////////
        // Asset management methods
        ////////////////////////////////////////////////
        void SetAssetNbElements(int NbElements) {m_nbDrawnElements = NbElements;}
        int GetAssetNbElements(){return m_nbDrawnElements;}

        void SetBreakingTension(double tension) {m_breakingTension = tension;}
        double GetBreakingTension() { return m_breakingTension;}

        void InitRangeTensionColor();

        /// Update the assets (position of the discretized cable for the visualization)
        virtual void UpdateAsset();

        /// Generate the assets (discretized cable for the visualization)
        void GenerateAssets();

        void AddAssetComponent(std::shared_ptr<FrAssetComponent> asset){
            AddAsset(asset->GetShapeAsset());
            AddAsset(asset->GetColorAsset());
        }
    };











    //>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> REFACTO

    // Forward declarations:
    class FrCatenaryLine_;
    class FrCatenaryForce_;


    /**
     * \class FrCatenaryLine_ FrCatenaryLine.h
     * \brief Class for catenary line objects, subclass of FrCable_
     * The catenary line can be specified elastic or not. However be careful not to stretch the line if it has been
     * defined as non elastic. Only an elastic line can be stretched !
     * The model for the catenary line is a quasi-static approach, based on uniform distributed load. In water, the
     * uniform load consists of the linear density of the cable and the hydrostatic restoring force per length of cable.
     *
     * Greco, L., "A procedure for the static analysis of cables structures following elastic catenary theory",
     * International Journal of Solids and Structures,pp 1521-1533, 2014
     */
    //TODO: check that the chrono_objects are deleted correctly, when the frydom objects are deleted (assets included)
    class FrCatenaryLine_ : public FrCable_ {

    private:

        //--------------------------------------------------------------------------------------------------------------
        // Catenary line properties
        bool m_elastic = true;              ///< Is the catenary line elastic
        mathutils::Vector3d<double> m_t0;   ///< Tension vector at the starting node
        double m_q;                         ///< Uniform distributed load weight (linear density + hydrostatic)
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
        std::shared_ptr<FrCatenaryForce_> m_startingForce;  ///< Force applied by the catenary line to the body at the
                                                            ///< starting node
        std::shared_ptr<FrCatenaryForce_> m_endingForce;    ///< Force applied by the catenary line to the body at the
                                                            ///< ending node
        //--------------------------------------------------------------------------------------------------------------

        //--------------------------------------------------------------------------------------------------------------
        // Asset parameters
        bool is_lineAsset = true;                           ///< Is the line asset shown
        unsigned int m_nbDrawnElements = 40;                ///< Numbers of asset elements depicted
        std::unique_ptr<FrCatenaryLineAsset_> m_lineAsset;  ///< Line asset
        //--------------------------------------------------------------------------------------------------------------

    public:

        /// Catenary line constructor, using two nodes and catenary line properties
        /// \param startingNode starting node of the catenary line
        /// \param endingNode ending node of the catenary line
        /// \param elastic true if the catenary line is elastic (remember only an elastic line can be stretched !)
        /// \param youngModulus Young modulus of the catenary line
        /// \param sectionArea Section area of the catenary line
        /// \param unstretchedLength Unstretched length of the catenary line
        /// \param linearDensity Uniformly distributed load of the catenary line
        /// \param fluid fluid type in which the catenary line is mostly in
        FrCatenaryLine_(const std::shared_ptr<FrNode_>& startingNode,
                        const std::shared_ptr<FrNode_>& endingNode,
                        bool elastic,
                        double youngModulus,
                        double sectionArea,
                        double unstretchedLength,
                        double linearDensity,
                        FLUID_TYPE fluid
        );

        //--------------------------------------------------------------------------------------------------------------
        // Asset
        /// Get the catenary line asset, created at the initialization of the catenary line (don't try to get it before initializing the line)
        /// \return catenary line asset
        FrCatenaryLineAsset_* GetLineAsset() const;

        /// Set the number of asset elements depicted
        /// \param n number of asset elements
        void SetNbElements(unsigned int n);;

        /// Get the number of asset elements depicted
        /// \return number of asset elements
        unsigned int GetNbElements();

        //--------------------------------------------------------------------------------------------------------------
        // TODO: avoir une methode pour detacher d'un noeud ou d'un corps. Dans ce cas, un nouveau noeud fixe est cree a
        // la position courante de la ligne.

        //--------------------------------------------------------------------------------------------------------------
        // Force accessors
        /// Get the starting force of the line
        /// \return the starting force of the line
        std::shared_ptr<FrCatenaryForce_> GetStartingForce();

        /// Get the ending force of the line
        /// \return the ending force of the line
        std::shared_ptr<FrCatenaryForce_> GetEndingForce();

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
        Force getStartingNodeTension(FRAME_CONVENTION fc) const;

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
        Position GetAbsPosition(double s, FRAME_CONVENTION fc) const override;

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

        /// Returns the current cable length by line discretization
        /// \return stretched cable length
        double GetStretchedLength() const override;

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
        // Accessor relative to the embedded Newton-Raphson solver
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
        // Initialize - Update - Finalize methods
        /// Catenary line initialization method
        void Initialize() override;

        /// Catenary line update method
        /// \param time time of the simulation
        void Update(double time) override;

        /// Update internal time and time step for dynamic behaviour of the cable
        /// \param time time of the simulation
        // TODO: Transfer it to FrCable?
        void UpdateTime(double time);

        /// Update the length of the cable if unrolling speed is defined.
        virtual void UpdateState();

        /// Method called at the send of a time step. Logging may be used here
        void StepFinalize() override {}

        //--------------------------------------------------------------------------------------------------------------
    private :
        /// Cached function to compute ||t(s)|| - u.t(s)
        /// \param s lagrangian coordinate
        /// \return rho function value
        double _rho(double s) const;

        /// Compute the jacobian matrix with respect to tension using its analytical expression
        /// \return jacobian matrix
        mathutils::Matrix33<double> analytical_jacobian() const;

        friend void FrCatenaryLineAsset_::Initialize();
    };

    std::shared_ptr<FrCatenaryLine_>
    make_catenary_line(const std::shared_ptr<FrNode_> &startingNode, const std::shared_ptr<FrNode_> &endingNode,
                       FrOffshoreSystem_ *system, bool elastic, double youngModulus, double sectionArea,
                       double unstretchedLength, double linearDensity, FLUID_TYPE fluid);

}// end namespace frydom

#endif //FRYDOM_FRCATENARYLINE_H
