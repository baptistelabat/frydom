//
// Created by frongere on 29/05/17.
//

#ifndef FRYDOM_FROFFSHORESYSTEM_H
#define FRYDOM_FROFFSHORESYSTEM_H

//#include <frydom/hydrodynamics/FrHydroDB.h>
//
//#include "frydom/core/FrException.h"
//
//
#include <frydom/utils/FrIrrApp.h>
//#include <frydom/cable/FrCable.h>
//#include <frydom/cable/FrCatway.h>
#include "chrono/physics/ChSystemSMC.h"
#include "chrono/physics/ChSystemNSC.h"
//
//
//#include "chrono/timestepper/ChState.h"
//#include "chrono/core/ChMatrixNM.h"
//#include "chrono/core/ChMatrix33.h"
//
#include "frydom/core/common/FrObject.h"
//#include "frydom/environment/waves/FrFreeSurface.h"
//#include "frydom/environment/current/FrCurrent.h"
//#include "frydom/core/FrBody.h"
//
//#include "frydom/environment/FrEnvironment.h"

#include "frydom/utils/FrIrrApp.h"


// TODO: les objets environnement devront etre mis dans une classe environnement qui encapsule tout l'environnement:
// vent, vagues, courant, fond...


namespace frydom {

    class FrBody;
    class FrEnvironment;
    class FrHydroMapper;
    class FrHydroDB;


    // TODO: voir aussi a deriver de ChSystemSMC pour comparer les 2 ? Avoir une classe de base virtuelle derivant de ChSystem ???
    class FrOffshoreSystem :
            public chrono::ChSystemSMC,
            public std::enable_shared_from_this<FrOffshoreSystem>,
            public FrObject
    {  // TODO: supprimer cette dependance !

    private:

        std::shared_ptr<FrBody> world_body;

        chrono::ChFrame<double> NEDframe;                            ///< Frame that has Z pointing down to have a well defined heading

        std::unique_ptr<FrEnvironment> m_environment;

        int m_nHDB = 0;
        std::vector<std::shared_ptr<FrHydroDB>> m_HDB;
        std::vector<std::shared_ptr<FrHydroMapper>> m_hydroMapper;  // TODO : patch vector hydro map multibody

        int m_NsampleOutput = 1;    ///< number of time sample between two outputs
        int m_NitterOutput = 0;     ///< current iteration between two outputs


    public:
        /// Default constructor
        explicit FrOffshoreSystem(bool use_material_properties = true,
                                  unsigned int max_objects = 16000,
                                  double scene_size = 500);

        /// Default destructor
        ~FrOffshoreSystem() = default;

        /// Copy constructor
        //FrOffshoreSystem(const FrOffshoreSystem& system) {};

        /// Default destructor
        //~FrOffshoreSystem() override {std::cout << "OffshoreSystem deleted" << "\n";};

        FrEnvironment* GetEnvironment() const;

        /// Get NED frame
        chrono::ChFrame<double> GetNEDFrame() const;

        /// Get the world body
        chrono::ChBody* GetWorldBodyPtr() const;

        std::shared_ptr<FrBody> GetWorldBody() const;

        void SetHydroMapper(std::shared_ptr<FrHydroMapper> hydroMapper);

        std::shared_ptr<FrHydroMapper> GetHydroMapper(const int id) const;

        void SetHydroDB(const std::string filename);

        FrHydroDB* GetHydroDB(const int id) const;

        int GetHydroMapNb() const;

        /// Updates all the auxiliary data and children of
        /// bodies, forces, links, given their current state
        /// as well as environment prior to everything.
        void Update(bool update_assets = true) override;

        void StateScatter(const chrono::ChState& x, const chrono::ChStateDelta& v, const double T) override;

        bool Integrate_Y() override;

        void CustomEndOfStep() override;

        void Initialize() override;

        void StepFinalize() override;

        void SetNsampleOutput(const int n) { m_NsampleOutput = n; }

        virtual void IntLoadResidual_Mv(const unsigned int off,
                                        chrono::ChVectorDynamic<>& R,
                                        const chrono::ChVectorDynamic<>& w,
                                        const double c) override;

        virtual void VariablesFbIncrementMq() override;

    };  // class FrOffshoreSystem








    // REFACTORING ---->>>>>>>>>>>>

    class FrOffshoreSystem_;

    /// Base class inheriting from chrono ChSystemSMC for physical system in which contact is modeled using a smooth
    /// (penalty-based) method. This class must not be used by external FRyDoM users.
    /// It is used in composition rule along with the FrOffshoreSystem_ FRyDoM class
    class _FrSystemBaseSMC : public chrono::ChSystemSMC {

    private:
        FrOffshoreSystem_* m_offshoreSystem_;   ///< pointer to the offshore system

    public:
        /// Constructor of the systemBase
        /// \param offshoreSystem pointer to the offshore system
        explicit _FrSystemBaseSMC(FrOffshoreSystem_* offshoreSystem);

        /// Update the state of the systemBase, called from chrono, call the Update of the offshore system
        /// \param update_assets check if the assets are updated
        void Update(bool update_assets = true) override;

        /// This method overrides a ChSystemSMC method, called by chrono, call StepFinalize of the offshore system
        /// at the end of each step
        void CustomEndOfStep() override;

//        void SetupInitial() override;

    };

    /// Base class inheriting from chrono ChSystemNSC for physical system in which contact is modeled using a non-smooth
    /// (complementarity-based) method.. This class must not be used by external FRyDoM users.
    /// It is used in composition rule along with the FrOffshoreSystem_ FRyDoM class
    class _FrSystemBaseNSC : public chrono::ChSystemNSC {
            // TODO
    };




    // Forward declarations
    class FrBody_;
    class FrLink_;
    class FrPhysicsItem_;
    class FrEnvironment_;
    class FrCable_;

    /// Main class for a FRyDoM offshore system. This class is used to represent a multibody physical system,
    /// so it acts also as a database for most items involved in simulations, most noticeably objects of FrBody and FrLink
    /// classes, which are used to represent mechanisms.
    /// Moreover, it also owns some global settings and features, like the environment components (ocean, atmosphere, etc.),
    /// the gravity acceleration, the global time and so on.
    ///This object will be responsible of performing the entire physical simulation (dynamics, kinematics, statics, etc.),
    /// so you need at least one FrOffshoreSystem_ object in your program, in order to perform simulations
    /// (you'll insert rigid bodies and links into it..).
    class FrOffshoreSystem_ : public FrObject {

    public:

        /// enum for contact methods
        enum SYSTEM_TYPE {
            SMOOTH_CONTACT,     ///< system using smooth (penalty) contact
            NONSMOOTH_CONTACT   ///< system using non-smooth (complementarity) contact
        };

        /// enum for timesteppers, i.e., time integrators that can advance a system state.
        enum TIME_STEPPER {
            EULER_IMPLICIT_LINEARIZED,  ///< Performs a step of Euler implicit for II order systems using the
                                        ///< Anitescu/Stewart/Trinkle single-iteration method, that is a bit like an
                                        ///< implicit Euler where one performs only the first Newton corrector iteration.
            EULER_IMPLICIT_PROJECTED,   ///< Performs a step of Euler implicit for II order systems using a semi implicit
                                        ///< Euler without constraint stabilization, followed by a projection.
                                        ///< That is: a speed problem followed by a position problem that keeps constraint
                                        ///< drifting 'closed' by using a projection. If using an underlying CCP
                                        ///< complementarity solver, this is the typical Tasora stabilized timestepper for DVIs.
            EULER_IMPLICIT,             ///< Performs a step of Euler implicit for II order systems.
            TRAPEZOIDAL,                ///< Performs a step of trapezoidal implicit for II order systems.
                                        ///< NOTE this is a modified version of the trapezoidal for DAE: the original
                                        ///< derivation would lead to a scheme that produces oscillatory reactions in
                                        ///< constraints, so this is a modified version that is first order in constraint
                                        ///< reactions. Use damped HHT or damped Newmark for more advanced options.
            TRAPEZOIDAL_LINEARIZED,     ///< Performs a step of trapezoidal implicit linearized for II order systems.
            HHT,                        ///< Implementation of the HHT implicit integrator for II order systems.
                                        ///< This timestepper allows use of an adaptive time-step, as well as optional use
                                        ///< of a modified Newton scheme for the solution of the resulting nonlinear problem.
//            HEUN,
            RUNGEKUTTA45,               ///< Performs a step of a 4th order explicit Runge-Kutta integration scheme.
            EULER_EXPLICIT,             ///< Euler explicit timestepper. This performs the typical
                                        ///< y_new = y+ dy/dt * dt integration with Euler formula.
//            LEAPFROG,
            NEWMARK,                    ///< Performs a step of Newmark constrained implicit for II order DAE systems.
                                        ///< See Negrut et al. 2007.
        };

        /// enum for solvers aimed at solving complementarity problems arising from QP optimization problems.
        enum SOLVER {
            SOR,                ///< An iterative solver based on projective fixed point method, with overrelaxation and
                                ///< immediate variable update as in SOR methods.
            SYMMSOR,            ///< An iterative solver based on symmetric projective fixed point method, with
                                ///< overrelaxation and immediate variable update as in SSOR methods.
            JACOBI,             ///< An iterative solver for VI (VI/CCP/LCP/linear problems,..) based on projective
                                ///< fixed point method, similar to a projected Jacobi method. Note: this method is here
                                ///< mostly for comparison and tests: we suggest you to use the more efficient ChSolverSOR
                                ///< - similar, but faster & converges better.
//            SOR_MULTITHREAD,
//            PMINRES,
            BARZILAIBORWEIN,    ///< An iterative solver based on modified Krylov iteration of spectral projected
                                ///< gradients with Barzilai-Borwein.
            PCG,                ///< An iterative solver based on modified Krylov iteration of projected conjugate gradient.
            APGD,               ///< An iterative solver based on Nesterov's Projected Gradient Descent.
            MINRES,             ///< An iterative solver based on modified Krylov iteration of MINRES type alternated
                                ///< with gradient projection (active set).
            SOLVER_SMC,         ///< A solver for problems arising in SMooth Contact (SMC) i.e. penalty formulations.
        };

        /// enum for statics methods, which solve the position of static equilibrium (and the reactions).
        enum STATICS_METHOD {
            LINEAR,     ///< This is a one-step only approach that solves the linear equilibrium.
                        ///< To be used mostly for FEM problems with small deformations.
            NONLINEAR,  ///< This tries to solve the equilibrium for the nonlinear problem (large displacements).
                        ///< The larger nsteps, the more the CPU time but the less likely the divergence.
            RELAXATION  ///< Since a truncated iterative method is used, you may need to call this method multiple times
                        ///< in case of large nonlinearities before coming to the precise static solution.
        };

        /// enum for smooth contact models (SMC)
        enum CONTACT_MODEL {
            HOOKE,      ///< linear Hookean model
            HERTZ,      ///< nonlinear Hertzian model
            COULOMB     ///< basic tangential force definition for non-granular bodies
        };

        /// enum for adhesion models (SMC)
        enum ADHESION_MODEL {
            CONSTANT,   ///< constant adhesion force
            DMT         ///< Derjagin-Muller-Toropov model.
        };

        /// enum for tangential displacement models (SMC)
        enum TANGENTIAL_DISP_MODEL {
            NONE,       ///< no tangential force
            ONE_STEP,   ///< use only current relative tangential velocity
            MULTI_STEP  ///< use contact history (from contact initiation)
        };



    private:

        std::unique_ptr<chrono::ChSystem> m_chronoSystem;   ///< The real Chrono system (may be SMC or NSC)

        std::shared_ptr<FrBody_> m_worldBody;               ///< A fixed body that span the world and where things may be attached

        std::unique_ptr<FrEnvironment_> m_environment;      ///< The offshore environment

        SYSTEM_TYPE     m_systemType;                       ///< type of contact method
        TIME_STEPPER    m_timeStepper;                      ///< timesteppers, i.e., time integrators that can advance a
                                                            ///< system state.
        SOLVER          m_solverType;                       ///< solver aimed at solving complementarity problems
                                                            ///< arising from QP optimization problems.

        int m_nbStepStatics = 10;                           ///< maximum number of iterative steps to find the static
                                                            ///< equilibrium, with the nonlinear and relaxation methods
        STATICS_METHOD  m_staticsMethod;                    ///< method to find the static equilibrium

        using BodyContainer = std::vector<std::shared_ptr<FrBody_>>;
        using LinkContainer = std::vector<std::shared_ptr<FrLink_>>;
//        using OtherPhysicsContainer = std::vector<std::shared_ptr<FrOtherPhysics_>>;

        using BodyIter          = BodyContainer::iterator; // TODO : bouger les iterateurs proche des methodes d'iteration...
        using ConstBodyIter     = BodyContainer::const_iterator;

        using LinkIter      = LinkContainer::iterator;
        using ConstLinkIter = LinkContainer::const_iterator;

//        using OtherPhysicsIter = OtherPhysicsContainer::iterator;
//        using ConstOtherPhysicsIter = OtherPhysicsContainer::const_iterator;


        BodyContainer m_bodyList;   ///< list of bodies managed by this offshore system
        LinkContainer m_linkList;   ///< list of links between bodies managed by this offhsore system
//        OtherPhysicsList m_otherPhysicsList;

//        using CatenaryCableContainer = std::vector<std::shared_ptr<FrCatway>>;
//        CatenaryCableContainer m_catenaryCables;


    public:

        /// Default constructor
        /// \param systemType contact method system (SMOOTH_CONTACT/NONSMOOTH_CONTACT)
        /// \param timeStepper time stepper type
        /// \param solver solver type
        explicit
        FrOffshoreSystem_(SYSTEM_TYPE systemType   = SMOOTH_CONTACT,
                          TIME_STEPPER timeStepper = EULER_IMPLICIT_LINEARIZED,
                          SOLVER solver            = MINRES);

        /// Destructor
        ~FrOffshoreSystem_();

        /// Add an item (body, link, etc.) to the offshore sytem
        /// \param item item to be added to the offshore system
        void Add(std::shared_ptr<FrObject> item); // TODO : faire des dynamic_pointer_cast sur les classes pouvant etre ajoutees...

        /// Add a body to the offshore system
        /// \param body body to add
        void AddBody(std::shared_ptr<FrBody_> body);

        /// Add a link between bodies to the offshore system
        /// \param link link to be added
//        void AddLink(std::shared_ptr<FrLink_> link);
        void AddLink(std::shared_ptr<chrono::ChLink> link);  // TODO : remplacer par du frydom full

        /// Add a cable to the offshore system
        /// \param cable to be added
        void AddCable(std::shared_ptr<FrCable_> cable);

//        void AddOtherPhysics(std::shared_ptr<FrOtherPhysics_> otherPhysics);
        /// Add other physics item to the offshore system
        /// \param otherPhysics other physic item to be added
        void AddPhysicsItem(std::shared_ptr<FrPhysicsItem_> otherPhysics);

        /// Get the environment embedded in the offshore system
        /// \return environment embedded in the offshore system
        FrEnvironment_* GetEnvironment() const;

        /// Get the world body embedded in the offshore system
        /// \return world body embedded in the offshore system
        std::shared_ptr<FrBody_> GetWorldBody() const;

        // TODO: voir si les 3 methodes ci-dessous doivent etre privees (pas Initialize)

        // Updates...

        /// Update in priority certain components of the offshore system (Environment)
        void PreUpdate();

        /// Update in last certain components of the offshore system
        void PostUpdate();

        /// Initialize the state of the offshore system and its components (Environment, systemBase)
        void Initialize() override;

        /// Method called at the send of a time step. Logging may be used here
        void StepFinalize() override;


        // Constraint solver

        /// Choose the solver type, to be used for the simultaneous solution of the constraints
        /// in dynamical simulations (as well as in kinematics, statics, etc.)
        ///   - Suggested solver for speed, but lower precision: SOR
        ///   - Suggested solver for higher precision: BARZILAIBORWEIN or APGD
        ///   - For problems that involve a stiffness matrix: MINRES
        ///
        /// *Notes*:
        ///   - Do not use CUSTOM type, as this type is reserved for external solvers
        ///     (set using SetSolver() and/or SetStabSolver())
        ///   - This function is a shortcut, internally equivalent to two calls to
        ///     SetSolver() and SetStabSolve()
        /// \param solver solver type to used in the simulation
        /// \param checkCompat if true, compatibility check between contact method, solver and time stepper is performed.
        void SetSolver(SOLVER solver, bool checkCompat=true);

        /// Turn ON/OFF the warm starting feature of both iterative solvers (the one for speed and the other for
        /// pos.stabilization).
        /// \param useWarm warm starting if true
        void SetSolverWarmStarting(bool useWarm);

        /// Adjust the omega overrelaxation parameter of both iterative solvers (the one for speed and the other for
        /// position stabilization) Note, usually a good omega for Jacobi or GPU solver is 0.2;
        /// for other iter.solvers can be up to 1.0
        /// \param omega overrelaxation parameter of both iterative solvers
        void SetSolverOverrelaxationParam(double omega);

        /// Adjust the 'sharpness lambda' parameter of both iterative solvers (the one for speed and the other for
        /// pos.stabilization). Note, usually a good sharpness value is in 1..0.8 range (the lower, the more it helps
        /// exact convergence, but overall convergence gets also much slower so maybe better to tolerate some error)
        /// \param momega 'sharpness lambda' parameter of both iterative solvers
        void SetSolverSharpnessParam(double momega);

        /// Changes the number of parallel threads (by default is n.of cores).
        /// Note that not all solvers use parallel computation.
        /// If you have a N-core processor, this should be set at least =N for maximum performance.
        /// \param nbThreads number of parallel threads (by default is n.of cores)
        void SetParallelThreadNumber(int nbThreads);

        /// When using an iterative solver (es. SOR) set the maximum number of iterations.
        /// The higher the iteration number, the more precise the simulation (but more CPU time).
        /// \param maxIter maximum number of iterations od the iterative solver
        void SetSolverMaxIterSpeed(int maxIter);

        /// When using an iterative solver (es. SOR) and a timestepping method
        /// requiring post-stabilization (e.g., EULER_IMPLICIT_PROJECTED), set the
        /// the maximum number of stabilization iterations. The higher the iteration
        /// number, the more precise the simulation (but more CPU time).
        /// \param maxIter maximum number of stabilization iterations
        void SetSolverMaxIterStab(int maxIter);

        /// Sets outer iteration limit for assembly constraints. When trying to keep constraints together,
        /// the iterative process is stopped if this max.number of iterations (or tolerance) is reached.
        /// \param maxIter max.number of iterations for assembly constraints
        void SetSolverMaxIterAssembly(int maxIter); // FIXME c'est quoi la diff avec les 2 precedent ?

        /// Sets tolerance (in m) for assembly constraints. When trying to keep constraints together,
        /// the iterative process is stopped if this tolerance (or max.number of iterations ) is reached
        /// \param tol tolerance (in m) for assembly constraints
        void SetSolverGeometricTolerance(double tol);

        /// Sets tolerance for satisfying constraints at the velocity level.
        /// The tolerance specified here is in fact a tolerance at the force level.
        /// this value is multiplied by the value of the current time step and then
        /// used as a stopping criteria for the iterative speed solver.
        /// \param tol tolerance for satisfying constraints at the velocity level
        void SetSolverForceTolerance(double tol);

        /// Turn on this feature to let the system put to sleep the bodies whose
        /// motion has almost come to a rest. This feature will allow faster simulation
        /// of large scenarios for real-time purposes, but it will affect the precision!
        /// This functionality can be turned off selectively for specific ChBodies.
        /// \param useSleeping put bodies to sleep if true
        void SetUseSleepingBodies(bool useSleeping);


        // Contact

        /// Set the contact method (SMOOTH or NONSMOOTH) ie which systemBase to use.
        /// A compatibility check between contact method, solver and time stepper can be performed.
        /// \param type contact method / system type
        /// \param checkCompat if true, compatibility check between contact method, solver and time stepper is performed.
        void SetSystemType(SYSTEM_TYPE type, bool checkCompat=true);

        /// The use of material properties is only for SMOOTH_CONTACT systems
        /// Enable/disable using physical contact material properties.
        /// If true, contact coefficients are estimated from physical material properties.
        /// Otherwise, explicit values of stiffness and damping coefficients are used.
        /// \param use material properties are used if true
        void UseMaterialProperties(bool use);

        /// Contact force model is only for SMOOTH_CONTACT systems
        /// Set the normal contact force model.
        /// \param model normal contact force model
        void SetContactForceModel(CONTACT_MODEL model);

        /// Adhesion force model is only for SMOOTH_CONTACT systems
        /// Set the adhesion force model.
        /// \param model adhesion force model.
        void SetAdhesionForceModel(ADHESION_MODEL model);

        /// Adhesion force model is only for SMOOTH_CONTACT systems
        /// Set the tangential displacement model.
        /// Note that currently MultiStep falls back to OneStep.
        /// \param model tangential displacement model
        void SetTangentialDisplacementModel(TANGENTIAL_DISP_MODEL model);

        /// StiffContact is only for SMOOTH_CONTACT systems
        /// Declare the contact forces as stiff.
        /// If true, this enables calculation of contact force Jacobians.
        /// \param isStiff contact force is stiff, if true
        void SetStiffContact(bool isStiff);

        /// Slip Velocity Threshold is only for SMOOTH_CONTACT systems
        /// Slip velocity threshold.
        /// No tangential contact forces are generated if the magnitude of the tangential
        /// relative velocity is below this value.
        /// \param velocity Slip velocity threshold
        void SetSlipVelocityThreshold(double velocity);

        /// Characteristic Impact Velocity is only for SMOOTH_CONTACT systems
        /// Characteristic impact velocity (Hooke contact force model).
        /// \param velocity Characteristic Impact Velocity
        void SetCharacteristicImpactVelocity(double velocity);

        ///
        /// For elastic collisions, with objects that have nonzero
        /// restitution coefficient: objects will rebounce only if their
        /// relative colliding speed is above this threshold. Default 0.15 m/s.
        /// If this is too low, aliasing problems can happen with small high frequency
        /// rebounces, and settling to static stacking might be more difficult.
        /// \param speed colliding speed threshold, default 0.15 m/s
        void SetMinBounceSpeed(double speed);

        /// For the default stepper, you can limit the speed of exiting from penetration
        /// situations. Usually set a positive value, about 0.1 .. 2 . (as exiting speed, in m/s)
        /// \param speed speed of exiting from penetration situations
        void SetMaxPenetrationRecoverySpeed(double speed);


        // Informations on system problem size

        /// Get the number of position coordinates (considering 7 coords for rigid bodies because of the 4 dof of quaternions).
        /// \return number of position coordinates
        int GetNbPositionCoords() const;

        /// Get the number of velocity coordinates (considering 6 coords for rigid bodies, 3 transl.+3rot.)
        /// \return number of velocity coordinates
        int GetNbVelocityCoords() const;

        /// Get the number of scalar constraints added to the system.
        /// \return number of scalar constraints
        int GetNbConstraintsCoords() const;

        /// Get the number of degrees of freedom of the system.
        /// \return number of degrees of freedom
        int GetNbDOF() const;

        /// Get the number of active bodies (so, excluding those that are sleeping or are fixed to ground).
        /// \return number of active bodies
        int GetNbBodies() const;

        /// Get the number of bodies that are fixed to ground.
        /// \return number of fixed bodies
        int GetNbFixedBodies() const;

        /// Get the number of bodies that are in sleeping mode (excluding fixed bodies).
        /// \return number of sleeping bodies
        int GetNbSleepingBodies() const;

        /// Get the gravity acceleration on the vertical axis
        /// \return gravity acceleration
        double GetGravityAcceleration() const;

        /// Set the gravity acceleration on the vertical axis
        /// \param gravityAcceleration gravity acceleration
        void SetGravityAcceleration(double gravityAcceleration);


        // Statics

        /// Set the maximum number of iterative steps to find the static equilibrium, with the nonlinear and relaxation
        /// methods.
        /// \param nSteps maximum number of iterative steps
        void SetNbStepsStatics(int nSteps);

        /// Solve the static equilibrium with the specified method, default is NONLINEAR.
        /// \param method method to find the static equilibrium
        /// \return true if the static equilibrium has been reached
        // FIXME : il semble que les solveurs retournent toujours true...
        bool SolveStaticEquilibrium(STATICS_METHOD method=NONLINEAR);


        // Time Stepping settings

        /// Set the method for time integration (time stepper type).
        ///   - Suggested for fast dynamics with hard (NSC) contacts: EULER_IMPLICIT_LINEARIZED
        ///   - Suggested for fast dynamics with hard (NSC) contacts and low inter-penetration: EULER_IMPLICIT_PROJECTED
        ///   - Suggested for finite element smooth dynamics: HHT, EULER_IMPLICIT_LINEARIZED
        ///
        /// \param type time stepper method
        /// \param checkCompat check compatibility between contact method, solver and time stepper, if true
        void SetTimeStepper(TIME_STEPPER type, bool checkCompat=true);

        /// Sets the time step used for integration (dynamical simulation).
        /// The lower this value, the more precise the simulation. Usually, values
        /// about 0.01 s are enough for simple simulations. It may be modified automatically
        /// by integration methods, if they support automatic time adaption.
        /// \param timeStep time step used for integration
        void SetTimeStep(double timeStep);

        /// Gets the current time step used for the integration (dynamical simulation).
        /// \return time step used for integration
        double GetTimeStep() const;

        /// Sets the lower limit for time step (only needed if using
        /// integration methods which support time step adaption).
        /// \param minTimeStep lower limit for time step
        void SetMinTimeStep(double minTimeStep);

        /// Sets the upper limit for time step (only needed if using
        /// integration methods which support time step adaption).
        /// \param maxTimeStep upper limit for time step
        void SetMaxTimeStep(double maxTimeStep);

        /// Gets the simulation time
        /// \return simulation time
        double GetTime() const;


        // Dynamics

        /// PERFORM AN INTEGRATION STEP. Advances a single time step. Note that time step can be modified if some
        /// variable-time stepper is used.
        /// \param stepSize size of the time step
        /// \return true if integration step went well
        bool AdvanceOneStep(double stepSize);

        /// Perform the dynamical integration, from current ChTime to the specified nextTime, and terminating the
        /// integration exactly on the nextTime. Therefore, the step of integration may get a little increment/decrement
        /// to have the last step ending in nextTime. Note that this function can be used in iterations to provide
        /// results in a evenly spaced frames of time, even if the steps are changing. Also note that if the time step
        /// is higher than the time increment requested to reach nextTime, the step is lowered.
        /// \param nextTime specified end time to reach
        /// \return true if dynamical integration went well
        bool AdvanceTo(double nextTime);

        /// Perform the dynamical integration with no limit specified on simulation time. The simulation will keep
        /// going on until the user makes it stop.
        /// \param frameStep time step
        /// \return false if the dynamical integration fails
        bool RunDynamics(double frameStep);


        // Adding body

        /// Create a new body, managed by the offshore system. The body characteristics can then be setted using the
        /// shared pointer returned by this method.
        /// \return new body
        std::shared_ptr<FrBody_> NewBody();

        /// Removes all bodies/marker/forces/links/contacts, also resets timers and events.
        void Clear();


        // Visualization

        /// Run the simulation in the viewer environment
        /// \param endTime end time of the simulation
        /// \param dist distance of the camera from the subject, in the viewer environment
        /// \param recordVideo record snapshots if turned true
        void RunInViewer(double endTime, double dist=100, bool recordVideo=false);

        /// Add an optional asset (it can be used to define visualization shapes, or textures, or custom attached
        /// properties that the user can define by creating his class inherited from FrAssetComponent)
        /// \param asset asset to be added to the offshore system
        void AddAsset(std::shared_ptr<chrono::ChAsset> asset);  // TODO : mettre en prive




    private:

        /// Create the world body (fixed body that span the world and where things may be attached) and
        /// add is to the offshore system
        void CreateWorldBody();

        /// Check the compatibility between the contact method, the solver and the time stepper.
        void CheckCompatibility() const;

        /// Check the compatibility between the system contact method and the specified body contact type
        bool CheckBodyContactMethod(std::shared_ptr<FrBody_> body);

        /// Get the systemBase, embedded in the offshore system
        /// \return systemBase
        chrono::ChSystem* GetChronoSystem();



//        friend class FrIrrApp_;




    public:
        // Defining iterators

        BodyIter        body_begin();
        ConstBodyIter   body_begin() const;
        BodyIter        body_end();
        ConstBodyIter   body_end() const;

        LinkIter link_begin();
        ConstLinkIter link_begin() const;
        LinkIter link_end();
        ConstLinkIter link_end() const;

//        OtherPhysicsIter otherphysics_begin();
//        ConstOtherPhysicsIter otherphysics_begin() const;
//        OtherPhysicsIter otherphysics_end();
//        ConstOtherPhysicsIter otherphysics_end() const;

    };


//    namespace internal {
//        void AddBodyToSystem(FrOffshoreSystem* system, std::shared_ptr<FrBody> body) {
//            system->AddBody()
//        }
//    }






} // end namespace frydom

#endif //FRYDOM_FROFFSHORESYSTEM_H
