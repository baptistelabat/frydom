//
// Created by frongere on 29/05/17.
//

#ifndef FRYDOM_FROFFSHORESYSTEM_H
#define FRYDOM_FROFFSHORESYSTEM_H

//#include <frydom/hydrodynamics/FrHydroDB.h>
//
////#include "frydom/core/FrException.h"
//
//
#include <frydom/utils/FrIrrApp.h>
#include <frydom/cable/FrCable.h>
#include <frydom/cable/FrCatway.h>
#include "chrono/physics/ChSystemSMC.h"
#include "chrono/physics/ChSystemNSC.h"
//
//
//#include "chrono/timestepper/ChState.h"
//#include "chrono/core/ChMatrixNM.h"
//#include "chrono/core/ChMatrix33.h"
//
#include "FrObject.h"
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

        virtual void IntLoadResidual_Mv(const unsigned int off,
                                        chrono::ChVectorDynamic<>& R,
                                        const chrono::ChVectorDynamic<>& w,
                                        const double c) override;

        virtual void VariablesFbIncrementMq() override;

    };  // class FrOffshoreSystem








    // REFACTORING ---->>>>>>>>>>>>

    class FrOffshoreSystem_;

    class _FrSystemBaseSMC : public chrono::ChSystemSMC {

    private:
        FrOffshoreSystem_* m_offshoreSystem_;

    public:
        _FrSystemBaseSMC(FrOffshoreSystem_* offshoreSystem);

        void Update(bool update_assets = true) override;

    };

    class _FrSystemBaseNSC : public chrono::ChSystemNSC {


    };




    // Forward declarations
    class FrBody_;
    class FrLink_;
    class FrEnvironment_;

    class FrOffshoreSystem_ : public FrObject {

    public:

        enum SYSTEM_TYPE {
            SMOOTH_CONTACT,
            NONSMOOTH_CONTACT
        };

        enum TIME_STEPPER {
            EULER_IMPLICIT_LINEARIZED,
            EULER_IMPLICIT_PROJECTED,
            EULER_IMPLICIT,
            TRAPEZOIDAL,
            TRAPEZOIDAL_LINEARIZED,
            HHT,
//            HEUN,
            RUNGEKUTTA45,
            EULER_EXPLICIT,
//            LEAPFROG,
            NEWMARK,
        };

        enum SOLVER {
            SOR,
            SYMMSOR,
            JACOBI,
//            SOR_MULTITHREAD,
//            PMINRES,
            BARZILAIBORWEIN,
            PCG,
            APGD,
            MINRES,
            SOLVER_SMC,
        };

        enum STATICS_METHOD {
            LINEAR,
            NONLINEAR,
            RELAXATION
        };

        enum CONTACT_MODEL {
            HOOKE,
            HERTZ,
            COULOMB
        };

        enum ADHESION_MODEL {
            CONSTANT,
            DMT
        };

        enum TANGENTIAL_DISP_MODEL {
            NONE,
            ONE_STEP,
            MULTI_STEP
        };



    private:

        std::unique_ptr<chrono::ChSystem> m_chronoSystem; ///< The  real Chrono system (may be SMC or NSC)

        std::shared_ptr<FrBody_> m_worldBody;            ///< A fixed body that span the world and where things may be attached

        std::unique_ptr<FrEnvironment_> m_environment;     ///< The offshore environment

        SYSTEM_TYPE     m_systemType;
        TIME_STEPPER    m_timeStepper;
        SOLVER          m_solverType;

        int m_nbStepStatics = 10;
        STATICS_METHOD  m_staticsMethod;

        using BodyContainer = std::vector<std::shared_ptr<FrBody_>>;
        using LinkContainer = std::vector<std::shared_ptr<FrLink_>>;
//        using OtherPhysicsContainer = std::vector<std::shared_ptr<FrOtherPhysics_>>;

        using BodyIter          = BodyContainer::iterator;
        using ConstBodyIter     = BodyContainer::const_iterator;

        using LinkIter = LinkContainer::iterator;
        using ConstLinkIter = LinkContainer::const_iterator;

//        using OtherPhysicsIter = OtherPhysicsContainer::iterator;
//        using ConstOtherPhysicsIter = OtherPhysicsContainer::const_iterator;


        BodyContainer m_bodyList;
        LinkContainer m_linkList;
//        OtherPhysicsList m_otherPhysicsList;

        using CatenaryCableContainer = std::vector<std::shared_ptr<FrCatway>>;
        CatenaryCableContainer m_catenaryCables;


    public:

        /// Default constructor
        explicit
        FrOffshoreSystem_(SYSTEM_TYPE systemType   = SMOOTH_CONTACT,
                          TIME_STEPPER timeStepper = EULER_IMPLICIT_LINEARIZED,
                          SOLVER solver            = MINRES);

        ~FrOffshoreSystem_();

        void AddBody(std::shared_ptr<FrBody_> body);

        void AddLink(std::shared_ptr<FrLink_> link);

        void AddCable(std::shared_ptr<FrCable_> cable);

//        void AddOtherPhysics(std::shared_ptr<FrOtherPhysics_> otherPhysics);

        FrEnvironment_* GetEnvironment() const;

        std::shared_ptr<FrBody_> GetWorldBody() const;

        // TODO: voir si les 3 methodes ci-dessous doivent etre privees (pas Initialize)

        // Updates...

        void PreUpdate();

        void PostUpdate();



        void Initialize() override;

        void StepFinalize() override;


        // Constraint solver

        void SetSolver(SOLVER solver, bool checkCompat=true);

        void SetSolverWarmStarting(bool useWarm);

        void SetSolverOverrelaxationParam(double omega);

        void SetSolverSharpnessParam(double momega);

        void SetParallelThreadNumber(int nbThreads);

        void SetSolverMaxIterSpeed(int maxIter);

        void SetSolverMaxIterStab(int maxIter);

        void SetSolverMaxIterAssembly(int maxIter); // FIXME c'est quoi la diff avec les 2 precedent ?

        void SetSolverGeometricTolerance(double tol);

        void SetSolverForceTolerance(double tol);

        void SetUseSleepingBodies(bool useSleeping);


        // Contact

        void SetSystemType(SYSTEM_TYPE type, bool checkCompat=true);

        void UseMaterialProperties(bool use);

        void SetContactForceModel(CONTACT_MODEL model);

        void SetAdhesionForceModel(ADHESION_MODEL model);

        void SetTangentialDisplacementModel(TANGENTIAL_DISP_MODEL model);

        void SetStiffContact(bool isStiff);

        void SetSlipVelocityThreshold(double velocity);

        void SetCharacteristicImpactVelocity(double velocity);

        void SetMinBounceSpeed(double speed);

        void SetMaxPenetrationRecoverySpeed(double speed);


        // Informations on system problem size

        int GetNbPositionCoords() const;

        int GetNbVelocityCoords() const;

        int GetNbConstraintsCoords() const;

        int GetNbDOF() const;

        int GetNbBodies() const;

        int GetNbFixedBodies() const;

        int GetNbSleepingBodies() const;

        double GetGravityAcceleration() const;

        void SetGravityAcceleration(double gravityAcceleration);


        // Statics

        void SetNbStepsStatics(int nSteps);

        bool SolveStaticEquilibrium(STATICS_METHOD method=NONLINEAR);


        // Time Stepping settings

        void SetTimeStepper(TIME_STEPPER type, bool checkCompat=true);

        void SetTimeStep(double timeStep);

        double GetTimeStep() const;

        void SetMinTimeStep(double minTimeStep);

        void SetMaxTimeStep(double maxTimeStep);

        double GetTime() const;


        // Dynamics

        bool AdvanceOneStep(double stepSize);

        bool AdvanceTo(double nextTime);

        bool RunDynamics(double frameStep);


        // Adding body

        std::shared_ptr<FrBody_> NewBody();

        void Clear();


        // Visualization

        void RunInViewer(double endTime, double dist=100, bool recordVideo=false);

        void AddAsset(std::shared_ptr<chrono::ChAsset> asset);  // TODO : mettre en prive






    private:

        void CreateWorldBody();

        void CheckCompatibility() const;

        bool CheckBodyContactMethod(std::shared_ptr<FrBody_> body);

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
