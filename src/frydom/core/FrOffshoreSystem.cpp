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


#include "FrOffshoreSystem.h"

#include "chrono/utils/ChProfiler.h"

#include "frydom/core/link/links_lib/FrLink.h"
#include "frydom/core/body/FrBody.h"
#include "frydom/environment/FrEnvironment.h"
#include "frydom/utils/FrIrrApp.h"


namespace frydom {

    namespace internal {

        FrSystemBaseSMC::FrSystemBaseSMC(frydom::FrOffshoreSystem *offshoreSystem) :
                chrono::ChSystemSMC(), m_offshoreSystem_(offshoreSystem) {}

        void FrSystemBaseSMC::Update(bool update_assets) {

            CH_PROFILE("Update");

            timer_update.start();  // Timer for profiling

            // Pre updates that are not about multibody dynamics
            m_offshoreSystem_->PreUpdate();

            // Executes the "forUpdate" in all controls of controlslist
            ExecuteControlsForUpdate();

            // Physics item that have to be updated before all
            m_offshoreSystem_->PrePhysicsUpdate(ChTime, update_assets);

            // Bodies updates  // FIXME : appeler les updates directement des objets frydom !
            for (auto &body : bodylist) {
                body->Update(ChTime, update_assets);
            }

            // Physics items that have to be updated between bodies and links
            m_offshoreSystem_->MidPhysicsUpdate(ChTime, update_assets);

            // Links updates  // FIXME : appeler les updates directement des objets frydom !
            for (auto &link : linklist) {
                link->Update(ChTime, update_assets);
            }

            // Physics items that have to be updated after all
            m_offshoreSystem_->PostPhysicsUpdate(ChTime, update_assets);

            // Update all contacts, if any
            contact_container->Update(ChTime, update_assets);

            // Post updates that are not about multibody dynamics
            m_offshoreSystem_->PostUpdate();

            timer_update.stop();

        }

        void FrSystemBaseSMC::CustomEndOfStep() {
            m_offshoreSystem_->StepFinalize();
        }

    }  // end namespace frydom::internal


    /// Default constructor
    /// \param systemType contact method system (SMOOTH_CONTACT/NONSMOOTH_CONTACT)
    /// \param timeStepper time stepper type
    /// \param solver solver type
    FrOffshoreSystem::FrOffshoreSystem(SYSTEM_TYPE systemType, TIME_STEPPER timeStepper, SOLVER solver) {

        SetLogged(true);

        // Creating the chrono System backend. It drives the way contact are modelled
        SetSystemType(systemType, false);

        // Setting the time stepper
        SetTimeStepper(timeStepper, false);

        // Setting the constraints solver
        SetSolver(solver, false);

        // Check compatibility between system contact model, time stepper and constraint solver
        CheckCompatibility();

        // Set different default values
        m_staticsMethod = NONLINEAR;

        // Creating a fixed world body to be able to attach anything to it (anchors...) // TODO: mettre dans une methode privee
        CreateWorldBody();

        // Creating the environment
        m_environment = std::make_unique<FrEnvironment>(this); // FIXME: voir bug dans FrEnvironment pour le reglage du systeme

        // Creating the log manager service
        m_pathManager = std::make_unique<FrPathManager>();

//        m_message = std::make_unique<hermes::Message>();

    }

    FrOffshoreSystem::~FrOffshoreSystem() = default;
    void FrOffshoreSystem::Add(std::shared_ptr<FrObject> newItem) {
        assert(std::dynamic_pointer_cast<FrBody>(newItem) ||
               std::dynamic_pointer_cast<FrLink>(newItem) ||
               std::dynamic_pointer_cast<FrPhysicsItem>(newItem));

        if (auto item = std::dynamic_pointer_cast<FrBody>(newItem)) {
            AddBody(item);
            return;
        }

        if (auto item = std::dynamic_pointer_cast<FrLink>(newItem)) {
            AddLink(item);
            return;
        }

        if (auto item = std::dynamic_pointer_cast<FrPrePhysicsItem>(newItem)) {
            AddPhysicsItem(item);
            return;
        }

        if (auto item = std::dynamic_pointer_cast<FrMidPhysicsItem>(newItem)) {
            AddPhysicsItem(item);
            return;
        }

        if (auto item = std::dynamic_pointer_cast<FrPostPhysicsItem>(newItem)) {
            AddPhysicsItem(item);
            return;
        }

    }

    void FrOffshoreSystem::AddBody(std::shared_ptr<FrBody> body) {

        if (!CheckBodyContactMethod(body)) { // TODO : voir si on set pas d'autorite le mode de contact a celui du systeme plutot que de faire un if...
            body->SetContactMethod(m_systemType);
        }

        m_chronoSystem->AddBody(body->GetChronoBody());  // Authorized because this method is a friend of FrBody
        m_bodyList.push_back(body);

        body->m_system = this;

    }


    void FrOffshoreSystem::AddLink(std::shared_ptr<FrLinkBase> link) {
        m_chronoSystem->AddLink(link->GetChronoLink());
        m_linkList.push_back(link);
    }

    void FrOffshoreSystem::AddPhysicsItem(std::shared_ptr<FrPrePhysicsItem> otherPhysics) {
        m_chronoSystem->AddOtherPhysicsItem(otherPhysics->GetChronoPhysicsItem());
        otherPhysics->m_system = this;
        m_PrePhysicsList.push_back(otherPhysics);
    }

    void FrOffshoreSystem::AddPhysicsItem(std::shared_ptr<FrMidPhysicsItem> otherPhysics) {
        m_chronoSystem->AddOtherPhysicsItem(otherPhysics->GetChronoPhysicsItem());
        otherPhysics->m_system = this;
        m_MidPhysicsList.push_back(otherPhysics);
    }

    void FrOffshoreSystem::AddPhysicsItem(std::shared_ptr<FrPostPhysicsItem> otherPhysics) {
        m_chronoSystem->AddOtherPhysicsItem(otherPhysics->GetChronoPhysicsItem());
        otherPhysics->m_system = this;
        m_PostPhysicsList.push_back(otherPhysics);
    }

    FrEnvironment *FrOffshoreSystem::GetEnvironment() const {
        return m_environment.get();
    }

    std::shared_ptr<FrBody> FrOffshoreSystem::GetWorldBody() const {
        return m_worldBody;
    }

    void FrOffshoreSystem::PreUpdate() {
        // TODO : voir si on ne met pas l'environnement comme un physics Item update en tant que PrePhysicsItem
        m_environment->Update(m_chronoSystem->GetChTime());
    }

    void FrOffshoreSystem::PostUpdate() {
        // TODO
    }

    void FrOffshoreSystem::PrePhysicsUpdate(double time, bool update_assets) {
        for (auto& item : m_PrePhysicsList) {
            item->Update(time);
        }
    }

    void FrOffshoreSystem::MidPhysicsUpdate(double time, bool update_assets) {
        for (auto& item : m_MidPhysicsList) {
            item->Update(time);
        }
    }

    void FrOffshoreSystem::PostPhysicsUpdate(double time, bool update_assets) {
        for (auto& item : m_PostPhysicsList) {
            item->Update(time);
        }
    }

    void FrOffshoreSystem::Initialize() {

        // Initializing environment before bodies
        m_environment->Initialize();

        for (auto& item : m_PrePhysicsList) {
            item->Initialize();
        }

        for (auto& item : m_bodyList){
            item->Initialize();
        }

        for (auto& item : m_MidPhysicsList) {
            item->Initialize();
        }

        for (auto& item : m_linkList) {
            item->Initialize();
        }

        for (auto& item : m_PostPhysicsList) {
            item->Initialize();
        }

        m_chronoSystem->Update();

        // Init the logs
        if (IsLogged()) InitializeLog();

        m_isInitialized = true;

    }

    void FrOffshoreSystem::StepFinalize() {
        m_environment->StepFinalize();

        for (auto& item : m_PrePhysicsList) {
            item->StepFinalize();
        }

        for (auto& item : m_bodyList){
            item->StepFinalize();
        }

        for (auto& item : m_MidPhysicsList) {
            item->StepFinalize();
        }

        for (auto& item : m_linkList) {
            item->StepFinalize();
        }

        for (auto& item : m_PostPhysicsList) {
            item->StepFinalize();
        }

        // Serialize and send the message log
        FrObject::SendLog();

    }

    void FrOffshoreSystem::SetSystemType(SYSTEM_TYPE type, bool checkCompat) {

        if (m_chronoSystem) Clear(); // Clear the system from every bodies etc...

        // Creating the chrono System backend. It drives the way contact are modelled
        switch (type) {
            case SMOOTH_CONTACT:
                m_chronoSystem = std::make_unique<internal::FrSystemBaseSMC>(this);
                break;
            case NONSMOOTH_CONTACT:
                std::cout << "NSC systems is not tested for now !!!!" << std::endl;
                m_chronoSystem = std::make_unique<internal::FrSystemBaseNSC>();
                break;
        }

        m_systemType = type;

        if (checkCompat) CheckCompatibility();
    }

    void FrOffshoreSystem::CheckCompatibility() const {
        // TODO : verifier la compatibilite entre type systeme, solveur et integrateur temporel



    }

    bool FrOffshoreSystem::CheckBodyContactMethod(std::shared_ptr<FrBody> body) {
        return m_systemType == body->GetContactType();
    }

    void FrOffshoreSystem::SetSolver(SOLVER solver, bool checkCompat) {

        using SOLVERS = chrono::ChSolver::Type;

        switch (solver) {
            case SOR:
                m_chronoSystem->SetSolverType(SOLVERS::SOR);
                break;
            case SYMMSOR:
                m_chronoSystem->SetSolverType(SOLVERS::SYMMSOR);
                break;
            case JACOBI:
                m_chronoSystem->SetSolverType(SOLVERS::JACOBI);
                break;
            case BARZILAIBORWEIN:
                m_chronoSystem->SetSolverType(SOLVERS::BARZILAIBORWEIN);
                break;
            case PCG:
                m_chronoSystem->SetSolverType(SOLVERS::PCG);
                break;
            case APGD:
                m_chronoSystem->SetSolverType(SOLVERS::APGD);
                break;
            case MINRES:
                m_chronoSystem->SetSolverType(SOLVERS::MINRES);
                break;
            case SOLVER_SMC:
                m_chronoSystem->SetSolverType(SOLVERS::SOLVER_SMC);
                break;
        }

        m_solverType = solver;

        if (checkCompat) CheckCompatibility();
    }

    void FrOffshoreSystem::SetSolverWarmStarting(bool useWarm) {
        m_chronoSystem->SetSolverWarmStarting(useWarm);
    }

    void FrOffshoreSystem::SetSolverOverrelaxationParam(double omega) {
        m_chronoSystem->SetSolverOverrelaxationParam(omega);
    }

    void FrOffshoreSystem::SetSolverSharpnessParam(double momega) {
        m_chronoSystem->SetSolverSharpnessParam(momega);
    }

    void FrOffshoreSystem::SetParallelThreadNumber(int nbThreads) {
        m_chronoSystem->SetParallelThreadNumber(nbThreads);
    }

    void FrOffshoreSystem::SetSolverMaxIterSpeed(int maxIter) {
        m_chronoSystem->SetMaxItersSolverSpeed(maxIter);
    }

    void FrOffshoreSystem::SetSolverMaxIterStab(int maxIter) {
        m_chronoSystem->SetMaxItersSolverStab(maxIter);
    }

    void FrOffshoreSystem::SetSolverMaxIterAssembly(int maxIter) {
        m_chronoSystem->SetMaxiter(maxIter);
    }

    void FrOffshoreSystem::SetSolverGeometricTolerance(double tol) {
        m_chronoSystem->SetTol(tol);
    }

    void FrOffshoreSystem::SetSolverForceTolerance(double tol) {
        m_chronoSystem->SetTolForce(tol);
    }

    void FrOffshoreSystem::UseMaterialProperties(bool use) {
        if (m_systemType == SMOOTH_CONTACT) {
            dynamic_cast<chrono::ChSystemSMC*>(m_chronoSystem.get())->UseMaterialProperties(use);
        } else {
            std::cerr << "The use of material properties is only for SMOOTH_CONTACT systems" << std::endl;
        }
    }

    void FrOffshoreSystem::SetContactForceModel(FrOffshoreSystem::CONTACT_MODEL model) {
        if (m_systemType == SMOOTH_CONTACT) {
            auto systemSMC = dynamic_cast<chrono::ChSystemSMC*>(m_chronoSystem.get());
            using ContactForceModel = chrono::ChSystemSMC::ContactForceModel;
            switch (model) {
                case HOOKE:
                    systemSMC->SetContactForceModel(ContactForceModel::Hooke);
                    break;
                case HERTZ:
                    systemSMC->SetContactForceModel(ContactForceModel::Hertz);
                    break;
                case COULOMB:
                    systemSMC->SetContactForceModel(ContactForceModel::PlainCoulomb);
                    break;
            }
        } else {
            std::cerr << "Contact force model is only for SMOOTH_CONTACT systems" << std::endl;
        }
    }

    void FrOffshoreSystem::SetAdhesionForceModel(FrOffshoreSystem::ADHESION_MODEL model) {
        if (m_systemType == SMOOTH_CONTACT) {
            auto systemSMC = dynamic_cast<chrono::ChSystemSMC*>(m_chronoSystem.get());
            using AdhesionForceModel = chrono::ChSystemSMC::AdhesionForceModel ;
            switch (model) {
                case CONSTANT:
                    systemSMC->SetAdhesionForceModel(AdhesionForceModel::Constant);
                    break;
                case DMT:
                    systemSMC->SetAdhesionForceModel(AdhesionForceModel::DMT);
                    break;
            }
        } else {
            std::cerr << "Adhesion force model is only for SMOOTH_CONTACT systems" << std::endl;
        }
    }

    void FrOffshoreSystem::SetTangentialDisplacementModel(FrOffshoreSystem::TANGENTIAL_DISP_MODEL model) {
        if (m_systemType == SMOOTH_CONTACT) {
            auto systemSMC = dynamic_cast<chrono::ChSystemSMC*>(m_chronoSystem.get());
            using TangentialDisplacementModel = chrono::ChSystemSMC::TangentialDisplacementModel ;
            switch (model) {
                case NONE:
                    systemSMC->SetTangentialDisplacementModel(TangentialDisplacementModel::None);
                    break;
                case ONE_STEP:
                    systemSMC->SetTangentialDisplacementModel(TangentialDisplacementModel::OneStep);
                    break;
                case MULTI_STEP:
                    systemSMC->SetTangentialDisplacementModel(TangentialDisplacementModel::MultiStep);
                    break;
            }
        } else {
            std::cerr << "Adhesion force model is only for SMOOTH_CONTACT systems" << std::endl;
        }
    }

    void FrOffshoreSystem::SetStiffContact(bool isStiff) {
        if (m_systemType == SMOOTH_CONTACT) {
            dynamic_cast<chrono::ChSystemSMC*>(m_chronoSystem.get())->SetStiffContact(isStiff);
        } else {
            std::cerr << "StiffContact is only for SMOOTH_CONTACT systems" << std::endl;
        }
    }

    void FrOffshoreSystem::SetSlipVelocityThreshold(double velocity) {
        if (m_systemType == SMOOTH_CONTACT) {
            dynamic_cast<chrono::ChSystemSMC*>(m_chronoSystem.get())->SetSlipVelocityThreshold(velocity);
        } else {
            std::cerr << "Slip Velocity Threshold is only for SMOOTH_CONTACT systems" << std::endl;
        }
    }

    void FrOffshoreSystem::SetCharacteristicImpactVelocity(double velocity) {
        if (m_systemType == SMOOTH_CONTACT) {
            dynamic_cast<chrono::ChSystemSMC*>(m_chronoSystem.get())->SetCharacteristicImpactVelocity(velocity);
        } else {
            std::cerr << "Characteristic Impact Velocity is only for SMOOTH_CONTACT systems" << std::endl;
        }
    }

    void FrOffshoreSystem::SetMinBounceSpeed(double speed) {
        m_chronoSystem->SetMinBounceSpeed(speed);
    }

    void FrOffshoreSystem::SetMaxPenetrationRecoverySpeed(double speed) {
        m_chronoSystem->SetMaxPenetrationRecoverySpeed(speed);
    }

    int FrOffshoreSystem::GetNbPositionCoords() const {
        return m_chronoSystem->GetNcoords();
    }

    int FrOffshoreSystem::GetNbVelocityCoords() const {
        return m_chronoSystem->GetNcoords_w();
    }

    int FrOffshoreSystem::GetNbConstraintsCoords() const {
        return m_chronoSystem->GetNdoc_w();
    }

    int FrOffshoreSystem::GetNbDOF() const {
        return m_chronoSystem->GetNdof();
    }

    int FrOffshoreSystem::GetNbBodies() const {
        return m_chronoSystem->GetNbodies();
    }

    int FrOffshoreSystem::GetNbFixedBodies() const {
        return m_chronoSystem->GetNbodiesFixed();
    }

    int FrOffshoreSystem::GetNbSleepingBodies() const {
        return m_chronoSystem->GetNbodiesSleeping();
    }

    void FrOffshoreSystem::SetUseSleepingBodies(bool useSleeping) {
        m_chronoSystem->SetUseSleeping(useSleeping);
    }

    double FrOffshoreSystem::GetGravityAcceleration() const {
        return fabs(m_chronoSystem->Get_G_acc()[2]);
    }

    void FrOffshoreSystem::SetGravityAcceleration(double gravityAcceleration) {
        m_chronoSystem->Set_G_acc(chrono::ChVector<double>(0., 0., -gravityAcceleration));
    }

    void FrOffshoreSystem::SetNbStepsStatics(int nSteps) {
        m_nbStepStatics = nSteps;
    }

    bool FrOffshoreSystem::SolveStaticEquilibrium(FrOffshoreSystem::STATICS_METHOD method) {
        switch (method) {
            case LINEAR:
                return m_chronoSystem->DoStaticLinear();
            case NONLINEAR:
                return m_chronoSystem->DoStaticNonlinear(m_nbStepStatics);
            case RELAXATION:
                return m_chronoSystem->DoStaticRelaxing(m_nbStepStatics);
        }
        // FIXME : il semble que les solveurs retournent toujours true...
    }

    void FrOffshoreSystem::SetTimeStepper(TIME_STEPPER type, bool checkCompat) {

        using timeStepperType = chrono::ChTimestepper::Type;

        switch (type) {
            case EULER_IMPLICIT_LINEARIZED:
                m_chronoSystem->SetTimestepperType(timeStepperType::EULER_IMPLICIT_LINEARIZED);
                break;
            case EULER_IMPLICIT_PROJECTED:
                m_chronoSystem->SetTimestepperType(timeStepperType::EULER_IMPLICIT_PROJECTED);
                break;
            case EULER_IMPLICIT:
                m_chronoSystem->SetTimestepperType(timeStepperType::EULER_IMPLICIT);
                break;
            case TRAPEZOIDAL:
                m_chronoSystem->SetTimestepperType(timeStepperType::TRAPEZOIDAL);
                break;
            case TRAPEZOIDAL_LINEARIZED:
                m_chronoSystem->SetTimestepperType(timeStepperType::TRAPEZOIDAL_LINEARIZED);
                break;
            case HHT:
                m_chronoSystem->SetTimestepperType(timeStepperType::HHT);
                break;
            case RUNGEKUTTA45:
                m_chronoSystem->SetTimestepperType(timeStepperType::RUNGEKUTTA45);
                break;
            case EULER_EXPLICIT:
                m_chronoSystem->SetTimestepperType(timeStepperType::EULER_EXPLICIT);
                break;
            case NEWMARK:
                m_chronoSystem->SetTimestepperType(timeStepperType::NEWMARK);
                break;
        }

        m_timeStepper = type;

        if (checkCompat) CheckCompatibility();

    }

    void FrOffshoreSystem::SetTimeStep(double timeStep) {
        m_chronoSystem->SetStep(timeStep);
    }

    double FrOffshoreSystem::GetTimeStep() const {
        return m_chronoSystem->GetStep();
    }

    void FrOffshoreSystem::SetMinTimeStep(double minTimeStep) {
        m_chronoSystem->SetStepMin(minTimeStep);
    }

    void FrOffshoreSystem::SetMaxTimeStep(double maxTimeStep) {
        m_chronoSystem->SetStepMax(maxTimeStep);
    }

    double FrOffshoreSystem::GetTime() const {
        return m_chronoSystem->GetChTime();
    }

    bool FrOffshoreSystem::AdvanceOneStep(double stepSize) {
        IsInitialized();
        return (bool)m_chronoSystem->DoStepDynamics(stepSize);
    }

    bool FrOffshoreSystem::AdvanceTo(double nextTime) {
        IsInitialized();
        return m_chronoSystem->DoFrameDynamics(nextTime);
    }

    bool FrOffshoreSystem::RunDynamics(double frameStep) {
        IsInitialized();
        m_chronoSystem->Setup();
        m_chronoSystem->DoAssembly(chrono::AssemblyLevel::POSITION |
                                   chrono::AssemblyLevel::VELOCITY |
                                   chrono::AssemblyLevel::ACCELERATION);

        while (true) {
            double nextTime = m_chronoSystem->GetChTime() + frameStep;
            if (!AdvanceTo(nextTime))
                return false;
        }
        return true;
    }

    void FrOffshoreSystem::CreateWorldBody() {
        m_worldBody = std::make_shared<FrBody>();
        m_worldBody->SetFixedInWorld(true);
        m_worldBody->SetName("WorldBody");
        m_worldBody->SetLogged(false);
        AddBody(m_worldBody);
    }

    std::shared_ptr<FrBody> FrOffshoreSystem::NewBody() {
        auto body = std::make_shared<FrBody>();  // TODO : suivant le type de systeme SMC ou NSC, regler le type de surface...

        switch (m_systemType) {
            case SMOOTH_CONTACT:
                body->SetSmoothContact();
                break;
            case NONSMOOTH_CONTACT:
                body->SetNonSmoothContact();
                break;
        }

        AddBody(body);
        return body;
    }

    void FrOffshoreSystem::Clear() {
        m_chronoSystem->Clear();

        m_bodyList.clear();
//        m_linkList.clear();
//        m_otherPhysicsList.clear(); // FIXME : continuer les clear !!!
    }

    chrono::ChSystem* FrOffshoreSystem::GetChronoSystem() {
        return m_chronoSystem.get();
    }


    // Irrlicht visualization

    void FrOffshoreSystem::RunInViewer(double endTime, double dist, bool recordVideo) {

        /// This subroutine runs the numerical simulation.

        /// \param endTime End time.
        /// \param dist Distance of the video camera.
        /// \param recordVideo True if the video is recorded, false otherwise.

        // Initialization of the system if not already done.
        IsInitialized();

        // Definition and initialization of the Irrlicht application.
        FrIrrApp app(m_chronoSystem.get(), dist);

        app.SetTimestep(m_chronoSystem->GetStep());
        app.SetVideoframeSave(recordVideo);
        app.Run(endTime); // The temporal loop is here.

    }

    void FrOffshoreSystem::Visualize( double dist, bool recordVideo) {

        Initialize();  // So that system is automatically initialized when run in viewer mode

        FrIrrApp app(m_chronoSystem.get(), dist);

        app.SetTimestep(m_chronoSystem->GetStep());
        app.SetVideoframeSave(recordVideo);
        app.Visualize();

    }

    void FrOffshoreSystem::AddAsset(std::shared_ptr<chrono::ChAsset> asset) {
        m_chronoSystem->AddAsset(std::move(asset));
    }

    void FrOffshoreSystem::IsInitialized() {
        if (!m_isInitialized) Initialize();
    }


    // Iterators

    FrOffshoreSystem::BodyIter FrOffshoreSystem::body_begin() {
        return m_bodyList.begin();
    }

    FrOffshoreSystem::ConstBodyIter FrOffshoreSystem::body_begin() const {
        return m_bodyList.cbegin();
    }

    FrOffshoreSystem::BodyIter FrOffshoreSystem::body_end() {
        return m_bodyList.end();
    }

    FrOffshoreSystem::ConstBodyIter FrOffshoreSystem::body_end() const {
        return m_bodyList.cend();
    }

    FrOffshoreSystem::LinkIter FrOffshoreSystem::link_begin() {
        return m_linkList.begin();
    }

    FrOffshoreSystem::ConstLinkIter FrOffshoreSystem::link_begin() const {
        return m_linkList.cbegin();
    }

    FrOffshoreSystem::LinkIter FrOffshoreSystem::link_end() {
        return m_linkList.end();
    }

    FrOffshoreSystem::ConstLinkIter FrOffshoreSystem::link_end() const {
        return m_linkList.cend();
    }

    void FrOffshoreSystem::InitializeLog() {

        if (IsLogged()) {

            m_pathManager->Initialize(this);

            auto logPath = m_pathManager->BuildPath(this, "system.csv");

            // Add the fields
            // TODO A completer
//            m_message->AddField<double>("time", "s", "Current time of the simulation",
//                    [this]() { return m_chronoSystem->GetChTime(); });

            // Init the message
            FrObject::InitializeLog(logPath);

            // Initializing environment before bodies
//            m_environment->InitializeLog();

            for (auto &item : m_PrePhysicsList) {
                item->InitializeLog();
            }

            for (auto &item : m_bodyList) {
                item->InitializeLog();
            }

            for (auto &item : m_MidPhysicsList) {
                item->InitializeLog();
            }

            for (auto &item : m_linkList) {
                item->InitializeLog();
            }

            for (auto &item : m_PostPhysicsList) {
                item->InitializeLog();
            }

        }
    }

    FrPathManager *FrOffshoreSystem::GetPathManager() const { return m_pathManager.get(); }


}  // end namespace frydom
