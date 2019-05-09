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
#include "chrono/fea/ChLinkPointFrame.h"
#include "chrono/physics/ChLinkMate.h"

#include "frydom/core/link/links_lib/FrLink.h"
#include "frydom/core/body/FrBody.h"
#include "frydom/core/common/FrFEAMesh.h"
#include "frydom/cable/FrDynamicCable.h"
#include "frydom/core/force/FrForce.h"
#include "frydom/environment/FrEnvironment.h"
#include "frydom/utils/FrIrrApp.h"
#include "frydom/core/statics/FrStaticAnalysis.h"

#include "frydom/core/math/functions/ramp/FrCosRampFunction.h"
#include "frydom/utils/FrSerializerFactory.h"


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

            for (auto &mesh : meshlist) {
                mesh->Update(ChTime, update_assets);
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

//        // -----------------------------------------------------------------------------
//        // **** PERFORM THE STATIC ANALYSIS, FINDING THE STATIC
//        // **** EQUILIBRIUM OF THE SYSTEM, WITH ITERATIVE SOLUTION
//        // -----------------------------------------------------------------------------
//
//        bool FrSystemBaseSMC::DoQuasiStatic(int niter, int nsteps) {
//
//            double m_undotime = GetChTime();
//            bool reach_tolerance = false;
//
//            if ((ncoords > 0) && (ndof >= 0)) {
//                for (int m_iter = 0; m_iter < niter; m_iter++) {
//                    // Get the speed of the bodies to check the convergence
//                    double bodyVel = 0;
//                    for (auto &body : bodylist) {
//                        bodyVel += body->GetPos_dt().Length2();
//                    }
//
//                    // Set no speed and accel. on bodies, meshes and other physics items
//                    Relax();
//
//                    std::cout<<m_iter<<", "<<GetChTime()<<", "<<bodyVel<<std::endl;
//                    // TODO : introduce a tolerance parameter
//                    if (bodyVel < 1E-5 && GetChTime()>m_undotime+step*nsteps) {
//                        reach_tolerance = true;
//                        break;
//                    }
//                    DoFrameDynamics(m_undotime + m_iter * step * nsteps);
//                }
//
//                // Set no speed and accel. on bodies, meshes and other physics items
//                Relax();
//            }
//
//            SetChTime(m_undotime);
//            return reach_tolerance;
//        }

        bool FrSystemBaseSMC::DoStaticLinear() {
            // Set no speed and accel. on bodies, meshes and other physics items
            for (auto &body : bodylist) {
                body->SetNoSpeedNoAcceleration();
            }
            for (auto& mesh : meshlist) {
                mesh->SetNoSpeedNoAcceleration();
            }
            for (auto &ip : otherphysicslist) {
                ip->SetNoSpeedNoAcceleration();
            }
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

        // Creating a fixed world body to be able to attach anything to it (anchors...) // TODO: mettre dans une methode privee
        CreateWorldBody();

        // Creating the environment
        m_environment = std::make_unique<FrEnvironment>(this); // FIXME: voir bug dans FrEnvironment pour le reglage du systeme

        // Creating the log manager service
        m_pathManager = std::make_shared<FrPathManager>();
        
        // Create the static analysis 
        m_statics = std::make_unique<FrStaticAnalysis>(this);

//        m_message = std::make_unique<hermes::Message>();

    }

    FrOffshoreSystem::~FrOffshoreSystem() = default;


    void FrOffshoreSystem::Add(std::shared_ptr<FrObject> newItem) {
        assert(std::dynamic_pointer_cast<FrBody>(newItem) ||
               std::dynamic_pointer_cast<FrLinkBase>(newItem) ||
               std::dynamic_pointer_cast<FrPhysicsItem>(newItem));

        if (auto item = std::dynamic_pointer_cast<FrBody>(newItem)) {
            AddBody(item);
            return;
        }

        if (auto item = std::dynamic_pointer_cast<FrLinkBase>(newItem)) {
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


    // ***** Body *****

    void FrOffshoreSystem::AddBody(std::shared_ptr<FrBody> body) {

        if (!CheckBodyContactMethod(body)) { // TODO : voir si on set pas d'autorite le mode de contact a celui du systeme plutot que de faire un if...
            body->SetContactMethod(m_systemType);
        }

        m_chronoSystem->AddBody(body->GetChronoBody());  // Authorized because this method is a friend of FrBody
        m_bodyList.push_back(body);

        body->m_system = this;

    }

    FrOffshoreSystem::BodyContainer FrOffshoreSystem::GetBodyList() {
        return m_bodyList;
    }

    void FrOffshoreSystem::RemoveBody(std::shared_ptr<FrBody> body) {

        m_chronoSystem->RemoveBody(body->GetChronoBody());

        auto it = std::find(body_begin(),body_end(),body);
        assert(it != body_end());
        m_bodyList.erase(it);
        body->m_system = nullptr;

    }

    void FrOffshoreSystem::RemoveAllBodies() {

        for (auto& body: m_bodyList)
            RemoveBody(body);

    }


    // ***** Link *****

    void FrOffshoreSystem::AddLink(std::shared_ptr<FrLinkBase> link) {
        m_chronoSystem->AddLink(link->GetChronoLink());
        m_linkList.push_back(link);
    }

    void FrOffshoreSystem::RemoveLink(std::shared_ptr<FrLinkBase> link) {

        m_chronoSystem->RemoveLink(link->GetChronoLink());

        auto it = std::find(link_begin(),link_end(),link);
        assert(it != link_end());
        m_linkList.erase(it);
        link->m_system = nullptr;

    }

    void FrOffshoreSystem::RemoveAllLinks() {

        for (auto& link: m_linkList)
            RemoveLink(link);

    }


    // ***** Physics Item *****

    void FrOffshoreSystem::AddPhysicsItem(std::shared_ptr<FrPrePhysicsItem> otherPhysics) {
        m_chronoSystem->AddOtherPhysicsItem(otherPhysics->GetChronoPhysicsItem());
        otherPhysics->m_system = this;
        m_PrePhysicsList.push_back(otherPhysics);
    }

    FrOffshoreSystem::PrePhysicsContainer FrOffshoreSystem::GetPrePhysicsItemList() {
        return m_PrePhysicsList;
    }

    void FrOffshoreSystem::AddPhysicsItem(std::shared_ptr<FrMidPhysicsItem> otherPhysics) {
        m_chronoSystem->AddOtherPhysicsItem(otherPhysics->GetChronoPhysicsItem());
        otherPhysics->m_system = this;
        m_MidPhysicsList.push_back(otherPhysics);
    }

    FrOffshoreSystem::MidPhysicsContainer FrOffshoreSystem::GetMidPhysicsItemList() {
        return m_MidPhysicsList;
    }

    void FrOffshoreSystem::AddPhysicsItem(std::shared_ptr<FrPostPhysicsItem> otherPhysics) {
        m_chronoSystem->AddOtherPhysicsItem(otherPhysics->GetChronoPhysicsItem());
        otherPhysics->m_system = this;
        m_PostPhysicsList.push_back(otherPhysics);
    }

    FrOffshoreSystem::PostPhysicsContainer FrOffshoreSystem::GetPostPhysicsItemList() {
        return m_PostPhysicsList;
    }

    void FrOffshoreSystem::RemovePhysicsItem(std::shared_ptr<FrPhysicsItem> item) {

        m_chronoSystem->RemoveOtherPhysicsItem(item->GetChronoPhysicsItem());

        auto it = std::find(m_PrePhysicsList.begin(),m_PrePhysicsList.end(),item);
        if (it!= m_PrePhysicsList.end())
            m_PrePhysicsList.erase(it);
        else {
            auto it = std::find(m_MidPhysicsList.begin(),m_MidPhysicsList.end(),item);
            if (it!= m_MidPhysicsList.end())
                m_MidPhysicsList.erase(it);
            else {
                auto it = std::find(m_PostPhysicsList.begin(),m_PostPhysicsList.end(),item);
                if (it!= m_PostPhysicsList.end())
                    m_PostPhysicsList.erase(it);
                else {
                    assert(("physics item can't be found in the list : ",it!= m_PostPhysicsList.end()));
                }
            }
        }


        item->m_system = nullptr;

    }

    void FrOffshoreSystem::RemoveAllPhysicsItem() {

        for (auto& item: m_PrePhysicsList)
            RemovePhysicsItem(item);

        for (auto& item: m_MidPhysicsList)
            RemovePhysicsItem(item);

        for (auto& item: m_PostPhysicsList)
            RemovePhysicsItem(item);

    }


    // ***** FEAMesh *****

    void FrOffshoreSystem::AddFEAMesh(std::shared_ptr<FrFEAMesh> feaMesh){
        m_chronoSystem->AddMesh(feaMesh->GetChronoMesh());  // Authorized because this method is a friend of FrFEAMesh

        feaMesh->m_system = this;
        m_feaMeshList.push_back(feaMesh);
    }

    void FrOffshoreSystem::Add(std::shared_ptr<FrDynamicCable> cable) {

        // Add the FEA mesh
        AddFEAMesh(cable);

        // Add the hinges
        m_chronoSystem->Add(dynamic_cast<internal::FrDynamicCableBase*>(cable->GetChronoMesh().get())->m_startingHinge);
        m_chronoSystem->Add(dynamic_cast<internal::FrDynamicCableBase*>(cable->GetChronoMesh().get())->m_endingHinge);

    }

    FrOffshoreSystem::FEAMeshContainer FrOffshoreSystem::GetFEAMeshList() {
        return m_feaMeshList;
    }

    void FrOffshoreSystem::RemoveFEAMesh(std::shared_ptr<FrFEAMesh> feamesh) {

        m_chronoSystem->RemoveMesh(feamesh->GetChronoMesh());

        auto it = std::find(m_feaMeshList.begin(),m_feaMeshList.end(),feamesh);
        assert(it != m_feaMeshList.end());
        m_feaMeshList.erase(it);
        feamesh->m_system = nullptr;

    }

    void FrOffshoreSystem::Remove(std::shared_ptr<FrDynamicCable> cable) {

        RemoveFEAMesh(cable);

        m_chronoSystem->RemoveOtherPhysicsItem(dynamic_cast<internal::FrDynamicCableBase*>(cable->GetChronoMesh().get())->m_startingHinge);
        m_chronoSystem->RemoveOtherPhysicsItem(dynamic_cast<internal::FrDynamicCableBase*>(cable->GetChronoMesh().get())->m_endingHinge);

    }


    // ***** Environment *****

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

        for (auto& item : m_feaMeshList) {
            item->Initialize();
        }

        for (auto& item : m_PostPhysicsList) {
            item->Initialize();
        }

        m_chronoSystem->Update();


        // Init the logs
        if (IsLogged()) {
            m_pathManager->Initialize(this);
            m_pathManager->SetRunPath("Dynamic");
            InitializeLog("");
        }

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

        for (auto& item : m_feaMeshList) {
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

    FrStaticAnalysis *FrOffshoreSystem::GetStaticAnalysis() const {
        return m_statics.get();
    }

    bool FrOffshoreSystem::SolveStaticWithRelaxation() {

        IsInitialized();

        return m_statics->SolveStatic();

    }

    void FrOffshoreSystem::Relax(FrStaticAnalysis::RELAXTYPE relax) {

        for (auto& body:m_bodyList) {
            switch (relax) {
                case FrStaticAnalysis::NORELAX :
                    break;
                case FrStaticAnalysis::VELOCITY :
                    body->SetVelocityInWorldNoRotation(Velocity(), NWU);
                    break;
                case FrStaticAnalysis::ACCELERATION :
                    body->SetAccelerationInBodyNoRotation(Acceleration(), NWU);
                    break;
                case FrStaticAnalysis::VELOCITYANDACCELERATION :
                    body->SetVelocityInWorldNoRotation(Velocity(), NWU);
                    body->SetAccelerationInBodyNoRotation(Acceleration(), NWU);
                    break;
            }
        }

        for (auto &mesh : m_feaMeshList) {
            mesh->Relax();
        }

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

    void FrOffshoreSystem::SetTime(double time) {
        m_chronoSystem->SetChTime(time);
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
        m_linkList.clear();
        m_feaMeshList.clear();
        m_PrePhysicsList.clear();
        m_MidPhysicsList.clear();
        m_PostPhysicsList.clear();

        m_isInitialized = false;
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
        FrIrrApp app(this, m_chronoSystem.get(), dist);

        app.SetTimestep(m_chronoSystem->GetStep());
        app.SetVideoframeSave(recordVideo);
        app.Run(endTime); // The temporal loop is here.

    }

    void FrOffshoreSystem::Visualize( double dist, bool recordVideo) {

        IsInitialized();  // So that system is automatically initialized when run in viewer mode

        FrIrrApp app(this, m_chronoSystem.get(), dist);

        app.SetTimestep(m_chronoSystem->GetStep());
        app.SetVideoframeSave(recordVideo);
        app.Visualize();

    }

    void FrOffshoreSystem::VisualizeStaticAnalysis( double dist, bool recordVideo) {

        IsInitialized();  // So that system is automatically initialized when run in viewer mode

        FrIrrApp app(this, m_chronoSystem.get(), dist);

        app.SetTimestep(m_chronoSystem->GetStep());
        app.SetVideoframeSave(recordVideo);
        app.VisualizeStaticAnalysis();

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

    void FrOffshoreSystem::InitializeLog_Dependencies(const std::string& systemPath) {

        if (IsLogged()) {

            // Initializing environment before bodies
//            m_environment->InitializeLog();

            for (auto &item : m_PrePhysicsList) {
                item->InitializeLog(systemPath);
            }

            for (auto &item : m_bodyList) {
                item->InitializeLog(systemPath);
            }

            for (auto &item : m_MidPhysicsList) {
                item->InitializeLog(systemPath);
            }

            for (auto &item : m_linkList) {
                item->InitializeLog(systemPath);
            }

            for (auto &item : m_feaMeshList) {
                item->InitializeLog(systemPath);
            }

            for (auto &item : m_PostPhysicsList) {
                item->InitializeLog(systemPath);
            }

        }
    }

    void FrOffshoreSystem::ClearLogs() {

        ClearMessage();

        for (auto &item : m_PrePhysicsList) {
            item->ClearMessage();
        }

        for (auto &item : m_bodyList) {
            item->ClearMessage();
            for (auto& force : item->GetForceList()) {
                force->ClearMessage();
            }
            for (auto& node : item->GetNodeList()) {
                node->ClearMessage();
            }
        }

        for (auto &item : m_MidPhysicsList) {
            item->ClearMessage();
        }

        for (auto &item : m_linkList) {
            item->ClearMessage();
        }

        for (auto &item : m_PostPhysicsList) {
            item->ClearMessage();
        }

    }

    std::string FrOffshoreSystem::BuildPath(const std::string &rootPath) {

        auto objPath= fmt::format("{}_{}", GetTypeName(), GetShortenUUID());

        auto logPath = GetPathManager()->BuildPath(objPath, fmt::format("{}_{}.csv", GetTypeName(), GetShortenUUID()));

        // Add a serializer
        m_message->AddSerializer(FrSerializerFactory::instance().Create(this, logPath));

        return objPath;
    }


}  // end namespace frydom
