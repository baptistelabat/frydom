//
// Created by frongere on 29/05/17.
//

#include <frydom/cable/FrDynamicCable.h>
#include <chrono/utils/ChProfiler.h>
#include "FrOffshoreSystem.h"

#include "frydom/core/body/FrBody.h"
#include "frydom/core/link/links_lib/FrLink.h"
#include "frydom/core/common/FrPhysicsItem.h"
#include "frydom/environment/FrEnvironment.h"
#include "frydom/environment/ocean/freeSurface/FrFreeSurface.h"

#include "frydom/hydrodynamics/seakeeping/linear/hdb/FrHydroMapper.h"
#include "frydom/hydrodynamics/seakeeping/linear/hdb/FrHydroDB.h"
#include "frydom/core/body/FrBody.h"


#include "frydom/cable/FrCable.h"



//#include "GeographicLib/LocalCartesian.hpp"
//#include "frydom/environment/FrTimeZone.h"
//#include "frydom/environment/seabed/FrSeabed.h"
//#include "frydom/environment/tidal/FrTidalModel.h"



namespace frydom {

    FrOffshoreSystem::FrOffshoreSystem(bool use_material_properties,
                                       unsigned int max_objects,
                                       double scene_size) :

            chrono::ChSystemSMC(use_material_properties, max_objects, scene_size),
//            m_gravity_acc_magnitude(9.81),
//            m_water_density(1025.),
            NEDframe(chrono::VNULL, M_PI, chrono::VECT_X) {

        // The world body is a virtual body with no mass that is fixed and used to fix nodes in the absolute frame
        world_body = std::make_shared<FrBody>();
        //world_body->SetSystem(this);
        world_body->SetBodyFixed(true);
        world_body->SetName("WorldBody");
        AddBody(world_body);

        m_environment = std::make_unique<FrEnvironment>();
        m_environment->SetSystem(this);
        AddBody(m_environment->GetFreeSurface()->GetBody());


        // Convention for z orientation is upward
        Set_G_acc(chrono::ChVector<>(0., 0., -m_environment->GetGravityAcceleration()));

    }

    void FrOffshoreSystem::Update(bool update_assets) {
        timer_update.start();  // Timer for profiling

        // TODO: Mettre ici a jour tous les elements de l'environnement
        // Update all environment models (waves, wind, current...)
        m_environment->Update(ChTime);

        // Executes the "forUpdate" in all controls of controlslist
        ExecuteControlsForUpdate();

        // Inherit parent class (recursively update sub objects bodies, links, etc)
        chrono::ChAssembly::Update(update_assets);

        // Update all contacts, if any
        contact_container->Update(ChTime, update_assets);

        timer_update.stop();
    }

    // From state Y={x,v} to system.
    void FrOffshoreSystem::StateScatter(const chrono::ChState &x, const chrono::ChStateDelta &v, const double T) {

        m_environment->Update(T);  // Updating environment

        IntStateScatter(0, x, 0, v,
                        T);  // TODO: voir pour faire un update de l'environnement juste avant cette ligne ...

//        Update();  //***TODO*** optimize because maybe IntStateScatter above might have already called Update?
    }

    bool FrOffshoreSystem::Integrate_Y() {
        ResetTimers();

        timer_step.start();

        // Executes "forStep" in all controls of controlslist
        ExecuteControlsForStep();  // C'est ici qu'on pourra trigger les calculs de controllers... Voir avec sof si
                                   // c'est le bon endroit / si l'objet control de chrono convient

        stepcount++;
        solvecount = 0;
        setupcount = 0;

        // Compute contacts and create contact constraints
        ComputeCollisions();

        // Counts dofs, statistics, etc. (not needed because already in Advance()...? ) // TODO: voir ce qu'il en est
        Setup();

        // Update everything - and put to sleep bodies that need it (not needed because already in Advance()...? )
        // No need to update visualization assets here.
//        Update(true);  // FIXME : Desactive car redondant avec ce qui est deja fait lors du system::StateScatter()...

        // Re-wake the bodies that cannot sleep because they are in contact with
        // some body that is not in sleep state.
//        ManageSleepingBodies(); // Proposer au chrono group que cette methode soit protected afi de pouvoir
                                  // completement deriver de Integrate_Y()...

        // Prepare lists of variables and constraints.
        DescriptorPrepareInject(*descriptor);
        descriptor->UpdateCountsAndOffsets();

        // Set some settings in timestepper object
        timestepper->SetQcDoClamp(true);
        timestepper->SetQcClamping(max_penetration_recovery_speed);
        // TODO: reactiver les 3 lignes suivantes pour autoriser l'utilisation des solveurs HHT et Newmark
//        if (std::dynamic_pointer_cast<ChTimestepperHHT>(timestepper) ||
//            std::dynamic_pointer_cast<ChTimestepperNewmark>(timestepper))
//            timestepper->SetQcDoClamp(false);

        // PERFORM TIME STEP HERE!
        timestepper->Advance(
                step);  // Ici, on passe du temps courant au temps suivant en utilisant le shema du timestepper choisi

        // Executes custom processing at the end of step
        CustomEndOfStep();

        // If there are some probe objects in the probe list,
        // tell them to record their variables (ususally x-y couples)
        RecordAllProbes(); // Voir a utiliser les ChProbe pour les capteurs controle. On pourra utiliser pour la radiation et l'enregistremet en buffer circulaire...

        // Call method to gather contact forces/torques in rigid bodies
        contact_container->ComputeContactForces();

        // TODO: ici, appeler une methode sur tout l'assembly permettant de faire une finalisation du pas de temps courant
        // ou alors le faire en tout debut de cette methode pour avoir aussi le t=0 ?
        // Cet appel permettra de trigger le log aux pas de temps fixes.
        // Voir aussi la methode CurtomEndOfStep() ci-dessus qui pourrait faire tout a fait l'affaire...

        // Time elapsed for step..
        timer_step.stop();


        std::cout << "Framerate = " << 1. / timer_step.GetTimeSeconds() << " FPS" << std::endl;

        return true;
    }

    void FrOffshoreSystem::CustomEndOfStep() {
        // TODO : Ici on a bon candidat pour trigger l'emission des donnees des objets...
        std::cout << "End of time step leading to time " << ChTime << std::endl;

        m_NitterOutput += 1;

        if (m_NsampleOutput == m_NitterOutput) {
            m_NitterOutput = 0;
            for (auto &ibody : bodylist) {
                auto body = dynamic_cast<FrBody *>(ibody.get());
                if (body) {
                    body->StepFinalize();
                }
            }
        }
    }

    void FrOffshoreSystem::Initialize() {
        // TODO: Ici, on initialise tous les composants de systeme. Ceci implique d'iterer sur ces derniers et qu'ils
        // possedent tous une methode Initialize()
        // On pourra faire deriver tous les objets d'une class FrObject qui apporte a la fois un UUID et a la
        // methode initialize comme methode virtuelle

        std::cout << "Initializing the system to make every component consistent" << std::endl;
        m_environment->Initialize();

        // Initializing physical items
        for (auto &ibody : bodylist) {
            auto body = dynamic_cast<FrBody *>(ibody.get());
            if (body) {
                body->Initialize();
            }
        }

        // We can now do an initial Update as everything has been initialized against everything
        Update(true);
    }

    void FrOffshoreSystem::StepFinalize() {

        std::cout << "Finalizing the step at time " << GetChTime() << std::endl;
        m_environment->StepFinalize();

        for (auto &ibody : bodylist) {
            auto body = dynamic_cast<FrBody *>(ibody.get());
            if (body) {
                body->StepFinalize();//TODO this line does not work, check why
            }
        }

    }

    void FrOffshoreSystem::IntLoadResidual_Mv(const unsigned int off,
                                              chrono::ChVectorDynamic<>& Res,
                                              const chrono::ChVectorDynamic<>& w,
                                              const double c) {
        unsigned int displ_v = off - offset_w;

        // Inherit: operate parent method on sub object (bodies, links, etc.)
        chrono::ChAssembly::IntLoadResidual_Mv(off, Res, w, c);
        // Use also on contact container:
        contact_container->IntLoadResidual_Mv(displ_v + contact_container->GetOffset_w(), Res, w, c);
        // Use also on hydro mapper:
        auto off_ChAw = chrono::ChAssembly::GetOffset_w();
        for (auto& mapper: m_hydroMapper){
            mapper->IntLoadResidual_Mv(off-off_ChAw, Res, w, c);
        }

    }

    void FrOffshoreSystem::VariablesFbIncrementMq() {
        // Inherit: operate parent method on sub object (bodies, links, etc.)
        chrono::ChAssembly::VariablesFbIncrementMq();
        // Use also on contact container:
        contact_container->VariablesFbIncrementMq();
        // Use also on hydro mapper
        /**
        for (auto& mapper: m_hydroMapper) {
			mapper->VariablesFbIncrementMq();
		}
        **/

    }

    FrEnvironment *FrOffshoreSystem::GetEnvironment() const {
        return m_environment.get();
    }

    chrono::ChFrame<double> FrOffshoreSystem::GetNEDFrame() const { return NEDframe; }

    chrono::ChBody *FrOffshoreSystem::GetWorldBodyPtr() const {
        return world_body.get();
    }

    std::shared_ptr<FrBody> FrOffshoreSystem::GetWorldBody() const {
        return world_body;
    }

    void FrOffshoreSystem::SetHydroMapper(std::shared_ptr<FrHydroMapper> hydroMapper) {
        m_hydroMapper.push_back(hydroMapper);
    }

    std::shared_ptr<FrHydroMapper> FrOffshoreSystem::GetHydroMapper(const int id) const {
        return m_hydroMapper[id];
    }

    void FrOffshoreSystem::SetHydroDB(const std::string filename) {
        m_HDB.push_back( std::make_shared<FrHydroDB>(filename) );
        m_nHDB += 1;
        m_hydroMapper.push_back( m_HDB[m_nHDB-1]->GetMapper() );
    }

    FrHydroDB *FrOffshoreSystem::GetHydroDB(const int id) const {
        return m_HDB[id].get();
    }

    int FrOffshoreSystem::GetHydroMapNb() const { return (int) m_hydroMapper.size(); }













    // REFACTORING ------------->>>>>>>>>>>>>>>

    _FrSystemBaseSMC::_FrSystemBaseSMC(frydom::FrOffshoreSystem_ *offshoreSystem) :
            chrono::ChSystemSMC(), m_offshoreSystem_(offshoreSystem) {}

    void _FrSystemBaseSMC::Update(bool update_assets) {

        CH_PROFILE( "Update");

        timer_update.start();  // Timer for profiling

        // Pre updates that are not about multibody dynamics
        m_offshoreSystem_->PreUpdate();

        // Executes the "forUpdate" in all controls of controlslist
        ExecuteControlsForUpdate();

        // Physics item that have to be updated before all
        m_offshoreSystem_->PrePhysicsUpdate(ChTime, update_assets);

        // Bodies updates  // FIXME : appeler les updates directement des objets frydom !
        for (auto& body : bodylist) {
            body->Update(ChTime, update_assets);
        }

        // Physics items that have to be updated between bodies and links
        m_offshoreSystem_->MidPhysicsUpdate(ChTime, update_assets);

        // Links updates  // FIXME : appeler les updates directement des objets frydom !
        for (auto& link : linklist) {
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

    void _FrSystemBaseSMC::CustomEndOfStep() {
        m_offshoreSystem_->StepFinalize();
    }

//    void _FrSystemBaseSMC::SetupInitial() {
//        chrono::ChSystem::SetupInitial();
//        m_offshoreSystem_->Initialize();
//    }

    FrOffshoreSystem_::FrOffshoreSystem_(SYSTEM_TYPE systemType, TIME_STEPPER timeStepper, SOLVER solver) {

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
        m_environment = std::make_unique<FrEnvironment_>(this); // FIXME: voir bug dans FrEnvironment pour le reglage du systeme

    }

    FrOffshoreSystem_::~FrOffshoreSystem_() = default;
    void FrOffshoreSystem_::Add(std::shared_ptr<FrObject> newItem) {
        assert(std::dynamic_pointer_cast<FrBody_>(newItem) ||
               std::dynamic_pointer_cast<FrLink_>(newItem) ||
               std::dynamic_pointer_cast<FrPhysicsItem_>(newItem));

        if (auto item = std::dynamic_pointer_cast<FrBody_>(newItem)) {
            AddBody(item);
            return;
        }

        if (auto item = std::dynamic_pointer_cast<FrLink_>(newItem)) {
            AddLink(item);
            return;
        }

        if (auto item = std::dynamic_pointer_cast<FrPrePhysicsItem_>(newItem)) {
            AddPhysicsItem(item);
            return;
        }

        if (auto item = std::dynamic_pointer_cast<FrMidPhysicsItem_>(newItem)) {
            AddPhysicsItem(item);
            return;
        }

        if (auto item = std::dynamic_pointer_cast<FrPostPhysicsItem_>(newItem)) {
            AddPhysicsItem(item);
            return;
        }

    }

    void FrOffshoreSystem_::AddBody(std::shared_ptr<FrBody_> body) {

        if (!CheckBodyContactMethod(body)) { // TODO : voir si on set pas d'autorite le mode de contact a celui du systeme plutot que de faire un if...
            body->SetContactMethod(m_systemType);
        }

        m_chronoSystem->AddBody(body->GetChronoBody());  // Authorized because this method is a friend of FrBody_
        m_bodyList.push_back(body);

        body->m_system = this;

    }


    void FrOffshoreSystem_::AddLink(std::shared_ptr<FrLinkBase_> link) {
        m_chronoSystem->AddLink(link->GetChronoLink());
        m_linkList.push_back(link);
    }


//
//    void FrOffshoreSystem_::AddOtherPhysics(std::shared_ptr<FrOtherPhysics_> otherPhysics) {
//        m_chronoSystem->AddOtherPhysicsItem(otherPhysics->GetChronoOtherPhysics());
//        m_otherPhysicsList.push_back(otherPhysics);
//    }


    void FrOffshoreSystem_::AddPhysicsItem(std::shared_ptr<FrPrePhysicsItem_> otherPhysics) {
        m_chronoSystem->AddOtherPhysicsItem(otherPhysics->GetChronoPhysicsItem());
        otherPhysics->m_system = this;
        m_PrePhysicsList.push_back(otherPhysics);
    }

    void FrOffshoreSystem_::AddPhysicsItem(std::shared_ptr<FrMidPhysicsItem_> otherPhysics) {
        m_chronoSystem->AddOtherPhysicsItem(otherPhysics->GetChronoPhysicsItem());
        otherPhysics->m_system = this;
        m_MidPhysicsList.push_back(otherPhysics);
    }

    void FrOffshoreSystem_::AddPhysicsItem(std::shared_ptr<FrPostPhysicsItem_> otherPhysics) {
        m_chronoSystem->AddOtherPhysicsItem(otherPhysics->GetChronoPhysicsItem());
        otherPhysics->m_system = this;
        m_PostPhysicsList.push_back(otherPhysics);
    }

    FrEnvironment_ *FrOffshoreSystem_::GetEnvironment() const {
        return m_environment.get();
    }

    std::shared_ptr<FrBody_> FrOffshoreSystem_::GetWorldBody() const {
        return m_worldBody;
    }

    void FrOffshoreSystem_::PreUpdate() {
        // TODO : voir si on ne met pas l'environnement comme un physics Item update en tant que PrePhysicsItem
        m_environment->Update(m_chronoSystem->GetChTime());
    }

    void FrOffshoreSystem_::PostUpdate() {
        // TODO
    }

    void FrOffshoreSystem_::PrePhysicsUpdate(double time, bool update_assets) {
        for (auto& item : m_PrePhysicsList) {
            item->Update(time);
        }
    }

    void FrOffshoreSystem_::MidPhysicsUpdate(double time, bool update_assets) {
        for (auto& item : m_MidPhysicsList) {
            item->Update(time);
        }
    }

    void FrOffshoreSystem_::PostPhysicsUpdate(double time, bool update_assets) {
        for (auto& item : m_PostPhysicsList) {
            item->Update(time);
        }
    }


    void FrOffshoreSystem_::Initialize() {

        // Initializing environment before bodies
        m_environment->Initialize();

//        for (auto& item : m_PrePhysicsList) {
//            item->SetupInitial();
//        }
//
//        for (auto& item : m_bodyList){
//            item->SetupInitial();
//        }
//
//        for (auto& item : m_MidPhysicsList) {
//            item->SetupInitial();
//        }
//
//        for (auto& item : m_linkList) {
//            item->SetupInitial();
//        }
//
//        for (auto& item : m_PostPhysicsList) {
//            item->SetupInitial();
//        }

//        m_chronoSystem->Update();

        m_chronoSystem->SetupInitial();


//        // Initializing embedded chrono system
//        m_chronoSystem->SetupInitial(); // Actually do nothing but called for consistency

//        // Initializing bodies
//        auto bodyIter = body_begin();
//        for (; bodyIter != body_end(); bodyIter++) {
//            (*bodyIter)->Initialize();
//        }
//
//        // Initializing links
//        auto linkIter = link_begin();
//        for (; linkIter != link_end(); linkIter++) {
//            (*linkIter)->Initialize();
//        }

        // TODO (pour la radiation notamment)
//        // Initializing otherPhysics
//        auto otherPhysicsIter = otherphysics_begin();
//        for (; otherPhysicsIter != otherphysics_end(); otherPhysicsIter++) {
//            (*otherPhysicsIter)->Initialize();
//        }


        m_isInitialized = true;

    }

    void FrOffshoreSystem_::StepFinalize() {
        m_environment->StepFinalize();

        for (auto& body : m_bodyList) {
            body->StepFinalize();
        }

        for (auto& link : m_linkList) {
            link->StepFinalize();
        }

        // TODO : faire aussi pour les physicsItems !

    }

    void FrOffshoreSystem_::SetSystemType(SYSTEM_TYPE type, bool checkCompat) {

        if (m_chronoSystem) Clear(); // Clear the system from every bodies etc...

        // Creating the chrono System backend. It drives the way contact are modelled
        switch (type) {
            case SMOOTH_CONTACT:
                m_chronoSystem = std::make_unique<_FrSystemBaseSMC>(this);
                break;
            case NONSMOOTH_CONTACT:
                std::cout << "NSC systems is not tested for now !!!!" << std::endl;
                m_chronoSystem = std::make_unique<_FrSystemBaseNSC>();
                break;
        }

        m_systemType = type;

        if (checkCompat) CheckCompatibility();
    }

    void FrOffshoreSystem_::CheckCompatibility() const {
        // TODO : verifier la compatibilite entre type systeme, solveur et integrateur temporel



    }

    bool FrOffshoreSystem_::CheckBodyContactMethod(std::shared_ptr<FrBody_> body) {
        return m_systemType == body->GetContactType();
    }

    void FrOffshoreSystem_::SetSolver(SOLVER solver, bool checkCompat) {

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

    void FrOffshoreSystem_::SetSolverWarmStarting(bool useWarm) {
        m_chronoSystem->SetSolverWarmStarting(useWarm);
    }

    void FrOffshoreSystem_::SetSolverOverrelaxationParam(double omega) {
        m_chronoSystem->SetSolverOverrelaxationParam(omega);
    }

    void FrOffshoreSystem_::SetSolverSharpnessParam(double momega) {
        m_chronoSystem->SetSolverSharpnessParam(momega);
    }

    void FrOffshoreSystem_::SetParallelThreadNumber(int nbThreads) {
        m_chronoSystem->SetParallelThreadNumber(nbThreads);
    }

    void FrOffshoreSystem_::SetSolverMaxIterSpeed(int maxIter) {
        m_chronoSystem->SetMaxItersSolverSpeed(maxIter);
    }

    void FrOffshoreSystem_::SetSolverMaxIterStab(int maxIter) {
        m_chronoSystem->SetMaxItersSolverStab(maxIter);
    }

    void FrOffshoreSystem_::SetSolverMaxIterAssembly(int maxIter) {
        m_chronoSystem->SetMaxiter(maxIter);
    }

    void FrOffshoreSystem_::SetSolverGeometricTolerance(double tol) {
        m_chronoSystem->SetTol(tol);
    }

    void FrOffshoreSystem_::SetSolverForceTolerance(double tol) {
        m_chronoSystem->SetTolForce(tol);
    }

    void FrOffshoreSystem_::UseMaterialProperties(bool use) {
        if (m_systemType == SMOOTH_CONTACT) {
            dynamic_cast<chrono::ChSystemSMC*>(m_chronoSystem.get())->UseMaterialProperties(use);
        } else {
            std::cerr << "The use of material properties is only for SMOOTH_CONTACT systems" << std::endl;
        }
    }

    void FrOffshoreSystem_::SetContactForceModel(FrOffshoreSystem_::CONTACT_MODEL model) {
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

    void FrOffshoreSystem_::SetAdhesionForceModel(FrOffshoreSystem_::ADHESION_MODEL model) {
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

    void FrOffshoreSystem_::SetTangentialDisplacementModel(FrOffshoreSystem_::TANGENTIAL_DISP_MODEL model) {
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

    void FrOffshoreSystem_::SetStiffContact(bool isStiff) {
        if (m_systemType == SMOOTH_CONTACT) {
            dynamic_cast<chrono::ChSystemSMC*>(m_chronoSystem.get())->SetStiffContact(isStiff);
        } else {
            std::cerr << "StiffContact is only for SMOOTH_CONTACT systems" << std::endl;
        }
    }

    void FrOffshoreSystem_::SetSlipVelocityThreshold(double velocity) {
        if (m_systemType == SMOOTH_CONTACT) {
            dynamic_cast<chrono::ChSystemSMC*>(m_chronoSystem.get())->SetSlipVelocitythreshold(velocity);
        } else {
            std::cerr << "Slip Velocity Threshold is only for SMOOTH_CONTACT systems" << std::endl;
        }
    }

    void FrOffshoreSystem_::SetCharacteristicImpactVelocity(double velocity) {
        if (m_systemType == SMOOTH_CONTACT) {
            dynamic_cast<chrono::ChSystemSMC*>(m_chronoSystem.get())->SetCharacteristicImpactVelocity(velocity);
        } else {
            std::cerr << "Characteristic Impact Velocity is only for SMOOTH_CONTACT systems" << std::endl;
        }
    }

    void FrOffshoreSystem_::SetMinBounceSpeed(double speed) {
        m_chronoSystem->SetMinBounceSpeed(speed);
    }

    void FrOffshoreSystem_::SetMaxPenetrationRecoverySpeed(double speed) {
        m_chronoSystem->SetMaxPenetrationRecoverySpeed(speed);
    }

    int FrOffshoreSystem_::GetNbPositionCoords() const {
        return m_chronoSystem->GetNcoords();
    }

    int FrOffshoreSystem_::GetNbVelocityCoords() const {
        return m_chronoSystem->GetNcoords_w();
    }

    int FrOffshoreSystem_::GetNbConstraintsCoords() const {
        return m_chronoSystem->GetNdoc_w();
    }

    int FrOffshoreSystem_::GetNbDOF() const {
        return m_chronoSystem->GetNdof();
    }

    int FrOffshoreSystem_::GetNbBodies() const {
        return m_chronoSystem->GetNbodies();
    }

    int FrOffshoreSystem_::GetNbFixedBodies() const {
        return m_chronoSystem->GetNbodiesFixed();
    }

    int FrOffshoreSystem_::GetNbSleepingBodies() const {
        return m_chronoSystem->GetNbodiesSleeping();
    }

    void FrOffshoreSystem_::SetUseSleepingBodies(bool useSleeping) {
        m_chronoSystem->SetUseSleeping(useSleeping);
    }

    double FrOffshoreSystem_::GetGravityAcceleration() const {
        return fabs(m_chronoSystem->Get_G_acc()[2]);
    }

    void FrOffshoreSystem_::SetGravityAcceleration(double gravityAcceleration) {
        m_chronoSystem->Set_G_acc(chrono::ChVector<double>(0., 0., -gravityAcceleration));
    }

    void FrOffshoreSystem_::SetNbStepsStatics(int nSteps) {
        m_nbStepStatics = nSteps;
    }

    bool FrOffshoreSystem_::SolveStaticEquilibrium(FrOffshoreSystem_::STATICS_METHOD method) {
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

    void FrOffshoreSystem_::SetTimeStepper(TIME_STEPPER type, bool checkCompat) {

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

    void FrOffshoreSystem_::SetTimeStep(double timeStep) {
        m_chronoSystem->SetStep(timeStep);
    }

    double FrOffshoreSystem_::GetTimeStep() const {
        return m_chronoSystem->GetStep();
    }

    void FrOffshoreSystem_::SetMinTimeStep(double minTimeStep) {
        m_chronoSystem->SetStepMin(minTimeStep);
    }

    void FrOffshoreSystem_::SetMaxTimeStep(double maxTimeStep) {
        m_chronoSystem->SetStepMax(maxTimeStep);
    }

    double FrOffshoreSystem_::GetTime() const {
        return m_chronoSystem->GetChTime();
    }

    bool FrOffshoreSystem_::AdvanceOneStep(double stepSize) {
        CheckIsInitialized();
        return (bool)m_chronoSystem->DoStepDynamics(stepSize);
    }

    bool FrOffshoreSystem_::AdvanceTo(double nextTime) {
        CheckIsInitialized();
        return m_chronoSystem->DoFrameDynamics(nextTime);
    }

    bool FrOffshoreSystem_::RunDynamics(double frameStep) {
        CheckIsInitialized();
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

    void FrOffshoreSystem_::CreateWorldBody() {
        m_worldBody = std::make_shared<FrBody_>();
        m_worldBody->SetFixedInWorld(true);
        m_worldBody->SetName("WorldBody");
        AddBody(m_worldBody);
    }

    std::shared_ptr<FrBody_> FrOffshoreSystem_::NewBody() {
        auto body = std::make_shared<FrBody_>();  // TODO : suivant le type de systeme SMC ou NSC, regler le type de surface...

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

    void FrOffshoreSystem_::Clear() {
        m_chronoSystem->Clear();

        m_bodyList.clear();
//        m_linkList.clear();
//        m_otherPhysicsList.clear();
    }

    chrono::ChSystem* FrOffshoreSystem_::GetChronoSystem() {
        return m_chronoSystem.get();
    }


    // Irrlicht visualization

    void FrOffshoreSystem_::RunInViewer(double endTime, double dist, bool recordVideo) {

        CheckIsInitialized();

        FrIrrApp_ app(m_chronoSystem.get(), dist);

        app.SetTimestep(m_chronoSystem->GetStep());
        app.SetVideoframeSave(recordVideo);
        app.Run(endTime);

    }

    void FrOffshoreSystem_::Visualize( double dist, bool recordVideo) {

        Initialize();  // So that system is automatically initialized when run in viewer mode

        FrIrrApp_ app(m_chronoSystem.get(), dist);

        app.SetTimestep(m_chronoSystem->GetStep());
        app.SetVideoframeSave(recordVideo);
        app.Visualize();

    }

    void FrOffshoreSystem_::AddAsset(std::shared_ptr<chrono::ChAsset> asset) {
        m_chronoSystem->AddAsset(std::move(asset));
    }

    void FrOffshoreSystem_::CheckIsInitialized() {
        if (!m_isInitialized) Initialize();
    }


    // Iterators

    FrOffshoreSystem_::BodyIter FrOffshoreSystem_::body_begin() {
        return m_bodyList.begin();
    }

    FrOffshoreSystem_::ConstBodyIter FrOffshoreSystem_::body_begin() const {
        return m_bodyList.cbegin();
    }

    FrOffshoreSystem_::BodyIter FrOffshoreSystem_::body_end() {
        return m_bodyList.end();
    }

    FrOffshoreSystem_::ConstBodyIter FrOffshoreSystem_::body_end() const {
        return m_bodyList.cend();
    }

    FrOffshoreSystem_::LinkIter FrOffshoreSystem_::link_begin() {
        return m_linkList.begin();
    }

    FrOffshoreSystem_::ConstLinkIter FrOffshoreSystem_::link_begin() const {
        return m_linkList.cbegin();
    }

    FrOffshoreSystem_::LinkIter FrOffshoreSystem_::link_end() {
        return m_linkList.end();
    }

    FrOffshoreSystem_::ConstLinkIter FrOffshoreSystem_::link_end() const {
        return m_linkList.cend();
    }


}  // end namespace frydom
