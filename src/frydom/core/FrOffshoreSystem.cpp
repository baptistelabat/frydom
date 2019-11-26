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
#include "chrono/solver/ChIterativeSolver.h"

#include "frydom/core/link/links_lib/FrLink.h"
#include "frydom/core/body/FrBody.h"
#include "frydom/core/common/FrFEAMesh.h"
#include "frydom/cable/FrDynamicCable.h"
#include "frydom/core/force/FrForce.h"
#include "frydom/environment/FrEnvironment.h"
#include "frydom/utils/FrIrrApp.h"
#include "frydom/core/statics/FrStaticAnalysis.h"

#include "frydom/core/math/functions/ramp/FrCosRampFunction.h"

#include "frydom/logging/FrLogManager.h"
#include "frydom/logging/FrPathManager.h"

#include "frydom/logging/FrTypeNames.h"

#include "frydom/logging/FrEventLogger.h"
#include "frydom/logging/FrSerializerFactory.h"

namespace frydom {


  namespace internal {

    FrSystemBaseSMC::FrSystemBaseSMC(frydom::FrOffshoreSystem *offshoreSystem) :
        chrono::ChSystemSMC(),
        m_offshoreSystem(offshoreSystem) {}

    void FrSystemBaseSMC::Update(bool update_assets) {

      // Note : there is no ChAssembly::Update() as it is better expanded here...

      CH_PROFILE("Update");

      timer_update.start();  // Timer for profiling

      // Pre updates that are not about multibody dynamics
      m_offshoreSystem->PreUpdate();

      // Executes the "forUpdate" in all controls of controlslist
      ExecuteControlsForUpdate();

      // Physics item that have to be updated before all
      m_offshoreSystem->PrePhysicsUpdate(ChTime, update_assets);

      // Bodies updates  // FIXME : appeler les updates directement des objets frydom !
      for (auto &body : bodylist) {
        body->Update(ChTime, update_assets);
//            body->Update(ChTime, update_assets);  // FIXME : Appel redondant
      }

      // Links updates  // FIXME : appeler les updates directement des objets frydom !
      for (auto &link : linklist) {
        link->Update(ChTime, update_assets);
      }

      for (auto &mesh : meshlist) {
        mesh->Update(ChTime, update_assets);
      }

      // Update all contacts, if any
      contact_container->Update(ChTime, update_assets);

      // Post updates that are not about multibody dynamics
      m_offshoreSystem->PostUpdate();

      timer_update.stop();

    }

//    void FrSystemBaseSMC::CustomEndOfStep() {
//      m_offshoreSystem->StepFinalize();
//    }

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
      for (auto &mesh : meshlist) {
        mesh->SetNoSpeedNoAcceleration();
      }
      for (auto &ip : otherphysicslist) {
        ip->SetNoSpeedNoAcceleration();
      }
      return true;
    }

//    int FrSystemBaseSMC::DoStepDynamics(double m_step) {
//      chrono::ChSystem::DoStepDynamics(m_step);
//      m_offshoreSystem->StepFinalize();
//    }

    bool FrSystemBaseSMC::Integrate_Y() {
      chrono::ChSystem::Integrate_Y();
      m_offshoreSystem->StepFinalize();
    }

  }  // end namespace frydom::internal



  /// Default constructor
  /// \param systemType contact method system (SMOOTH_CONTACT/NONSMOOTH_CONTACT)
  /// \param timeStepper time stepper type
  /// \param solver solver type
  FrOffshoreSystem::FrOffshoreSystem(const std::string &name,
                                     SYSTEM_TYPE systemType,
                                     TIME_STEPPER timeStepper,
                                     SOLVER solver) :
      FrLoggable(name, TypeToString(this), nullptr),
      m_monitor_real_time(false),
      m_config_file() {

    // Creating the chrono System backend. It drives the way contact are modelled
    SetSystemType(systemType, false);

    // Creating the log manager service
    m_LogManager = std::make_unique<FrLogManager>(this);

    // Setting the time stepper
    SetTimeStepper(timeStepper, false);

    // Setting the constraints solver
    SetSolver(solver, false);


    // Check compatibility between system contact model, time stepper and constraint solver
    CheckCompatibility();

    // Creating the environment
    m_environment = std::make_unique<FrEnvironment>(this);

    // Creating the path manager service
    m_pathManager = std::make_unique<FrPathManager>();
    m_pathManager->RegisterTreeNode(this);

//    // Creating the log manager service
//    m_LogManager = std::make_unique<FrLogManager>(this);

    // Creating the static analysis
    m_statics = std::make_unique<FrStaticAnalysis>(this);

    // Creating a fixed world body to be able to attach anything to it (anchors...)
    CreateWorldBody();
  }

  FrOffshoreSystem::~FrOffshoreSystem() = default;


//  void FrOffshoreSystem::Add(std::shared_ptr<FrObject> newItem) {
//    assert(std::dynamic_pointer_cast<FrBody>(newItem) ||
//           std::dynamic_pointer_cast<FrLinkBase>(newItem) ||
//           std::dynamic_pointer_cast<FrPhysicsItem>(newItem));
//
//    if (auto item = std::dynamic_pointer_cast<FrBody>(newItem)) {
//      AddBody(item);
//      return;
//    }
//
//    if (auto item = std::dynamic_pointer_cast<FrLinkBase>(newItem)) {
//      AddLink(item);
//      return;
//    }
//
//    if (auto item = std::dynamic_pointer_cast<FrPrePhysicsItem>(newItem)) {
//      AddPhysicsItem(item);
//      return;
//    }
//
//  }
  const FrConfig& FrOffshoreSystem::config_file() {
    return m_config_file;
  }

// ***** Body *****

  void FrOffshoreSystem::AddBody(std::shared_ptr<FrBody> body, std::shared_ptr<internal::FrBodyBase> chrono_body) {

    // TODO : voir si on set pas d'autorite le mode de contact a celui du systeme plutot que de faire un if...
    if (!CheckBodyContactMethod(body)) {
      body->SetContactMethod(m_systemType);
    }

    m_chronoSystem->AddBody(chrono_body);  // Authorized because this method is a friend of FrBody
    m_bodyList.push_back(body);

    event_logger::info(GetTypeName(), GetName(),
                       "Body {} has been ADDED to the system", body->GetName());
  }

  FrOffshoreSystem::BodyContainer &FrOffshoreSystem::GetBodyList() {
    return m_bodyList;
  }

  void FrOffshoreSystem::RemoveBody(std::shared_ptr<FrBody> body, std::shared_ptr<internal::FrBodyBase> chrono_body) {

    m_chronoSystem->RemoveBody(chrono_body);

    auto it = std::find(body_begin(), body_end(), body);
    assert(it != body_end());
    m_bodyList.erase(it);

    body->RemoveAllForces();
    body->RemoveAllNodes();

    event_logger::info(GetTypeName(), GetName(), "Body {} has been REMOVED from the system", body->GetName());

    // FIXME : we should launch removal of FrNode and FrForce objects attached to this body from the logManager...

  }

  void FrOffshoreSystem::RemoveAllBodies() {

    for (auto &body: m_bodyList)
      Remove(body);

    event_logger::info(GetTypeName(), GetName(), "Every bodies have been removed from the system");

  }


// ***** Link *****

  void FrOffshoreSystem::AddLink(std::shared_ptr<FrLinkBase> link, std::shared_ptr<chrono::ChLink> chrono_link) {
    m_chronoSystem->AddLink(chrono_link);
    m_linkList.push_back(link);
    event_logger::info(GetTypeName(), GetName(), "Link {} has been ADDED to the system", link->GetName());
  }

  void FrOffshoreSystem::RemoveLink(std::shared_ptr<FrLinkBase> link, std::shared_ptr<chrono::ChLink> chrono_link) {

    m_chronoSystem->RemoveLink(chrono_link);

    auto it = std::find(link_begin(), link_end(), link);
    assert(it != link_end());
    if (it == link_end()) {
      event_logger::error(GetTypeName(), GetName(),
          "Fail to remove link {} as it is not registered", link->GetName());
      return;
    }

    m_linkList.erase(it);
    event_logger::info(GetTypeName(), GetName(), "Link {} has been REMOVED from the system", link->GetName());
  }

  void FrOffshoreSystem::RemoveAllLinks() {

    for (auto &link: m_linkList)
      Remove(link);
  }


// ***** Physics Item *****

  void FrOffshoreSystem::AddPhysicsItem(std::shared_ptr<FrPrePhysicsItem> otherPhysics,
                                        std::shared_ptr<internal::FrPhysicsItemBase> chrono_physics_item) {

    m_chronoSystem->AddOtherPhysicsItem(chrono_physics_item);
    m_PrePhysicsList.push_back(otherPhysics);
    event_logger::info(GetTypeName(), GetName(), "A Physics Item has been ADDED to the system");
  }

  FrOffshoreSystem::PrePhysicsContainer FrOffshoreSystem::GetPrePhysicsItemList() {
    return m_PrePhysicsList;
  }

  void FrOffshoreSystem::RemovePhysicsItem(std::shared_ptr<FrPhysicsItem> item,
                                           std::shared_ptr<internal::FrPhysicsItemBase> chrono_physics_item) {

    m_chronoSystem->RemoveOtherPhysicsItem(chrono_physics_item);

    auto it = std::find(m_PrePhysicsList.begin(), m_PrePhysicsList.end(), item);
    if (it != m_PrePhysicsList.end())
      m_PrePhysicsList.erase(it);
    event_logger::info(GetTypeName(), GetName(), "A Physics Item has been REMOVED to the system");
  }

//  void FrOffshoreSystem::RemoveAllPhysicsItem() {
//
//    for (auto &item: m_PrePhysicsList)
//      Remove(item);
//
//  }


// ***** FEAMesh *****

  void FrOffshoreSystem::AddFEAMesh(std::shared_ptr<FrFEAMesh> feaMesh,
                                    std::shared_ptr<chrono::fea::ChMesh> chrono_mesh) {

    m_chronoSystem->AddMesh(chrono_mesh);  // Authorized because this method is a friend of FrFEAMesh

//      feaMesh->m_system = this;
    m_feaMeshList.push_back(feaMesh);
  }

  void FrOffshoreSystem::AddDynamicCable(std::shared_ptr<FrDynamicCable> cable,
                                         std::shared_ptr<chrono::fea::ChMesh> chrono_mesh) {

    // Add the FEA mesh
    AddFEAMesh(cable, chrono_mesh);

    // Add the hinges
    m_chronoSystem->Add(dynamic_cast<internal::FrDynamicCableBase *>(chrono_mesh.get())->m_startingHinge);
    m_chronoSystem->Add(dynamic_cast<internal::FrDynamicCableBase *>(chrono_mesh.get())->m_endingHinge);

  }

  FrOffshoreSystem::FEAMeshContainer FrOffshoreSystem::GetFEAMeshList() {
    return m_feaMeshList;
  }

  void FrOffshoreSystem::RemoveFEAMesh(std::shared_ptr<FrFEAMesh> feamesh,
                                       std::shared_ptr<chrono::fea::ChMesh> chrono_mesh) {

    m_chronoSystem->RemoveMesh(chrono_mesh);

    auto it = std::find(m_feaMeshList.begin(), m_feaMeshList.end(), feamesh);
    assert(it != m_feaMeshList.end());
    m_feaMeshList.erase(it);
//      feamesh->m_system = nullptr;

  }

  void FrOffshoreSystem::RemoveDynamicCable(std::shared_ptr<FrDynamicCable> cable,
                                            std::shared_ptr<chrono::fea::ChMesh> chrono_mesh) {

    Remove(cable);

    m_chronoSystem->RemoveOtherPhysicsItem(
        dynamic_cast<internal::FrDynamicCableBase *>(chrono_mesh.get())->m_startingHinge);
    m_chronoSystem->RemoveOtherPhysicsItem(
        dynamic_cast<internal::FrDynamicCableBase *>(chrono_mesh.get())->m_endingHinge);

  }

  void FrOffshoreSystem::MonitorRealTimePerfs(bool val) {
    m_monitor_real_time = val;
    event_logger::info(GetTypeName(), GetName(), "Monitoring time performance set to {}", val);
  }


// ***** Environment *****

  FrEnvironment *FrOffshoreSystem::GetEnvironment() const {
    return m_environment.get();
  }

  std::shared_ptr<FrBody> FrOffshoreSystem::GetWorldBody() const {
    return m_worldBody;
  }

  std::shared_ptr<FrNode> FrOffshoreSystem::NewWorldFixedNode(const std::string &name) {
    return m_worldBody->NewNode(name);
  }

  void FrOffshoreSystem::PreUpdate() {
    // TODO : voir si on ne met pas l'environnement comme un physics Item update en tant que PrePhysicsItem
    m_environment->Update(m_chronoSystem->GetChTime());
  }

  void FrOffshoreSystem::PostUpdate() {
    // TODO
  }

  void FrOffshoreSystem::PrePhysicsUpdate(double time, bool update_assets) {
    for (auto &item : m_PrePhysicsList) {
      item->Update(time);
    }
  }

  void FrOffshoreSystem::Initialize() {

    if (m_isInitialized)
      return;

    event_logger::info(GetTypeName(), GetName(), "BEGIN OffshoreSystem initialization");
    event_logger::flush();

    // Initializing environment before bodies
    m_environment->Initialize();

    for (auto &item : m_PrePhysicsList) {
      item->Initialize();
    }

    for (auto &item : m_bodyList) {
      item->Initialize();
    }

    for (auto &item : m_linkList) {
      item->Initialize();
    }

    for (auto &item : m_feaMeshList) {
      item->Initialize();
    }

    m_chronoSystem->Update();

    m_LogManager->Initialize();

    m_isInitialized = true;

    event_logger::info(GetTypeName(), GetName(), "END OffshoreSystem initialization");
    event_logger::flush();

  }

  void FrOffshoreSystem::ForceInitialize() {
    m_isInitialized = false;
    Initialize();
  }

  void FrOffshoreSystem::DefineLogMessages() {  // FIXME : doit etre appele par logManager !!!

    // Solver related messages

    auto msg = NewMessage("Solver_message", "Messages relative to the dynamic solver and constraint solvers");

    msg->AddField<double>("time", "s", "Current time of the simulation", [this]() { return GetTime(); });

    msg->AddField<int>("iter", "", "number of total iterations taken by the solver", [this]() {
      return dynamic_cast<chrono::ChIterativeSolver *>(m_chronoSystem->GetSolver().get())->GetTotalIterations();
    });

    if (dynamic_cast<chrono::ChIterativeSolver *>(m_chronoSystem->GetSolver().get())->GetRecordViolation()) {

      msg->AddField<double>("violationResidual", "", "constraint violation", [this]() {
        return dynamic_cast<chrono::ChIterativeSolver *>(m_chronoSystem->GetSolver().get())->GetViolationHistory().back();
      });

      msg->AddField<double>("LagrangeResidual", "", "maximum change in Lagrange multipliers", [this]() {
        return dynamic_cast<chrono::ChIterativeSolver *>(m_chronoSystem->GetSolver().get())->GetDeltalambdaHistory().back();
      });

    }

  }

  void FrOffshoreSystem::StepFinalize() {

    m_environment->StepFinalize();

    for (auto &item : m_PrePhysicsList) {
      item->StepFinalize();
    }

    for (auto &item : m_bodyList) {
      item->StepFinalize();
    }

    for (auto &item : m_linkList) {
      item->StepFinalize();
    }

    for (auto &item : m_feaMeshList) {
      item->StepFinalize();
    }

    // Logging
    m_LogManager->StepFinalize();

    if (m_monitor_real_time) {
      double ratio = m_chronoSystem->GetTimerStep() / GetTimeStep();

      std::string msg = (ratio > 1.) ? "slower that real time" : "faster than real time";

      std::cout << "At time : "
                << GetTime()
                << ";\t"
                << msg
                << "(1 physical second computed in "
                << ratio
                << " seconds)"
                << std::endl;
    }

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
      dynamic_cast<chrono::ChSystemSMC *>(m_chronoSystem.get())->UseMaterialProperties(use);
    } else {
      std::cerr << "The use of material properties is only for SMOOTH_CONTACT systems" << std::endl;
    }
  }

  void FrOffshoreSystem::SetContactForceModel(FrOffshoreSystem::CONTACT_MODEL model) {
    if (m_systemType == SMOOTH_CONTACT) {
      auto systemSMC = dynamic_cast<chrono::ChSystemSMC *>(m_chronoSystem.get());
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
      auto systemSMC = dynamic_cast<chrono::ChSystemSMC *>(m_chronoSystem.get());
      using AdhesionForceModel = chrono::ChSystemSMC::AdhesionForceModel;
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
      auto systemSMC = dynamic_cast<chrono::ChSystemSMC *>(m_chronoSystem.get());
      using TangentialDisplacementModel = chrono::ChSystemSMC::TangentialDisplacementModel;
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
      dynamic_cast<chrono::ChSystemSMC *>(m_chronoSystem.get())->SetStiffContact(isStiff);
      event_logger::info(GetTypeName(), GetName(),
          "Stiff contact {}", (isStiff) ? "activated" : "deactivated");
    } else {
      event_logger::error(GetTypeName(), GetName(),
                          "StiffContact is only for SMOOTH_CONTACT systems. Action ignored");
    }
  }

  void FrOffshoreSystem::SetSlipVelocityThreshold(double velocity) {
    if (m_systemType == SMOOTH_CONTACT) {
      dynamic_cast<chrono::ChSystemSMC *>(m_chronoSystem.get())->SetSlipVelocityThreshold(velocity);
      event_logger::info(GetTypeName(), GetName(),
                         "Slip Velocity Threshold set to {} m/s", velocity);
    } else {
      event_logger::error(GetTypeName(), GetName(),
                          "Slip Velocity Threshold is only for SMOOTH_CONTACT systems. Action ignored");
    }
  }

  void FrOffshoreSystem::SetCharacteristicImpactVelocity(double velocity) {
    if (m_systemType == SMOOTH_CONTACT) {
      dynamic_cast<chrono::ChSystemSMC *>(m_chronoSystem.get())->SetCharacteristicImpactVelocity(velocity);
      event_logger::info(GetTypeName(), GetName(),
                          "Characteristic Impact Velocity set to {} m/s", velocity);
    } else {
      event_logger::error(GetTypeName(), GetName(),
          "Characteristic Impact Velocity is only for SMOOTH_CONTACT systems. Action ignored");
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
    event_logger::info(GetTypeName(), GetName(),
                       "Gravity acceleration set to {} m/s2", gravityAcceleration);
    m_chronoSystem->Set_G_acc(chrono::ChVector<double>(0., 0., -gravityAcceleration));
  }

  bool FrOffshoreSystem::DoAssembly() {
    event_logger::info(GetTypeName(), GetName(), "Solving assembly");
    return m_chronoSystem->DoFullAssembly();
  }

  FrStaticAnalysis *FrOffshoreSystem::GetStaticAnalysis() const {
    return m_statics.get();
  }

  bool FrOffshoreSystem::SolveStaticWithRelaxation() {

    event_logger::info(GetTypeName(), GetName(), "Static analysis by dynamic relaxation STARTED");
    Initialize();
    return m_statics->SolveStatic();
  }

  void FrOffshoreSystem::Relax(FrStaticAnalysis::RELAXTYPE relax) {

    for (auto &body:m_bodyList) {
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

  void FrOffshoreSystem::SetTimeStepper(TIME_STEPPER type, bool check_compatibility) {

    using timeStepperType = chrono::ChTimestepper::Type;

    switch (type) {
      case EULER_IMPLICIT_LINEARIZED:
        m_chronoSystem->SetTimestepperType(timeStepperType::EULER_IMPLICIT_LINEARIZED);
        event_logger::info(GetTypeName(), GetName(), "Time stepper set to EULER_IMPLICIT_LINEARIZED");
        break;
      case EULER_IMPLICIT_PROJECTED:
        m_chronoSystem->SetTimestepperType(timeStepperType::EULER_IMPLICIT_PROJECTED);
        event_logger::info(GetTypeName(), GetName(), "Time stepper set to EULER_IMPLICIT_PROJECTED");
        break;
      case EULER_IMPLICIT:
        m_chronoSystem->SetTimestepperType(timeStepperType::EULER_IMPLICIT);
        event_logger::info(GetTypeName(), GetName(), "Time stepper set to EULER_IMPLICIT");
        break;
      case TRAPEZOIDAL:
        m_chronoSystem->SetTimestepperType(timeStepperType::TRAPEZOIDAL);
        event_logger::info(GetTypeName(), GetName(), "Time stepper set to TRAPEZOIDAL");
        break;
      case TRAPEZOIDAL_LINEARIZED:
        m_chronoSystem->SetTimestepperType(timeStepperType::TRAPEZOIDAL_LINEARIZED);
        event_logger::info(GetTypeName(), GetName(), "Time stepper set to TRAPEZOIDAL_LINEARIZED");
        break;
      case HHT:
        m_chronoSystem->SetTimestepperType(timeStepperType::HHT);
        event_logger::info(GetTypeName(), GetName(), "Time stepper set to HHT");
        break;
      case RUNGEKUTTA45:
        m_chronoSystem->SetTimestepperType(timeStepperType::RUNGEKUTTA45);
        event_logger::info(GetTypeName(), GetName(), "Time stepper set to RUNGEKUTTA45");
        break;
      case EULER_EXPLICIT:
        m_chronoSystem->SetTimestepperType(timeStepperType::EULER_EXPLICIT);
        event_logger::info(GetTypeName(), GetName(), "Time stepper set to EULER_EXPLICIT");
        break;
      case NEWMARK:
        m_chronoSystem->SetTimestepperType(timeStepperType::NEWMARK);
        event_logger::info(GetTypeName(), GetName(), "Time stepper set to NEWMARK");
        break;
    }

    m_timeStepper = type;

    if (check_compatibility) CheckCompatibility();

  }

  void FrOffshoreSystem::SetTimeStepper(TIME_STEPPER type) {
    SetTimeStepper(type, true);
  }

  void FrOffshoreSystem::SetTimeStep(double timeStep) {
    m_chronoSystem->SetStep(timeStep);
    event_logger::info(GetTypeName(), GetName(), "Time step set to {} s", timeStep);
  }

  double FrOffshoreSystem::GetTimeStep() const {
    return m_chronoSystem->GetStep();
  }

  void FrOffshoreSystem::SetMinTimeStep(double minTimeStep) {
    m_chronoSystem->SetStepMin(minTimeStep);
    event_logger::info(GetTypeName(), GetName(), "Set minimum time step to {} s", minTimeStep);
  }

  void FrOffshoreSystem::SetMaxTimeStep(double maxTimeStep) {
    m_chronoSystem->SetStepMax(maxTimeStep);
    event_logger::info(GetTypeName(), GetName(), "Set maximum time step to {} s", maxTimeStep);
  }

  double FrOffshoreSystem::GetTime() const {
    return m_chronoSystem->GetChTime();
  }

  void FrOffshoreSystem::SetTime(double time) {
    m_chronoSystem->SetChTime(time);
  }

  bool FrOffshoreSystem::AdvanceOneStep(double stepSize) {
    Initialize();
    return (bool) m_chronoSystem->DoStepDynamics(stepSize);
  }

  bool FrOffshoreSystem::AdvanceTo(double nextTime) {
    Initialize();
    return m_chronoSystem->DoFrameDynamics(nextTime);
  }

  bool FrOffshoreSystem::RunDynamics(double frameStep) {
    Initialize();
    event_logger::info(GetTypeName(), GetName(), "Dynamic simulation STARTED");
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
    m_worldBody = std::make_shared<FrBody>("world_body", this);
    m_worldBody->SetFixedInWorld(true);
//      m_worldBody->SetName("WorldBody");
//      m_worldBody->SetLogged(false);
    switch (m_systemType) {
      case SMOOTH_CONTACT:
        m_worldBody->SetSmoothContact();
        break;
      case NONSMOOTH_CONTACT:
        m_worldBody->SetNonSmoothContact();
        break;
    }
    Add(m_worldBody);
    m_worldBody->LogThis(false);  // No log for the world body
  }

  std::shared_ptr<FrBody> FrOffshoreSystem::NewBody(const std::string &name) {
    auto body = std::make_shared<FrBody>(name, this);
    // TODO : suivant le type de systeme SMC ou NSC, regler le type de surface...

    switch (m_systemType) {
      case SMOOTH_CONTACT:
        body->SetSmoothContact();
        break;
      case NONSMOOTH_CONTACT:
        body->SetNonSmoothContact();
        break;
    }

    Add(body);
    return body;
  }

  void FrOffshoreSystem::Clear() {
    m_chronoSystem->Clear();

    m_bodyList.clear();
    m_linkList.clear();
    m_feaMeshList.clear();
    m_PrePhysicsList.clear();

    m_isInitialized = false;
  }


// Irrlicht visualization

  FrIrrApp* FrOffshoreSystem::GetIrrApp() const {
    return m_irrApp.get();
  }

  void FrOffshoreSystem::RunInViewer(double endTime, double dist, bool recordVideo, int videoFrameSaveInterval) {

    // Initialization of the system if not already done.
    Initialize();

    // Definition and initialization of the Irrlicht application.
    // Definition and initialization of the Irrlicht application.
    m_irrApp = std::make_unique<FrIrrApp>(this, m_chronoSystem.get(), dist);

    m_irrApp->SetTimestep(m_chronoSystem->GetStep());
    m_irrApp->SetVideoframeSave(recordVideo);
    m_irrApp->SetVideoframeSaveInterval(videoFrameSaveInterval);

    event_logger::info(GetTypeName(), GetName(),
                       "Dynamic simulation STARTED in viewer with endTime = {} s, video recording set to {}",
                       endTime, recordVideo);

    m_irrApp->Run(endTime); // The temporal loop is here.

  }

  void FrOffshoreSystem::RunInViewer(double endTime, double dist, bool recordVideo) {
    RunInViewer(endTime, dist, recordVideo, 10);
  }

  void FrOffshoreSystem::RunInViewer(double endTime, double dist) {
    RunInViewer(endTime, dist, false);
  }

  void FrOffshoreSystem::RunInViewer(double endTime) {
    RunInViewer(endTime, 100);
  }

  void FrOffshoreSystem::RunInViewer() {
    RunInViewer(0);
  }

  void FrOffshoreSystem::Visualize(double dist, bool recordVideo) {

    Initialize();

    FrIrrApp app(this, m_chronoSystem.get(), dist);

    app.SetTimestep(m_chronoSystem->GetStep());
    app.SetVideoframeSave(recordVideo);
    app.Visualize();

  }

  void FrOffshoreSystem::Visualize(double dist) {
    Visualize(dist, false);
  }

  void FrOffshoreSystem::Visualize() {
    Visualize(100);
  }

  void FrOffshoreSystem::VisualizeStaticAnalysis(double dist, bool recordVideo) {

    Initialize();  // So that system is automatically initialized when run in viewer mode

    FrIrrApp app(this, m_chronoSystem.get(), dist);

    app.SetTimestep(m_chronoSystem->GetStep());
    app.SetVideoframeSave(recordVideo);
    app.VisualizeStaticAnalysis();

  }

  void FrOffshoreSystem::VisualizeStaticAnalysis(double dist) {
    VisualizeStaticAnalysis(dist, false);
  }

  void FrOffshoreSystem::VisualizeStaticAnalysis() {
    VisualizeStaticAnalysis(100);
  }

  void FrOffshoreSystem::AddAsset(std::shared_ptr<chrono::ChAsset> asset) {
    m_chronoSystem->AddAsset(std::move(asset));
  }

  FrLogManager *FrOffshoreSystem::GetLogManager() const {
    return m_LogManager.get();
  }

  FrPathManager *FrOffshoreSystem::GetPathManager() const {
    return m_pathManager.get();
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

//    void FrOffshoreSystem::InitializeLog_Dependencies(const std::string& systemPath) {
//
//        if (IsLogged()) {
//
//            // Initializing environment before bodies
////            m_environment->InitializeLog();
//
//            for (auto &item : m_PrePhysicsList) {
//                item->SetPathManager(GetPathManager());
//                item->InitializeLog(systemPath);
//            }
//
//            for (auto &item : m_bodyList) {
//                item->SetPathManager(GetPathManager());
//                item->InitializeLog(systemPath);
//            }
//
//            for (auto &item : m_linkList) {
//                item->SetPathManager(GetPathManager());
//                item->InitializeLog(systemPath);
//            }
//
//            for (auto &item : m_feaMeshList) {
//                item->SetPathManager(GetPathManager());
//                item->InitializeLog(systemPath);
//            }
//
//        }
//    }
//
//    void FrOffshoreSystem::ClearLogs() {
//
//        ClearMessage();
//
//        for (auto &item : m_PrePhysicsList) {
//            item->ClearMessage();
//        }
//
//        for (auto &item : m_bodyList) {
//            item->ClearMessage();
//            for (auto& force : item->GetForceList()) {
//                force->ClearMessage();
//            }
//            for (auto& node : item->GetNodeList()) {
//                node->ClearMessage();
//            }
//        }
//
//        for (auto &item : m_linkList) {
//            item->ClearMessage();
//        }
//
//    }
//
//    void FrOffshoreSystem::AddFields() {
//        m_message->AddField<double>("time", "s", "Current time of the simulation",
//                                    [this]() { return GetTime(); });
//
//        m_message->AddField<int>("iter", "", "number of total iterations taken by the solver", [this]() {
//            return dynamic_cast<chrono::ChIterativeSolver*>(m_chronoSystem->GetSolver().get())->GetTotalIterations();
//        });
//
//        if (dynamic_cast<chrono::ChIterativeSolver*>(m_chronoSystem->GetSolver().get())->GetRecordViolation()) {
//
//            m_message->AddField<double>("violationResidual", "", "constraint violation", [this]() {
//                return dynamic_cast<chrono::ChIterativeSolver *>(m_chronoSystem->GetSolver().get())->GetViolationHistory().back();
//                                        });
//
//            m_message->AddField<double>("LagrangeResidual", "", "maximum change in Lagrange multipliers", [this]() {
//                return dynamic_cast<chrono::ChIterativeSolver *>(m_chronoSystem->GetSolver().get())->GetDeltalambdaHistory().back();
//            });
//
//        }
//
//    }
//
//    std::string FrOffshoreSystem::GetDataPath(const std::string& relPath) const {
//        return GetPathManager()->GetDataPath(relPath);
//    }
//
//    std::string FrOffshoreSystem::BuildPath(const std::string &rootPath) {
//
//        auto objPath= fmt::format("{}_{}", GetTypeName(), GetShortenUUID());
//
//        auto logPath = GetPathManager()->BuildPath(objPath, fmt::format("{}_{}.csv", GetTypeName(), GetShortenUUID()));
//
//        // Add a serializer
//        m_message->AddSerializer(FrSerializerFactory::instance().Create(this, logPath));
//
//        return objPath;
//    }

  void FrOffshoreSystem::SetSolverVerbose(bool verbose) {
    m_chronoSystem->GetSolver()->SetVerbose(verbose);
    dynamic_cast<chrono::ChIterativeSolver *>(m_chronoSystem->GetSolver().get())->SetRecordViolation(verbose);
  }


  void FrOffshoreSystem::Add(std::shared_ptr<FrTreeNodeBase> item) {

    // FIXME : mettre des gardes au cas ou RegisterTreeNode renvoie false !!!

    // BODY
    if (auto body = std::dynamic_pointer_cast<FrBody>(item)) {
      AddBody(body, body->GetChronoBody());
      m_pathManager->RegisterTreeNode(body.get());

      // LINK
    } else if (auto link = std::dynamic_pointer_cast<FrLinkBase>(item)) {
      AddLink(link, link->GetChronoLink());
      m_pathManager->RegisterTreeNode(link.get());

      //PHYSICS ITEM
    } else if (auto physics_item = std::dynamic_pointer_cast<FrPrePhysicsItem>(item)) {
      AddPhysicsItem(physics_item, physics_item->GetChronoPhysicsItem());
//      m_pathManager->RegisterTreeNode(physics_item.get());

      // FEA MESH
    } else if (auto fea_mesh = std::dynamic_pointer_cast<FrFEAMesh>(item)) {
      AddFEAMesh(fea_mesh, fea_mesh->GetChronoMesh());
//      m_pathManager->RegisterTreeNode(fea_mesh.get());

      // DYNAMIC CABLE
    } else if (auto dynamic_cable = std::dynamic_pointer_cast<FrDynamicCable>(item)) {
      AddDynamicCable(dynamic_cable, dynamic_cable->GetChronoMesh());
      m_pathManager->RegisterTreeNode(dynamic_cable.get());

      // UNKNOWN
    } else {
      std::cerr << "Unknown object type " << std::endl;
      exit(EXIT_FAILURE);
    }

    if (auto loggable = std::dynamic_pointer_cast<FrLoggableBase>(item)) {
      m_LogManager->Add(loggable);
    }

  }

  void FrOffshoreSystem::Remove(std::shared_ptr<FrTreeNodeBase> item) {

    // BODY
    if (auto body = std::dynamic_pointer_cast<FrBody>(item)) {
      RemoveBody(body, body->GetChronoBody());

      // LINK
    } else if (auto link = std::dynamic_pointer_cast<FrLinkBase>(item)) {
      RemoveLink(link, link->GetChronoLink());

      //PHYSICS ITEM
    } else if (auto physics_item = std::dynamic_pointer_cast<FrPrePhysicsItem>(item)) {
      RemovePhysicsItem(physics_item, physics_item->GetChronoPhysicsItem());

      // FEA MESH
    } else if (auto fea_mesh = std::dynamic_pointer_cast<FrFEAMesh>(item)) {
      RemoveFEAMesh(fea_mesh, fea_mesh->GetChronoMesh());

      // DYNAMIC CABLE
    } else if (auto dynamic_cable = std::dynamic_pointer_cast<FrDynamicCable>(item)) {
      RemoveDynamicCable(dynamic_cable, dynamic_cable->GetChronoMesh());

      // UNKNOWN
    } else {
      std::cerr << "Unknown object type " << std::endl;
      exit(EXIT_FAILURE);
    }

    if (auto loggable = std::dynamic_pointer_cast<FrLoggableBase>(item)) {
      m_LogManager->Remove(loggable);
    }

  }

}  // end namespace frydom
