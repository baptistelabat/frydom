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
#include "frydom/utils/FrSerializerFactory.h"


namespace frydom {

  namespace internal {

    template<typename OffshoreSystemType>
    FrSystemBaseSMC<OffshoreSystemType>::FrSystemBaseSMC(frydom::FrOffshoreSystem<OffshoreSystemType> *offshoreSystem) :
        chrono::ChSystemSMC(), m_offshoreSystem_(offshoreSystem) {}

    template<typename OffshoreSystemType>
    void FrSystemBaseSMC<OffshoreSystemType>::Update(bool update_assets) {

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

    template<typename OffshoreSystemType>
    void FrSystemBaseSMC<OffshoreSystemType>::CustomEndOfStep() {
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
    template<typename OffshoreSystemType>
    bool FrSystemBaseSMC<OffshoreSystemType>::DoStaticLinear() {
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
    }

  }  // end namespace frydom::internal

  /// Default constructor
  /// \param systemType contact method system (SMOOTH_CONTACT/NONSMOOTH_CONTACT)
  /// \param timeStepper time stepper type
  /// \param solver solver type
  template<typename OffshoreSystemType>
  FrOffshoreSystem<OffshoreSystemType>::FrOffshoreSystem(SYSTEM_TYPE systemType, TIME_STEPPER timeStepper, SOLVER solver) {

    this->SetLogged(true);

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
    m_environment = std::make_unique<FrEnvironment>(
        this); // FIXME: voir bug dans FrEnvironment pour le reglage du systeme

    // Creating the log manager service
    this->m_pathManager = std::make_shared<FrPathManager<OffshoreSystemType>>();

    // Create the static analysis
    m_statics = std::make_unique<FrStaticAnalysis>(this);

//        m_message = std::make_unique<hermes::Message>();

  }

  template<typename OffshoreSystemType>
  FrOffshoreSystem<OffshoreSystemType>::~FrOffshoreSystem<OffshoreSystemType>() = default;

  template<typename OffshoreSystemType>
  void FrOffshoreSystem<OffshoreSystemType>::Add(std::shared_ptr<FrObject<OffshoreSystemType>> newItem) {
    assert(std::dynamic_pointer_cast<FrBody>(newItem) ||
           std::dynamic_pointer_cast<FrLinkBase<OffshoreSystemType>>(newItem) ||
           std::dynamic_pointer_cast<FrPhysicsItem>(newItem));

    if (auto item = std::dynamic_pointer_cast<FrBody>(newItem)) {
      AddBody(item);
      return;
    }

    if (auto item = std::dynamic_pointer_cast<FrLinkBase<OffshoreSystemType>>(newItem)) {
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
  template<typename OffshoreSystemType>
  void FrOffshoreSystem<OffshoreSystemType>::AddBody(std::shared_ptr<FrBody<OffshoreSystemType>> body) {

    if (!CheckBodyContactMethod(
        body)) { // TODO : voir si on set pas d'autorite le mode de contact a celui du systeme plutot que de faire un if...
      body->SetContactMethod(m_systemType);
    }

    m_chronoSystem->AddBody(body->GetChronoBody());  // Authorized because this method is a friend of FrBody
    m_bodyList.push_back(body);

    body->m_system = this;

  }

  template<typename OffshoreSystemType>
  typename FrOffshoreSystem<OffshoreSystemType>::BodyContainer FrOffshoreSystem<OffshoreSystemType>::GetBodyList() {
    return m_bodyList;
  }

  template<typename OffshoreSystemType>
  void FrOffshoreSystem<OffshoreSystemType>::RemoveBody(std::shared_ptr<FrBody<OffshoreSystemType>> body) {

    m_chronoSystem->RemoveBody(body->GetChronoBody());

    auto it = std::find(body_begin(), body_end(), body);
    assert(it != body_end());
    m_bodyList.erase(it);
    body->m_system = nullptr;

  }

  template<typename OffshoreSystemType>
  void FrOffshoreSystem<OffshoreSystemType>::RemoveAllBodies() {

    for (auto &body: m_bodyList)
      RemoveBody(body);

  }


  // ***** Link *****
  template<typename OffshoreSystemType>
  void FrOffshoreSystem<OffshoreSystemType>::AddLink(std::shared_ptr<FrLinkBase<OffshoreSystemType>> link) {
    m_chronoSystem->AddLink(link->GetChronoLink());
    m_linkList.push_back(link);
  }

  template<typename OffshoreSystemType>
  void FrOffshoreSystem<OffshoreSystemType>::RemoveLink(std::shared_ptr<FrLinkBase<OffshoreSystemType>> link) {

    m_chronoSystem->RemoveLink(link->GetChronoLink());

    auto it = std::find(link_begin(), link_end(), link);
    assert(it != link_end());
    m_linkList.erase(it);
    link->m_system = nullptr;

  }

  template<typename OffshoreSystemType>
  void FrOffshoreSystem<OffshoreSystemType>::RemoveAllLinks() {

    for (auto &link: m_linkList)
      RemoveLink(link);

  }


  // ***** Physics Item *****
  template<typename OffshoreSystemType>
  void FrOffshoreSystem<OffshoreSystemType>::AddPhysicsItem(std::shared_ptr<FrPrePhysicsItem> otherPhysics) {
    m_chronoSystem->AddOtherPhysicsItem(otherPhysics->GetChronoPhysicsItem());
    otherPhysics->m_system = this;
    m_PrePhysicsList.push_back(otherPhysics);
  }

  template<typename OffshoreSystemType>
  FrOffshoreSystem<OffshoreSystemType>::PrePhysicsContainer FrOffshoreSystem<OffshoreSystemType>::GetPrePhysicsItemList() {
    return m_PrePhysicsList;
  }

  template<typename OffshoreSystemType>
  void FrOffshoreSystem<OffshoreSystemType>::AddPhysicsItem(std::shared_ptr<FrMidPhysicsItem<OffshoreSystemType>> otherPhysics) {
    m_chronoSystem->AddOtherPhysicsItem(otherPhysics->GetChronoPhysicsItem());
    otherPhysics->m_system = this;
    m_MidPhysicsList.push_back(otherPhysics);
  }

  template<typename OffshoreSystemType>
  typename FrOffshoreSystem<OffshoreSystemType>::MidPhysicsContainer FrOffshoreSystem<OffshoreSystemType>::GetMidPhysicsItemList() {
    return m_MidPhysicsList;
  }

  template<typename OffshoreSystemType>
  void FrOffshoreSystem<OffshoreSystemType>::AddPhysicsItem(std::shared_ptr<FrPostPhysicsItem<OffshoreSystemType>> otherPhysics) {
    m_chronoSystem->AddOtherPhysicsItem(otherPhysics->GetChronoPhysicsItem());
    otherPhysics->m_system = this;
    m_PostPhysicsList.push_back(otherPhysics);
  }

  template<typename OffshoreSystemType>
  typename FrOffshoreSystem<OffshoreSystemType>::PostPhysicsContainer FrOffshoreSystem<OffshoreSystemType>::GetPostPhysicsItemList() {
    return m_PostPhysicsList;
  }

  template<typename OffshoreSystemType>
  void FrOffshoreSystem<OffshoreSystemType>::RemovePhysicsItem(std::shared_ptr<FrPhysicsItem<OffshoreSystemType>> item) {

    m_chronoSystem->RemoveOtherPhysicsItem(item->GetChronoPhysicsItem());

    auto it = std::find(m_PrePhysicsList.begin(), m_PrePhysicsList.end(), item);
    if (it != m_PrePhysicsList.end())
      m_PrePhysicsList.erase(it);
    else {
      auto it = std::find(m_MidPhysicsList.begin(), m_MidPhysicsList.end(), item);
      if (it != m_MidPhysicsList.end())
        m_MidPhysicsList.erase(it);
      else {
        auto it = std::find(m_PostPhysicsList.begin(), m_PostPhysicsList.end(), item);
        if (it != m_PostPhysicsList.end())
          m_PostPhysicsList.erase(it);
        else {
          assert(("physics item can't be found in the list : ", it != m_PostPhysicsList.end()));
        }
      }
    }


    item->m_system = nullptr;

  }

  template<typename OffshoreSystemType>
  void FrOffshoreSystem<OffshoreSystemType>::RemoveAllPhysicsItem() {

    for (auto &item: m_PrePhysicsList)
      RemovePhysicsItem(item);

    for (auto &item: m_MidPhysicsList)
      RemovePhysicsItem(item);

    for (auto &item: m_PostPhysicsList)
      RemovePhysicsItem(item);

  }


  // ***** FEAMesh *****
  template<typename OffshoreSystemType>
  void FrOffshoreSystem<OffshoreSystemType>::AddFEAMesh(std::shared_ptr<FrFEAMesh> feaMesh) {
    m_chronoSystem->AddMesh(feaMesh->GetChronoMesh());  // Authorized because this method is a friend of FrFEAMesh

    feaMesh->m_system = this;
    m_feaMeshList.push_back(feaMesh);
  }

  template<typename OffshoreSystemType>
  void FrOffshoreSystem<OffshoreSystemType>::Add(std::shared_ptr<FrDynamicCable> cable) {

    // Add the FEA mesh
    AddFEAMesh(cable);

    // Add the hinges
    m_chronoSystem->Add(dynamic_cast<internal::FrDynamicCableBase *>(cable->GetChronoMesh().get())->m_startingHinge);
    m_chronoSystem->Add(dynamic_cast<internal::FrDynamicCableBase *>(cable->GetChronoMesh().get())->m_endingHinge);

  }

  template<typename OffshoreSystemType>
  FrOffshoreSystem<OffshoreSystemType>::FEAMeshContainer FrOffshoreSystem<OffshoreSystemType>::GetFEAMeshList() {
    return m_feaMeshList;
  }

  template<typename OffshoreSystemType>
  void FrOffshoreSystem<OffshoreSystemType>::RemoveFEAMesh(std::shared_ptr<FrFEAMesh> feamesh) {

    m_chronoSystem->RemoveMesh(feamesh->GetChronoMesh());

    auto it = std::find(m_feaMeshList.begin(), m_feaMeshList.end(), feamesh);
    assert(it != m_feaMeshList.end());
    m_feaMeshList.erase(it);
    feamesh->m_system = nullptr;

  }

  template<typename OffshoreSystemType>
  void FrOffshoreSystem<OffshoreSystemType>::Remove(std::shared_ptr<FrDynamicCable> cable) {

    RemoveFEAMesh(cable);

    m_chronoSystem->RemoveOtherPhysicsItem(
        dynamic_cast<internal::FrDynamicCableBase *>(cable->GetChronoMesh().get())->m_startingHinge);
    m_chronoSystem->RemoveOtherPhysicsItem(
        dynamic_cast<internal::FrDynamicCableBase *>(cable->GetChronoMesh().get())->m_endingHinge);

  }


  // ***** Environment *****
  template<typename OffshoreSystemType>
  FrEnvironment *FrOffshoreSystem<OffshoreSystemType>::GetEnvironment() const {
    return m_environment.get();
  }

  template<typename OffshoreSystemType>
  std::shared_ptr<FrBody> FrOffshoreSystem<OffshoreSystemType>::GetWorldBody() const {
    return m_worldBody;
  }

  template<typename OffshoreSystemType>
  void FrOffshoreSystem<OffshoreSystemType>::PreUpdate() {
    // TODO : voir si on ne met pas l'environnement comme un physics Item update en tant que PrePhysicsItem
    m_environment->Update(m_chronoSystem->GetChTime());
  }

  template<typename OffshoreSystemType>
  void FrOffshoreSystem<OffshoreSystemType>::PostUpdate() {
    // TODO
  }

  template<typename OffshoreSystemType>
  void FrOffshoreSystem<OffshoreSystemType>::PrePhysicsUpdate(double time, bool update_assets) {
    for (auto &item : m_PrePhysicsList) {
      item->Update(time);
    }
  }

  template<typename OffshoreSystemType>
  void FrOffshoreSystem<OffshoreSystemType>::MidPhysicsUpdate(double time, bool update_assets) {
    for (auto &item : m_MidPhysicsList) {
      item->Update(time);
    }
  }

  template<typename OffshoreSystemType>
  void FrOffshoreSystem<OffshoreSystemType>::PostPhysicsUpdate(double time, bool update_assets) {
    for (auto &item : m_PostPhysicsList) {
      item->Update(time);
    }
  }

  template<typename OffshoreSystemType>
  void FrOffshoreSystem<OffshoreSystemType>::Initialize() {


    // Initializing environment before bodies
    m_environment->Initialize();

    for (auto &item : m_PrePhysicsList) {
      item->Initialize();
    }

    for (auto &item : m_bodyList) {
      item->Initialize();
    }

    for (auto &item : m_MidPhysicsList) {
      item->Initialize();
    }

    for (auto &item : m_linkList) {
      item->Initialize();
    }

    for (auto &item : m_feaMeshList) {
      item->Initialize();
    }

    for (auto &item : m_PostPhysicsList) {
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

  template<typename OffshoreSystemType>
  void FrOffshoreSystem<OffshoreSystemType>::StepFinalize() {
    m_environment->StepFinalize();

    for (auto &item : m_PrePhysicsList) {
      item->StepFinalize();
    }

    for (auto &item : m_bodyList) {
      item->StepFinalize();
    }

    for (auto &item : m_MidPhysicsList) {
      item->StepFinalize();
    }

    for (auto &item : m_linkList) {
      item->StepFinalize();
    }

    for (auto &item : m_feaMeshList) {
      item->StepFinalize();
    }

    for (auto &item : m_PostPhysicsList) {
      item->StepFinalize();
    }

    // Serialize and send the message log
    FrObject::SendLog();

  }

  template<typename OffshoreSystemType>
  void FrOffshoreSystem<OffshoreSystemType>::SetSystemType(SYSTEM_TYPE type, bool checkCompat) {

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

  template<typename OffshoreSystemType>
  void FrOffshoreSystem<OffshoreSystemType>::CheckCompatibility() const {
    // TODO : verifier la compatibilite entre type systeme, solveur et integrateur temporel



  }

  template<typename OffshoreSystemType>
  bool FrOffshoreSystem<OffshoreSystemType>::CheckBodyContactMethod(std::shared_ptr<FrBody> body) {
    return m_systemType == body->GetContactType();
  }

  template<typename OffshoreSystemType>
  void FrOffshoreSystem<OffshoreSystemType>::SetSolver(SOLVER solver, bool checkCompat) {

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

  template<typename OffshoreSystemType>
  void FrOffshoreSystem<OffshoreSystemType>::SetSolverWarmStarting(bool useWarm) {
    m_chronoSystem->SetSolverWarmStarting(useWarm);
  }

  template<typename OffshoreSystemType>
  void FrOffshoreSystem<OffshoreSystemType>::SetSolverOverrelaxationParam(double omega) {
    m_chronoSystem->SetSolverOverrelaxationParam(omega);
  }

  template<typename OffshoreSystemType>
  void FrOffshoreSystem<OffshoreSystemType>::SetSolverSharpnessParam(double momega) {
    m_chronoSystem->SetSolverSharpnessParam(momega);
  }

  template<typename OffshoreSystemType>
  void FrOffshoreSystem<OffshoreSystemType>::SetParallelThreadNumber(int nbThreads) {
    m_chronoSystem->SetParallelThreadNumber(nbThreads);
  }

  template<typename OffshoreSystemType>
  void FrOffshoreSystem<OffshoreSystemType>::SetSolverMaxIterSpeed(int maxIter) {
    m_chronoSystem->SetMaxItersSolverSpeed(maxIter);
  }

  template<typename OffshoreSystemType>
  void FrOffshoreSystem<OffshoreSystemType>::SetSolverMaxIterStab(int maxIter) {
    m_chronoSystem->SetMaxItersSolverStab(maxIter);
  }

  template<typename OffshoreSystemType>
  void FrOffshoreSystem<OffshoreSystemType>::SetSolverMaxIterAssembly(int maxIter) {
    m_chronoSystem->SetMaxiter(maxIter);
  }

  template<typename OffshoreSystemType>
  void FrOffshoreSystem<OffshoreSystemType>::SetSolverGeometricTolerance(double tol) {
    m_chronoSystem->SetTol(tol);
  }

  template<typename OffshoreSystemType>
  void FrOffshoreSystem<OffshoreSystemType>::SetSolverForceTolerance(double tol) {
    m_chronoSystem->SetTolForce(tol);
  }

  template<typename OffshoreSystemType>
  void FrOffshoreSystem<OffshoreSystemType>::UseMaterialProperties(bool use) {
    if (m_systemType == SMOOTH_CONTACT) {
      dynamic_cast<chrono::ChSystemSMC *>(m_chronoSystem.get())->UseMaterialProperties(use);
    } else {
      std::cerr << "The use of material properties is only for SMOOTH_CONTACT systems" << std::endl;
    }
  }

  template<typename OffshoreSystemType>
  void FrOffshoreSystem<OffshoreSystemType>::SetContactForceModel(FrOffshoreSystem<OffshoreSystemType>::CONTACT_MODEL model) {
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

  template<typename OffshoreSystemType>
  void FrOffshoreSystem<OffshoreSystemType>::SetAdhesionForceModel(FrOffshoreSystem<OffshoreSystemType>::ADHESION_MODEL model) {
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

  template<typename OffshoreSystemType>
  void FrOffshoreSystem<OffshoreSystemType>::SetTangentialDisplacementModel(FrOffshoreSystem<OffshoreSystemType>::TANGENTIAL_DISP_MODEL model) {
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

  template<typename OffshoreSystemType>
  void FrOffshoreSystem<OffshoreSystemType>::SetStiffContact(bool isStiff) {
    if (m_systemType == SMOOTH_CONTACT) {
      dynamic_cast<chrono::ChSystemSMC *>(m_chronoSystem.get())->SetStiffContact(isStiff);
    } else {
      std::cerr << "StiffContact is only for SMOOTH_CONTACT systems" << std::endl;
    }
  }

  template<typename OffshoreSystemType>
  void FrOffshoreSystem<OffshoreSystemType>::SetSlipVelocityThreshold(double velocity) {
    if (m_systemType == SMOOTH_CONTACT) {
      dynamic_cast<chrono::ChSystemSMC *>(m_chronoSystem.get())->SetSlipVelocityThreshold(velocity);
    } else {
      std::cerr << "Slip Velocity Threshold is only for SMOOTH_CONTACT systems" << std::endl;
    }
  }

  template<typename OffshoreSystemType>
  void FrOffshoreSystem<OffshoreSystemType>::SetCharacteristicImpactVelocity(double velocity) {
    if (m_systemType == SMOOTH_CONTACT) {
      dynamic_cast<chrono::ChSystemSMC *>(m_chronoSystem.get())->SetCharacteristicImpactVelocity(velocity);
    } else {
      std::cerr << "Characteristic Impact Velocity is only for SMOOTH_CONTACT systems" << std::endl;
    }
  }

  template<typename OffshoreSystemType>
  void FrOffshoreSystem<OffshoreSystemType>::SetMinBounceSpeed(double speed) {
    m_chronoSystem->SetMinBounceSpeed(speed);
  }

  template<typename OffshoreSystemType>
  void FrOffshoreSystem<OffshoreSystemType>::SetMaxPenetrationRecoverySpeed(double speed) {
    m_chronoSystem->SetMaxPenetrationRecoverySpeed(speed);
  }

  template<typename OffshoreSystemType>
  int FrOffshoreSystem<OffshoreSystemType>::GetNbPositionCoords() const {
    return m_chronoSystem->GetNcoords();
  }

  template<typename OffshoreSystemType>
  int FrOffshoreSystem<OffshoreSystemType>::GetNbVelocityCoords() const {
    return m_chronoSystem->GetNcoords_w();
  }

  template<typename OffshoreSystemType>
  int FrOffshoreSystem<OffshoreSystemType>::GetNbConstraintsCoords() const {
    return m_chronoSystem->GetNdoc_w();
  }

  template<typename OffshoreSystemType>
  int FrOffshoreSystem<OffshoreSystemType>::GetNbDOF() const {
    return m_chronoSystem->GetNdof();
  }

  template<typename OffshoreSystemType>
  int FrOffshoreSystem<OffshoreSystemType>::GetNbBodies() const {
    return m_chronoSystem->GetNbodies();
  }

  template<typename OffshoreSystemType>
  int FrOffshoreSystem<OffshoreSystemType>::GetNbFixedBodies() const {
    return m_chronoSystem->GetNbodiesFixed();
  }

  template<typename OffshoreSystemType>
  int FrOffshoreSystem<OffshoreSystemType>::GetNbSleepingBodies() const {
    return m_chronoSystem->GetNbodiesSleeping();
  }

  template<typename OffshoreSystemType>
  void FrOffshoreSystem<OffshoreSystemType>::SetUseSleepingBodies(bool useSleeping) {
    m_chronoSystem->SetUseSleeping(useSleeping);
  }

  template<typename OffshoreSystemType>
  double FrOffshoreSystem<OffshoreSystemType>::GetGravityAcceleration() const {
    return fabs(m_chronoSystem->Get_G_acc()[2]);
  }

  template<typename OffshoreSystemType>
  void FrOffshoreSystem<OffshoreSystemType>::SetGravityAcceleration(double gravityAcceleration) {
    m_chronoSystem->Set_G_acc(chrono::ChVector<double>(0., 0., -gravityAcceleration));
  }

  template<typename OffshoreSystemType>
  bool FrOffshoreSystem<OffshoreSystemType>::DoAssembly() {
    return m_chronoSystem->DoFullAssembly();
  }

  template<typename OffshoreSystemType>
  FrStaticAnalysis *FrOffshoreSystem<OffshoreSystemType>::GetStaticAnalysis() const {
    return m_statics.get();
  }

  template<typename OffshoreSystemType>
  bool FrOffshoreSystem<OffshoreSystemType>::SolveStaticWithRelaxation() {

    IsInitialized();

    return m_statics->SolveStatic();

  }

  template<typename OffshoreSystemType>
  void FrOffshoreSystem<OffshoreSystemType>::Relax(FrStaticAnalysis::RELAXTYPE relax) {

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

  template<typename OffshoreSystemType>
  void FrOffshoreSystem<OffshoreSystemType>::SetTimeStepper(TIME_STEPPER type, bool checkCompat) {

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

  template<typename OffshoreSystemType>
  void FrOffshoreSystem<OffshoreSystemType>::SetTimeStepper(TIME_STEPPER type) {
    SetTimeStepper(type, true);
  }

  template<typename OffshoreSystemType>
  void FrOffshoreSystem<OffshoreSystemType>::SetTimeStep(double timeStep) {
    m_chronoSystem->SetStep(timeStep);
  }

  template<typename OffshoreSystemType>
  double FrOffshoreSystem<OffshoreSystemType>::GetTimeStep() const {
    return m_chronoSystem->GetStep();
  }

  template<typename OffshoreSystemType>
  void FrOffshoreSystem<OffshoreSystemType>::SetMinTimeStep(double minTimeStep) {
    m_chronoSystem->SetStepMin(minTimeStep);
  }

  template<typename OffshoreSystemType>
  void FrOffshoreSystem<OffshoreSystemType>::SetMaxTimeStep(double maxTimeStep) {
    m_chronoSystem->SetStepMax(maxTimeStep);
  }

  template<typename OffshoreSystemType>
  double FrOffshoreSystem<OffshoreSystemType>::GetTime() const {
    return m_chronoSystem->GetChTime();
  }

  template<typename OffshoreSystemType>
  void FrOffshoreSystem<OffshoreSystemType>::SetTime(double time) {
    m_chronoSystem->SetChTime(time);
  }

  template<typename OffshoreSystemType>
  bool FrOffshoreSystem<OffshoreSystemType>::AdvanceOneStep(double stepSize) {
    IsInitialized();
    return (bool) m_chronoSystem->DoStepDynamics(stepSize);
  }

  template<typename OffshoreSystemType>
  bool FrOffshoreSystem<OffshoreSystemType>::AdvanceTo(double nextTime) {
    IsInitialized();
    return m_chronoSystem->DoFrameDynamics(nextTime);
  }

  template<typename OffshoreSystemType>
  bool FrOffshoreSystem<OffshoreSystemType>::RunDynamics(double frameStep) {
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

  template<typename OffshoreSystemType>
  void FrOffshoreSystem<OffshoreSystemType>::CreateWorldBody() {
    m_worldBody = std::make_shared<FrBody>();
    m_worldBody->SetFixedInWorld(true);
    m_worldBody->SetName("WorldBody");
    m_worldBody->SetLogged(false);
    switch (m_systemType) {
      case SMOOTH_CONTACT:
        m_worldBody->SetSmoothContact();
        break;
      case NONSMOOTH_CONTACT:
        m_worldBody->SetNonSmoothContact();
        break;
    }
    AddBody(m_worldBody);
  }

  template<typename OffshoreSystemType>
  std::shared_ptr<FrBody> FrOffshoreSystem<OffshoreSystemType>::NewBody() {
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

  template<typename OffshoreSystemType>
  void FrOffshoreSystem<OffshoreSystemType>::Clear() {
    m_chronoSystem->Clear();

    m_bodyList.clear();
    m_linkList.clear();
    m_feaMeshList.clear();
    m_PrePhysicsList.clear();
    m_MidPhysicsList.clear();
    m_PostPhysicsList.clear();

    m_isInitialized = false;
  }

  template<typename OffshoreSystemType>
  chrono::ChSystem *FrOffshoreSystem<OffshoreSystemType>::GetChronoSystem() {
    return m_chronoSystem.get();
  }


  // Irrlicht visualization
  template<typename OffshoreSystemType>
  void FrOffshoreSystem<OffshoreSystemType>::RunInViewer(double endTime, double dist, bool recordVideo, int videoFrameSaveInterval) {

    // Initialization of the system if not already done.
    IsInitialized();

    // Definition and initialization of the Irrlicht application.
    FrIrrApp app(this, m_chronoSystem.get(), dist);

    app.SetTimestep(m_chronoSystem->GetStep());
    app.SetVideoframeSave(recordVideo);
    app.SetVideoframeSaveInterval(videoFrameSaveInterval);
    app.Run(endTime); // The temporal loop is here.

  }

  template<typename OffshoreSystemType>
  void FrOffshoreSystem<OffshoreSystemType>::RunInViewer(double endTime, double dist, bool recordVideo) {
    RunInViewer(endTime, dist, recordVideo, 10);
  }

  template<typename OffshoreSystemType>
  void FrOffshoreSystem<OffshoreSystemType>::RunInViewer(double endTime, double dist) {
    RunInViewer(endTime, dist, false);
  }

  template<typename OffshoreSystemType>
  void FrOffshoreSystem<OffshoreSystemType>::RunInViewer(double endTime) {
    RunInViewer(endTime, 100);
  }

  template<typename OffshoreSystemType>
  void FrOffshoreSystem<OffshoreSystemType>::RunInViewer() {
    RunInViewer(0);
  }

  template<typename OffshoreSystemType>
  void FrOffshoreSystem<OffshoreSystemType>::Visualize(double dist, bool recordVideo) {

    IsInitialized();  // So that system is automatically initialized when run in viewer mode

    FrIrrApp app(this, m_chronoSystem.get(), dist);

    app.SetTimestep(m_chronoSystem->GetStep());
    app.SetVideoframeSave(recordVideo);
    app.Visualize();

  }

  template<typename OffshoreSystemType>
  void FrOffshoreSystem<OffshoreSystemType>::Visualize(double dist) {
    Visualize(dist, false);
  }

  template<typename OffshoreSystemType>
  void FrOffshoreSystem<OffshoreSystemType>::Visualize() {
    Visualize(100);
  }

  template<typename OffshoreSystemType>
  void FrOffshoreSystem<OffshoreSystemType>::VisualizeStaticAnalysis(double dist, bool recordVideo) {

    IsInitialized();  // So that system is automatically initialized when run in viewer mode

    FrIrrApp app(this, m_chronoSystem.get(), dist);

    app.SetTimestep(m_chronoSystem->GetStep());
    app.SetVideoframeSave(recordVideo);
    app.VisualizeStaticAnalysis();

  }

  template<typename OffshoreSystemType>
  void FrOffshoreSystem<OffshoreSystemType>::VisualizeStaticAnalysis(double dist) {
    VisualizeStaticAnalysis(dist, false);
  }

  template<typename OffshoreSystemType>
  void FrOffshoreSystem<OffshoreSystemType>::VisualizeStaticAnalysis() {
    VisualizeStaticAnalysis(100);
  }

  template<typename OffshoreSystemType>
  void FrOffshoreSystem<OffshoreSystemType>::AddAsset(std::shared_ptr<chrono::ChAsset> asset) {
    m_chronoSystem->AddAsset(std::move(asset));
  }

  template<typename OffshoreSystemType>
  void FrOffshoreSystem<OffshoreSystemType>::IsInitialized() {
    if (!m_isInitialized) Initialize();
  }

  // Iterators
  template<typename OffshoreSystemType>
  FrOffshoreSystem<OffshoreSystemType>::BodyIter FrOffshoreSystem<OffshoreSystemType>::body_begin() {
    return m_bodyList.begin();
  }

  template<typename OffshoreSystemType>
  FrOffshoreSystem<OffshoreSystemType>::ConstBodyIter FrOffshoreSystem<OffshoreSystemType>::body_begin() const {
    return m_bodyList.cbegin();
  }

  template<typename OffshoreSystemType>
  FrOffshoreSystem<OffshoreSystemType>::BodyIter FrOffshoreSystem<OffshoreSystemType>::body_end() {
    return m_bodyList.end();
  }

  template<typename OffshoreSystemType>
  FrOffshoreSystem<OffshoreSystemType>::ConstBodyIter FrOffshoreSystem<OffshoreSystemType>::body_end() const {
    return m_bodyList.cend();
  }

  template<typename OffshoreSystemType>
  FrOffshoreSystem<OffshoreSystemType>::LinkIter FrOffshoreSystem<OffshoreSystemType>::link_begin() {
    return m_linkList.begin();
  }

  template<typename OffshoreSystemType>
  FrOffshoreSystem<OffshoreSystemType>::ConstLinkIter FrOffshoreSystem<OffshoreSystemType>::link_begin() const {
    return m_linkList.cbegin();
  }

  template<typename OffshoreSystemType>
  FrOffshoreSystem<OffshoreSystemType>::LinkIter FrOffshoreSystem<OffshoreSystemType>::link_end() {
    return m_linkList.end();
  }

  template<typename OffshoreSystemType>
  FrOffshoreSystem<OffshoreSystemType>::ConstLinkIter FrOffshoreSystem<OffshoreSystemType>::link_end() const {
    return m_linkList.cend();
  }

  template<typename OffshoreSystemType>
  void FrOffshoreSystem<OffshoreSystemType>::InitializeLog_Dependencies(const std::string &systemPath) {

    if (IsLogged()) {

      // Initializing environment before bodies
//            m_environment->InitializeLog();

      for (auto &item : m_PrePhysicsList) {
        item->SetPathManager(GetPathManager());
        item->InitializeLog(systemPath);
      }

      for (auto &item : m_bodyList) {
        item->SetPathManager(GetPathManager());
        item->InitializeLog(systemPath);
      }

      for (auto &item : m_MidPhysicsList) {
        item->SetPathManager(GetPathManager());
        item->InitializeLog(systemPath);
      }

      for (auto &item : m_linkList) {
        item->SetPathManager(GetPathManager());
        item->InitializeLog(systemPath);
      }

      for (auto &item : m_feaMeshList) {
        item->SetPathManager(GetPathManager());
        item->InitializeLog(systemPath);
      }

      for (auto &item : m_PostPhysicsList) {
        item->SetPathManager(GetPathManager());
        item->InitializeLog(systemPath);
      }

    }
  }

  template<typename OffshoreSystemType>
  void FrOffshoreSystem<OffshoreSystemType>::ClearLogs() {

    ClearMessage();

    for (auto &item : m_PrePhysicsList) {
      item->ClearMessage();
    }

    for (auto &item : m_bodyList) {
      item->ClearMessage();
      for (auto &force : item->GetForceList()) {
        force->ClearMessage();
      }
      for (auto &node : item->GetNodeList()) {
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

  template<typename OffshoreSystemType>
  void FrOffshoreSystem<OffshoreSystemType>::AddFields() {
    m_message->AddField<double>("time", "s", "Current time of the simulation",
                                [this]() { return GetTime(); });

    m_message->AddField<int>("iter", "", "number of total iterations taken by the solver", [this]() {
      return dynamic_cast<chrono::ChIterativeSolver *>(m_chronoSystem->GetSolver().get())->GetTotalIterations();
    });

    if (dynamic_cast<chrono::ChIterativeSolver *>(m_chronoSystem->GetSolver().get())->GetRecordViolation()) {

      m_message->AddField<double>("violationResidual", "", "constraint violation", [this]() {
        return dynamic_cast<chrono::ChIterativeSolver *>(m_chronoSystem->GetSolver().get())->GetViolationHistory().back();
      });

      m_message->AddField<double>("LagrangeResidual", "", "maximum change in Lagrange multipliers", [this]() {
        return dynamic_cast<chrono::ChIterativeSolver *>(m_chronoSystem->GetSolver().get())->GetDeltalambdaHistory().back();
      });

    }

  }

  template<typename OffshoreSystemType>
  std::string FrOffshoreSystem<OffshoreSystemType>::GetDataPath(const std::string &relPath) const {
    return GetPathManager()->GetDataPath(relPath);
  }

  template<typename OffshoreSystemType>
  std::string FrOffshoreSystem<OffshoreSystemType>::BuildPath(const std::string &rootPath) {

    auto objPath = fmt::format("{}_{}", GetTypeName(), GetShortenUUID());

    auto logPath = GetPathManager()->BuildPath(objPath, fmt::format("{}_{}.csv", GetTypeName(), GetShortenUUID()));

    // Add a serializer
    m_message->AddSerializer(FrSerializerFactory::instance().Create(this, logPath));

    return objPath;
  }

  template<typename OffshoreSystemType>
  void FrOffshoreSystem<OffshoreSystemType>::SetSolverVerbose(bool verbose) {
    m_chronoSystem->GetSolver()->SetVerbose(verbose);
    dynamic_cast<chrono::ChIterativeSolver *>(m_chronoSystem->GetSolver().get())->SetRecordViolation(verbose);
  }


}  // end namespace frydom
