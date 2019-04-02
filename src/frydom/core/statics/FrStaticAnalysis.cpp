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

#include "FrStaticAnalysis.h"
#include "frydom/core/FrOffshoreSystem.h"
#include "frydom/core/body/FrBody.h"
#include "frydom/core/link/FrLinkBase.h"
#include "frydom/core/common/FrPhysicsItem.h"
#include "frydom/core/force/FrForce.h"

#include "frydom/environment/FrEnvironment.h"

namespace frydom{


    void FrStaticAnalysis::SetLog(bool log) {
        m_logStatic = log;
    }

    bool FrStaticAnalysis::GetLog() const {
        return m_logStatic;
    }

    void FrStaticAnalysis::SetNbSteps(int nSteps) {
        m_nSteps = nSteps;
    }

    int FrStaticAnalysis::GetNbSteps() const {
        return m_nSteps;
    }

    void FrStaticAnalysis::SetNbIteration(int nIter) {
        m_nIterations = nIter;
    }

    int FrStaticAnalysis::GetNbIteration() const {
        return m_nIterations;
    }

    void FrStaticAnalysis::SetRelaxation(FrStaticAnalysis::RELAXTYPE relax) {
        m_relax = relax;
    }

    FrStaticAnalysis::RELAXTYPE FrStaticAnalysis::GetRelaxation() const {
        return m_relax;
    }

    void FrStaticAnalysis::SetTolerance(double tol) {
        m_tolerance = tol;
    }

    double FrStaticAnalysis::GetTolerance() const {
        return m_tolerance;
    }

    void FrStaticAnalysis::InitializeStatic() {
// Store the starting time of the simulation
        m_undoTime = m_system->GetTime();
        // Store the time ramp before setting it to 1
        m_ramp = m_system->GetEnvironment()->GetTimeRamp();
        m_system->GetEnvironment()->GetTimeRamp()->SetByTwoPoints(0.,1.,1.,1.);

        for (auto& body : m_system->GetBodyList()) {
            m_map.emplace(body.get(),std::make_pair(body->IsActive(),body->IsLogged()));
            body->SetSleeping(!body->IncludedInStaticAnalysis());
            body->SetLogged(m_logStatic);
            for (auto& force : body->GetForceList()) {
                m_map.emplace(force.get(),std::make_pair(force->IsActive(),force->IsLogged()));
                force->SetActive(force->IncludedInStaticAnalysis());
                force->SetLogged(m_logStatic);
            }
        }

        for (auto& link : m_system->GetLinkList()) {
            m_map.emplace(link.get(),std::make_pair(link->IsActive(),link->IsLogged()));
            link->SetDisabled(!link->IncludedInStaticAnalysis());
            link->SetLogged(m_logStatic);
        }

        for (auto& pi : m_system->GetPrePhysiscsItemList()) {
            m_map.emplace(pi.get(),std::make_pair(pi->IsActive(),pi->IsLogged()));
            pi->SetActive(pi->IncludedInStaticAnalysis());
            pi->SetLogged(m_logStatic);
        }

        for (auto& pi : m_system->GetMidPhysiscsItemList()) {
            m_map.emplace(pi.get(),std::make_pair(pi->IsActive(),pi->IsLogged()));
            pi->SetActive(pi->IncludedInStaticAnalysis());
            pi->SetLogged(m_logStatic);
        }

        for (auto& pi : m_system->GetPostPhysiscsItemList()) {
            m_map.emplace(pi.get(),std::make_pair(pi->IsActive(),pi->IsLogged()));
            pi->SetActive(pi->IncludedInStaticAnalysis());
            pi->SetLogged(m_logStatic);
        }
    }

    void FrStaticAnalysis::FinalizeStatic() {

        for (auto& body : m_system->GetBodyList()) {

            body->SetSleeping(!m_map.find(body.get())->second.first);
            body->SetLogged(m_map.find(body.get())->second.second);

            for (auto& force : body->GetForceList()) {
                force->SetActive(m_map.find(force.get())->second.first);
                force->SetLogged(m_map.find(force.get())->second.second);
            }

        }

        for (auto& link : m_system->GetLinkList()) {

            link->SetDisabled(!m_map.find(link.get())->second.first);
            link->SetLogged(m_map.find(link.get())->second.second);

        }

        for (auto& pi : m_system->GetPrePhysiscsItemList()) {

            pi->SetActive(m_map.find(pi.get())->second.first);
            pi->SetLogged(m_map.find(pi.get())->second.second);

        }

        for (auto& pi : m_system->GetMidPhysiscsItemList()) {

            pi->SetActive(m_map.find(pi.get())->second.first);
            pi->SetLogged(m_map.find(pi.get())->second.second);

        }

        for (auto& pi : m_system->GetPostPhysiscsItemList()) {

            pi->SetActive(m_map.find(pi.get())->second.first);
            pi->SetLogged(m_map.find(pi.get())->second.second);

        }

        // Set no speed and accel. on bodies, meshes and other physics items
        m_system->Relax(m_relax);

        // Set the simulation time to its init value
        m_system->SetTime(m_undoTime);

        // Set the ramp to its init state
        double x0,y0,x1,y1;
        m_ramp->GetByTwoPoints(x0,y0,x1,y1);
        m_system->GetEnvironment()->GetTimeRamp()->SetByTwoPoints(x0,y0,x1,y1);

    }

    bool FrStaticAnalysis::SolveStatic() {


        InitializeStatic();

        bool reach_tolerance = false;
        int iter = 0;

//        for (int m_iter = 0; m_iter < nIter; m_iter++) {
        while (!(reach_tolerance || iter==m_nIterations)) {

            // Set no speed and accel. on bodies, meshes and other physics items
            m_system->Relax(m_relax);

            m_system->AdvanceTo(m_undoTime + iter * m_system->GetTimeStep() * m_nSteps);

            // Get the speed of the bodies to check the convergence
            double bodyVel = 0;
            for (auto &body : m_system->GetBodyList()) {
                bodyVel += body->GetVelocityInWorld(NWU).norm();
            }

            std::cout<<iter<<", "<<m_system->GetTime()<<", "<<bodyVel<<std::endl;

            if (bodyVel < m_tolerance &&
                m_system->GetTime()>m_undoTime+m_system->GetTimeStep()*m_nSteps) {
                reach_tolerance = true;
            }

            iter ++;

        }

        FinalizeStatic();

        return reach_tolerance;
    }

    FrOffshoreSystem *FrStaticAnalysis::GetSystem() {
        return m_system;
    }


} //end namespace frydom