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


#include "FrRadiationModel.h"

#include "FrAddedMassBase.h"
#include "frydom/hydrodynamics/seakeeping/linear/hdb/FrLinearHDBInc.h"
#include "frydom/core/body/FrBody.h"
#include "frydom/hydrodynamics/FrEquilibriumFrame.h"

#include "FrRadiationForce.h"


namespace frydom {


    // TODO : generaliser la méthode de mathutils pour les vecteurs Eigen

    Vector6d<double> TrapzLoc(std::vector<double>& x, std::vector<Vector6d<double>>& y) {

        unsigned long N = y.size();

        //assert(N > 1);
        if (N <= 1) {
            auto res = Vector6d<double>();
            res.SetNull();
            return res;
        }

        assert(x.size() == N);

        double dx1 = x[1] - x[0];
        double dxN_1 = x[N-1] - x[N-2];

        Vector6d<double> sum;
        sum.SetNull();
        double dxi, dxii;

        dxii = dx1;

        for (unsigned long i=1; i<N-1; i++) {
            dxi = dxii;
            dxii = x[i+1] - x[i];
            sum += y[i] * (dxi + dxii);
        }

        return 0.5 * (sum + y[0]*dx1 + y[N-1]*dxN_1);

    };

    // ----------------------------------------------------------------
    // Radiation model
    // ----------------------------------------------------------------

    FrRadiationModel::FrRadiationModel() {
        m_chronoPhysicsItem = std::make_shared<internal::FrAddedMassBase>(this);
    }

    FrRadiationModel::FrRadiationModel(std::shared_ptr<FrHydroDB> HDB) : m_HDB(HDB) {

        // Creation of an AddedMassBase object.
        m_chronoPhysicsItem = std::make_shared<internal::FrAddedMassBase>(this); // this = FrRadiationModel
    }

    void FrRadiationModel::Initialize() {
        FrPrePhysicsItem::Initialize();
        m_chronoPhysicsItem->SetupInitial();
    }

    FrHydroMapper* FrRadiationModel::GetMapper() const {
        return m_HDB->GetMapper();
    }

    Force FrRadiationModel::GetRadiationForce(FrBEMBody *BEMBody) const {
        return m_radiationForce.at(BEMBody).GetForce();
    }

    Force FrRadiationModel::GetRadiationForce(FrBody* body) const {
        auto BEMBody = m_HDB->GetBody(body);
        return m_radiationForce.at(BEMBody).GetForce();
    }

    Torque FrRadiationModel::GetRadiationTorque(FrBEMBody* BEMBody) const {
        return m_radiationForce.at(BEMBody).GetTorque();
    }

    Torque FrRadiationModel::GetRadiationTorque(FrBody* body) const {
        auto BEMBody = m_HDB->GetBody(body);
        return m_radiationForce.at(BEMBody).GetTorque();
    }

    void FrRadiationModel::Compute(double time) {

    }

    // ----------------------------------------------------------------
    // Radiation model with convolution
    // ----------------------------------------------------------------

    FrRadiationConvolutionModel::FrRadiationConvolutionModel(std::shared_ptr<FrHydroDB> HDB)
        : FrRadiationModel(HDB) { // Initialization of the the parent class FrRadiationModel.

        // Constructor of the class FrRadiationConvolutionModel.

        // FIXME : a passer dans la méthode initialize pour eviter les pb de précédence vis a vis de la HDB

        // Loop over every body subject to hydrodynamic loads.
        for (auto BEMBody=m_HDB->begin(); BEMBody!=m_HDB->end(); ++BEMBody) {
            auto body = m_HDB->GetBody(BEMBody->first);
            body->AddExternalForce(std::make_shared<FrRadiationConvolutionForce>(this)); // Addition of the hydrodynamic loads to every body.
        }

    }

    void FrRadiationConvolutionModel::Initialize() {

        FrRadiationModel::Initialize();

        unsigned int N;
        if (m_Te < DBL_EPSILON or m_dt < DBL_EPSILON) GetImpulseResponseSize(m_Te, m_dt, N);

        for (auto BEMBody=m_HDB->begin(); BEMBody!=m_HDB->end(); ++BEMBody) {

            if (m_recorder.find(BEMBody->first) == m_recorder.end()) {
                m_recorder[BEMBody->first] = FrTimeRecorder<GeneralizedVelocity>(m_Te, m_dt);
            }
            m_recorder[BEMBody->first].Initialize();
        }
    }

    void FrRadiationConvolutionModel::Clear() {
        for (auto &BEMBody : *m_HDB) {
            m_recorder[BEMBody.first].Clear();
        }
    }

    void FrRadiationConvolutionModel::Compute(double time) {

        if (std::abs(time - GetSystem()->GetTime()) < 0.1*GetSystem()->GetTimeStep() and
                time > FLT_EPSILON) return;

        // Update speed recorder
        for (auto BEMBody = m_HDB->begin(); BEMBody != m_HDB->end(); BEMBody++) {
            auto eqFrame = m_HDB->GetMapper()->GetEquilibriumFrame(BEMBody->first);
            m_recorder[BEMBody->first].Record(time, eqFrame->GetPerturbationGeneralizedVelocityInFrame());
        }

        for (auto BEMBody=m_HDB->begin(); BEMBody!=m_HDB->end(); ++BEMBody) {

            auto radiationForce = GeneralizedForce();
            radiationForce.SetNull();

            for (auto BEMBodyMotion = m_HDB->begin(); BEMBodyMotion != m_HDB->end(); ++BEMBodyMotion) {

                auto velocity = m_recorder[BEMBodyMotion->first].GetData();

                auto vtime = m_recorder[BEMBodyMotion->first].GetTime();

                for (auto idof : BEMBodyMotion->first->GetListDOF()) {

                    auto interpK = BEMBody->first->GetIRFInterpolatorK(BEMBodyMotion->first, idof);

                    std::vector<mathutils::Vector6d<double>> kernel;
                    kernel.reserve(vtime.size());
                    for (unsigned int it = 0; it < vtime.size(); ++it) {
                        kernel.push_back(interpK->Eval(vtime[it]) * velocity.at(it).at(idof));
                    }
                    radiationForce += TrapzLoc(vtime, kernel);
                }
            }

            auto eqFrame = m_HDB->GetMapper()->GetEquilibriumFrame(BEMBody->first);
            auto meanSpeed = eqFrame->GetVelocityInFrame();

            if (meanSpeed.squaredNorm() > FLT_EPSILON) {
                radiationForce += ConvolutionKu(meanSpeed.norm());
            }

            auto forceInWorld = eqFrame->ProjectVectorFrameInParent(radiationForce.GetForce(), NWU);
            auto TorqueInWorld = eqFrame->ProjectVectorFrameInParent(radiationForce.GetTorque(), NWU);

            m_radiationForce[BEMBody->first] = - GeneralizedForce(forceInWorld, TorqueInWorld);
        }
    }

    void FrRadiationConvolutionModel::StepFinalize() {
        // Serialize and send the message log
        FrObject::SendLog();
    }

    void FrRadiationConvolutionModel::GetImpulseResponseSize(double &Te, double &dt, unsigned int &N) const {

        auto timeStep = m_system->GetTimeStep();

        auto freqStep = m_HDB->GetStepFrequency();

        Te = 0.5 * MU_2PI / freqStep;

        N = (unsigned int)floor(Te / timeStep);

        dt = Te / double(N-1);
    }

    GeneralizedForce FrRadiationConvolutionModel::ConvolutionKu(double meanSpeed) const {

        auto radiationForce = GeneralizedForce();
        radiationForce.SetNull();

        for (auto BEMBody = m_HDB->begin(); BEMBody != m_HDB->end(); BEMBody++) {

            auto Ainf = BEMBody->first->GetSelfInfiniteAddedMass();

            for (auto BEMBodyMotion = m_HDB->begin(); BEMBodyMotion != m_HDB->end(); BEMBodyMotion++) {

                auto velocity = m_recorder.at(BEMBodyMotion->first).GetData();
                auto vtime = m_recorder.at(BEMBodyMotion->first).GetTime();

                for (unsigned int idof=4; idof<6; idof++) {

                    auto interpKu = BEMBody->first->GetIRFInterpolatorKu(BEMBodyMotion->first, idof);

                    std::vector<mathutils::Vector6d<double>> kernel;
                    for (unsigned int it = 0; it < vtime.size(); ++it) {
                        kernel.push_back(interpKu->Eval(vtime[it]) * velocity[it].at(idof));
                    }
                    radiationForce += TrapzLoc(vtime, kernel) * meanSpeed ;
                }

                auto eqFrame = m_HDB->GetMapper()->GetEquilibriumFrame(BEMBodyMotion->first);
                auto angular = eqFrame->GetAngularPerturbationVelocityInFrame();

                auto damping = Ainf.col(2) * angular.y() - Ainf.col(1) * angular.z();
                radiationForce += meanSpeed * damping;
            }

        }

        return radiationForce;

    }

    void FrRadiationConvolutionModel::SetImpulseResponseSize(FrBEMBody *BEMBody, double Te, double dt) {
        m_recorder[BEMBody] = FrTimeRecorder<GeneralizedVelocity>(Te, dt);
    }

    void FrRadiationConvolutionModel::SetImpulseResponseSize(FrBody* body, double Te, double dt) {
        auto BEMBody = m_HDB->GetBody(body);
        this->SetImpulseResponseSize(BEMBody, Te, dt);
    }

    void FrRadiationConvolutionModel::SetImpulseResponseSize(double Te, double dt) {
        assert(Te > DBL_EPSILON);
        assert(dt > DBL_EPSILON);
        m_Te = Te;
        m_dt = dt;
    }

    void FrRadiationConvolutionModel::InitializeLog() {

        if (IsLogged()) {

            // Build the path to the radiation convolution model log
            auto logPath = m_system->GetPathManager()->BuildPath(this, fmt::format("{}_{}.csv",GetTypeName(),GetShortenUUID()));

            // Add the fields to be logged here
            // TODO: A completer
            m_message->AddField<double>("time", "s", "Current time of the simulation",
                                        [this]() { return m_system->GetTime(); });

            // Initialize the message
            FrObject::InitializeLog(logPath);

        }

    }

    std::shared_ptr<FrRadiationConvolutionModel>
    make_radiation_convolution_model(std::shared_ptr<FrHydroDB> HDB, FrOffshoreSystem* system){

        // This subroutine creates and adds the radiation convulation model to the offshore system from the HDB.

        // Construction and initialization of the classes dealing with radiation models.
        auto radiationModel = std::make_shared<FrRadiationConvolutionModel>(HDB);

        // Addition to the system.
        system->AddPhysicsItem(radiationModel);

        return radiationModel;
    }

}  // end namespace frydom

