//
// Created by frongere on 11/01/18.
//

#include "FrRadiationModel.h"

#include "FrRadiationForce.h"

/// <<<<<<<<<<<<<<<<<<<<<< Refactoring

#include "frydom/hydrodynamics/FrEquilibriumFrame.h"

namespace frydom {



    FrRadiationConvolutionModel::FrRadiationConvolutionModel(FrHydroDB *HDB, FrOffshoreSystem *system) : FrRadiationModel(HDB, system) {}

    void FrRadiationConvolutionModel::Initialize() {

        // TODO: verifier que m_HDB et m_system sont bien renseignes

        // Base class initialization
        FrRadiationModel::Initialize();

        // ======================
        // Initializing recorders
        // ======================

        // Getting the required characteristics of the impulse response functions (Te, dt, N)
        double Te, dt;
        unsigned int N;
        GetImpulseResponseSize(Te, dt, N);

        // Initialisation of every recorder we need
        auto NbBodies = m_HDB->GetNbBodies();

        // Getting the hydrodynamic mapper
        auto mapper = m_system->GetHydroMapper(m_HydroMapIndex);

        // We have one velocity recorder by hydrodynamic body
        m_recorders.reserve(NbBodies);

        FrHydroBody* hydroBody;
        for (unsigned int ibody=0; ibody<NbBodies; ibody++) {

            auto recorder = FrPerturbationVelocityRecorder();

            hydroBody = mapper->GetHydroBody(ibody);

            recorder.SetBody(hydroBody);

            recorder.SetSize(N);
            recorder.Initialize();

            m_recorders.push_back(recorder);

        }

        // Initializing Impulse response functions with correct parameters (same as recorder)
        m_HDB->GenerateImpulseResponseFunctions(Te, dt);

        if (m_speed_dependent) {
            m_HDB->GenerateSpeedDependentIRF();
        }

    }

    void FrRadiationConvolutionModel::Update(double time) {

        double stepSize = m_system->GetStep();  // FIXME: le pas de temps n'est pas le bon ici !! (mais on s'en fout un peu en fait)

        if (std::abs(m_time - time) < 0.1*stepSize) return; // No update if the model is already at the current simulation time

        m_time = time;

        // Recording the current velocities
        for (auto& recorder : m_recorders) {
            recorder.RecordVelocity();
        }

        // Here we compute the radiation forces by computing the convolutions
        ResetRadiationForceVector();

        auto nbBodies = GetNbInteractingBodies();
//            auto N = m_recorders[0].GetSize();

        auto N = m_HDB ->GetNbTimeSamples() - 1; // FIXME: le -1 est un quick hack pour le moment, la longueur des IRFs n'est pas la bonne...
        double val;

        // Loop on bodies that get force
        for (unsigned int iforceBody=0; iforceBody<nbBodies; iforceBody++) {
            chrono::ChVectorDynamic<double>& generalizedForce = m_radiationForces[iforceBody];
            // TODO: verifier qu'on a bie zero ici !!
            auto bemBody_i = m_HDB->GetBody(iforceBody);

            // Loop on bodies that are moving
            for (unsigned int imotionBody=0; imotionBody<nbBodies; imotionBody++) {

                // Loop on DOF of body imotioBody
                for (unsigned int idof=0; idof<6; idof++) {

                    // Getting the historic of motion of body imotionBody along DOF imotion
                    auto velocity = m_recorders[imotionBody].GetRecordOnDOF(idof);

                    // Loop over the force elements
                    for (unsigned int iforce = 0; iforce < 6; iforce++) {

                        // Getting the convolution kernel that goes well...
                        auto kernel = bemBody_i->GetImpulseResponseFunction(imotionBody, idof, iforce);

                        // Performing the multiplication
                        auto product = std::vector<double>(N);
                        for (unsigned int itime = 0; itime < N; itime++) {
                            product[itime] = kernel[itime] * velocity[itime];
                        }

                        // Numerical integration
                        val = Trapz(product, stepSize);

                        // Update of the force
                        // FIXME: verification du signe  !!!
                        generalizedForce.ElementN(
                                iforce) += val;  // TODO: bien verifier qu'on fait bien du inplace dans m_radiationForces !

                    }  // End loop iforce

                }  // End loop imotion of body imotionBody
            }  // End loop imotionBody

            if (m_speed_dependent) {
                UpdateSpeedDependentTerm(bemBody_i, iforceBody, generalizedForce);
            }

        } // End loop iforceBody

    }

    void FrRadiationConvolutionModel::UpdateSpeedDependentTerm(std::shared_ptr<FrBEMBody> &bemBody_i,
                                                               unsigned int iforceBody,
                                                               chrono::ChVectorDynamic<double> &generalizedForce) {

        double stepSize = m_system->GetStep();
        auto N = m_HDB->GetNbTimeSamples() - 1;
        auto hydroBody = m_system->GetHydroMapper(m_HydroMapIndex)->GetHydroBody(iforceBody);

        auto Ainf = bemBody_i->GetSelfInfiniteAddedMass();

        auto nbBodies = GetNbInteractingBodies();
        double val;

        auto steady_speed = hydroBody->GetSteadyVelocity();
        auto mean_speed = steady_speed.Length();

        for (unsigned int imotionBody=0; imotionBody<nbBodies; imotionBody++) {

            for (unsigned int idof=0; idof<6; idof++) {

                auto velocity = m_recorders[imotionBody].GetRecordOnDOF(idof);

                for (unsigned int iforce = 0; iforce < 6; iforce++) {               // FIXME : verifier si le max est pas 3

                    auto kernel = bemBody_i->GetSpeedDependentIRF(imotionBody, idof, iforce);
                    auto product = std::vector<double>(N);

                    for (unsigned int itime = 0; itime < N; itime++) {
                        product[itime] = kernel[itime] * velocity[itime];
                    }

                    val = Trapz(product, stepSize);
                    generalizedForce.ElementN(iforce) += mean_speed * val;
                }
            }

            auto velocity = hydroBody->GetAngularVelocityPert();

            for (unsigned int iforce=0; iforce<6; iforce++) {

                val = Ainf(iforce, 2) * velocity.y() - Ainf(iforce, 1) * velocity.z();
                generalizedForce.ElementN(iforce) += mean_speed * val;

            }

        }
    }

    void FrRadiationConvolutionModel::StepFinalize() {
        // Ici, on est a la fi du pas de temps, on peut declencher l'enregistrement de toutes les vitesses de corps
        // dans les recorders

        for (auto& recorder : m_recorders) {
            recorder.RecordVelocity();
        }

        // O declenche maintenant le recalcul des forces hydro
//            Update(m_environment->GetChTime()); // TODO: voir si c'est le bo endroit pour declencher ??
        // Pas certain, pour des solveurs avec plus de pas intermediaires, les forces de radiation ne seront pas a jour !!!

    }

    void FrRadiationConvolutionModel::GetImpulseResponseSize(double &Te, double &dt, unsigned int &N) const {

        // Getting the simulation time step
        auto timeStep = m_system->GetStep();

        // Frequency step used in the HDB
        auto freqStep = m_HDB->GetStepFrequency();

        // Maximum usefull time for the impulse response function
        Te = 0.5 * MU_2PI / freqStep;

        // Size of recorders that will store Te duration of velocities at timeStep sampling
        N = (unsigned int)floor(Te / timeStep);

        dt = Te / double(N-1);

    }

    FrRadiationModel::FrRadiationModel(FrHydroDB *HDB, FrOffshoreSystem *system) : m_HDB(HDB), m_system(system) {}

    void FrRadiationModel::SetHydroMapIndex(const int id) { m_HydroMapIndex = id; }

    int FrRadiationModel::GetHydroMapIndex() const { return m_HydroMapIndex; }

    void FrRadiationModel::SetHydroDB(FrHydroDB *HDB) { m_HDB = HDB; }

    FrHydroDB *FrRadiationModel::GetHydroDB() const { return m_HDB; }

    void FrRadiationModel::SetSystem(FrOffshoreSystem *system) { m_system = system; }

    FrOffshoreSystem *FrRadiationModel::GetSystem() const { return m_system; }

    unsigned int FrRadiationModel::GetNbInteractingBodies() const {
        return m_HDB->GetNbBodies();
    }

    void FrRadiationModel::Initialize() {
        // Initializing the radiation force vector with sufficient number of elements
        auto nbBodies = GetNbInteractingBodies();

        m_radiationForces.reserve(nbBodies);
        for (unsigned int ibody=0; ibody<nbBodies; ibody++) {
            m_radiationForces.emplace_back(chrono::ChVectorDynamic<double>(6));
        }

    }

    std::shared_ptr<FrHydroMapper> FrRadiationModel::GetMapper() const {
        // TODO: le mapper doit etre cree lors de l'instanciation de radModel et pas stocke dans system !!
        // On parlera plutot de FrLinearHydroModel !! auquel on attache une HDB (et on a un loadHDB dans cette classe)
        return m_system->GetHydroMapper(m_HydroMapIndex);
    }

    void FrRadiationModel::ResetRadiationForceVector() {
        // Setting everything to zero to prepare for a new summation
        for (auto& force : m_radiationForces) {
            for (unsigned int k=0; k<6; k++) {
                force.ElementN(k) = 0.;
            }
        }
    }

    void FrRadiationModel::GetRadiationForce(FrHydroBody *hydroBody, chrono::ChVector<double> &force,
                                             chrono::ChVector<double> &moment) {

        auto mapper = m_system->GetHydroMapper(m_HydroMapIndex);

        auto iBEMBody = mapper->GetBEMBodyIndex(hydroBody);  // FIXME: le mapper renvoie pour le moment un pointeur vers le BEMBody, pas son index !!

        auto generalizedForce = m_radiationForces[iBEMBody];

        force.x() = - generalizedForce.ElementN(0);
        force.y() = - generalizedForce.ElementN(1);
        force.z() = - generalizedForce.ElementN(2);

        moment.x() = - generalizedForce.ElementN(3);
        moment.y() = - generalizedForce.ElementN(4);
        moment.z() = - generalizedForce.ElementN(5);

        // Warning, the moment that is returned is expressed int the absolute NWU frame such as done in Seakeeping

    }

    void FrRadiationModel::SetSpeedDependent(bool time_dependent) { m_speed_dependent = time_dependent; }

    std::shared_ptr<FrRadiationConvolutionForce>
    FrRadiationConvolutionModel::AddRadiationForceToHydroBody(std::shared_ptr<FrHydroBody> hydroBody) {

        // Getting the bem index of the body
        auto iBEMBody = m_system->GetHydroMapper(m_HydroMapIndex)->GetBEMBodyIndex(hydroBody);

        auto radiationForce = std::make_shared<FrRadiationConvolutionForce>(shared_from_this());

        // Activate radiation model to include added mass to the system
        m_system->GetHydroMapper(m_HydroMapIndex)->GetBEMBody(hydroBody)->SetRadiationActive(true);
        m_system->GetHydroMapper(m_HydroMapIndex)->GetBEMBody(hydroBody)->SetBEMVariables();

        // Adding the force to the body
        hydroBody->AddForce(radiationForce);

        return radiationForce;
    }

























    /// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< REFACTORING

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

    FrRadiationModel_::FrRadiationModel_() {
        m_chronoPhysicsItem = std::make_shared<internal::FrAddedMassBase>(this);
    }

    FrRadiationModel_::FrRadiationModel_(std::shared_ptr<FrHydroDB_> HDB) : m_HDB(HDB) {
        m_chronoPhysicsItem = std::make_shared<internal::FrAddedMassBase>(this);
    }

    void FrRadiationModel_::Initialize() {

    }

    FrHydroMapper_* FrRadiationModel_::GetMapper() const {
        return m_HDB->GetMapper();
    }

    Force FrRadiationModel_::GetRadiationForce(FrBEMBody_ *BEMBody) const {
        return m_radiationForce.at(BEMBody).GetForce();
    }

    Force FrRadiationModel_::GetRadiationForce(FrBody_* body) const {
        auto BEMBody = m_HDB->GetBody(body);
        return m_radiationForce.at(BEMBody).GetForce();
    }

    Torque FrRadiationModel_::GetRadiationTorque(FrBEMBody_* BEMBody) const {
        return m_radiationForce.at(BEMBody).GetTorque();
    }

    Torque FrRadiationModel_::GetRadiationTorque(FrBody_* body) const {
        auto BEMBody = m_HDB->GetBody(body);
        return m_radiationForce.at(BEMBody).GetTorque();
    }

    void FrRadiationModel_::Update(double time) {

    }

    // ----------------------------------------------------------------
    // Radiation model with convolution
    // ----------------------------------------------------------------

    void FrRadiationConvolutionModel_::Initialize() {

        double Te, dt;
        unsigned int N;
        GetImpulseResponseSize(Te, dt, N);

        std::cout << "Initialize radiation convolution model" << std::endl;

        for (auto BEMBody=m_HDB->begin(); BEMBody!=m_HDB->end(); ++BEMBody) {

            m_recorder[BEMBody->get()] = FrTimeRecorder_<GeneralizedVelocity>(Te, dt);
            m_recorder[BEMBody->get()].Initialize();

            auto body = m_HDB->GetBody(BEMBody->get());
            body->AddExternalForce(std::make_shared<FrRadiationConvolutionForce_>(this));

            std::cout << "Adding radiation force" << std::endl;
        }
    }

    void FrRadiationConvolutionModel_::Update(double time) {

        // Update speed recorder
        for (auto BEMBody = m_HDB->begin(); BEMBody != m_HDB->end(); BEMBody++) {
            auto eqFrame = m_HDB->GetMapper()->GetEquilibriumFrame(BEMBody->get());
            m_recorder[BEMBody->get()].Record(time, eqFrame->GetPerturbationGeneralizedVelocityInFrame());

            //##CC
            //auto vp = eqFrame->GetPerturbationGeneralizedVelocityInFrame().GetVelocity();
            //auto vrp = eqFrame->GetPerturbationGeneralizedVelocityInFrame().GetAngularVelocity();
            //std::cout << "debug: radiation force: record perturbation velocity:"
            //          << vp.x() <<  ";" << vp.y() << ";" << vp.z()
            //          << vrp.x() << ";"  << vrp.y() << ";"  << vrp.z() << std::endl;
            //##CC
        }

        for (auto BEMBody=m_HDB->begin(); BEMBody!=m_HDB->end(); ++BEMBody) {

            auto radiationForce = GeneralizedForce();

            for (auto BEMBodyMotion = m_HDB->begin(); BEMBodyMotion != m_HDB->end(); ++BEMBodyMotion) {

                auto velocity = m_recorder[BEMBodyMotion->get()].GetData();

                auto vtime = m_recorder[BEMBodyMotion->get()].GetTime();

                for (unsigned int idof = 0; idof < 6; idof++) {

                    auto interpK = BEMBody->get()->GetIRFInterpolatorK(BEMBodyMotion->get(), idof);

                    std::vector<mathutils::Vector6d<double>> kernel;
                    kernel.reserve(vtime.size());
                    for (unsigned int it = 0; it < vtime.size(); ++it) {
                        kernel.push_back(interpK->Eval(vtime[it]).cwiseProduct(velocity.at(it)));

                        //##CC
                        //std::cout << "debug: radiation force: time= " << vtime[it] << ";"
                        //          << "kernel:" << kernel[it].at(0) << ";" << kernel[it].at(1) << ";" << kernel[it].at(2)
                        //          << ";" << kernel[it].at(3) << ";" << kernel[it].at(4) << ";" << kernel[it].at(5)
                        //          << std::endl;
                        //##CC
                    }
                    radiationForce += TrapzLoc(vtime, kernel);
                }
            }

            auto eqFrame = m_HDB->GetMapper()->GetEquilibriumFrame(BEMBody->get());
            auto meanSpeed = eqFrame->GetVelocityInFrame();

            // ##CC
            //std::cout << "debug: radiation force: meanSpeed = " << meanSpeed.x() << ";"
            //          << meanSpeed.y() << ";" << meanSpeed.z() << std::endl;
            //##CC

            if (meanSpeed.squaredNorm() > FLT_EPSILON) {

                //##CC
                //std::cout << "debug: radiation force: compute Ku" << std::endl;
                //##CC
                radiationForce += ConvolutionKu(meanSpeed.norm());
            }

            auto forceInWorld = eqFrame->ProjectVectorFrameInParent(radiationForce.GetForce(), NWU);
            auto TorqueInWorld = eqFrame->ProjectVectorFrameInParent(radiationForce.GetTorque(), NWU);

            // ##CC
            //std::cout << "debug: radiation force Fx=" << forceInWorld.GetFx() << " Fy=" << forceInWorld.GetFy()
            //          << " Fz=" << forceInWorld.GetFz() << " Mx=" << TorqueInWorld.GetMx()
            //          << " My=" << TorqueInWorld.GetMy() << " Mz=" << TorqueInWorld.GetMz() << std::endl;
            // ##CC

            m_radiationForce[BEMBody->get()] = - GeneralizedForce(forceInWorld, TorqueInWorld);
        }
    }

    void FrRadiationConvolutionModel_::StepFinalize() {

    }

    void FrRadiationConvolutionModel_::GetImpulseResponseSize(double &Te, double &dt, unsigned int &N) const {

        auto timeStep = m_system->GetTimeStep();

        auto freqStep = m_HDB->GetStepFrequency();

        Te = 0.5 * MU_2PI / freqStep;

        N = (unsigned int)floor(Te / timeStep);

        dt = Te / double(N-1);
    }

    GeneralizedForce FrRadiationConvolutionModel_::ConvolutionKu(double meanSpeed) const {

        auto radiationForce = GeneralizedForce();

        for (auto BEMBody = m_HDB->begin(); BEMBody != m_HDB->end(); BEMBody++) {

            auto Ainf = BEMBody->get()->GetSelfInfiniteAddedMass();

            for (auto BEMBodyMotion = m_HDB->begin(); BEMBodyMotion != m_HDB->end(); BEMBodyMotion++) {

                auto velocity = m_recorder.at(BEMBodyMotion->get()).GetData();
                auto vtime = m_recorder.at(BEMBodyMotion->get()).GetTime();

                for (unsigned int idof=0; idof<6; idof++) {

                    auto interpKu = BEMBody->get()->GetIRFInterpolatorKu(BEMBodyMotion->get(), idof);

                    std::vector<mathutils::Vector6d<double>> kernel;
                    for (unsigned int it = 0; it < vtime.size(); ++it) {
                        kernel.push_back(interpKu->Eval(vtime[it]).cwiseProduct(velocity[it]));
                    }
                    radiationForce += TrapzLoc(vtime, kernel) * meanSpeed ;
                }

                auto eqFrame = m_HDB->GetMapper()->GetEquilibriumFrame(BEMBodyMotion->get());
                auto angular = eqFrame->GetAngularPerturbationVelocityInFrame();

                auto damping = Ainf.col(2) * angular.y() - Ainf.col(1) * angular.x();
                radiationForce += meanSpeed * damping;
            }

        }

        return radiationForce;

    }












}

