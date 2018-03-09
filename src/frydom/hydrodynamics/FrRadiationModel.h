//
// Created by frongere on 11/01/18.
//

#ifndef FRYDOM_FRRADIATIONMODEL_H
#define FRYDOM_FRRADIATIONMODEL_H

#include "frydom/core/FrOffshoreSystem.h"
#include "frydom/core/FrObject.h"
#include "FrHydroDB.h"
//#include "FrRadiationForce.h
#include "FrHydroMapper.h"
#include "FrVelocityRecorder.h"

namespace frydom {

    /// Base class
    class FrRadiationModel : public FrObject {

    protected:
        FrHydroDB* m_HDB = nullptr;
        FrOffshoreSystem* m_system = nullptr;

        double m_time = -1.;  // Quick hack to force update at first time step...

        std::vector<chrono::ChVectorDynamic<double>> m_radiationForces;


    public:
        FrRadiationModel() = default;

        explicit FrRadiationModel(FrHydroDB* HDB, FrOffshoreSystem* system) : m_HDB(HDB), m_system(system) {}


        void SetHydroDB(FrHydroDB* HDB) { m_HDB = HDB; }

        FrHydroDB* GetHydroDB() const { return m_HDB; }

        void SetSystem(FrOffshoreSystem* system) { m_system = system; }

        FrOffshoreSystem* GetSystem() const { return m_system; }

        unsigned int GetNbInteractingBodies() const {
            return m_HDB->GetNbBodies();
        }

        void Initialize() override {
            // Initializing the radiation force vector with sufficient number of elements
            auto nbBodies = GetNbInteractingBodies();

            m_radiationForces.reserve(nbBodies);
            for (unsigned int ibody=0; ibody<nbBodies; ibody++) {
                m_radiationForces.emplace_back(chrono::ChVectorDynamic<double>(6));
            }

        }

        std::shared_ptr<FrHydroMapper> GetMapper() const {
            // TODO: le mapper doit etre cree lors de l'instanciation de radModel et pas stocke dans system !!
            // On parlera plutot de FrLinearHydroModel !! auquel on attache une HDB (et on a un loadHDB dans cette classe)
            return m_system->GetHydroMapper();
        }

        void ResetRadiationForceVector() {
            // Setting everything to zero to prepare for a new summation
            for (auto& force : m_radiationForces) {
                for (unsigned int k=0; k<6; k++) {
                    force.ElementN(k) = 0.;
                }
            }
        }

        virtual void Update(double time) = 0;

        void GetRadiationForce(FrHydroBody* hydroBody,
                               chrono::ChVector<double>& force, chrono::ChVector<double>& moment) {

            auto mapper = m_system->GetHydroMapper();

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


    };

    class FrRadiationConvolutionForce;


    class FrRadiationConvolutionModel :
            public FrRadiationModel,
            public std::enable_shared_from_this<FrRadiationConvolutionModel>
    {

    private:
        // Recorders for the velocity of bodies that are in interaction
        std::vector<FrVelocityRecorder> m_recorders;

    public:

        FrRadiationConvolutionModel(FrHydroDB* HDB, FrOffshoreSystem* system) : FrRadiationModel(HDB, system) {}


        void Initialize() override {

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
            auto mapper = m_system->GetHydroMapper();

            // We have one velocity recorder by hydrodynamic body
            m_recorders.reserve(NbBodies);

            FrHydroBody* hydroBody;
            for (unsigned int ibody=0; ibody<NbBodies; ibody++) {

                // Gettig the corresponding HydroBody


                auto recorder = FrVelocityRecorder();

                hydroBody = mapper->GetHydroBody(ibody);

                recorder.SetBody(hydroBody);

                recorder.SetSize(N);
                recorder.Initialize();

                m_recorders.push_back(recorder);

            }

            // Initializing Impulse response functions with correct parameters (same as recorder)
            m_HDB->GenerateImpulseResponseFunctions(Te, dt);

        }

        // It has to be called by force models and updated only once, the first time it is called
        void Update(double time) override {

            double stepSize = m_system->GetStep();  // FIXME: le pas de temps n'est pas le bon ici !! (mais on s'en fout un peu en fait)

            if (std::abs(m_time - time) < 0.1*stepSize) return; // No update if the model is already at the current simulation time

            m_time = time;

            // Recording the current velocities
            for (auto& recorder : m_recorders) {
                recorder.RecordVelocity();
            }


            // Here we compute the radiation forces by computin the convolutions
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
                        for (unsigned int iforce=0; iforce<6; iforce++) {

                            // Getting the convolution kernel that goes well...
                            auto kernel = bemBody_i->GetImpulseResponseFunction(imotionBody, idof, iforce);

//                            std::cout << kernel.rows() << std::endl;

                            // Performing the multiplication
                            auto product = std::vector<double>(N);
                            for (unsigned int itime=0; itime<N; itime++) {
                                product[itime] = kernel[itime] * velocity[itime];
                            }

                            // Numerical integration
                            val = Trapz(product, stepSize);

                            // Update of the force
                            // FIXME: verification du signe  !!!
                            generalizedForce.ElementN(iforce) -= val;  // TODO: bien verifier qu'on fait bien du inplace dans m_radiationForces !

                        }  // End loop iforce
                    }  // End loop imotion of body imotionBody
                }  // End loop imotionBody
            } // End loop iforceBody

        }

        std::shared_ptr<FrRadiationConvolutionForce> AddRadiationForceToHydroBody(std::shared_ptr<FrHydroBody>);


        void StepFinalize() override {
            // Ici, on est a la fi du pas de temps, on peut declencher l'enregistrement de toutes les vitesses de corps
            // dans les recorders

            for (auto& recorder : m_recorders) {
                recorder.RecordVelocity();
            }

            // O declenche maintenant le recalcul des forces hydro
//            Update(m_system->GetChTime()); // TODO: voir si c'est le bo endroit pour declencher ??
            // Pas certain, pour des solveurs avec plus de pas intermediaires, les forces de radiation ne seront pas a jour !!!

        }




    private:
        void GetImpulseResponseSize(double& Te, double &dt, unsigned int& N) const {

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


    };



}  // end namespace frydom


#endif //FRYDOM_FRRADIATIONMODEL_H
