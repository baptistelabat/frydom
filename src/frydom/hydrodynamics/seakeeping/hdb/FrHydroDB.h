//
// Created by frongere on 17/10/17.
//

#ifndef FRYDOM_FRHYDRODB_H
#define FRYDOM_FRHYDRODB_H

#include <vector>

#include "boost/bimap.hpp"


#include "MathUtils/MathUtils.h"
//#include "frydom/utils/FrEigen.h" // TODO: Eigen est maintenant deja importe de MathUtils... ne plus reposer sur le sous module frydom

#include "frydom/core/FrVector.h"

#include "frydom/core/FrBody.h"


#include "FrBEMBody.h"

// TODO: utiliser plutot des std::vector a la place des matrices eigen ...

using namespace mathutils;

namespace frydom {

    // Forward declarations
    class FrHydroMapper;


    class FrHydroDB {

    // =================================================================================================================

    private:

        double m_GravityAcc            = 9.81;
        double m_NormalizationLength   = 1.;
        double m_WaterDensity          = 1000.;
        double m_WaterDepth            = 0.;

        std::vector<std::shared_ptr<FrBEMBody>> m_Bodies;
        FrDiscretization1D m_FrequencyDiscretization;
        FrDiscretization1D m_WaveDirectionDiscretization;
        FrDiscretization1D m_TimeDiscretization;

    public:

        FrHydroDB() = default;

        FrHydroDB(std::string h5file);

        unsigned int GetNbBodies() const { return (unsigned int)m_Bodies.size(); }

        double GetGravityAcc() const { return m_GravityAcc; }

        void SetGravityAcc(const double GravityAcc) { m_GravityAcc = GravityAcc; }

        double GetNormalizationLength() const { return m_NormalizationLength; }

        void SetNormalizationLength(const double NormalizationLength) { m_NormalizationLength = NormalizationLength; }

        double GetWaterDensity() const { return m_WaterDensity; }

        void SetWaterDensity(const double WaterDensity) { m_WaterDensity = WaterDensity; }

        double GetWaterDepth() const { return m_WaterDepth; }

        void SetWaterDepth(const double WaterDepth) { m_WaterDepth = WaterDepth; }

        void SetFrequencyDiscretization(const double MinFreq, const double MaxFreq, const unsigned int NbFreq) {
            m_FrequencyDiscretization.SetMin(MinFreq);
            m_FrequencyDiscretization.SetMax(MaxFreq);
            m_FrequencyDiscretization.SetNbSample(NbFreq);
        }

        std::vector<double> GetFrequencies() const { return m_FrequencyDiscretization.GetVector(); } // TODO: gerer les unites...

        void SetWaveDirectionDiscretization(const double MinAngle, const double MaxAngle, const unsigned int NbAngle) {
            m_WaveDirectionDiscretization.SetMin(MinAngle);
            m_WaveDirectionDiscretization.SetMax(MaxAngle);
            m_WaveDirectionDiscretization.SetNbSample(NbAngle);
        }

        std::vector<double> GetWaveDirections() const { return m_WaveDirectionDiscretization.GetVector(); } // TODO: gerer les unites...

        void SetTimeDiscretization(const double FinalTime, const unsigned int NbTimeSamples) {
            m_TimeDiscretization.SetMin(0.);
            m_TimeDiscretization.SetMax(FinalTime);
            m_TimeDiscretization.SetNbSample(NbTimeSamples);
        }

        unsigned int GetNbFrequencies() const { return m_FrequencyDiscretization.GetNbSample(); }

        double GetMaxFrequency() const { return m_FrequencyDiscretization.GetMax(); }

        double GetMinFrequency() const { return m_FrequencyDiscretization.GetMin(); }

        double GetStepFrequency() const {return m_FrequencyDiscretization.GetStep(); }

        unsigned int GetNbWaveDirections() const { return m_WaveDirectionDiscretization.GetNbSample(); }

        unsigned int GetNbTimeSamples() const { return m_TimeDiscretization.GetNbSample(); }

        double GetFinalTime() const { return m_TimeDiscretization.GetMax(); }

        double GetTimeStep() const { return m_TimeDiscretization.GetStep(); }

        std::shared_ptr<FrBEMBody> NewBody(std::string BodyName) {
            auto BEMBody = std::make_shared<FrBEMBody>(GetNbBodies(), BodyName);
            BEMBody->SetHDB(this);
            m_Bodies.push_back(BEMBody);
            return BEMBody;
        }

        std::shared_ptr<FrBEMBody> GetBody(unsigned int ibody) { return m_Bodies[ibody]; }

        void GenerateImpulseResponseFunctions(double tf = 30., double dt = 0.);

        void GenerateSpeedDependentIRF();

        std::shared_ptr<FrHydroMapper> GetMapper();

        void IntLoadResidual_Mv(const unsigned int off, chrono::ChVectorDynamic<>& Res,
                                const chrono::ChVectorDynamic<>& w, const double c) ;

        //void VariablesFbIncrementMq();

    };  // end class FrHydroDB




















    ////////////// REFACTORING --------------->>>>>>>>>>>>>>>>>>>>

    // Forward declaration
    class FrHydroDB_;

    class FrBEMBody_ {

        // =================================================================================================================

    private:
//        FrHydroBody* m_hydroBody = nullptr;  // TODO: est-ce qu'on utilise un pointeur vers un hydrobody ou bien une bimap dans la HDB ??


        FrHydroDB_* m_HDB = nullptr;

        unsigned int m_ID;
        std::string m_bodyName;

        Position m_BodyPosition;  // Ne peut etre utilise pour renvoyer un warning si le corps n'est pas proche de cette position en seakeeping sans vitesse d'avance

        std::vector<FrBEMForceMode> m_ForceModes;
        std::vector<FrBEMMotionMode> m_MotionModes;

        Eigen::Matrix<bool, Eigen::Dynamic, Eigen::Dynamic> m_ExcitationMask;
        std::vector<Eigen::MatrixXcd> m_Diffraction;
        std::vector<Eigen::MatrixXcd> m_FroudeKrylov;
        std::vector<Eigen::MatrixXcd> m_Excitation;

        std::vector<Eigen::Matrix<bool, Eigen::Dynamic, Eigen::Dynamic>> m_RadiationMask;
        std::vector<Eigen::MatrixXd> m_InfiniteAddedMass;
        std::vector<std::vector<Eigen::MatrixXd>> m_AddedMass;
        std::vector<std::vector<Eigen::MatrixXd>> m_RadiationDamping;
        std::vector<std::vector<Eigen::MatrixXd>> m_ImpulseResponseFunction;
        std::vector<std::vector<Eigen::MatrixXd>> m_SpeedDependentIRF;

        std::vector<std::vector<Interp1dLinear<double, std::complex<double>>>> m_waveDirInterpolators;

        bool m_radiation_active;

//        friend FrHydroDB_ LoadHDB5_(std::string);

    public:

//        FrBEMBody_() = default;
        FrBEMBody_(unsigned int ID, std::string& BodyName, FrHydroDB_* hdb);

//        FrHydroDB* GetHDB() const { return m_HDB; }

        void SetName(const std::string& BodyName) { m_bodyName = BodyName; }
        void SetWorldPosition(const Eigen::Vector3d &BodyPosition) { m_BodyPosition = BodyPosition; }

        unsigned int GetNbForceMode() const { return (uint)m_ForceModes.size(); }

        unsigned int GetNbMotionMode() const { return (uint)m_MotionModes.size(); }


        FrBEMForceMode* GetForceMode(unsigned int imode) {
            assert(imode < GetNbForceMode());
            return &m_ForceModes[imode];
        }

        FrBEMMotionMode* GetMotionMode(unsigned int imode) {
            assert(imode < GetNbMotionMode());
            return &m_MotionModes[imode];
        }

        unsigned int GetID() const { return m_ID; }

        void AddForceMode(FrBEMForceMode& mode) {
            m_ForceModes.push_back(mode);
        }

        void AddMotionMode(FrBEMMotionMode& mode) {
            m_MotionModes.push_back(mode);
        }

        unsigned int GetNbFrequencies() const;

        std::vector<double> GetFrequencies() const;

        std::vector<std::vector<double>>
        GetEncounterFrequencies(std::vector<double> waveFrequencies,
                                std::vector<double> waveDirections,
                                std::vector<double> waveNumbers,
                                chrono::ChVector<double> frame_velocity,
                                ANGLE_UNIT angleUnit);

        unsigned int GetNbWaveDirections() const;

        std::vector<double> GetWaveDirections() const;

        void FilterRadiation();

        void FilterExcitation();

        void Initialize();

        void Finalize() {
            ComputeExcitation();
            FilterExcitation();
            FilterRadiation();

            // TODO: Ici, on construit les interpolateurs
            BuildInterpolators();
        }

        void BuildInterpolators();

        void BuildWaveExcitationInterpolators();  // TODO: voir si on construit les interpolateurs pour la diffraction et Froude-Krylov

        std::vector<Eigen::MatrixXcd>
        GetExcitationInterp(std::vector<double> waveFrequencies, std::vector<double> waveDirections, ANGLE_UNIT angleUnit=DEG);

        std::vector<Eigen::MatrixXcd>
        GetExcitationInterp(std::vector<double> waveFrequencies, std::vector<double> waveDirection,
                            std::vector<double> waveNumbers, chrono::ChVector<double> frame_velocity,
                            ANGLE_UNIT angleUnit=DEG);

        void SetDiffraction(unsigned int iangle, const Eigen::MatrixXcd& diffractionMatrix);

        void SetFroudeKrylov(unsigned int iangle, const Eigen::MatrixXcd& froudeKrylovMatrix);

        void ComputeExcitation();

        void SetInfiniteAddedMass(unsigned int ibody, const Eigen::MatrixXd& CMInf);

        void SetAddedMass(unsigned int ibody, unsigned int idof, const Eigen::MatrixXd& CM);

        void SetRadiationDamping(unsigned int ibody, unsigned int idof, const Eigen::MatrixXd& CA);

        void SetImpulseResponseFunction(unsigned int ibody, unsigned int idof, const Eigen::MatrixXd& IRF);

        void SetSpeedDependentIRF(const unsigned int ibody, const unsigned int idof, const Eigen::MatrixXd& IRF);

        // FIXME: les GetDiffraction etc ne doivent pas specialement etre accessible en dehors des interpolations...
        // On utilisera donc plutot GetExcitation(omega, angles) ...

        Eigen::MatrixXcd GetDiffraction(unsigned int iangle) const;

        Eigen::VectorXcd GetDiffraction(unsigned int iangle, unsigned int iforce) const;

        Eigen::MatrixXcd GetFroudeKrylov(unsigned int iangle) const;

        Eigen::VectorXcd GetFroudeKrylov(unsigned int iangle, unsigned int iforce) const;

        Eigen::MatrixXcd GetExcitation(unsigned int iangle) const;

        Eigen::VectorXcd GetExcitation(unsigned int iangle, unsigned int iforce) const;

        Eigen::MatrixXd GetInfiniteAddedMass(unsigned int ibody) const;

        Eigen::MatrixXd GetSelfInfiniteAddedMass() const;

        Eigen::MatrixXd GetAddedMass(unsigned int ibody, unsigned int idof) const;

        Eigen::VectorXd GetAddedMass(unsigned int ibody, unsigned int idof, unsigned int iforce) const;

        Eigen::MatrixXd GetSelfAddedMass(unsigned int idof) const;

        Eigen::VectorXd GetSelfAddedMass(unsigned int idof, unsigned int iforce) const;

        Eigen::MatrixXd GetRadiationDamping(unsigned int ibody, unsigned int idof) const;

        Eigen::VectorXd GetRadiationDamping(unsigned int ibody, unsigned int idof, unsigned int iforce) const;

        Eigen::MatrixXd GetSelfRadiationDamping(unsigned int idof) const;

        Eigen::VectorXd GetselfRadiationDamping(unsigned int idof, unsigned int iforce) const;

        std::vector<Eigen::MatrixXd> GetImpulseResponseFunction(unsigned int ibody) const;

        Eigen::MatrixXd GetImpulseResponseFunction(unsigned int ibody, unsigned int idof) const;

        Eigen::VectorXd GetImpulseResponseFunction(unsigned int ibody, unsigned int idof, unsigned int iforce) const;

        Eigen::MatrixXd GetSelfImpulseResponseFunction(unsigned int idof) const;

        Eigen::VectorXd GetSelfImpulseResponseFunction(unsigned int idof, unsigned int iforce) const;

        std::vector<Eigen::MatrixXd> GetSpeedDependentIRF(unsigned int ibody) const;

        Eigen::MatrixXd GetSpeedDependentIRF(unsigned int ibody, unsigned int idof) const;

        Eigen::VectorXd GetSpeedDependentIRF(unsigned int ibody, unsigned int idof, unsigned int iforce) const;

        void GenerateImpulseResponseFunctions();

        void GenerateSpeedDependentIRF();

        virtual void IntLoadResidual_Mv(const unsigned int off,
                                        chrono::ChVectorDynamic<>& R,
                                        const chrono::ChVectorDynamic<>& w,
                                        const double c);

        void SetRadiationActive(const bool is_active) { m_radiation_active = is_active; }

        bool GetRadiationActive() const { return m_radiation_active; }

        //void VariablesFbIncrementMq();

        //void Compute_inc_Mb_v(chrono::ChMatrix<double>& result, const chrono::ChMatrix<double>& vect) const;

        void SetBEMVariables();
    };


//    typedef boost::bimaps::bimap<FrBody_*, FrBEMBody_*> myBimap;
//    typedef myBimap::value_type mapping;


    class FrBEMBodyMapper_ {

    private:

        using BodyMapper = boost::bimaps::bimap<FrBody_*, FrBEMBody_*>;

//        FrHydroDB* m_HDB;
        BodyMapper m_mapper;


    public:

        FrBEMBodyMapper_();

//        void Map(const std::shared_ptr<FrHydroBody> hydroBody, unsigned int iBEMBody);
//
//        unsigned int GetNbMappings() const;
//
//        FrHydroBody* GetHydroBody(unsigned int iBEMBody) const;
//
//        unsigned int GetBEMBodyIndex(std::shared_ptr<FrHydroBody> hydroBody);
//
//        unsigned int GetBEMBodyIndex(FrHydroBody* hydroBody);
//
//        std::shared_ptr<FrBEMBody> GetBEMBody(std::shared_ptr<FrHydroBody> hydroBody);
//
//        std::shared_ptr<FrBEMBody> GetBEMBody(FrHydroBody* hydroBody);
//
//        virtual void IntLoadResidual_Mv(const unsigned int off,
//                                        chrono::ChVectorDynamic<>& R,
//                                        const chrono::ChVectorDynamic<>& w,
//                                        const double c);
//
//        //virtual void VariablesFbIncrementMq() { m_HDB->VariablesFbIncrementMq(); }


    };











    // Forward declaration
//    class FrHydroDB_;





    // Forward declarations
    class FrHydroMapper_;


    class FrHydroDB_ {

    // =================================================================================================================

    private:

        double m_GravityAcc            = 9.81;
        double m_NormalizationLength   = 1.;
        double m_WaterDensity          = 1000.;
        double m_WaterDepth            = 0.;

        std::vector<std::unique_ptr<FrBEMBody_>> m_Bodies;
        FrDiscretization1D m_FrequencyDiscretization;
        FrDiscretization1D m_WaveDirectionDiscretization;
        FrDiscretization1D m_TimeDiscretization;

//        friend FrHydroDB_ LoadHDB5_(std::string);



    public:

//        FrHydroDB_() = default;

        FrHydroDB_(std::string hdb5File);

        unsigned int GetNbBodies() const { return (unsigned int)m_Bodies.size(); }

//        double GetGravityAcc() const { return m_GravityAcc; }
//
//        void SetGravityAcc(const double GravityAcc) { m_GravityAcc = GravityAcc; }
//
//        double GetNormalizationLength() const { return m_NormalizationLength; }
//
//        void SetNormalizationLength(const double NormalizationLength) { m_NormalizationLength = NormalizationLength; }
//
//        double GetWaterDensity() const { return m_WaterDensity; }
//
//        void SetWaterDensity(const double WaterDensity) { m_WaterDensity = WaterDensity; }
//
//        double GetWaterDepth() const { return m_WaterDepth; }
//
//        void SetWaterDepth(const double WaterDepth) { m_WaterDepth = WaterDepth; }
//
//        void SetFrequencyDiscretization(const double MinFreq, const double MaxFreq, const unsigned int NbFreq) {
//            m_FrequencyDiscretization.SetMin(MinFreq);
//            m_FrequencyDiscretization.SetMax(MaxFreq);
//            m_FrequencyDiscretization.SetNbSample(NbFreq);
//        }
//
//        std::vector<double> GetFrequencies() const { return m_FrequencyDiscretization.GetVector(); } // TODO: gerer les unites...
//
//        void SetWaveDirectionDiscretization(const double MinAngle, const double MaxAngle, const unsigned int NbAngle) {
//            m_WaveDirectionDiscretization.SetMin(MinAngle);
//            m_WaveDirectionDiscretization.SetMax(MaxAngle);
//            m_WaveDirectionDiscretization.SetNbSample(NbAngle);
//        }
//
//        std::vector<double> GetWaveDirections() const { return m_WaveDirectionDiscretization.GetVector(); } // TODO: gerer les unites...
//
//        void SetTimeDiscretization(const double FinalTime, const unsigned int NbTimeSamples) {
//            m_TimeDiscretization.SetMin(0.);
//            m_TimeDiscretization.SetMax(FinalTime);
//            m_TimeDiscretization.SetNbSample(NbTimeSamples);
//        }
//
//        unsigned int GetNbFrequencies() const { return m_FrequencyDiscretization.GetNbSample(); }
//
//        double GetMaxFrequency() const { return m_FrequencyDiscretization.GetMax(); }
//
//        double GetMinFrequency() const { return m_FrequencyDiscretization.GetMin(); }
//
//        double GetStepFrequency() const {return m_FrequencyDiscretization.GetStep(); }
//
//        unsigned int GetNbWaveDirections() const { return m_WaveDirectionDiscretization.GetNbSample(); }
//
//        unsigned int GetNbTimeSamples() const { return m_TimeDiscretization.GetNbSample(); }
//
//        double GetFinalTime() const { return m_TimeDiscretization.GetMax(); }
//
//        double GetTimeStep() const { return m_TimeDiscretization.GetStep(); }
//
        FrBEMBody_* NewBEMBody(std::string BodyName) {
            m_Bodies.emplace_back(std::make_unique<FrBEMBody_>(GetNbBodies(), BodyName, this));
            return m_Bodies.back().get();
        }
//
//        std::shared_ptr<FrBEMBody> GetBody(unsigned int ibody) { return m_Bodies[ibody]; }
//
//        void GenerateImpulseResponseFunctions(double tf = 30., double dt = 0.);
//
//        void GenerateSpeedDependentIRF();
//
//        std::shared_ptr<FrHydroMapper> GetMapper();
//
//        void IntLoadResidual_Mv(const unsigned int off, chrono::ChVectorDynamic<>& Res,
//                                const chrono::ChVectorDynamic<>& w, const double c) ;
//
//        //void VariablesFbIncrementMq();

    };  // end class FrHydroDB



    std::shared_ptr<FrHydroDB_> LoadHDB5_(std::string hdb5File);



}  // end namespace frydom


#endif //FRYDOM_FRHYDRODB_H
