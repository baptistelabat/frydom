//
// Created by frongere on 17/10/17.
//

#ifndef FRYDOM_FRHYDRODB_H
#define FRYDOM_FRHYDRODB_H

#include <vector>

//#include "boost/bimap.hpp"
#include <unordered_map>


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
    class FrBEMBody_;

    class FrBEMBodyMapper_ {

    private:

        std::unordered_map<FrBEMBody_*, FrBody_*> m_BEMToFrydomBodyMap;
        std::unordered_map<FrBody_*, FrBEMBody_*> m_FrydomToBEMBodyMap;

    public:

        FrBEMBodyMapper_() = default;

        bool AddEntry(FrBEMBody_* bemBody, FrBody_* frydomBody);

        FrBody_* GetFrydomBody(FrBEMBody_* bemBody) const;

        FrBEMBody_* GetBEMBody(FrBody_* frydomBody) const;

        unsigned long GetNbMappedBodies() const;

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

        using BEMBodyContainer = std::vector<std::unique_ptr<FrBEMBody_>>;
        BEMBodyContainer m_bodies;

        std::unordered_map<std::string, std::pair<unsigned long, FrBEMBody_*>> m_namedBodyMap;
//        std::unordered_map<FrBEMBody_*, unsigned long> m_bemBodyToIndex;

        FrDiscretization1D m_FrequencyDiscretization;
        FrDiscretization1D m_WaveDirectionDiscretization;
        FrDiscretization1D m_TimeDiscretization;

        std::unique_ptr<FrBEMBodyMapper_> m_bodyMapper;



    public:

        /// Constructor from a HDB5 file that is loaded
        explicit FrHydroDB_(std::string hdb5File);

        /// Get the number of interacting bodies
        unsigned long GetNbBodies() const { return m_bodies.size(); }

        /// Bind a frydom body to a BEMBody defined by its name as specified into the HDB
        void Bind(std::string bemBodyName, FrBody_* frydomBody);

        /// Get a pointer to the BEM body that has been bind to the Frydom body given as argument
        FrBEMBody_* GetBEMBody(FrBody_* frydomBody);

        FrBEMBody_* GetBEMBody(std::string bemBodyName);

        /// Get a pointer to the frydom body that has been bind to the bemBody given as argument
        FrBody_* GetFrydomBody(FrBEMBody_* bemBody);

        bool IsFullyConnected() const;





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
        std::vector<double> GetWaveDirections() const { return m_WaveDirectionDiscretization.GetVector(); } // TODO: gerer les unites...
//
//        void SetTimeDiscretization(const double FinalTime, const unsigned int NbTimeSamples) {
//            m_TimeDiscretization.SetMin(0.);
//            m_TimeDiscretization.SetMax(FinalTime);
//            m_TimeDiscretization.SetNbSample(NbTimeSamples);
//        }
//
        unsigned int GetNbFrequencies() const { return m_FrequencyDiscretization.GetNbSample(); }

        double GetMaxFrequency() const { return m_FrequencyDiscretization.GetMax(); }

        double GetMinFrequency() const { return m_FrequencyDiscretization.GetMin(); }

        double GetStepFrequency() const {return m_FrequencyDiscretization.GetStep(); }

        unsigned int GetNbWaveDirections() const { return m_WaveDirectionDiscretization.GetNbSample(); }

        unsigned int GetNbTimeSamples() const { return m_TimeDiscretization.GetNbSample(); }

        double GetFinalTime() const { return m_TimeDiscretization.GetMax(); }

        double GetTimeStep() const { return m_TimeDiscretization.GetStep(); }

        FrBEMBody_* NewBEMBody(std::string BodyName) {
            m_bodies.emplace_back(std::make_unique<FrBEMBody_>(BodyName, this));
            FrBEMBody_* newBody = m_bodies.back().get();

            m_namedBodyMap.insert({BodyName, {GetNbBodies()-1, newBody}});

            return newBody;
        }
//
//        FrBEMBody_* GetBody(unsigned int ibody) { return m_bodies[ibody].get(); }

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




        /// Iterators on bodies that are interacting into the hydrodynamic database

        struct BEMBodyIter : public BEMBodyContainer::iterator {

            BEMBodyIter() : BEMBodyContainer::iterator() {}
            BEMBodyIter(BEMBodyContainer::iterator it) : BEMBodyContainer::iterator(it) {}

            FrBEMBody_* GetBEMBody() {
                return BEMBodyContainer::iterator::operator*().get();
            }

            FrBody_* GetFrydomBody() {
                FrBEMBody_* bemBody = GetBEMBody();
                return bemBody->m_HDB->GetFrydomBody(bemBody);
            }

        };


//        struct BEMBodyConstIter : public BEMBodyContainer::const_iterator {
//
//            BEMBodyConstIter() : BEMBodyContainer::const_iterator() {}
//            BEMBodyConstIter(BEMBodyContainer::const_iterator it) : BEMBodyContainer::const_iterator(it) {}
//
//            const FrBEMBody_* GetBEMBody() const {
//                return BEMBodyContainer::const_iterator::operator*().get();
//            }
//
//            const FrBody_* GetFrydomBody() const {
//                const FrBEMBody_* bemBody = GetBEMBody();
//                return bemBody->m_HDB->GetFrydomBody(bemBody);
//            }
//
//        };


        BEMBodyIter begin_body();
        BEMBodyIter end_body();

//        BEMBodyConstIter begin_body() const;
//        BEMBodyConstIter end_body() const;
    private:

        friend unsigned long FrBEMBody_::GetIndexInHDB() const;

        unsigned long GetBodyIndex(const FrBEMBody_* bemBody);


    };  // end class FrHydroDB



    std::shared_ptr<FrHydroDB_> LoadHDB5_(std::string hdb5File);



}  // end namespace frydom


#endif //FRYDOM_FRHYDRODB_H
