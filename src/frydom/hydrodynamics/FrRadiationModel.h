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

#include <iostream>
#include <fstream>


namespace frydom {

    /// Base class
    class FrRadiationModel : public FrObject {

    protected:

        FrHydroDB* m_HDB = nullptr;

        FrOffshoreSystem* m_system = nullptr;

        double m_time = -1.;  // Quick hack to force update at first time step...

        std::vector<chrono::ChVectorDynamic<double>> m_radiationForces;

        int m_HydroMapIndex = 0; // TODO : patch hydro map multibody

        bool m_speed_dependent =  false;


    public:

        FrRadiationModel() = default;

        explicit FrRadiationModel(FrHydroDB* HDB, FrOffshoreSystem* system);

        void SetHydroMapIndex(const int id);  // TODO : patch hydro map multibody

        int GetHydroMapIndex() const;  // TODO : patch hydro map multibody

        void SetHydroDB(FrHydroDB* HDB);

        FrHydroDB* GetHydroDB() const;

        void SetSystem(FrOffshoreSystem* system);

        FrOffshoreSystem* GetSystem() const;

        unsigned int GetNbInteractingBodies() const;

        void Initialize() override;

        std::shared_ptr<FrHydroMapper> GetMapper() const;

        void ResetRadiationForceVector();

        virtual void Update(double time) = 0;

        void GetRadiationForce(FrHydroBody* hydroBody, chrono::ChVector<double>& force, chrono::ChVector<double>& moment);

        void SetSpeedDependent(bool time_dependent = true);



    };

    class FrRadiationConvolutionForce;


    class FrRadiationConvolutionModel :
            public FrRadiationModel,
            public std::enable_shared_from_this<FrRadiationConvolutionModel> {

    private:
        // Recorders for the velocity of bodies that are in interaction
        std::vector<FrPerturbationVelocityRecorder> m_recorders;

    public:

        FrRadiationConvolutionModel(FrHydroDB* HDB, FrOffshoreSystem* system);


        void Initialize() override;

        // It has to be called by force models and updated only once, the first time it is called
        void Update(double time) override;

        /// Update the convolution term relative to the advance speed of the vessel
        void UpdateSpeedDependentTerm(std::shared_ptr<FrBEMBody>& bemBody_i,
                                      unsigned int iforceBody,
                                      chrono::ChVectorDynamic<double>& generalizedForce);

        std::shared_ptr<FrRadiationConvolutionForce> AddRadiationForceToHydroBody(std::shared_ptr<FrHydroBody>);

        void StepFinalize() override;


    private:

        void GetImpulseResponseSize(double& Te, double &dt, unsigned int& N) const;

    };



}  // end namespace frydom


#endif //FRYDOM_FRRADIATIONMODEL_H
