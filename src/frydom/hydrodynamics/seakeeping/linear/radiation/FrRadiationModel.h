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


#ifndef FRYDOM_FRRADIATIONMODEL_H
#define FRYDOM_FRRADIATIONMODEL_H

#include "frydom/core/FrOffshoreSystem.h"
#include "frydom/core/common/FrObject.h"
#include "frydom/hydrodynamics/seakeeping/linear/hdb/FrHydroDB.h"
//#include "FrRadiationForce.h
#include "frydom/hydrodynamics/seakeeping/linear/hdb/FrHydroMapper.h"
#include "frydom/hydrodynamics/FrVelocityRecorder.h"

#include <iostream>
#include <fstream>
///  <<<<<<<<<<<<< Refactoring include

#include "frydom/utils/FrRecorder.h"

#include "frydom/core/common/FrPhysicsItem.h"

#include "frydom/hydrodynamics/seakeeping/linear/radiation/FrAddedMassBase.h"


namespace frydom {

    /// Base class
    /**
     * \class FrRadiationModel
     * \brief Class for computing the radiation loads.
     */
    class FrRadiationModel : public FrObject {

    protected:

        FrHydroDB *m_HDB = nullptr;

        FrOffshoreSystem *m_system = nullptr;

        double m_time = -1.;  // Quick hack to force update at first time step...

        std::vector<chrono::ChVectorDynamic<double>> m_radiationForces;

        int m_HydroMapIndex = 0; // TODO : patch hydro map multibody

        bool m_speed_dependent = false;


    public:

        FrRadiationModel() = default;

        explicit FrRadiationModel(FrHydroDB *HDB, FrOffshoreSystem *system);

        void SetHydroMapIndex(const int id);  // TODO : patch hydro map multibody

        int GetHydroMapIndex() const;  // TODO : patch hydro map multibody

        void SetHydroDB(FrHydroDB *HDB);

        FrHydroDB *GetHydroDB() const;

        void SetSystem(FrOffshoreSystem *system);

        FrOffshoreSystem *GetSystem() const;

        unsigned int GetNbInteractingBodies() const;

        void Initialize() override;

        std::shared_ptr<FrHydroMapper> GetMapper() const;

        void ResetRadiationForceVector();

        virtual void Update(double time) = 0;

        void
        GetRadiationForce(FrHydroBody *hydroBody, chrono::ChVector<double> &force, chrono::ChVector<double> &moment);

        void SetSpeedDependent(bool time_dependent = true);


    };

    class FrRadiationConvolutionForce;

    /**
     * \class FrRadiationConvolutionModel
     * \brief Class for computing the convolution integrals.
     */
    class FrRadiationConvolutionModel :
            public FrRadiationModel,
            public std::enable_shared_from_this<FrRadiationConvolutionModel> {

    private:
        // Recorders for the velocity of bodies that are in interaction
        std::vector<FrPerturbationVelocityRecorder> m_recorders;

    public:

        FrRadiationConvolutionModel(FrHydroDB *HDB, FrOffshoreSystem *system);

        void Initialize() override;

        // It has to be called by force models and updated only once, the first time it is called
        void Update(double time) override;

        /// Update the convolution term relative to the advance speed of the vessel
        void UpdateSpeedDependentTerm(std::shared_ptr<FrBEMBody> &bemBody_i,
                                      unsigned int iforceBody,
                                      chrono::ChVectorDynamic<double> &generalizedForce);

        std::shared_ptr<FrRadiationConvolutionForce> AddRadiationForceToHydroBody(std::shared_ptr<FrHydroBody>);

        void StepFinalize() override;


    private:

        void GetImpulseResponseSize(double &Te, double &dt, unsigned int &N) const;

    };


    /// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< REFACTORING

    //namespace internal {
    //    class FrAddedMassBase;
    //}

    // ---------------------------------------------------------------------
    // Radiation model
    // ---------------------------------------------------------------------

    /**
     * \class FrRadiationModel_
     * \brief Class for computing the radiation loads.
     */
    class FrRadiationModel_ : public FrPrePhysicsItem_ {

    protected:

        std::shared_ptr<FrHydroDB_> m_HDB;
        std::unordered_map<FrBEMBody_*, GeneralizedForce> m_radiationForce;
        std::shared_ptr<internal::FrAddedMassBase> m_addedMass;

    public:

        FrRadiationModel_();

        explicit FrRadiationModel_(std::shared_ptr<FrHydroDB_> HDB);

        FrHydroDB_* GetHydroDB() const { return m_HDB.get(); }

        Force GetRadiationForce(FrBEMBody_* BEMBody) const;

        Force GetRadiationForce(FrBody_* body) const;

        Torque GetRadiationTorque(FrBEMBody_* BEMBody) const;

        Torque GetRadiationTorque(FrBody_* body) const;

        void Initialize() override;

        FrHydroMapper_* GetMapper() const;

        void Update(double time) override;

    };


    // -------------------------------------------------------------------------
    // Radiation model with convolution
    // -------------------------------------------------------------------------

    /**
     * \class FrRadiationConvolutionModel_
     * \brief Class for computing the convolution integrals.
     */
    class FrRadiationConvolutionModel_ : public FrRadiationModel_ {

    private:
        std::unordered_map<FrBEMBody_*, FrTimeRecorder_<GeneralizedVelocity> > m_recorder;
        double m_Te = -9.;
        double m_dt = -9.;

    public:

        // Constructor.
        FrRadiationConvolutionModel_(std::shared_ptr<FrHydroDB_> HDB);

        void Initialize() override;

        void Update(double time) override;

        void StepFinalize() override;

        void SetImpulseResponseSize(FrBEMBody_* BEMBody, double Te, double dt);

        void SetImpulseResponseSize(FrBody_* body, double Te, double dt);

        void SetImpulseResponseSize(double Te, double dt);

    private:

        void GetImpulseResponseSize(double& Te, double &dt, unsigned int& N) const;

        GeneralizedForce ConvolutionKu(double meanSpeed) const;

    };

    std::shared_ptr<FrRadiationConvolutionModel_>
    make_radiation_convolution_model(std::shared_ptr<FrHydroDB_> HDB, FrOffshoreSystem_* system);

}  // end namespace frydom


#endif //FRYDOM_FRRADIATIONMODEL_H
