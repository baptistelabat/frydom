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

    /**
     * \class FrRadiationModel_
     * \brief Class for computing the radiation loads.
     */
    class FrRadiationModel_ : public FrPrePhysicsItem_ {

    protected:

        std::shared_ptr<FrHydroDB_> m_HDB;
        std::unordered_map<FrBEMBody_*, GeneralizedForce> m_radiationForce;
        std::shared_ptr<internal::FrAddedMassBase> m_addedMass; // FIXME : a supprimer

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
