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

#include <memory>
#include <unordered_map>

#include "frydom/utils/FrRecorder.h"
#include "frydom/core/math/FrVector.h"
#include "frydom/core/common/FrPhysicsItem.h"


namespace frydom {


    // Forward declarations
    class FrHydroDB;
    class FrBEMBody;
    class FrHydroMapper;
    class FrBody;
    class FrOffshoreSystem;

    namespace internal {
        class FrAddedMassBase;
    }


    /**
     * \class FrRadiationModel_
     * \brief Class for computing the radiation loads.
     */
    class FrRadiationModel : public FrPrePhysicsItem {

    protected:

        std::shared_ptr<FrHydroDB> m_HDB;
        std::unordered_map<FrBEMBody*, GeneralizedForce> m_radiationForce;
        std::shared_ptr<internal::FrAddedMassBase> m_addedMass; // FIXME : a supprimer

    public:

        FrRadiationModel();

        explicit FrRadiationModel(std::shared_ptr<FrHydroDB> HDB);

        FrHydroDB* GetHydroDB() const { return m_HDB.get(); }

        Force GetRadiationForce(FrBEMBody* BEMBody) const;

        Force GetRadiationForce(FrBody* body) const;

        Torque GetRadiationTorque(FrBEMBody* BEMBody) const;

        Torque GetRadiationTorque(FrBody* body) const;

        void Initialize() override;

        FrHydroMapper* GetMapper() const;

        void Update(double time) override;

    };


    // -------------------------------------------------------------------------
    // Radiation model with convolution
    // -------------------------------------------------------------------------

    /**
     * \class FrRadiationConvolutionModel_
     * \brief Class for computing the convolution integrals.
     */
    class FrRadiationConvolutionModel : public FrRadiationModel {

    private:
        std::unordered_map<FrBEMBody*, FrTimeRecorder<GeneralizedVelocity> > m_recorder;
        double m_Te = -9.;
        double m_dt = -9.;

    public:

        // Constructor.
        FrRadiationConvolutionModel(std::shared_ptr<FrHydroDB> HDB);

        void Initialize() override;

        void Update(double time) override;

        void StepFinalize() override;

        void SetImpulseResponseSize(FrBEMBody* BEMBody, double Te, double dt);

        void SetImpulseResponseSize(FrBody* body, double Te, double dt);

        void SetImpulseResponseSize(double Te, double dt);

    private:

        void GetImpulseResponseSize(double& Te, double &dt, unsigned int& N) const;

        GeneralizedForce ConvolutionKu(double meanSpeed) const;

    };

    std::shared_ptr<FrRadiationConvolutionModel>
    make_radiation_convolution_model(std::shared_ptr<FrHydroDB> HDB, FrOffshoreSystem* system);

}  // end namespace frydom


#endif //FRYDOM_FRRADIATIONMODEL_H
