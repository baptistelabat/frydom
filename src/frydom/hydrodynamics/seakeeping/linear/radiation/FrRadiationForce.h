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


#ifndef FRYDOM_FRRADIATIONFORCE_H
#define FRYDOM_FRRADIATIONFORCE_H


#include "frydom/core/force/FrForce.h"


namespace frydom {

    // Forward declaration
    class FrRadiationModel_;

    /**
     * \class FrRadiationForce_
     * \brief Class for computing the radiation loads.
     */
    class FrRadiationForce_ : public FrForce_ {

    protected:

        FrRadiationModel_* m_radiationModel;

    public:

        FrRadiationForce_() = default;

        explicit FrRadiationForce_(FrRadiationModel_* radiationModel);

        void SetRadiationModel(FrRadiationModel_* radiationModel);

//        FrRadiationModel_* GetRadiationModel() const;

        void StepFinalize() override;

    };


    // Forward declaration
    class FrRadiationConvolutionModel_;

    /**
     * \class FrRadiationConvolutionForce_
     * \brief Class for computing the hydrodynamic damping loads.
     */
    class FrRadiationConvolutionForce_: public FrRadiationForce_ {

    public:

        explicit FrRadiationConvolutionForce_(FrRadiationConvolutionModel_* radiationModel);

        void Initialize() override;

        void Update(double time) override;

    };

}  // end namespace frydom

#endif //FRYDOM_FRRADIATIONFORCE_H
