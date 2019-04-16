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
    class FrRadiationModel;

    /**
     * \class FrRadiationForce
     * \brief Class for computing the radiation loads.
     */
    class FrRadiationForce : public FrForce {

    protected:

        FrRadiationModel* m_radiationModel;

    public:

        FrRadiationForce() = default;

        explicit FrRadiationForce(FrRadiationModel* radiationModel);

        void SetRadiationModel(FrRadiationModel* radiationModel);

//        FrRadiationModel* GetRadiationModel() const;

        void StepFinalize() override;

    };


    // Forward declaration
    class FrRadiationConvolutionModel;

    /**
     * \class FrRadiationConvolutionForce
     * \brief Class for computing the hydrodynamic damping loads.
     */
    class FrRadiationConvolutionForce: public FrRadiationForce {

    public:

        /// Get the type name of this object
        /// \return type name of this object
        std::string GetTypeName() const override { return "RadiationConvolutionForce"; }

        explicit FrRadiationConvolutionForce(FrRadiationConvolutionModel* radiationModel);

        void Initialize() override;

    private:

        /// Compute the radiation force via convolution
        /// \param time Current time of the simulation from beginning, in seconds
        void Compute(double time) override;

    };

}  // end namespace frydom

#endif //FRYDOM_FRRADIATIONFORCE_H
