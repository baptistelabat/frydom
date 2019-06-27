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

        FrRadiationModel* m_radiationModel;     ///< radiation model

    public:

        /// Default constructor
        FrRadiationForce() = default;

        /// Constructor with the radiation model
        /// \param radiationModel Radiation model where the radiation force is applied
        explicit FrRadiationForce(FrRadiationModel* radiationModel);

        /// Define the radiation model where the radiation force is applied
        /// \param radiationModel Radiation model where the radiation force is applied
        void SetRadiationModel(FrRadiationModel* radiationModel);

        /// Method to be applied at the end of steps
        //void StepFinalize() override;

    };


    // Forward declaration
    class FrRadiationConvolutionModel;

    /**
     * \class FrRadiationConvolutionForce
     * \brief Class for computing the hydrodynamic damping loads.
     */
    class FrRadiationConvolutionForce: public FrRadiationForce {

    private:

        Force c_forceInertiaPart;
        Torque c_torqueInertiaPart;

    public:

        /// Get the type name of this object
        /// \return type name of this object
        std::string GetTypeName() const override { return "RadiationConvolutionForce"; }

        /// Constructor with the radiation model
        /// \param radiationModel Radiation model where the radiation force is applied
        explicit FrRadiationConvolutionForce(FrRadiationConvolutionModel* radiationModel);

        void AddFields() override;

        /// Method to initialize the radiation convolution force
        void Initialize() override;

        /// Methods to be applied at the end of each time step
        void StepFinalize() override;

    private:

        /// Compute the radiation force via convolution
        /// \param time Current time of the simulation from beginning, in seconds
        void Compute(double time) override;

        /// Update the part of the radiation force linked with body acceleration (for logging)
        void UpdateForceInertiaPart();

        /// Return the force component of the inertia part of the radiation force in body reference frame
        /// \param fc Frame convention
        Force GetForceInertiaPartInBody(FRAME_CONVENTION fc) const;

        /// Return the torque component of the inertia part of the radiation force in body reference frame
        /// \param fc Frame convention
        Torque GetTorqueInertiaPartInBody(FRAME_CONVENTION fc) const;

        /// Return the force component of the inertia part of the radiation force in  world reference frame
        /// \param fc Frame convention
        Force GetForceInertiaPartInWorld(FRAME_CONVENTION fc) const;

        /// Return the torque component of the inertia part of the radiation force in world reference frame
        /// \param fc Frame convention
        Torque GetTorqueInertiaPartInWorld(FRAME_CONVENTION fc) const;

    };

}  // end namespace frydom

#endif //FRYDOM_FRRADIATIONFORCE_H
