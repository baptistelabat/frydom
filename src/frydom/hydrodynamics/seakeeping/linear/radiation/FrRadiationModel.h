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
    template<typename OffshoreSystemType>
    class FrHydroDB;

    template<typename OffshoreSystemType>
    class FrBEMBody;

    template<typename OffshoreSystemType>
    class FrHydroMapper;

    template<typename OffshoreSystemType>
    class FrBody;

    template<typename OffshoreSystemType>
    class FrOffshoreSystem;

    namespace internal {
        template<typename OffshoreSystemType>
        class FrRadiationModelBase;
    }

    /**
     * \class FrRadiationModel
     * \brief Class for computing the radiation loads.
     */
    template<typename OffshoreSystemType>
    class FrRadiationModel : public FrPrePhysicsItem<OffshoreSystemType> {

     protected:

      std::shared_ptr<FrHydroDB<OffshoreSystemType>> m_HDB;
      std::unordered_map<FrBEMBody<OffshoreSystemType> *, GeneralizedForce> m_radiationForce;

     public:

      /// Default constructor
      FrRadiationModel();

      /// Constructor with specified hydrodynamic database
      /// \param HDB Hydrodynamic database
      explicit FrRadiationModel(std::shared_ptr<FrHydroDB<OffshoreSystemType>> HDB);

      /// Get the type name of this object
      /// \return type name of this object
      std::string GetTypeName() const override { return "RadiationModel"; }

      /// Return true if the radiation model is included in the static analysis
      bool IncludedInStaticAnalysis() const override { return false; }

      /// Return the hydrodynamic database linked with the radiation model
      /// \return Hydrodynamic database
      FrHydroDB<OffshoreSystemType> *GetHydroDB() const { return m_HDB.get(); }

      /// Return the radiation force applied on a body
      /// \param BEMBody BEM body database
      /// \return Radiation force
      Force GetRadiationForce(FrBEMBody<OffshoreSystemType> *BEMBody) const;

      /// Return the radiation force applied on a body
      /// \param body body (frydom object)
      /// \return Radiation force
      Force GetRadiationForce(FrBody<OffshoreSystemType> *body) const;

      /// Return the radiation torque applied on a body
      /// \param BEMBody BEM body database
      /// \return Radiation torque
      Torque GetRadiationTorque(FrBEMBody<OffshoreSystemType> *BEMBody) const;

      /// Return the radiation torque applied on a body
      /// \param body body (frydom object)
      /// \return Radiation torque
      Torque GetRadiationTorque(FrBody<OffshoreSystemType> *body) const;

      /// Method to initialize the radiation model
      void Initialize() override;

      /// Return the mapper between body and BEM body database
      /// \return Mapper
      FrHydroMapper<OffshoreSystemType> *GetMapper() const;

     private:

      /// Compute the internal states of the Radiation model
      /// \param time Current time of the simulation from beginning, in seconds
      void Compute(double time) override;

    };


    // -------------------------------------------------------------------------
    // Radiation model with convolution
    // -------------------------------------------------------------------------

    /**
     * \class FrRadiationConvolutionModel
     * \brief Class for computing the convolution integrals.
     */
    template<typename OffshoreSystemType>
    class FrRadiationConvolutionModel : public FrRadiationModel<OffshoreSystemType> {

     private:
      std::unordered_map<FrBEMBody<OffshoreSystemType> *, FrTimeRecorder<GeneralizedVelocity> > m_recorder;    ///< Recorder of the perturbation velocity of the body at COG
      double m_Te = -9.;      ///< Persistence time of the recorder
      double m_dt = -9.;      ///< Time step of the recorder

     public:
      /// Default constructor
      explicit FrRadiationConvolutionModel(std::shared_ptr<FrHydroDB<OffshoreSystemType>> HDB);

      /// Get the type name of this object
      /// \return type name of this object
      std::string GetTypeName() const override { return "RadiationConvolutionModel"; }

      /// Method to initialize the radiation model
      void Initialize() override;

      /// Clear the recorder
      void Clear();

      /// Method to be applied at the end of each time step
      void StepFinalize() override;

      /// Set the impulse response function size
      /// \param BEMBody BEM body database corresponding to the body to which the radiation force is applied
      /// \param Te Time length
      /// \param dt Time step
      void SetImpulseResponseSize(FrBEMBody<OffshoreSystemType> *BEMBody, double Te, double dt);

      /// Set the impulse response function size
      /// \param body Body to which the radiation force is applied
      /// \param Te Time length
      /// \param dt Time step
      void SetImpulseResponseSize(FrBody<OffshoreSystemType> *body, double Te, double dt);

      /// Set the impulse response function size
      /// \param Te Time length
      /// \param dt Time step
      void SetImpulseResponseSize(double Te, double dt);

      /// Return the generalized force part relative to the added mass term
      /// \param body Body for which the motion is considered
      /// \return Part the the radiation force linked with the acceleration of the body
      GeneralizedForce GetRadiationInertiaPart(FrBody<OffshoreSystemType> *body) const;

     private:

      /// Compute the radiation convolution.
      /// \param time Current time of the simulation from beginning, in seconds
      void Compute(double time) override;

      /// Return the impulse response function size
      /// \param Te Time length
      /// \param dt Time step
      /// \param N Number of time step
      void GetImpulseResponseSize(double &Te, double &dt, unsigned int &N) const;

      /// Compute the the convolution part of the radiation force linked with steady speed
      /// \param meanSpeed Steady speed of the body
      /// \return Generalized force
      GeneralizedForce ConvolutionKu(double meanSpeed) const;
    };

    template<typename OffshoreSystemType>
    std::shared_ptr<FrRadiationConvolutionModel<OffshoreSystemType>>
    make_radiation_convolution_model(std::shared_ptr<FrHydroDB<OffshoreSystemType>> HDB,
                                     FrOffshoreSystem<OffshoreSystemType> *system);

}  // end namespace frydom


#endif //FRYDOM_FRRADIATIONMODEL_H
