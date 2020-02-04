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


#ifndef FRYDOM_FRFLOWBASE_H
#define FRYDOM_FRFLOWBASE_H


#include <memory>

#include "frydom/core/common/FrObject.h"
#include "frydom/core/math/FrVector.h"
#include "frydom/core/common/FrConvention.h"
#include "FrUniformField.h"  // TODO : enlever et utiliser du virtual copy constructor

namespace frydom {

  // Forward declarations
  class FrEnvironment;

  class FrFrame;

  class FrFieldBase;

  /**
   * \class FrFlowBase
   * \brief Class for defining in general the type of flow used (current or wind).
   */
  class FrFlowBase : public FrObject {

   private:
    std::unique_ptr<FrFieldBase> m_field;        ///< Flow field model

   protected:
    double m_time = 0.;
    double c_ramp = 1.;   ///> cache value of the time ramp applied on the flow field // TODO : ne pas passer par cette valeur de cache... utiliser directement la fonction !

   public:

    /// Default constructor
    explicit FrFlowBase();

    /// Destructor of the flow base object
    ~FrFlowBase() = default;

    /// Return the flow velocity at a given point in world frame
    /// \param worldPos Position of the Point in world frame
    /// \param fc Frame convention (NED/NWU)
    /// \return Velocity in world frame
    Velocity GetFluxVelocityInWorld(const Position &worldPos, FRAME_CONVENTION fc) const;

    /// Return the flow velocity observed from the local frame
    /// \param frame Local frame in which the velocity is computed
    /// \param worldVel Translation velocity of the frame in world frame
    /// \param fc Frame convention (NED/NWU)
    /// \return Velocity in local frame
    Velocity GetRelativeVelocityInFrame(const FrFrame &frame, const Velocity &worldVel, FRAME_CONVENTION fc) const;

    /// Define a new field with derived type
    /// \tparam T Derived type of the field
    /// \param field Smart pointer to the new field
    /// \return Field raw pointer with the derived type
    template<typename T>
    T *SetField(std::unique_ptr<T> &&field) {
      auto ptr = field.get();
      m_field = std::move(field);
      return ptr;
    }

    /// Set a new field of the specified derived type
    /// \tparam T Type of the field
    /// \return Field raw pointer with the derived type
    template<typename T>
    T *NewField() {
      return SetField(std::make_unique<T>());
    }

    /// Create a uniform field
    void MakeFieldUniform();

    /// Return the field model of the flow
    /// \tparam Field Field model
    /// \return Field model pointer
    template<class T=FrUniformField>
    T *GetField() const {
      return dynamic_cast<T *>(m_field.get());
    }

    /// Get the field as a uniform field
    /// \return uniform field
    FrUniformField *GetFieldUniform() const;

    /// Method of initialization of the flow base model
    void Initialize() override;

    /// Update the state of the flow base object
    /// \param time Current time of the simulation
    virtual void Update(double time);

    //// Method to be applied at the end of each time step
    void StepFinalize() override;

    virtual FrEnvironment *GetEnvironment() const = 0;

  };


} // end of namespace frydom

#endif //FRYDOM_FRFLOWBASE_H
