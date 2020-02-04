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


#ifndef FRYDOM_FRFLOWFORCE_H
#define FRYDOM_FRFLOWFORCE_H


#include "frydom/core/force/FrForce.h"
#include "frydom/core/math/FrVector.h"

#include "MathUtils/LookupTable1D.h"


// TODO : splitter en force de vent et de courant et generique (FlowForce) !!!

namespace frydom {

  /**
   * \class FrFlowForce
   * \brief Class for computing the flow force.
   */
  class FrFlowForce : public FrForce {

   protected:

    Velocity m_fluxVelocityInBody;             ///< relative velocity of the flow in the body frame
    mathutils::LookupTable1D<double, mathutils::Vector3d<double>> m_table;  ///< Table of polar coefficient

    double m_frontal_area;
    double m_lateral_area;
    double m_length;

   public:

    /// Default constructor
//      FrFlowForce() = default;

    /// Constructor of the flow force with polar coeffients from json table
    /// \param jsonFile Name of the json file containing the polar coefficients
    explicit FrFlowForce(const std::string &name,
                         const std::string &type_name,
                         FrBody *body,
                         const std::string &jsonFile);

    /// Extract polar coeffients from json table
    /// \param jsonFile Name of the json file containing the polar coefficients
    void ReadTable(const std::string &jsonFile);

   protected:

    /// Update the state of the flow force
    /// \param time Current time of the simulation
    void Compute(double time) override;

    virtual double GetFluidDensity() const = 0;

  };


  /**
   * \class FrCurrentForce
   * \brief Class for computing the current loads.
   */
  class FrCurrentForce : public FrFlowForce {


   public:

    FrCurrentForce(const std::string &name, FrBody *body, const std::string &jsonFile);

   private:

    void Compute(double time) override;

    double GetFluidDensity() const override;

  };


  /**
   * \class FrWindForce
   * \brief Class for computing the wind loads.
   */
  class FrWindForce : public FrFlowForce {

   public:

    explicit FrWindForce(const std::string &name, FrBody *body, const std::string &jsonFile);


   private:

    /// Compute thd wind force
    /// \param time Current time of the simulation from beginning, in seconds
    void Compute(double time) override;

    double GetFluidDensity() const override;

  };

  std::shared_ptr<FrCurrentForce> make_current_force(const std::string &name,
                                                     std::shared_ptr<FrBody> body,
                                                     const std::string &jsonFile);

  std::shared_ptr<FrWindForce> make_wind_force(const std::string &name,
                                               std::shared_ptr<FrBody> body,
                                               const std::string &jsonFile);


} // end of namespace frydom

#endif //FRYDOM_FRFLOWFORCE_H
