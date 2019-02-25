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

#include "MathUtils/Vector3d.h"
#include "frydom/core/force/FrForce.h"
#include "MathUtils/MathUtils.h"

namespace frydom {

    /**
     * \class FrFlowForce
     * \brief Class for computing the flow force.
     */
    class FrFlowForce : public FrForce_ {

    protected:

        Velocity m_fluxVelocityInBody;             ///< relative velocity of the flow in the body frame
        mathutils::LookupTable1D<double, mathutils::Vector3d<double>> m_table;  ///< Table of polar coefficient

    public:

        /// Default constructor
        FrFlowForce() = default;

        /// Constructor of the flow force with polar coeffients from YAML table
        /// \param yamlFile Name of the YAML file containing the polar coefficients
        explicit FrFlowForce(const std::string& yamlFile);

        /// Extract polar coeffients from YAML table
        /// \param yamlFile Name of the YAML file containing the polar coefficients
        void ReadTable(const std::string& yamlFile);

        /// Update the state of the flow force
        /// \param time Current time of the simulation
        void Update(double time) override;

        /// Initialize the state of the flow force
        void Initialize() override;

        /// Method called at the send of a time step. Logging may be used here
        void StepFinalize() override;

    };


    /**
     * \class FrCurrentForce2_
     * \brief Class for computing the current loads.
     */
    class FrCurrentForce2_ : public FrFlowForce {


    public:
        explicit FrCurrentForce2_(const std::string& yamlFile) : FrFlowForce(yamlFile) { }

        void Update(double time) override;
    };


    /**
     * \class FrWindForce2_
     * \brief Class for computing the wind loads.
     */
    class FrWindForce2_ : public FrFlowForce {

    public:
        explicit FrWindForce2_(const std::string& yamlFile) : FrFlowForce(yamlFile) { }

        void Update(double time) override;

    };

    std::shared_ptr<FrCurrentForce2_> make_current_force(const std::string& yamlFile, std::shared_ptr<FrBody_> body);

    std::shared_ptr<FrWindForce2_> make_wind_force(const std::string& yamlFile, std::shared_ptr<FrBody_> body);

} // end of namespace frydom

#endif //FRYDOM_FRFLOWFORCE_H
