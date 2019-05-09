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

namespace frydom {

    /**
     * \class FrFlowForce
     * \brief Class for computing the flow force.
     */
    class FrFlowForce : public FrForce {

    protected:

        Velocity m_fluxVelocityInBody;             ///< relative velocity of the flow in the body frame
        mathutils::LookupTable1D<double, mathutils::Vector3d<double>> m_table;  ///< Table of polar coefficient

    public:

        /// Default constructor
        FrFlowForce() = default;

        /// Constructor of the flow force with polar coeffients from json table
        /// \param jsonFile Name of the json file containing the polar coefficients
        explicit FrFlowForce(const std::string& jsonFile);

        /// Get the type name of this object
        /// \return type name of this object
        std::string GetTypeName() const override { return "FlowForce"; }

        /// Extract polar coeffients from json table
        /// \param jsonFile Name of the json file containing the polar coefficients
        void ReadTable(const std::string& jsonFile);

    protected:

        /// Update the state of the flow force
        /// \param time Current time of the simulation
        void Compute(double time) override;

    };


    /**
     * \class FrCurrentForce
     * \brief Class for computing the current loads.
     */
    class FrCurrentForce : public FrFlowForce {


    public:

        /// Get the type name of this object
        /// \return type name of this object
        std::string GetTypeName() const override { return "CurrentForce"; }

        explicit FrCurrentForce(const std::string& jsonFile) : FrFlowForce(jsonFile) { }

    private:

        void Compute(double time) override;

    };


    /**
     * \class FrWindForce
     * \brief Class for computing the wind loads.
     */
    class FrWindForce : public FrFlowForce {

    public:

        /// Get the type name of this object
        /// \return type name of this object
        std::string GetTypeName() const override { return "WindForce"; }

        explicit FrWindForce(const std::string& jsonFile) : FrFlowForce(jsonFile) { }

    private:

        /// Compute thd wind force
        /// \param time Current time of the simulation from beginning, in seconds
        void Compute(double time) override;

    };

    std::shared_ptr<FrCurrentForce> make_current_force(const std::string& jsonFile, std::shared_ptr<FrBody> body);

    std::shared_ptr<FrWindForce> make_wind_force(const std::string& jsonFile, std::shared_ptr<FrBody> body);


} // end of namespace frydom

#endif //FRYDOM_FRFLOWFORCE_H
