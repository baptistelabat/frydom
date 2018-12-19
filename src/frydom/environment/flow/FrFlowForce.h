//
// Created by camille on 15/11/18.
//

#ifndef FRYDOM_FRFLOWFORCE_H
#define FRYDOM_FRFLOWFORCE_H

#include "MathUtils/Vector3d.h"
#include "frydom/core/FrForce.h"
#include "MathUtils/MathUtils.h"

namespace frydom {

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
        void Initialize() override {}

        /// Method called at the send of a time step. Logging may be used here
        void StepFinalize() override {}

    };


    class FrCurrentForce2_ : public FrFlowForce {


    public:
        explicit FrCurrentForce2_(const std::string& yamlFile) : FrFlowForce(yamlFile) { }

        void Update(double time) override;
    };


    class FrWindForce2_ : public FrFlowForce {

    public:
        explicit FrWindForce2_(const std::string& yamlFile) : FrFlowForce(yamlFile) { }

        void Update(double time) override;

    };

} // end of namespace frydom

#endif //FRYDOM_FRFLOWFORCE_H
