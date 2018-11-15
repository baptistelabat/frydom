//
// Created by camille on 15/11/18.
//

#ifndef FRYDOM_FRFLOWFORCE_H
#define FRYDOM_FRFLOWFORCE_H

#include "frydom/core/FrForce.h"
#include "MathUtils/MathUtils.h"

namespace frydom {

    class FrFlowForce : public FrForce_ {

    private:

        mathutils::LookupTable1D<double> m_table;   ///< table of coeffient

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

        void Initialize() override {}

        void StepFinalize() override {}

    };

} // end of namespace frydom

#endif //FRYDOM_FRFLOWFORCE_H
