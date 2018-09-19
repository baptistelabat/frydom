//
// Created by frongere on 03/07/17.
//

#ifndef FRYDOM_FRWINDFORCE_H
#define FRYDOM_FRWINDFORCE_H

#include "frydom/core/FrForce.h"

#include "MathUtils/MathUtils.h"

namespace frydom {


    class FrWindForce : public FrForce {

    public:

        /// Constructor from yaml file
        explicit FrWindForce(const std::string& yaml_file);

        /// Read the drag and lift coefficient from yaml file
        void ReadTable(const std::string& yaml_file);

        //
        //  UPDATE
        //

        /// Update procedure containing implementation of the wind drag force model
        void UpdateState() override;

    private:

        chrono::ChVector<> m_wind_relative_velocity;    ///< Relative velocity of the wind inflow
        double m_wind_relative_angle;                   ///< Relative angle of the wind
        mathutils::LookupTable1D<double> m_table;       ///< table of coefficient

    };


}  // end namespace frydom
#endif //FRYDOM_FRWINDFORCE_H
