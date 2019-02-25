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


#ifndef FRYDOM_FRCURRENTFORCE_H
#define FRYDOM_FRCURRENTFORCE_H

#include "frydom/core/force/FrForce.h"
//#include "frydom/core/FrHydroBody.h"
#include "FrCurrentPolarCoeffs.h"

namespace frydom {

    // Forward declarations
    class FrCurrentPolarCoeffs;
    class FrCurrent;
    class FrHydroBody;

    /**
     * \class FrCurrentForce
     * \brief Class for computing the current loads.
     */
    class FrCurrentForce : public FrForce {

    private:

        FrCurrentPolarCoeffs coeffs_table;

    public:

        /// Default constructor
        FrCurrentForce() = default;

        /// Constructor from a YAML file
        explicit FrCurrentForce(std::string yaml_file);

        /// Get the parent body that must be a FrHydroBody
//        FrHydroBody* GetBody() override { return dynamic_cast<FrHydroBody*>(Body); }

        /// Sets the parent body (the force belongs to this rigid body)
//        void SetBody(FrHydroBody* newRB) { Body = newRB; }

        /// Update the state of the force
        void UpdateState() override;

        void SetCoeffs(FrCurrentPolarCoeffs table) {
            //coeffs_table = std::move(table);
        }

        //
        // LOG
        //

        /// Define the prefix of the message of the current force
        void SetLogPrefix(std::string prefix_name) override;

    };








    // REFACTORING ------>>>>>>>>>>>>>>

    /**
     * \class FrCurrentForce_
     * \brief Class for computing the current loads.
     */
    class FrCurrentForce_ : public FrForce_ {

    private:
        FrCurrentPolarCoeffs m_coeffsTable;

    public:

        /// Default constructor
        FrCurrentForce_() = default;

        /// Constructor from YAML file
        explicit FrCurrentForce_(std::string yamlFile);

        /// Update the state of the force
        void Update(double time) override;

        void Initialize() override {};

        void StepFinalize() override;;

        //void SetCoeffs(FrCurrentPolarCoeffs coeffsTable);

    };


}  // end namespace frydom

#endif //FRYDOM_FRCURRENTFORCE_H
