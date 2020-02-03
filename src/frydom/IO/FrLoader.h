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


#ifndef FRYDOM_FRLOADER_H
#define FRYDOM_FRLOADER_H


#include <string>
#include <vector>

#include "MathUtils/LookupTable1D.h"
#include "MathUtils/Vector3d.h"

#include "frydom/core/common/FrConvention.h"
#include "frydom/core/common/FrUnits.h"


namespace frydom {

  // =========================================================================================================
  // IO for flow force model
  // =========================================================================================================

  /// Extract coefficients for the flow force model from JSON file
  /// \param jsonFile Name of the json file containing the polar coefficient
  /// \param angle Flow direction
  /// \param cx Polar coefficient in surge
  /// \param cy Polar coefficient in sway
  /// \param cn Polar coefficient in yaw
  /// \param unit Unit of the angle direction
  /// \param fc frame convention (NED/NWU)
  /// \param dc direction convention (GOTO/COMEFROM)
  void LoadFlowPolarCoeffFromJson(const std::string &jsonFile,
                                  std::vector<double> &angle,
                                  std::vector<double> &cx,
                                  std::vector<double> &cy,
                                  std::vector<double> &cn,
                                  double &frontal_area,
                                  double &lateral_area,
                                  double &length,
                                  ANGLE_UNIT &unit,
                                  FRAME_CONVENTION &fc,
                                  DIRECTION_CONVENTION &dc);

  /// This function reads the flow polar coefficients from a Json input file.
  void LoadFlowPolarCoeffFromJson(const std::string &jsonFile,
                                  std::vector<std::pair<double, mathutils::Vector3d<double>>> &polar,
                                  double &frontal_area,
                                  double &lateral_area,
                                  double &length,
                                  ANGLE_UNIT &unit,
                                  FRAME_CONVENTION &fc,
                                  DIRECTION_CONVENTION &dc);


}  // end namespace frydom


#endif //FRYDOM_FRLOADER_H
