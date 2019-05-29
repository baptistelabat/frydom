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

#ifndef FRYDOM_FRYDOM_H
#define FRYDOM_FRYDOM_H

#include <cstdlib>
#include <irrlicht.h>

#include "chrono/solver/ChSolverMINRES.h" // FIXME: trouver moyen d'avoir un import plus global des headers chrono...

// FRyDoM related headers
#include "core/FrCore.h"
#include "environment/FrEnvironmentInc.h"
#include "frydom/hydrodynamics/FrHydrodynamicsInc.h"
#include "cable/FrCableInc.h"
#include "frydom/IO/FrIOInc.h"  // TODO: pour respecter le nommage, paser le repertoire IO en io
#include "mesh/FrMeshInc.h"
#include "utils/FrIrrApp.h"
#include "asset/FrAssetInc.h"
#include "collision/FrCollisionModel.h"

#include <H5Cpp.h>


#endif //FRYDOM_FRYDOM_H
