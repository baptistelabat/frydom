//
// Created by frongere on 22/11/17.
//

#ifndef FRYDOM_FRYDOM_H
#define FRYDOM_FRYDOM_H

#include <cstdlib>
#include <irrlicht.h>


// Chrono related headers
#include "chrono/solver/ChSolverMINRES.h" // FIXME: trouver moyen d'avoir un import plus global des headers chrono...

// FRyDoM related headers
#include "asset/FrAssetInc.h"
#include "core/FrCore.h"
#include "environment/FrEnvironmentInc.h"
#include "frydom/hydrodynamics/FrHydrodynamicsInc.h"
#include "cable/FrCableInc.h"
#include "IO/FrIO.h"  // TODO: pour respecter le nommage, paser le repertoire IO en io
#include "mesh/FrMeshInc.h"
#include "utils/FrIrrApp.h"

#include <H5Cpp.h>

#endif //FRYDOM_FRYDOM_H
