//
// Created by frongere on 22/11/17.
//

#ifndef FRYDOM_FRYDOM_H
#define FRYDOM_FRYDOM_H

#include <cstdlib>

// Chrono related headers
#include "chrono/solver/ChSolverMINRES.h" // FIXME: trouver moyen d'avoir un import plus global des headers chrono...


// FRyDoM related headers
#include "core/FrCore.h"
#include "hydrodynamics/FrHydrodynamics.h"
#include "cable/FrCableInc.h"
#include "IO/FrIO.h"  // TODO: pour respecter le nommage, paser le repertoire IO en io
#include "mesh/FrMeshInc.h"
#include "utils/FrIrrApp.h"

#include "Eigen/Dense"


#endif //FRYDOM_FRYDOM_H
