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


#ifndef FRYDOM_FRHYDRODYNAMICS_H
#define FRYDOM_FRHYDRODYNAMICS_H

// FIXME : developper les includes dans chacun des dossiers une fois le refacto réalisé

#include "seakeeping/linear/excitation/FrLinearExcitationForce.h"
#include "seakeeping/linear/excitation/FrFroudeKrylov.h"

#include "seakeeping/linear/hdb/FrBEMBody.h"
#include "seakeeping/linear/hdb/FrHydroDB.h"
#include "seakeeping/linear/hdb/FrHydroDBLoader.h"
#include "seakeeping/linear/hdb/FrHydroMapper.h"

#include "seakeeping/linear/radiation/FrRadiationForce.h"
#include "seakeeping/linear/radiation/FrRadiationModel.h"
#include "seakeeping/linear/radiation/FrAddedMassBase.h"
#include "seakeeping/linear/radiation/FrVariablesAddedMassBase.h"

#include "wave_resistance/FrITTCResistance.h"

#include "damping/FrLinearDamping.h"
#include "damping/FrQuadraticDamping.h"

#include "hydrostatic/FrLinearHydrostaticForce.h"
#include "hydrostatic/FrLinearHydrostaticStiffnessMatrix.h"

#include "morison/FrMorrisonElement.h"
#include "morison/FrMorrisonForce.h"
#include "morison/FrMorisonModel.h"
#include "morison/FrMorisonForce.h"

#include "manoeuvring/FrManoeuvringDamping.h"
#include "seakeeping/linear/second_order_drift/FrWaveDriftForce.h"

//#include "FrVelocityRecorder.h"
//#include "FrPositionRecorder.h"

//#include "FrSteadyPitchTorque.h"
//#include "FrSteadyHeaveForce.h"

#include "FrEquilibriumFrame.h"
//#include "FrVelocityRecorder.h"

#endif //FRYDOM_FRHYDRODYNAMICS_H
