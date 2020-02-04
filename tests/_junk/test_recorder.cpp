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

#include "frydom/frydom.h"

using namespace frydom;

int main(int argc, char *argv[]) {

  // 1 - Creation du recorder
  auto recorder = FrRecorder<chrono::ChVector<double>>();
  recorder.SetTimePersistence(1);

  // 2 - Creation d'un vecteur temps
  auto time = arange<double>(0, 100, 0.01);

  // 3 - Incrementally adding the data into the recorder
  for (auto t: time) {

    // Random vector for velocity or angles...
    auto data = Eigen::Vector3d();
    data.setRandom();

    recorder.record(t, ChEig(data));

  }

  return 0;
}