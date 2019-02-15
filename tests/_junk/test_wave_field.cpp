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

// =====================================================================================================================
void TestRegularWaveField(const std::vector<double>& xVect, const std::vector<double>& yVect) {
    auto waveField = FrLinearWaveField(LINEAR_REGULAR);
    waveField.SetRegularWaveHeight(3);
    waveField.SetRegularWavePeriod(9);
    waveField.SetMeanWaveDirection(0., DEG);

    waveField.GetSteadyElevation(0, 0);

    auto eta = waveField.GetElevation(xVect, yVect);
}

// =====================================================================================================================
void TestIrregularWaveField(const std::vector<double>& xVect, const std::vector<double>& yVect) {

    auto waveField = FrLinearWaveField(LINEAR_IRREGULAR);
    waveField.SetRegularWaveHeight(3);
    waveField.SetRegularWavePeriod(9);
    waveField.SetMeanWaveDirection(0., DEG);

    waveField.GetSteadyElevation(0, 0);

    auto eta = waveField.GetElevation(xVect, yVect);

}

// =====================================================================================================================
void TestDirectionalWaveField(const std::vector<double>& xVect, const std::vector<double>& yVect) {
    // TODO
}

// =====================================================================================================================
int main(int argc, char* argv[]) {

    // Grid definition
    auto xVect = linspace<double>(-1000., 1000., 100);
    auto yVect = linspace<double>(-1000., 1000., 100);

    TestRegularWaveField(xVect, yVect);

    TestIrregularWaveField(xVect, yVect);

    TestDirectionalWaveField(xVect, yVect);

    return 0;
}