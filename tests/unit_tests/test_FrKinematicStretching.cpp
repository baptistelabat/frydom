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

#include "gtest/gtest.h"

using namespace frydom;

TEST(FrKinematicStretching,FrKinematicStretching){

    double konde = 5.4;
    double x = 6.0, y = 0.;
    double z = 0.25;
    double depth = 100;

    FrOffshoreSystem system;
    auto wavefield = system.GetEnvironment()->GetOcean()->GetFreeSurface()->SetAiryRegularWaveField(1.,10.,0.,RAD,NWU,GOTO);

    system.Initialize();

    // No kinematic stretching
    FrKinematicStretching kinStretch;

    // Test Ez
    auto testEz = cosh(konde*(z+depth)) / sinh(konde*depth);
    EXPECT_NEAR(testEz, kinStretch.Eval(z,konde,depth), 1E-8);

    auto testDiffEz = konde* sinh(konde*(z+depth)) / sinh(konde*depth);
    // Test diffEz
    EXPECT_NEAR(testDiffEz, kinStretch.EvalDZ(z,konde,depth), 1E-8);

    // Test that it gives different values above FS
    EXPECT_NE(kinStretch.Eval(0.,konde,depth), kinStretch.Eval(1.,konde,depth));
    EXPECT_NE(kinStretch.EvalDZ(0.,konde,depth), kinStretch.EvalDZ(1.,konde,depth));

    // Vertical stretching
    FrKinStretchingVertical_ vertStretch;

    EXPECT_NEAR(vertStretch.Eval(0.,konde,depth), vertStretch.Eval(z,konde,depth), 1E-8);
    EXPECT_NEAR(vertStretch.EvalDZ(0.,konde,depth), vertStretch.EvalDZ(z,konde,depth), 1E-8);

    // Extrapolation stretching
    FrKinStretchingExtrapol_ extStretch;

    EXPECT_NEAR(extStretch.Eval(0.,konde,depth) + konde*z, extStretch.Eval(z,konde,depth), 1E-8);
    EXPECT_NEAR(extStretch.EvalDZ(0.,konde,depth) + konde, extStretch.EvalDZ(z,konde,depth), 1E-8);

    // Wheeler stretching
    FrKinStretchingWheeler_ wheelStretch(wavefield);
    auto eta = wavefield->GetElevation(x, y, NWU);
    auto rz = (depth + z) / (depth + eta);
    auto zp = depth * (rz - 1.);
    auto drz = depth / (depth + eta);

    EXPECT_NEAR(kinStretch.Eval(zp,konde,depth), wheelStretch.Eval(x,y,z,konde,depth), 1E-8);
    EXPECT_NEAR(kinStretch.EvalDZ(zp,konde,depth)*drz, wheelStretch.EvalDZ(x,y,z,konde,depth), 1E-8);

    // Chakrabarti stretching
    FrKinStretchingChakrabarti_ ChakStretching(wavefield);

    auto test = cosh(konde * (z + depth)) / sinh(konde * (depth + eta));
    EXPECT_NEAR(test, ChakStretching.Eval(x,y,z,konde,depth), 1E-8);

    test = konde * sinh(konde * (z + depth)) / sinh(konde * (depth + eta));
    EXPECT_NEAR(test, ChakStretching.EvalDZ(x,y,z,konde,depth), 1E-8);

    // Delta stretching
    FrKinStretchingDelta_ DeltaStretching(wavefield);
    double delta = 0.3, hd = 0.5;
    DeltaStretching.SetParam(hd,delta);

    if (z > -hd) {
        zp = (z + hd) * (hd + delta * eta) / (hd + eta) - hd;
        drz =(hd + delta * eta) / (hd + eta);
    } else {
        zp = z;
        drz = 1;
    }

    EXPECT_NEAR(extStretch.Eval(zp,konde,depth), DeltaStretching.Eval(x,y,z,konde,depth), 1E-8);
    EXPECT_NEAR((extStretch.EvalDZ(zp,konde,depth)-konde)*drz + konde, DeltaStretching.EvalDZ(x,y,z,konde,depth), 1E-8);

    DeltaStretching.SetParam(depth,0);
    EXPECT_NEAR(wheelStretch.Eval(x,y,z,konde,depth), DeltaStretching.Eval(x,y,z,konde,depth), 1E-8);
    EXPECT_NEAR(wheelStretch.EvalDZ(x,y,z,konde,depth), DeltaStretching.EvalDZ(x,y,z,konde,depth), 1E-8);

    DeltaStretching.SetParam(depth,1);
    EXPECT_NEAR(extStretch.Eval(z,konde,depth), DeltaStretching.Eval(x,y,z,konde,depth), 1E-8);
    EXPECT_NEAR(extStretch.EvalDZ(z,konde,depth), DeltaStretching.EvalDZ(x,y,z,konde,depth), 1E-8);

    // HDelta stretching
    FrKinStretchingHDelta_ HDeltaStretching(wavefield);
    HDeltaStretching.SetDelta(delta);
    DeltaStretching.SetParam(depth,delta);

    EXPECT_NEAR(DeltaStretching.Eval(x,y,z,konde,depth), HDeltaStretching.Eval(x,y,z,konde,depth), 1E-8);
    EXPECT_NEAR(DeltaStretching.EvalDZ(x,y,z,konde,depth), HDeltaStretching.EvalDZ(x,y,z,konde,depth), 1E-8);
}

