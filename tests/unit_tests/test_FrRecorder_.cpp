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

#include "frydom/utils/FrRecorder.h"

using namespace frydom;

// --------------------------------------------------------------------
//
// TEST OF THE RECORDER
//
// --------------------------------------------------------------------

TEST(TestRecorder, ParamSetter) {

    auto recorder = std::make_shared<FrTimeRecorder_<double>>();

    recorder->SetTimePersistence(10.);
    EXPECT_FLOAT_EQ(10, recorder->GetTimePersistence());

    recorder->SetTimeStep(0.1);
    EXPECT_FLOAT_EQ(0.1, recorder->GetTimeStep());

    recorder = std::make_shared<FrTimeRecorder_<double>>(150.);
    EXPECT_FLOAT_EQ(150., recorder->GetTimePersistence());

    recorder = std::make_shared<FrTimeRecorder_<double>>(100., 0.5);
    EXPECT_FLOAT_EQ(100., recorder->GetTimePersistence());
    EXPECT_FLOAT_EQ(0.5, recorder->GetTimeStep());

}

