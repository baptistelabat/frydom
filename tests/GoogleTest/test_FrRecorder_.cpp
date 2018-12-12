//
// Created by camille on 07/12/18.
//

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

