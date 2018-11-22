//
// Created by camille on 21/11/18.
//

#include "frydom/frydom.h"
#include "frydom/utils/FrRecorder.h"

#include "matplotlibcpp.h"

using namespace frydom;
using namespace mathutils;

int main(int argc, char* argv[]) {


    double value;
    std::vector<double> ftime, fval;

    FrRecorder_<double> recorder;

    recorder.SetTimePersistence(10.);
    recorder.SetTimeStep(0.5);
    recorder.Initialize();

    for (double time=0; time<20.15; time += 0.01) {
        value = sin(2.*M_PI / 10. * time) + cos(4.*M_PI / 10. * time);
        //value = 2.*M_PI / 10. * time;
        recorder.Record(time, value);

        ftime.push_back(time);
        fval.push_back(value);
    }


    // ------------------- PLOT OUTPUT ----------------------------

    std::vector<double> rtime, rval;
    std::vector<double> rtimeReal;

    for (auto val: recorder.GetData()) {
        rval.push_back(val);
    }

    rtime = recorder.GetTime();
    auto last_time = recorder.GetLastTime();

    for (auto val: rtime) { rtimeReal.push_back(last_time - val); }

    std::cout << "Size of the record : " << recorder.GetData().size() << std::endl;
    std::cout << "Mean value : " << recorder.GetMean() << std::endl;

    matplotlibcpp::subplot(2, 1, 1);
    matplotlibcpp::named_plot("function", ftime, fval);
    matplotlibcpp::named_plot("recorder", rtimeReal, rval, "o-");

    matplotlibcpp::subplot(2, 1, 2);
    matplotlibcpp::named_plot("recorder", rtime, rval, "o-");

    matplotlibcpp::show();
}