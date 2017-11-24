//
// Created by frongere on 20/10/17.
//

#include "frydom/frydom.h"

using namespace frydom;

int main(int argc, char* argv[]) {

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