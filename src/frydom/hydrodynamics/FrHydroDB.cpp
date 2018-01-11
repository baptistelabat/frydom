//
// Created by frongere on 17/10/17.
//

//#include <boost/filesystem.hpp>

#include "yaml-cpp/yaml.h"

#include "FrHydroDB.h"
#include "FrHydroMapper.h"


using namespace mathutils;


// TODO: utiliser boost/multi_array.hpp a la place des vector<vector<Eigen::Matrix>>> ?????


namespace frydom {

    void FrHydroDB::GenerateImpulseResponseFunctions(double tf, double dt) {

        // Generate time informations
        if (dt == 0.) {
            // Ensuring a time step satisfying largely the shannon theorem (5x by security instead of the theoretical 2 ...)
            dt = MU_2PI / (5. * GetMaxFrequency());  // TODO: non, on veut avoir un nombre d'echantillon en puissance de 2 !!!
        }

        // Registering the time discretization into the database
        m_TimeDiscretization.SetMin(0.);
        m_TimeDiscretization.SetMax(tf);
        m_TimeDiscretization.SetStep(dt);

        // Computing the Impulse response functions for every bodies
        auto nbBody = GetNbBodies();
        for (unsigned int iBody=0; iBody<nbBody; ++iBody) {
            GetBody(iBody)->GenerateImpulseResponseFunctions();
        }

    }


    std::shared_ptr<FrHydroMapper> FrHydroDB::GetMapper() {
        return std::make_shared<FrHydroMapper>(this);
    }

}  // end namespace frydom