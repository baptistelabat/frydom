// =============================================================================
// FRyDoM - frydom-ce.gitlab.host.io
//
// Copyright (c) D-ICE Engineering and Ecole Centrale de Nantes (LHEEA lab.)
// All rights reserved.
//
// Use of this source code is governed by a GPLv3 license that can be found
// in the LICENSE file of FRyDOM.
//
// =============================================================================


#ifndef FRYDOM_FRIRRAPP_H
#define FRYDOM_FRIRRAPP_H

#include "chrono_irrlicht/ChIrrApp.h"

#include "FrIrrCamera.h"

namespace frydom {

    class FrOffshoreSystem;

    /**
     * \class FrIrrApp
     * \brief Class for Irrlicht applications (visualization).
     */
    class FrIrrApp : public chrono::irrlicht::ChIrrApp {

    private:
        bool m_verbose = true;

      public:

        FrIrrApp(FrOffshoreSystem& system, const double dist=100);

        /// Create the application with Irrlicht context (3D view, device, etc.)
        FrIrrApp(FrOffshoreSystem* system,
                 const wchar_t* title = 0,
                 irr::core::dimension2d<irr::u32> dimens = irr::core::dimension2d<irr::u32>(800, 600));

        virtual ~FrIrrApp();

        /// Create a skybox that has Z pointing up.
        /// Note that the default ChIrrApp::AddTypicalSky() uses Y up.
        void SetSkyBox();

        FrIrrCamera* AddCustomCamera(irr::core::vector3df mpos = irr::core::vector3df(0, 0, -8),
                                                                irr::core::vector3df mtarg = irr::core::vector3df(0, 0, 0));

        void AddCustomLights(irr::core::vector3df pos1 = irr::core::vector3df(-100.f, -30.f, 30.f),
                             irr::core::vector3df pos2 = irr::core::vector3df(80.f, 30.f, -30.f),
                             double rad1 = 290,
                             double rad2 = 190,
                             irr::video::SColorf col1 = irr::video::SColorf(0.7f, 0.7f, 0.7f, 1.0f),
                             irr::video::SColorf col2 = irr::video::SColorf(0.7f, 0.8f, 0.8f, 1.0f)) ;

        void Run(bool infiniteLoop = true, bool verbose=true, int stepMax = 200) {
            int stepCounter = 0;

            AssetBindAll();
            AssetUpdateAll();

            while (GetDevice()->run() && stepCounter < stepMax) {
                BeginScene();
                DrawAll();

                if (verbose) {

                    std::cout << "\n\n"
                              << "Integration from " << GetSystem()->GetChTime()
                              << " To " << GetSystem()->GetChTime()+GetTimestep()
                              << "\n";
                }

                DoStep();
                EndScene();

                if(!infiniteLoop){
                    stepCounter++;
                }
            }
        }

    };














    // REFACTORING ------------->>>>>>>>>>>>

    // Forward declaration
    class FrOffshoreSystem_;


    /**
     * \class FrIrrApp_
     * \brief Class for Irrlicht applications (visualization).
     */
    class FrIrrApp_ : public chrono::irrlicht::ChIrrApp {

    private:
        chrono::ChSystem* m_system;

    public:

        explicit FrIrrApp_(chrono::ChSystem* system, double dist=100);

        ~FrIrrApp_() final;

        /// Create a skybox that has Z pointing up.
        /// Note that the default ChIrrApp::AddTypicalSky() uses Y up.
        void SetSkyBox();

        FrIrrCamera* AddCustomCamera(irr::core::vector3df mpos = irr::core::vector3df(0, 0, -8),
                                     irr::core::vector3df mtarg = irr::core::vector3df(0, 0, 0));

        void AddCustomLights(irr::core::vector3df pos1 = irr::core::vector3df(-100.f, -30.f, 30.f),
                             irr::core::vector3df pos2 = irr::core::vector3df(80.f, 30.f, -30.f),
                             double rad1 = 290,
                             double rad2 = 190,
                             irr::video::SColorf col1 = irr::video::SColorf(0.7f, 0.7f, 0.7f, 1.0f),
                             irr::video::SColorf col2 = irr::video::SColorf(0.7f, 0.8f, 0.8f, 1.0f)) ;

        void Run(double endTime);

        void Visualize();

    };

} // end namespace frydom

#endif //FRYDOM_FRIRRAPP_H
