//
// Created by frongere on 02/06/17.
//

#ifndef FRYDOM_FRIRRAPP_H
#define FRYDOM_FRIRRAPP_H

#include "chrono_irrlicht/ChIrrApp.h"
//#include "chrono_irrlicht/ChIrrAppInterface.h"
//#include "chrono_irrlicht/ChIrrAssetConverter.h"

#include "../core/FrOffshoreSystem.h"

namespace frydom {

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

        void Run() {

            AssetBindAll();
            AssetUpdateAll();

            while (GetDevice()->run()) {
                BeginScene();
                DrawAll();
                DoStep();
                EndScene();

                if (m_verbose) {
//                    std::cout << "Time: " << GetSystem()->GetChTime() << std::endl;
                }
            }

        }



    };



} // end namespace frydom

#endif //FRYDOM_FRIRRAPP_H
