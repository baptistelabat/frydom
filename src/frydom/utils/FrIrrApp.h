//
// Created by frongere on 02/06/17.
//

#ifndef FRYDOM_FRIRRAPP_H
#define FRYDOM_FRIRRAPP_H

#include "chrono_irrlicht/ChIrrApp.h"
//#include "chrono_irrlicht/ChIrrAppInterface.h"
//#include "chrono_irrlicht/ChIrrAssetConverter.h"

#include "../core/FrOffshoreSystem.h"

#include "FrIrrCamera.h"

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

        FrIrrCamera* AddCustomCamera(irr::core::vector3df mpos = irr::core::vector3df(0, 0, -8),
                                                                irr::core::vector3df mtarg = irr::core::vector3df(0, 0, 0));

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



} // end namespace frydom

#endif //FRYDOM_FRIRRAPP_H
