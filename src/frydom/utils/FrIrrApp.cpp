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

#include "FrIrrApp.h"

#include "FrIrrCamera.h"

#include "frydom/core/FrOffshoreSystem.h"


#define SQ2_2 (sqrt(2.)/2.)

namespace frydom {

    FrIrrApp::FrIrrApp(FrOffshoreSystem* frSystem, chrono::ChSystem* system, double dist)
            : chrono::irrlicht::ChIrrApp(system,
                                         L"FRyDoM viewer",
                                         irr::core::dimension2d<irr::u32>(800, 600),
                                         false,
                                         false,
                                         true,
                                         irr::video::EDT_OPENGL),
                                         m_system(frSystem)
                                         {

        SetSkyBox();
        AddCustomLights();

        AddCustomCamera(irr::core::vector3df(-(irr::f32)dist, (irr::f32)dist*0, (irr::f32)dist),
                         irr::core::vector3df(0, (irr::f32)SQ2_2, (irr::f32)SQ2_2));
        AddTypicalLogo(std::string(RESOURCES_VIZU_PATH) + "frydom_logo.png");
    }

    FrIrrApp::~FrIrrApp() = default;

    // -----------------------------------------------------------------------------
    // Create a skybox that has Z pointing up.
    // Note that the default ChIrrApp::AddTypicalSky() uses Y up.
    // -----------------------------------------------------------------------------
    void FrIrrApp::SetSkyBox() {
        std::string resources_path = std::string(RESOURCES_VIZU_PATH);
        std::string str_lf = resources_path + "skybox/sky_lf.jpg";
        std::string str_up = resources_path + "skybox/sky_up.jpg";
        std::string str_dn = resources_path + "skybox/sky_dn.jpg";

        irr::video::ITexture* map_skybox_side = GetVideoDriver()->getTexture(str_lf.c_str());
        irr::scene::ISceneNode* mbox = GetSceneManager()->addSkyBoxSceneNode(
                GetVideoDriver()->getTexture(str_up.c_str()),
                GetVideoDriver()->getTexture(str_dn.c_str()),
                map_skybox_side,
                map_skybox_side,
                map_skybox_side,
                map_skybox_side
        );
        // Turning around x to make z pointing up. Note that it is +90 as Irrlicht uses left-handed frames... WTF !!!
        mbox->setRotation(irr::core::vector3df(90, 0, 0));

    }

    FrIrrCamera* FrIrrApp::AddCustomCamera(irr::core::vector3df mpos, irr::core::vector3df mtarg) {

        // create and init camera
        auto camera = new FrIrrCamera(GetDevice(), GetSceneManager()->getRootSceneNode(), GetSceneManager(),
                                                                              -1, -160.0f, 1.0f, 10.0f);

        camera->setPosition(mpos);
        camera->setTarget(mtarg);

        camera->setNearValue(0.1f);
        camera->setMinZoom(0.6f);

        return camera;
    }

    void FrIrrApp::AddCustomLights(irr::core::vector3df pos1, irr::core::vector3df pos2,
                                   double rad1, double rad2,
                                   irr::video::SColorf col1, irr::video::SColorf col2)  {
        // create lights
        irr::scene::ILightSceneNode* mlight1 = GetDevice()->getSceneManager()->addLightSceneNode(0, pos1, col1, (irr::f32)rad1);

        irr::scene::ILightSceneNode* mlight2 = GetDevice()->getSceneManager()->addLightSceneNode(0, pos2, col2, (irr::f32)rad2);

        mlight2->enableCastShadow(false);
    }

    void FrIrrApp::Run(double endTime) {

        AssetBindAll();
        AssetUpdateAll();

        // Temporal loop.
        while (GetDevice()->run()) {
            std::cout << "Time : " << m_system->GetTime() << std::endl;
            BeginScene();
            DrawAll();
            // Time-stepping.
            DoStep(); // m_system->GetChTime() is also updated here.
            EndScene();

            // Condition to stop the time-domain simulation using the time after time-stepping.
            if (endTime > 0. && m_system->GetTime() > endTime) break; // If the endTime given is negative or null, the loop is infinite :)


        }

    }

    void FrIrrApp::Visualize() {
        AssetBindAll();
        AssetUpdateAll();

        while (GetDevice()->run()) {
            BeginScene();
            DrawAll();
            EndScene();
        }

    }

    void FrIrrApp::VisualizeStaticAnalysis(){
        AssetBindAll();
        AssetUpdateAll();

        while (GetDevice()->run()) {
            BeginScene();
            DrawAll();

            m_system->GetStaticAnalysis()->SetNbIteration(2);
            m_system->SolveStaticWithRelaxation();

            EndScene();
        }

    }

}  // end namespace frydom
