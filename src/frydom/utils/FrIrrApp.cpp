//
// Created by frongere on 02/06/17.
//

#include "FrIrrApp.h"
#include "chrono_irrlicht/ChIrrWizard.h"

#include "frydom/core/FrOffshoreSystem.h"

#define SQ2_2 (sqrt(2.)/2.)

namespace frydom {

    FrIrrApp::FrIrrApp(FrOffshoreSystem& system, const double dist)
            : chrono::irrlicht::ChIrrApp(&system,
                                         L"FRyDoM viewer",
                                         irr::core::dimension2d<irr::u32>(800, 600),
                                         false,
                                         false,
                                         true,
                                         irr::video::EDT_OPENGL){

        SetSkyBox();
        AddCustomLights();
        AddCustomCamera(irr::core::vector3df(-(irr::f32)dist, (irr::f32)dist*0, (irr::f32)dist),
                         irr::core::vector3df(0, (irr::f32)SQ2_2, (irr::f32)SQ2_2));
        AddTypicalLogo("frydom_logo.png");
    }


    FrIrrApp::FrIrrApp(FrOffshoreSystem* system,
                       const wchar_t* title,
                       irr::core::dimension2d<irr::u32> dimens)
        : chrono::irrlicht::ChIrrApp(system, title, dimens, false, false, true, irr::video::EDT_OPENGL){

        SetSkyBox();
    }

    FrIrrApp::~FrIrrApp() {}

    // -----------------------------------------------------------------------------
    // Create a skybox that has Z pointing up.
    // Note that the default ChIrrApp::AddTypicalSky() uses Y up.
    // -----------------------------------------------------------------------------
    void FrIrrApp::SetSkyBox() {
//        std::string mtexturedir = "../data/skybox/";
        std::string str_lf = "sky_lf.jpg";
        std::string str_up = "sky_up.jpg";
        std::string str_dn = "sky_dn.jpg";

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














    /// REFACTORING ------------->>>>>>>>>>>>>>>>



    FrIrrApp_::FrIrrApp_(chrono::ChSystem* system, double dist)
            : chrono::irrlicht::ChIrrApp(system,
                                         L"FRyDoM viewer",
                                         irr::core::dimension2d<irr::u32>(800, 600),
                                         false,
                                         false,
                                         true,
                                         irr::video::EDT_OPENGL),
                                         m_system(system)
                                         {

        SetSkyBox();
        AddCustomLights();

        AddCustomCamera(irr::core::vector3df(-(irr::f32)dist, (irr::f32)dist*0, (irr::f32)dist),
                         irr::core::vector3df(0, (irr::f32)SQ2_2, (irr::f32)SQ2_2));
        AddTypicalLogo("frydom_logo.png");
    }

    FrIrrApp_::~FrIrrApp_() = default;

    // -----------------------------------------------------------------------------
    // Create a skybox that has Z pointing up.
    // Note that the default ChIrrApp::AddTypicalSky() uses Y up.
    // -----------------------------------------------------------------------------
    void FrIrrApp_::SetSkyBox() {
        std::string str_lf = "sky_lf.jpg";
        std::string str_up = "sky_up.jpg";
        std::string str_dn = "sky_dn.jpg";

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

    FrIrrCamera* FrIrrApp_::AddCustomCamera(irr::core::vector3df mpos, irr::core::vector3df mtarg) {

        // create and init camera
        auto camera = new FrIrrCamera(GetDevice(), GetSceneManager()->getRootSceneNode(), GetSceneManager(),
                                                                              -1, -160.0f, 1.0f, 10.0f);

        camera->setPosition(mpos);
        camera->setTarget(mtarg);

        camera->setNearValue(0.1f);
        camera->setMinZoom(0.6f);

        return camera;
    }

    void FrIrrApp_::AddCustomLights(irr::core::vector3df pos1, irr::core::vector3df pos2,
                                   double rad1, double rad2,
                                   irr::video::SColorf col1, irr::video::SColorf col2)  {
        // create lights
        irr::scene::ILightSceneNode* mlight1 = GetDevice()->getSceneManager()->addLightSceneNode(0, pos1, col1, (irr::f32)rad1);

        irr::scene::ILightSceneNode* mlight2 = GetDevice()->getSceneManager()->addLightSceneNode(0, pos2, col2, (irr::f32)rad2);

        mlight2->enableCastShadow(false);
    }

    void FrIrrApp_::Run(double endTime) {

        AssetBindAll();
        AssetUpdateAll();

        while (GetDevice()->run()) {
            BeginScene();
            DrawAll();
            DoStep();
            EndScene();

            if (endTime > 0. && m_system->GetChTime() > endTime) break; // If the endTime given is negative or null, the loop is infinite :)

        }

    }



















}  // end namespace frydom