//
// Created by frongere on 02/06/17.
//

#include "FrIrrApp.h"

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
        //AddTypicalLights();
        AddTypicalLights(irr::core::vector3df(-30.f, -100.f, 30.f),
                irr::core::vector3df(-30.f, -80.f, -30.f),
                 290,
                 190,
                irr::video::SColorf(0.7f, 0.7f, 0.7f, 1.0f),
                irr::video::SColorf(0.7f, 0.8f, 0.8f, 1.0f));
        AddTypicalCamera(irr::core::vector3df(0, (irr::f32)dist, (irr::f32)dist),
                         irr::core::vector3df(0, (irr::f32)SQ2_2, (irr::f32)SQ2_2));
        //AddTypicalLogo("frydom_logo.png");
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












}  // end namespace frydom