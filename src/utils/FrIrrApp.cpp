//
// Created by frongere on 02/06/17.
//

#include "FrIrrApp.h"

namespace frydom {


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
        std::string mtexturedir = "../data/skybox/";
        std::string str_lf = mtexturedir + "sky_lf.jpg";
        std::string str_up = mtexturedir + "sky_up.jpg";
        std::string str_dn = mtexturedir + "sky_dn.jpg";

        irr::video::ITexture* map_skybox_side = GetVideoDriver()->getTexture(str_lf.c_str());
        irr::scene::ISceneNode* mbox = GetSceneManager()->addSkyBoxSceneNode(
                GetVideoDriver()->getTexture(str_up.c_str()),
                GetVideoDriver()->getTexture(str_dn.c_str()),
                map_skybox_side,
                map_skybox_side,
                map_skybox_side,
                map_skybox_side
        );
        mbox->setRotation(irr::core::vector3df(90, 0, 0));

    };










}  // end namespace frydom