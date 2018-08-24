//
// Created by Lucas Letournel on 24/08/18.
//

#ifndef FRYDOM_FRIRRCAMERA_H
#define FRYDOM_FRIRRCAMERA_H


#include <chrono_irrlicht/ChIrrCamera.h>
//#include "chrono_irrlicht/ChApiIrr.h"


class FrIrrCamera :public chrono::irrlicht::RTSCamera {
public:

    FrIrrCamera(irr::IrrlichtDevice* devicepointer,
                irr::scene::ISceneNode* parent,
                irr::scene::ISceneManager* smgr,
                irr::s32 id,
                irr::f32 rotateSpeed = -160.0f,
                irr::f32 zoomSpeed = 20.0f,
                irr::f32 translationSpeed = 20.0f);


private:


    virtual void OnAnimate(irr::u32 timeMs);



};


#endif //FRYDOM_FRIRRCAMERA_H
