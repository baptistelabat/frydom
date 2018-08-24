//
// Created by Lucas Letournel on 24/08/18.
//

#include "FrIrrCamera.h"

FrIrrCamera::FrIrrCamera(irr::IrrlichtDevice *devicepointer, irr::scene::ISceneNode *parent,
                         irr::scene::ISceneManager *smgr, irr::s32 id, irr::f32 rotateSpeed, irr::f32 zoomSpeed,
                         irr::f32 translationSpeed)
        : RTSCamera(devicepointer, parent, smgr, id, rotateSpeed, zoomSpeed, translationSpeed) {
}

void FrIrrCamera::OnAnimate(irr::u32 timeMs) {
    RTSCamera::OnAnimate(timeMs);


}
