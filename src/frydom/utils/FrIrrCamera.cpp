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


#include "FrIrrCamera.h"


namespace frydom {

  using namespace irr;
  using namespace irr::scene;

  FrIrrCamera::FrIrrCamera(IrrlichtDevice *devicepointer,
                           ISceneNode *parent,
                           ISceneManager *smgr,
                           s32 id,
                           f32 rs,
                           f32 zs,
                           f32 ts)
      : ICameraSceneNode(parent,
                         smgr,
                         id,
                         core::vector3df(1.0f, 1.0f, 1.0f),
                         core::vector3df(0.0f, 0.0f, 0.0f),
                         core::vector3df(1.0f, 1.0f, 1.0f)),
        InputReceiverEnabled(true) {
    device = devicepointer;
    BBox.reset(0, 0, 0);

    UpVector.set(0.0f, 0.0f, 1.0f);

    Fovy = core::PI / 2.5f;
    Aspect = 4.0f / 3.0f;
    ZNear = 1.0f;
    ZFar = 3000.0f;

    atMinDistance = false;

    video::IVideoDriver *d = smgr->getVideoDriver();
    if (d) {
      screenDim.Width = (f32) d->getCurrentRenderTargetSize().Width;
      screenDim.Height = (f32) d->getCurrentRenderTargetSize().Height;
      Aspect = screenDim.Width / screenDim.Height;
    }

    zooming = false;
    rotating = false;
    moving = false;
    translating = false;
    zoomSpeed = zs;
    rotateSpeed = rs;
    translateSpeed = ts;
    currentZoom = 100.0f;
    targetMinDistance = 1.0f;
    targetMaxDistance = 2000.0f;
    Target.set(0.0f, 0.0f, 0.0f);
    rotX = 0;
    rotY = 0;
    oldTarget = Target;

    atMinDistance = false;
    atMaxDistance = false;

    allKeysUp();
    allMouseKeysUp();

    recalculateProjectionMatrix();
    recalculateViewArea();

    smgr->setActiveCamera(this);
  }

  bool FrIrrCamera::OnEvent(const SEvent &event) {
    if (!InputReceiverEnabled)
      return false;

    core::dimension2d<u32> ssize = SceneManager->getVideoDriver()->getScreenSize();

    if (event.EventType == EET_MOUSE_INPUT_EVENT) {
      switch (event.MouseInput.Event) {
        case EMIE_LMOUSE_PRESSED_DOWN:
          // selectednode =
          // SceneManager->getSceneCollisionManager()->getSceneNodeFromScreenCoordinatesBB(device->getCursorControl()->getPosition(),0xFF,false);

          // if(selectednode)
          //   pointCameraAtNode(selectednode);
          // else
        {
          selectednode = NULL;
          MouseKeys[0] = true;
        }
          break;
        case EMIE_RMOUSE_PRESSED_DOWN:
          MouseKeys[2] = true;
          break;
        case EMIE_MMOUSE_PRESSED_DOWN:
          MouseKeys[1] = true;
          break;
        case EMIE_LMOUSE_LEFT_UP:
          MouseKeys[0] = false;
          break;
        case EMIE_RMOUSE_LEFT_UP:
          MouseKeys[2] = false;
          break;
        case EMIE_MMOUSE_LEFT_UP:
          MouseKeys[1] = false;
          break;
        case EMIE_MOUSE_MOVED:
          MousePos.X = event.MouseInput.X / (f32) ssize.Width;
          MousePos.Y = event.MouseInput.Y / (f32) ssize.Height;
          break;
        case EMIE_MOUSE_WHEEL:
          currentZoom -= event.MouseInput.Wheel * zoomSpeed;

          if (currentZoom <= targetMinDistance)
            atMinDistance = true;
          else if (currentZoom >= targetMaxDistance)
            atMaxDistance = true;
          else {
            atMinDistance = false;
            atMaxDistance = false;
          }

          break;
        default:
          break;
      }
      return true;
    }

    if (event.EventType == EET_KEY_INPUT_EVENT) {
      Keys[event.KeyInput.Key] = event.KeyInput.PressedDown;
      return true;
    }

    return false;
  }

  void FrIrrCamera::OnRegisterSceneNode() {
    video::IVideoDriver *driver = SceneManager->getVideoDriver();
    if (!driver)
      return;

    if (SceneManager->getActiveCamera() == this) {
      screenDim.Width = (f32) driver->getCurrentRenderTargetSize().Width;
      screenDim.Height = (f32) driver->getCurrentRenderTargetSize().Height;

      driver->setTransform(video::ETS_PROJECTION, Projection);

      // If UpVector and Vector to Target are the same, we have a problem.
      // Correct it.
      core::vector3df pos = getAbsolutePosition();
      core::vector3df tgtv = Target - pos;
      tgtv.normalize();

      core::vector3df up = UpVector;
      up.normalize();

      f32 dp = tgtv.dotProduct(up);
      if ((dp > -1.0001f && dp < -0.9999f) || (dp < 1.0001f && dp > 0.9999f))
        up.X += 1.0f;

      View.buildCameraLookAtMatrixLH(pos, Target, up);  // Right hand camera: use "..RH" here and in the following
      recalculateViewArea();

      SceneManager->registerNodeForRendering(this, ESNRP_CAMERA);
    }

    if (IsVisible)
      ISceneNode::OnRegisterSceneNode();
  }

  void FrIrrCamera::render() {
    video::IVideoDriver *driver = SceneManager->getVideoDriver();
    if (!driver)
      return;

    driver->setTransform(video::ETS_VIEW, View);
  }

  void FrIrrCamera::OnAnimate(u32 timeMs) {
    animate();

    ISceneNode::setPosition(Pos);
    updateAbsolutePosition();

    // TODO Add Animators
  }

  bool FrIrrCamera::isInputReceiverEnabled() const {
    _IRR_IMPLEMENT_MANAGED_MARSHALLING_BUGFIX;
    return InputReceiverEnabled;
  }

  void FrIrrCamera::setNearValue(f32 f) {
    ZNear = f;
    recalculateProjectionMatrix();
  }

  void FrIrrCamera::setFarValue(f32 f) {
    ZFar = f;
    recalculateProjectionMatrix();
  }

  void FrIrrCamera::setAspectRatio(f32 f) {
    Aspect = f;
    recalculateProjectionMatrix();
  }

  void FrIrrCamera::setFOV(f32 f) {
    Fovy = f;
    recalculateProjectionMatrix();
  }

  void FrIrrCamera::setViewMatrixAffector(const core::matrix4 &affector) {
    Affector = affector;
  }

  void FrIrrCamera::setUpVector(const core::vector3df &pos) {
    UpVector = pos;
  }

  void FrIrrCamera::setProjectionMatrix(const core::matrix4 &projection, bool isOrthogonal) {
    Projection = projection;
  }

  void FrIrrCamera::setPosition(const core::vector3df &pos) {
    Pos = pos;
    updateAnimationState();

    ISceneNode::setPosition(pos);
  }

  void FrIrrCamera::setTarget(const core::vector3df &pos) {
    Target = oldTarget = pos;
    updateAnimationState();
  }

  void FrIrrCamera::setZoomSpeed(f32 value) {
    zoomSpeed = value;
  }

  void FrIrrCamera::setTranslateSpeed(f32 value) {
    translateSpeed = value;
  }

  void FrIrrCamera::setRotationSpeed(f32 value) {
    rotateSpeed = value;
  }

  void FrIrrCamera::pointCameraAtNode(ISceneNode *selectednode) {
    core::vector3df totarget = getPosition() - getTarget();
    setPosition(selectednode->getPosition() + (totarget.normalize() * 100));
    setTarget(selectednode->getPosition());
    updateAnimationState();
  }

  void FrIrrCamera::setMinZoom(f32 amount) {
    targetMinDistance = amount;
  }

  void FrIrrCamera::setMaxZoom(f32 amount) {
    targetMaxDistance = amount;
  }

  void FrIrrCamera::recalculateProjectionMatrix() {
    Projection.buildProjectionMatrixPerspectiveFovLH(Fovy, Aspect, ZNear,
                                                     ZFar);  // Right hand camera: use "..RH" here and in the following
  }

  void FrIrrCamera::recalculateViewArea() {
    core::matrix4 mat = Projection * View;
    ViewArea = SViewFrustum(mat);

    ViewArea.cameraPosition = getAbsolutePosition();
    ViewArea.recalculateBoundingBox();
  }

  void FrIrrCamera::allKeysUp() {
    for (int i = 0; i < KEY_KEY_CODES_COUNT; i++)
      Keys[i] = false;
  }

  void FrIrrCamera::allMouseKeysUp() {
    for (s32 i = 0; i < 3; ++i)
      MouseKeys[i] = false;
  }

  bool FrIrrCamera::isKeyDown(s32 key) {
    return Keys[key];
  }

  bool FrIrrCamera::isMouseKeyDown(s32 key) {
    return MouseKeys[key];
  }

  void FrIrrCamera::animate() {
    // Rotation Vals
    f32 nRotX = rotX;
    f32 nRotY = rotY;
    f32 nZoom = currentZoom;

    // Translation Vals
    core::vector3df translate(oldTarget);
    core::vector3df direction = Pos - Target;
    core::vector3df tvectX = direction.crossProduct(UpVector);
    tvectX.normalize();
    core::vector3df tvectY = direction.crossProduct(tvectX);
    tvectY.normalize();

    // Zoom
    if (isMouseKeyDown(MOUSE_BUTTON_RIGHT) && isMouseKeyDown(MOUSE_BUTTON_LEFT)) {
      if (!zooming) {
        zoomStartX = MousePos.X;
        zoomStartY = MousePos.Y;
        zooming = true;
        nZoom = currentZoom;
      } else {
        f32 old = nZoom;
        nZoom += (zoomStartX - MousePos.X) * zoomSpeed * 100;

        if (nZoom < targetMinDistance)
          nZoom = targetMinDistance;
        else if (nZoom > targetMaxDistance)
          nZoom = targetMaxDistance;

        if (nZoom < 0)
          nZoom = old;
      }
    } else {
      if (zooming) {
        f32 old = currentZoom;
        currentZoom = currentZoom + (zoomStartX - MousePos.X) * zoomSpeed;
        nZoom = currentZoom;

        if (nZoom < 0)
          nZoom = currentZoom = old;
      }
      zooming = false;
    }

    // Rotation
    if (isMouseKeyDown(MOUSE_BUTTON_LEFT) && !zooming) {
      if (!rotating) {
        rotateStartX = MousePos.X;
        rotateStartY = MousePos.Y;
        rotating = true;
        nRotX = rotX;
        nRotY = rotY;
      } else {
        nRotX += (rotateStartX - MousePos.X) * rotateSpeed;
        nRotY += (rotateStartY - MousePos.Y) * rotateSpeed;
      }
    } else {
      if (rotating) {
        rotX = rotX + (rotateStartX - MousePos.X) * rotateSpeed;
        rotY = rotY + (rotateStartY - MousePos.Y) * rotateSpeed;
        nRotX = rotX;
        nRotY = rotY;
      }

      rotating = false;
    }
    //Translate
    if (isMouseKeyDown(MOUSE_BUTTON_MIDDLE) && !zooming) {
      if (!translating) {
        translateStartX = MousePos.X;
        translateStartY = MousePos.Y;
        translating = true;
        oldTarget = Target;
      } else {
        translate =
            -10 * (tvectX * (MousePos.X - translateStartX) + tvectY * (MousePos.Y - translateStartY)) * translateSpeed;
        Target = oldTarget + translate;
      }
    } else if (isKeyDown(KEY_UP) && !zooming) {
      if (!translating) {
        translating = true;
        oldTarget = Target;
      } else {
        translate = tvectY * translateSpeed;
        Target = oldTarget + translate;
      }
    } else if (isKeyDown(KEY_DOWN) && !zooming) {
      if (!translating) {
        translating = true;
        oldTarget = Target;
      } else {
        translate = -tvectY * translateSpeed;
        Target = oldTarget + translate;
      }
    } else if (isKeyDown(KEY_LEFT) && !zooming) {
      if (!translating) {
        translating = true;
        oldTarget = Target;
      } else {
        translate = tvectX * translateSpeed;
        Target = oldTarget + translate;
      }
    } else if (isKeyDown(KEY_RIGHT) && !zooming) {
      if (!translating) {
        translating = true;
        oldTarget = Target;
      } else {
        translate = -tvectX * translateSpeed;
        Target = oldTarget + translate;
      }
    } else if (isKeyDown(KEY_PRIOR) && !zooming)  // ALE
    {
      if (!translating) {
        translating = true;
        oldTarget = Target;
      } else {
        translate = -UpVector * translateSpeed;
        Target = oldTarget + translate;
      }
    } else if (isKeyDown(KEY_NEXT) && !zooming)  // ALE
    {
      if (!translating) {
        translating = true;
        oldTarget = Target;
      } else {
        translate = UpVector * translateSpeed;
        Target = oldTarget + translate;
      }
    } else if (translating) {
      translating = false;
    }

    // Set Position

    Pos.X = nZoom + Target.X;
    Pos.Y = Target.Y;
    Pos.Z = Target.Z;

    Pos.rotateXZBy(nRotY, Target);
    Pos.rotateXYBy(nRotX, Target);

    // Correct Rotation Error
    UpVector.set(0, 0, 1);
    UpVector.rotateXZBy(-nRotY, core::vector3df(0, 0, 0));
    UpVector.rotateXYBy(nRotX + 180.f, core::vector3df(0, 0, 0));
  }

  void FrIrrCamera::updateAnimationState() {
    core::vector3df pos(Pos - Target);

    // X rotation
    core::vector2df vec2d(pos.X, pos.Y);
    rotX = (f32) vec2d.getAngle();

    // Y rotation
    pos.rotateXYBy(rotX, core::vector3df());
    vec2d.set(pos.X, pos.Z);
    rotY = -(f32) vec2d.getAngle();

    // Zoom
    currentZoom = (f32) Pos.getDistanceFrom(Target);
  }

  void FrIrrCamera::updateMatrices() {}

}  // end namespace frydom
