//
// Created by Lucas Letournel on 24/08/18.
//

#ifndef FRYDOM_FRIRRCAMERA_H
#define FRYDOM_FRIRRCAMERA_H


//#include <irrlicht.h>
//#include <chrono_irrlicht/ChIrrCamera.h>
//#include "chrono_irrlicht/ChApiIrr.h"

#include "chrono_irrlicht/ChIrrTools.h"

namespace frydom {

/// Copied from ChIrrCamera and modified to get UpVector on Z.
/// Class to create an interactive videocamera in Irrlicht, that is similar to
/// the Maya camera but hasn't the problems that the Maya camera has in
/// Irrlicht 1.5.
/// This code is based on work by "CmdKewin" (from the Irrlicht forum).
    class FrIrrCamera : public irr::scene::ICameraSceneNode {
    public:

        FrIrrCamera(irr::IrrlichtDevice *devicepointer,
                    irr::scene::ISceneNode *parent,
                    irr::scene::ISceneManager *smgr,
                    irr::s32 id,
                    irr::f32 rotateSpeed = -160.0f,
                    irr::f32 zoomSpeed = 1.0f,
                    irr::f32 translationSpeed = 1.0f);


        virtual ~FrIrrCamera() {}

        // Events
        virtual void render();

        virtual bool OnEvent(const irr::SEvent &event);

        virtual void OnRegisterSceneNode();

        virtual void OnAnimate(irr::u32 timeMs);

        // Setup
        virtual void setInputReceiverEnabled(bool enabled) { InputReceiverEnabled = enabled; }

        virtual bool isInputReceiverEnabled() const;

        // Gets
        virtual const irr::core::aabbox3d<irr::f32> &getBoundingBox() const { return BBox; }

        virtual const irr::core::matrix4 &getProjectionMatrix() const { return Projection; }

        virtual const irr::scene::SViewFrustum *getViewFrustum() const { return &ViewArea; }

        virtual const irr::core::matrix4 &getViewMatrix() const { return View; }

        virtual const irr::core::matrix4 &getViewMatrixAffector() const { return Affector; }

        virtual const irr::core::vector3df &getUpVector() const { return UpVector; }

        virtual const irr::core::vector3df &getTarget() const { return Target; }

        virtual irr::f32 getNearValue() const { return ZNear; }

        virtual irr::f32 getFarValue() const { return ZFar; }

        virtual irr::f32 getAspectRatio() const { return Aspect; }

        virtual irr::f32 getFOV() const { return Fovy; }

        // Sets
        virtual void setNearValue(irr::f32 zn);

        virtual void setFarValue(irr::f32 zf);

        virtual void setAspectRatio(irr::f32 aspect);

        virtual void setFOV(irr::f32 fovy);

        virtual void setViewMatrixAffector(const irr::core::matrix4 &affector);

        virtual void setUpVector(const irr::core::vector3df &pos);

        virtual void setProjectionMatrix(const irr::core::matrix4 &projection, bool isOrthogonal = false);

        virtual void setPosition(const irr::core::vector3df &newpos);

        virtual void setTarget(const irr::core::vector3df &newpos);

        virtual void setRotation(const irr::core::vector3df &rotation) {}

        virtual void setZoomSpeed(irr::f32 value);

        virtual void setTranslateSpeed(irr::f32 value);

        virtual void setRotationSpeed(irr::f32 value);

        virtual void bindTargetAndRotation(bool bound) {}

        virtual bool getTargetAndRotationBinding(void) const { return false; }

        // Helper Functions
        void pointCameraAtNode(ISceneNode *selectednode);

        void setMinZoom(irr::f32 amount);

        void setMaxZoom(irr::f32 amount);

        // Type Return
        virtual irr::scene::ESCENE_NODE_TYPE getType() const { return irr::scene::ESNT_CAMERA; }

        // Public Attributes
        bool atMinDistance;
        bool atMaxDistance;
        ISceneNode *selectednode;

    protected:
        // Properties
        irr::core::vector3df Target;
        irr::core::vector3df UpVector;
        irr::core::matrix4 Projection;
        irr::core::matrix4 View;
        irr::core::matrix4 Affector;  //**ALEX
        irr::scene::SViewFrustum ViewArea;
        irr::core::aabbox3d<irr::f32> BBox;
        bool InputReceiverEnabled;
        irr::core::dimension2d<irr::f32> screenDim;
        irr::f32 Fovy;    /// Field of view, in radians.
        irr::f32 Aspect;  /// Aspect ratio.
        irr::f32 ZNear;   /// Value of the near view-plane.
        irr::f32 ZFar;    /// Z-value of the far view-plane.

        void recalculateProjectionMatrix();

        void recalculateViewArea();

    private:
        irr::IrrlichtDevice *device;
        irr::core::vector3df Pos;
        bool zooming, rotating, moving, translating;
        irr::f32 zoomSpeed;
        irr::f32 translateSpeed;
        irr::f32 rotateSpeed;
        irr::f32 rotateStartX, rotateStartY;
        irr::f32 zoomStartX, zoomStartY;
        irr::f32 translateStartX, translateStartY;
        irr::f32 currentZoom;
        irr::f32 rotX, rotY;
        irr::core::vector3df oldTarget;
        irr::core::vector2df MousePos;
        bool Keys[irr::KEY_KEY_CODES_COUNT];
        bool MouseKeys[3];
        irr::f32 targetMinDistance;
        irr::f32 targetMaxDistance;

        enum MOUSE_BUTTON {
            MOUSE_BUTTON_LEFT, MOUSE_BUTTON_MIDDLE, MOUSE_BUTTON_RIGHT
        };

        void allKeysUp();

        void allMouseKeysUp();

        bool isKeyDown(irr::s32 key);

        bool isMouseKeyDown(irr::s32 key);

        void animate();

        void updateAnimationState();

        // Hammad: Adding this so that GCC and clang on osx do not complain
        void updateMatrices();


    };
}

#endif //FRYDOM_FRIRRCAMERA_H
