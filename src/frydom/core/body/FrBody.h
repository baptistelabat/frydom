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


#ifndef FRYDOM_FRBODY_H
#define FRYDOM_FRBODY_H

#include "chrono/physics/ChBodyAuxRef.h"

#include "frydom/core/common/FrObject.h"
#include "frydom/core/FrOffshoreSystem.h"
#include "FrInertiaTensor.h"
#include "frydom/core/misc/FrColors.h"
#include "frydom/core/common/FrNode.h"
#include "frydom/asset/FrAssetOwner.h"
#include "frydom/mesh/FrMesh.h"

// TODO : voir si il n'y a pas moyen de passer ces includes
#include "frydom/hydrodynamics/seakeeping/linear/radiation/FrAddedMassBase.h"
#include "frydom/hydrodynamics/seakeeping/linear/radiation/FrAddedMassBase2.h"
#include "frydom/hydrodynamics/seakeeping/linear/radiation/FrVariablesAddedMassBase.h"


#define DEFAULT_MAX_SPEED (float)10.
#define DEFAULT_MAX_ROTATION_SPEED (float)(180.*DEG2RAD)

namespace frydom {


    // Forward declarations
    class FrUnitQuaternion;
    class FrBody;


    namespace internal {

        /// Base class inheriting from chrono ChBodyAuxRef
        /// This class must not be used by external FRyDoM users. It is used in composition rule along with the FrBody_ FRyDoM class
        struct FrBodyBase : public chrono::ChBodyAuxRef {

            FrBody *m_frydomBody;                      ///< pointer to the FrBody containing this bodyBase

            /// Constructor of the bodyBase
            /// \param body body containing this bodyBase
            explicit FrBodyBase(FrBody *body);

            /// Initial setup of the bodyBase, called from chrono, call the Initialize of the body
            void SetupInitial() override;

            /// Update the state of the bodyBase, called from chrono, call the Update of the body
            /// \param update_assets check if the assets are updated
            void Update(bool update_assets) override;

            /// Update the state of the body when this one is moved by its setters.
            /// It is not called during simulation by chrono
            void UpdateAfterMove();

            /// Update the markers position, relatively to the new Center Of Gravity given
            /// \param newCOG new Center Of Gravity
            void UpdateMarkerPositionToCOG(const chrono::ChVector<> newCOG);

            /// Removes an asset given its shared pointer
            void RemoveAsset(std::shared_ptr<chrono::ChAsset> asset);

        };

    }  // end namespace internal


    // Forward declarations
    class FrForce;
    class FrFrame;
    class FrRotation;
    class FrGeographicCoord;
    class FrAsset;
    class FrBodyDOFMask;
    class FrLink;
    class FrTriangleMeshConnected;

    /// Main class for a FRyDoM rigid body
    /**
     * \class FrBody
     * \brief Class for defining a body.
     */
    class FrBody : public FrObject, public FrAssetOwner {

    protected:

        std::shared_ptr<internal::FrBodyBase> m_chronoBody;  ///< Embedded Chrono body Object

        FrOffshoreSystem* m_system;                ///< Pointer to the FrOffshoreSystem where the body has been registered

        using ForceContainer = std::vector<std::shared_ptr<FrForce>>;
        ForceContainer m_externalForces;            ///< Container of the external forces acting on body

        using NodeContainer = std::vector<std::shared_ptr<FrNode>>;
        NodeContainer m_nodes;                    ///< Container of the nodes belonging to the body

        using CONTACT_TYPE = FrOffshoreSystem::SYSTEM_TYPE;
        CONTACT_TYPE m_contactType = CONTACT_TYPE::SMOOTH_CONTACT; ///< The contact method that has to be consistent with that of the FrOffshoreSystem


        std::unique_ptr<FrBodyDOFMask> m_DOFMask;
        std::shared_ptr<FrLink> m_DOFLink;

    public:

        /// Default constructor
        FrBody();

        /// Get the FrOffshoreSystem where the body has been registered
        FrOffshoreSystem* GetSystem() const;

        /// Make the body fixed
        /// \param state true if body is fixed, false otherwise
        void SetFixedInWorld(bool state);

        /// Enable/disable option for setting bodies to "sleep".
        /// If use sleeping = true, bodies which stay in same place
        /// for long enough time will be deactivated, for optimization.
        /// The realism is limited, but the simulation is faster.
        void SetUseSleeping(bool state);

        /// Return true if 'sleep' mode is activated.
        bool GetUseSleeping() const;

        /// Force the body in sleeping mode or not (usually this state change is not
        /// handled by users, anyway, because it is mostly automatic).
        void SetSleeping(bool state);

        /// Return true if this body is currently in 'sleep' mode.
        bool GetSleeping() const;

        /// Test if a body could go in sleeping state if requirements are satisfied.
        /// Return true if state could be changed from no sleep to sleep.
        bool TrySleeping();

        /// Return true if the body is active; i.e. it is neither fixed to ground
        /// nor is it in "sleep" mode. Return false otherwise.
        bool IsActive();
        
        /// Return true if the body is included in the static analysis
        bool IncludedInStaticAnalysis() const {return true;}

        /// Get the type name of this object
        /// \return type name of this object
        std::string GetTypeName() const override { return "Body"; }

        // =============================================================================================================
        // LOGGING
        // =============================================================================================================

        void InitializeLog();


        // =============================================================================================================
        // PRINCIPAL INERTIAL PARAMETERS
        // =============================================================================================================

        /// Get the body mass in kg
        double GetMass() const;

        /// Get the inertia parameters as a FrInertiaTensor object
        // TODO : gerer la frame convention !
        FrInertiaTensor GetInertiaTensor(FRAME_CONVENTION fc) const; // TODO : voir pour une methode renvoyant une reference non const

        /// Set the inertia parameters as a FrInertiaTensor object
        void SetInertiaTensor(const FrInertiaTensor &inertia);

        // =============================================================================================================
        // CONTACT
        // =============================================================================================================

        /// Set the contact method to SMOOTH
        /// The system where the body is registered must be consistent
        void SetSmoothContact();

        /// Set the contact method to NONSMOOTH
        /// The system where the body is registered must be consistent
        void SetNonSmoothContact();

        /// Set the contact method (SMOOTH or NONSMOOTH)
        /// The system where the body is registered must be consistent
        /// \param contactType contact method to be used (SMOOTH/NONSMOOTH)
        void SetContactMethod(CONTACT_TYPE contactType);

        /// Get the contact method of this body
        /// \return contact method used (SMOOTH/NONSMOOTH)
        CONTACT_TYPE GetContactType() const;

        /// Set the collide mode. If true, a collision shape must be set and the body will participate in physical
        /// collision with other physical collision enabled items
        /// \param isColliding true if a collision model is to be defined, false otherwise
        void AllowCollision(bool isColliding);

        std::shared_ptr<chrono::ChMaterialSurfaceSMC> GetMaterialSurface() {return m_chronoBody->GetMaterialSurfaceSMC();}

        // TODO : ajouter de quoi definir des shapes de collision !!!

        // =============================================================================================================
        // VISUAL ASSETS
        // =============================================================================================================
//        void AssetActive() // TODO

        /// Add a mesh as an asset for visualization given a WaveFront .obj file name
        /// \param obj_filename filename of the asset to be added
        void AddMeshAsset(std::string obj_filename);

        /// Add a mesh as an asset for visualization given a FrTriangleMeshConnected mesh object
        /// \param mesh mesh of the asset to be added
        void AddMeshAsset(std::shared_ptr<FrTriangleMeshConnected> mesh);

        // =============================================================================================================
        // SPEED LIMITATIONS TO STABILIZE SIMULATIONS
        // =============================================================================================================

        /// Enable the maximum speed limits in both linear and angular speed (beyond this limit it will be clamped).
        /// This is useful in virtual reality and real-time simulations, because
        /// it reduces the risk of bad collision detection.
        /// The realism is limited, but the simulation is more stable.
        /// \param activate true if the speed limit is activated, false otherwise
        void ActivateSpeedLimits(bool activate);

        /// Set the maximum linear speed (beyond this limit it will be clamped). In m/s
        /// This is useful in virtual reality and real-time simulations, because
        /// it reduces the risk of bad collision detection.
        /// This speed limit is active only if you set  SetLimitSpeed(true);
        /// \param maxSpeed maximum linear speed, for the speed limit feature
        void SetMaxSpeed(double maxSpeed);

        /// Set the maximum angular speed (beyond this limit it will be clamped). In rad/s
        /// This is useful in virtual reality and real-time simulations, because
        /// it reduces the risk of bad collision detection.
        /// This speed limit is active only if you set  SetLimitSpeed(true);
        /// \param maxSpeed maximum angular speed, for the speed limit feature
        void SetMaxRotationSpeed(double wMax);

        /// [DEBUGGING MODE] Remove the gravity by adding a anti-gravity. This is a debugging method and should not be
        /// used in projects
        /// \param val true if the gravity is to be removed, false otherwise.
        void RemoveGravity(bool val);


        // =============================================================================================================
        // FORCES
        // =============================================================================================================

        /// Add an external force to the body
        /// \param force force to be added to the body
        void AddExternalForce(std::shared_ptr<FrForce> force);

        /// Remove an external force to the body
        /// \param force force to be removed to the body
        void RemoveExternalForce(std::shared_ptr<FrForce> force);

        /// Remove all forces from the body
        void RemoveAllForces();

        /// Get the list of all external forces
        /// \return List of all external forces
        ForceContainer GetForceList() const;

        // ##CC adding for monitoring force

        Force GetTotalExtForceInWorld(FRAME_CONVENTION fc) const;

        Force GetTotalExtForceInBody(FRAME_CONVENTION fc) const;

        Torque GetTotalTorqueInBodyAtCOG(FRAME_CONVENTION fc) const;
        // ##CC

        // =============================================================================================================
        // NODES
        // =============================================================================================================

        /// Generates a new node attached to the body which position and orientation are coincident with the body
        /// reference frame
        /// \return node created
        std::shared_ptr<FrNode> NewNode();

        /// Get the list of all nodes added to the body
        /// \return List of all nodes added to the body
        NodeContainer GetNodeList() const;

        // TODO : permettre de definir un frame a l'aide des parametres de Denavit-Hartenberg modifies ?? --> dans FrFrame !

        // =============================================================================================================
        // POSITIONS
        // =============================================================================================================

        // Position of the body reference frame in world -->

        /// Get the position in world frame of the origin of the body reference frame
        /// \param fc frame convention (NED/NWU)
        /// \return position in world frame of the origin of the body reference frame
        Position GetPosition(FRAME_CONVENTION fc) const;

        /// Get the geographic position in world frame of the origin of the body reference frame
        /// \param fc frame convention (NED/NWU)
        /// \return geographic position of the origin of the body reference frame
        FrGeographicCoord GetGeoPosition(FRAME_CONVENTION fc) const;

        /// Set the position in world frame of the origin of the body reference frame
        /// Note that it moves the entire body along with its nodes and other attached elements to the body (nodes...)
        /// which are updated
        /// \param worldPos position in world frame of the origin of the body reference frame
        /// \param fc frame convention (NED/NWU)
        void SetPosition(const Position& worldPos, FRAME_CONVENTION fc);

        /// Set the position in world frame of the origin of the body reference frame, at a geographic position
        /// Note that it moves the entire body along with its nodes and other attached elements to the body (nodes...)
        /// which are updated
        /// \param geoPos geographic destination for the origin of the body reference frame
        void SetGeoPosition(const FrGeographicCoord& geoPos);

        /// Get the rotation object that represents the orientation of the body reference frame in the world frame
        /// \return rotation object that represents the orientation of the body reference frame in the world frame
        FrRotation GetRotation() const;

        /// Set the orientation of the body reference frame in world using a rotation object
        /// Note that it moves the entire body along with its nodes and other attached elements to the body (nodes...)
        /// which are updated
        /// \param rotation orientation of the body reference frame in world frame
        void SetRotation(const FrRotation& rotation);

        /// Get the quaternion object that represents the orientation of the body reference frame in the world frame
        /// \return quaternion object that represents the orientation of the body reference frame in the world frame
        FrUnitQuaternion GetQuaternion() const;

        /// Set the orientation of the body reference frame in world frame using a quaterion object
        /// Note that it moves the entire body along with its nodes and other attached elements to the body (nodes...)
        /// which are updated
        /// \param quaternion orientation of the body reference frame in world frame
        void SetRotation(const FrUnitQuaternion& quaternion);

        //TODO : ajouter ici toutes les methodes portant sur d'autres representations de la rotation



        /// Get the body reference frame expressed in the world frame
        /// \return body reference frame expressed in the world frame
        FrFrame GetFrame() const;

        /// Set the body reference frame expressed in the world frame
        /// Note that it moves the entire body along with its nodes and other attached elements to the body (nodes...)
        /// which are updated
        /// \param worldFrame body reference frame expressed in the world frame
        void SetFrame(const FrFrame& worldFrame);

        /// Get a frame object whose origin is located at a body point expressed in BODY frame and orientation is that
        /// of the body reference frame
        /// \param bodyPoint origin of the frame to return
        /// \param fc frame convention (NED/NWU)
        /// \return body frame
        FrFrame GetFrameAtPoint(const Position& bodyPoint, FRAME_CONVENTION fc);

        /// Get a frame object whose origin is locate at COG and orientation is that of the body reference frame
        /// \param fc frame convention (NED/NWU)
        /// \return body frame
        FrFrame GetFrameAtCOG(FRAME_CONVENTION fc);


        /// Get the position in world frame of a body fixed point whose position is given in body reference frame
        /// \param bodyPos position in the body reference frame of the point to be returned in world reference frame
        /// \param fc frame convention (NED/NWU)
        /// \return point position in world reference frame
        Position GetPointPositionInWorld(const Position& bodyPos, FRAME_CONVENTION fc) const;

        /// Get the position in body reference frame of a body fixed point whose position is given in world frame
        /// \param worldPos position in the world reference frame of the point to be returned in body reference frame
        /// \param fc frame convention (NED/NWU)
        /// \return point position in body reference frame
        Position GetPointPositionInBody(const Position& worldPos, FRAME_CONVENTION fc) const;

        /// Get the body COG position in world frame (coordinates are expressed in world frame)
        /// \param fc frame convention (NED/NWU)
        /// \return COG position in body reference frame
        Position GetCOGPositionInWorld(FRAME_CONVENTION fc) const;

        /// Get the COG position in the body reference frame
        /// \param fc frame convention (NED/NWU)
        Position GetCOG(FRAME_CONVENTION fc) const;

        // GeographicServices

        /// Get the geographic position in world frame of a body fixed point whose position is given in world reference frame
        /// \param worldPos position of a point given in the world reference frame
        /// \param fc frame convention (NED/NWU)
        /// \return geographic position of the point mentioned above
        FrGeographicCoord GetGeoPointPositionInWorld(const Position& worldPos, FRAME_CONVENTION fc) const;

        /// Get the geographic position in body reference frame of a body fixed point whose position is given in body frame
        /// \param bodyPos position of a point given in the body reference frame
        /// \param fc frame convention (NED/NWU)
        /// \return geographic position of the point mentioned above
        FrGeographicCoord GetGeoPointPositionInBody(const Position& bodyPos, FRAME_CONVENTION fc) const;

        /// Get the body COG geographic position
        /// \return geographic position of the COG
        FrGeographicCoord GetCOGGeoPosition() const;



        /// Set the position in WORLD frame of a body fixed point whose position is defined wrt body reference frame
        /// Note that it moves the entire body along with its nodes and other attached elements to the body (nodes...)
        /// which are updated
        /// \param bodyPoint position of a point given in the body reference frame
        /// \param worldPos destination position for the point, given in world reference frame
        /// \param fc frame convention (NED/NWU)
        void SetPositionOfBodyPoint(const Position &bodyPoint, const Position &worldPos, FRAME_CONVENTION fc);

        /// Translate the body along a translation vector whose coordinates are given in the world frame
        /// Note that it moves the entire body along with its nodes and other attached elements to the body (nodes...)
        /// which are updated
        /// \param worldTranslation translation to be applied to the body, expressed in world reference frame
        /// \param fc frame convention (NED/NWU)
        void TranslateInWorld(const Translation& worldTranslation, FRAME_CONVENTION fc);

        /// Translate the body along a translation vector whose coordinates are given in the world frame
        /// Note that it moves the entire body along with its nodes and other attached elements to the body (nodes...)
        /// which are updated
        /// \param worldTranslation translation to be applied to the body, expressed in world reference frame
        /// \param fc frame convention (NED/NWU)
        void TranslateInWorld(double x, double y, double z, FRAME_CONVENTION fc);

        /// Translate the body along a translation vector whose coordinates are given in the body frame
        /// Note that it moves the entire body along with its nodes and other attached elements to the body (nodes...)
        /// which are updated
        /// \param bodyTranslation translation to be applied to the body, expressed in body reference frame
        /// \param fc frame convention (NED/NWU)
        void TranslateInBody(const Translation& bodyTranslation, FRAME_CONVENTION fc);

        /// Translate the body along a translation vector whose coordinates are given in the body frame
        /// Note that it moves the entire body along with its nodes and other attached elements to the body (nodes...)
        /// which are updated
        /// \param bodyTranslation translation to be applied to the body, expressed in body reference frame
        /// \param fc frame convention (NED/NWU)
        void TranslateInBody(double x, double y, double z, FRAME_CONVENTION fc);

        /// Rotate the body with respect to its current orientation in world using a rotation object
        /// Note that it moves the entire body along with its nodes and other attached elements to the body (nodes...)
        /// which are updated
        /// \param relRotation relative rotation to be applied
        void Rotate(const FrRotation& relRotation);

        /// Rotate the body with respect to its current orientation in world using a quaternion object
        /// Note that it moves the entire body along with its nodes and other attached elements to the body (nodes...)
        /// which are updated
        /// \param relQuaternion relative rotation, defined with quaternion, to be applied
        void Rotate(const FrUnitQuaternion& relQuaternion);
        // FIXME : reflechir de nouveau a ce que sont les eux methodes precedentes... on tourne autour de quoi ?
        // Possible que ca n'ait pas de sens...

        /// Rotate the body around a point, given in world reference frame,  with respect to its current orientation
        /// in world using a rotation object.
        /// Note that it moves the entire body along with its nodes and other attached elements to the body (nodes...)
        /// which are updated
        /// \param rot rotation to be applied
        /// \param worldPos point position around which the body is to be rotated, given in world reference frame
        /// \param fc frame convention (NED/NWU)
        void RotateAroundPointInWorld(const FrRotation& rot, const Position& worldPos, FRAME_CONVENTION fc);

        /// Rotate the body around a point, given in body reference frame,  with respect to its current orientation
        /// in world using a rotation object.
        /// Note that it moves the entire body along with its nodes and other attached elements to the body (nodes...)
        /// which are updated
        /// \param rot rotation to be applied
        /// \param worldPos point position around which the body is to be rotated, given in body reference frame
        /// \param fc frame convention (NED/NWU)
        void RotateAroundPointInBody(const FrRotation& rot, const Position& bodyPos, FRAME_CONVENTION fc);

        /// Rotate the body around COG with respect to its current orientation in world using a rotation object.
        /// Note that it moves the entire body along with its nodes and other attached elements to the body (nodes...)
        /// which are updated
        /// \param rot rotation to be applied
        /// \param fc frame convention (NED/NWU)
        void RotateAroundCOG(const FrRotation& rot, FRAME_CONVENTION fc);

        /// Rotate the body around a point, given in world reference frame,  with respect to its current orientation
        /// in world using a quaternion object.
        /// Note that it moves the entire body along with its nodes and other attached elements to the body (nodes...)
        /// which are updated
        /// \param rot rotation to be applied, with a quaternion object
        /// \param worldPos point position around which the body is to be rotated, given in world reference frame
        /// \param fc frame convention (NED/NWU)
        void RotateAroundPointInWorld(const FrUnitQuaternion& rot, const Position& worldPos, FRAME_CONVENTION fc);

        /// Rotate the body around a point, given in body reference frame,  with respect to its current orientation
        /// in world using a quaternion object.
        /// Note that it moves the entire body along with its nodes and other attached elements to the body (nodes...)
        /// which are updated
        /// \param rot rotation to be applied, with a quaternion object
        /// \param worldPos point position around which the body is to be rotated, given in body reference frame
        /// \param fc frame convention (NED/NWU)
        void RotateAroundPointInBody(const FrUnitQuaternion& rot, const Position& bodyPos, FRAME_CONVENTION fc);

        /// Rotate the body around COG with respect to its current orientation in world using a quaternion object.
        /// Note that it moves the entire body along with its nodes and other attached elements to the body (nodes...)
        /// which are updated
        /// \param rot rotation to be applied, with a quaternion object
        /// \param fc frame convention (NED/NWU)
        void RotateAroundCOG(const FrUnitQuaternion& rot, FRAME_CONVENTION fc);




        // TODO : ajouter aussi les RotateX, RotateAxisAngle, RotateEuler...


        /// Set the velocity of the body reference frame with a vector expressed in WORLD frame
        /// \param worldVel body velocity in world reference frame
        /// \param fc frame convention (NED/NWU)
        void SetVelocityInWorldNoRotation(const Velocity& worldVel, FRAME_CONVENTION fc);

        /// Set the velocity of the body reference frame with a vector expressed in BODY frame
        /// \param bodyVel body velocity in body reference frame
        /// \param fc frame convention (NED/NWU)
        void SetVelocityInBodyNoRotation(const Velocity& bodyVel, FRAME_CONVENTION fc);

        /// Set the generalized velocity of the body reference frame with vectors expressed in WORLD frame
        /// \param worldVel body velocity in world reference frame
        /// \param worldAngVel body angular velocity in world reference frame
        /// \param fc frame convention (NED/NWU)
        void SetGeneralizedVelocityInWorld(const Velocity& worldVel, const AngularVelocity& worldAngVel, FRAME_CONVENTION fc);

        /// Set the generalized velocity of the body reference frame with vectors expressed in BODY frame
        /// \param bodyVel body velocity in body reference frame
        /// \param bodyAngVel body angular velocity in body reference frame
        /// \param fc frame convention (NED/NWU)
        void SetGeneralizedVelocityInBody(const Velocity& bodyVel, const AngularVelocity& bodyAngVel, FRAME_CONVENTION fc);


        /// Get the velocity of the body reference frame with a vector expressed in WORLD frame
        /// \param fc frame convention (NED/NWU)
        /// \return body velocity in world reference frame
        Velocity GetVelocityInWorld(FRAME_CONVENTION fc) const;

        /// Get the velocity of the body reference frame with a vector expressed in BODY frame
        /// \param fc frame convention (NED/NWU)
        /// \return body velocity in body reference frame
        Velocity GetVelocityInBody(FRAME_CONVENTION fc) const;


        /// Get the velocity of the body COG with a vector expressed in WORLD frame
        /// \param fc frame convention (NED/NWU)
        /// \return COG velocity in world reference frame
        Velocity GetCOGVelocityInWorld(FRAME_CONVENTION fc) const;

        /// Get the velocity of the body COG with a vector expressed in BODY frame
        /// \param fc frame convention (NED/NWU)
        /// \return COG velocity in body reference frame
        Velocity GetCOGVelocityInBody(FRAME_CONVENTION fc) const;

        /// Set the acceleration of the body COG with a vector expressed in WORLD frame
        /// \param worldAcc body acceleration in world reference frame
        /// \param fc frame convention (NED/NWU)
        void SetAccelerationInWorldNoRotation(const Acceleration &worldAcc, FRAME_CONVENTION fc);

        /// Set the acceleration of the body COG with a vector expressed in BODY frame
        /// \param bodyAcc body acceleration in body reference frame
        /// \param fc frame convention (NED/NWU)
        void SetAccelerationInBodyNoRotation(const Acceleration &bodyAcc, FRAME_CONVENTION fc);

        /// Get the acceleration of the body reference frame with a vector expressed in WORLD frame
        /// \param fc frame convention (NED/NWU)
        /// \return body acceleration in world reference frame
        Acceleration GetAccelerationInWorld(FRAME_CONVENTION fc) const;

        /// Get the acceleration of the body reference frame with a vector expressed in BODY frame
        /// \param fc frame convention (NED/NWU)
        /// \return body acceleration in body reference frame
        Acceleration GetAccelerationInBody(FRAME_CONVENTION fc) const;

        /// Get the acceleration of the body COG with a vector expressed in WORLD frame
        /// \param fc frame convention (NED/NWU)
        /// \return COG acceleration in world reference frame
        Acceleration GetCOGAccelerationInWorld(FRAME_CONVENTION fc) const;

        /// Get the acceleration of the body COG with a vector expressed in BODY frame
        /// \param fc frame convention (NED/NWU)
        /// \return COG acceleration in body reference frame
        Acceleration GetCOGAccelerationInBody(FRAME_CONVENTION fc) const;



        /// Set the body angular velocity from a vector expressed in WORLD frame
        /// \param worldAngVel body angular velocity in world reference frame
        /// \param fc frame convention (NED/NWU)
        void SetAngularVelocityInWorld(const AngularVelocity& worldAngVel, FRAME_CONVENTION fc);

        void SetCOGAngularVelocityInWorld(const AngularVelocity& worldAngVel, FRAME_CONVENTION fc);

        /// Set the body angular velocity from a vector expressed in BODY frame
        /// \param bodyAngVel body angular velocity in body reference frame
        /// \param fc frame convention (NED/NWU)
        void SetAngularVelocityInBody(const AngularVelocity& bodyAngVel, FRAME_CONVENTION fc);


        /// Get the body angular velocity from a vector expressed in WORLD frame
        /// \param fc frame convention (NED/NWU)
        /// \return body angular velocity in world reference frame
        AngularVelocity GetAngularVelocityInWorld(FRAME_CONVENTION fc) const;

        /// Get the body angular velocity from a vector expressed in BODY frame
        /// \param fc frame convention (NED/NWU)
        /// \return body angular velocity in body reference frame
        AngularVelocity GetAngularVelocityInBody(FRAME_CONVENTION fc) const;



        /// Set the body angular acceleration from a vector expressed in WORLD frame
        /// \param worldAngAcc body angular acceleration in world reference frame
        /// \param fc frame convention (NED/NWU)
        void SetAngularAccelerationInWorld(const AngularAcceleration& worldAngAcc, FRAME_CONVENTION fc);

        /// Set the body angular acceleration from a vector expressed in BODY frame
        /// \param bodyAngAcc body angular acceleration in body reference frame
        /// \param fc frame convention (NED/NWU)
        void SetAngularAccelerationInBody(const AngularAcceleration& bodyAngAcc, FRAME_CONVENTION fc);


        /// Get the body angular acceleration from a vector expressed in WORLD frame
        /// \param fc frame convention (NED/NWU)
        /// \return body angular acceleration in world reference frame
        AngularAcceleration GetAngularAccelerationInWorld(FRAME_CONVENTION fc) const;

        /// Get the body angular acceleration from a vector expressed in BODY frame
        /// \param fc frame convention (NED/NWU)
        /// \return body angular acceleration in body reference frame
        AngularAcceleration GetAngularAccelerationInBody(FRAME_CONVENTION fc) const;



        /// Get the velocity expressed in world frame of a body fixed point whose coordinates are given in world frame
        /// \param worldPoint point position in world reference frame, at which the velocity is requested
        /// \param fc frame convention (NED/NWU)
        /// \return body velocity expressed in world reference frame
        Velocity GetVelocityInWorldAtPointInWorld(const Position& worldPoint, FRAME_CONVENTION fc) const;

        /// Get the velocity expressed in world frame of a body fixed point whose coordinates are given in body frame
        /// \param bodyPoint point position in body reference frame, at which the velocity is requested
        /// \param fc frame convention (NED/NWU)
        /// \return body velocity expressed in world reference frame
        Velocity GetVelocityInWorldAtPointInBody(const Position& bodyPoint, FRAME_CONVENTION fc) const;

        /// Get the velocity expressed in body frame of a body fixed point whose coordinates are given in world frame
        /// \param worldPoint point position in world reference frame, at which the velocity is requested
        /// \param fc frame convention (NED/NWU)
        /// \return body velocity expressed in body reference frame
        Velocity GetVelocityInBodyAtPointInWorld(const Position& worldPoint, FRAME_CONVENTION fc) const;

        /// Get the velocity expressed in body frame of a body fixed point whose coordinates are given in body frame
        /// \param bodyPoint point position in body reference frame, at which the velocity is requested
        /// \param fc frame convention (NED/NWU)
        /// \return body velocity expressed in body reference frame
        Velocity GetVelocityInBodyAtPointInBody(const Position& bodyPoint, FRAME_CONVENTION fc) const;



        /// Get the acceleration expressed in world frame of a body fixed point whose coordinates are given in world frame
        /// \param worldPoint point position in world reference frame, at which the acceleration is requested
        /// \param fc frame convention (NED/NWU)
        /// \return body acceleration expressed in world reference frame
        Acceleration GetAccelerationInWorldAtPointInWorld(const Position& worldPoint, FRAME_CONVENTION fc) const;

        /// Get the acceleration expressed in world frame of a body fixed point whose coordinates are given in body frame
        /// \param bodyPoint point position in body reference frame, at which the acceleration is requested
        /// \param fc frame convention (NED/NWU)
        /// \return body acceleration expressed in world reference frame
        Acceleration GetAccelerationInWorldAtPointInBody(const Position& bodyPoint, FRAME_CONVENTION fc) const;

        /// Get the acceleration expressed in body frame of a body fixed point whose coordinates are given in world frame
        /// \param worldPoint point position in world reference frame, at which the acceleration is requested
        /// \param fc frame convention (NED/NWU)
        /// \return body acceleration expressed in body reference frame
        Acceleration GetAccelerationInBodyAtPointInWorld(const Position& worldPoint, FRAME_CONVENTION fc) const;

        /// Get the acceleration expressed in body frame of a body fixed point whose coordinates are given in body frame
        /// \param bodyPoint point position in body reference frame, at which the acceleration is requested
        /// \param fc frame convention (NED/NWU)
        /// \return body acceleration expressed in body reference frame
        Acceleration GetAccelerationInBodyAtPointInBody(const Position& bodyPoint, FRAME_CONVENTION fc) const;



        /// Set the velocity expressed in WORLD frame of a body fixed point whose coordinates are given in WORLD frame
        /// along with the angular velocity expressed in WORLD frame so that the velocity state is totally defined
        /// \param worldPoint point position in world reference frame, at which the generalized velocity is set
        /// \param worldVel body linear velocity in world reference frame
        /// \param worldAngVel body angular velocity in world reference frame
        /// \param fc frame convention (NED/NWU)
        void SetGeneralizedVelocityInWorldAtPointInWorld(const Position& worldPoint,
                const Velocity& worldVel, const AngularVelocity& worldAngVel, FRAME_CONVENTION fc);

        /// Set the velocity expressed in WORLD frame of a body fixed point whose coordinates are given in BODY frame
        /// along with the angular velocity expressed in WORLD frame so that the velocity state is totally defined
        /// \param bodyPoint point position in body reference frame, at which the generalized velocity is set
        /// \param worldVel body linear velocity in world reference frame
        /// \param worldAngVel body angular velocity in world reference frame
        /// \param fc frame convention (NED/NWU)
        void SetGeneralizedVelocityInWorldAtPointInBody(const Position& bodyPoint,
                const Velocity& worldVel, const AngularVelocity& worldAngVel, FRAME_CONVENTION fc);

        /// Set the velocity expressed in BODY frame of a body fixed point whose coordinates are given in WORLD frame
        /// along with the angular velocity expressed in BODY frame so that the velocity state is totally defined
        /// \param worldPoint point position in world reference frame, at which the generalized velocity is set
        /// \param bodyVel body linear velocity in body reference frame
        /// \param bodyAngVel body angular velocity in body reference frame
        /// \param fc frame convention (NED/NWU)
        void SetGeneralizedVelocityInBodyAtPointInWorld(const Position& worldPoint,
                const Velocity& bodyVel, const AngularVelocity& bodyAngVel, FRAME_CONVENTION fc);

        /// Set the velocity expressed in BODY frame of a body fixed point whose coordinates are given in BODY frame
        /// along with the angular velocity expressed in BODY frame so that the velocity state is totally defined
        /// \param bodyPoint point position in body reference frame, at which the generalized velocity is set
        /// \param bodyVel body linear velocity in body reference frame
        /// \param bodyAngVel body angular velocity in body reference frame
        /// \param fc frame convention (NED/NWU)
        void SetGeneralizedVelocityInBodyAtPointInBody(const Position& bodyPoint,
                const Velocity& bodyVel, const AngularVelocity& bodyAngVel, FRAME_CONVENTION fc);

        /// Set the COG acceleration along with the angular velocity expressed in BODY frame,
        /// so that the acceleration state is totally defined
        /// \param bodyAcc body linear acceleration in body reference frame
        /// \param bodyAngAcc body angular acceleration in body reference frame
        /// \param fc frame convention (NED/NWU)
        void SetGeneralizedAccelerationInBodyAtCOG(const Acceleration& bodyAcc, const AngularAcceleration& bodyAngAcc, FRAME_CONVENTION fc);

        /// Set the COG acceleration along with the angular velocity expressed in WORLD frame,
        /// so that the acceleration state is totally defined
        /// \param worldAcc body linear acceleration in world reference frame
        /// \param worldAngAcc body angular acceleration in world reference frame
        /// \param fc frame convention (NED/NWU)
        void SetGeneralizedAccelerationInWorldAtCOG(const Acceleration& worldAcc, const AngularAcceleration& worldAngAcc, FRAME_CONVENTION fc);

        // =============================================================================================================
        // PROJECTIONS
        // =============================================================================================================

        // Projection of 3D vectors defined in FrVector.h

        /// Project a vector given in body reference frame, to the world reference frame
        /// \tparam Vector type of the vector defined in FrVector.h
        /// \param bodyVector vector given in body reference frame
        /// \param fc frame convention (NED/NWU)
        /// \return vector in world reference frame
        template <class Vector>
        Vector ProjectVectorInWorld(const Vector& bodyVector, FRAME_CONVENTION fc) const {
            return GetQuaternion().Rotate<Vector>(bodyVector, fc);
        }

        /// Project in place a vector given in body reference frame, to the world reference frame
        /// \tparam Vector type of the vector defined in FrVector.h
        /// \param bodyVector vector given in body reference frame
        /// \param fc frame convention (NED/NWU)
        /// \return vector in world reference frame
        template <class Vector>
        Vector& ProjectVectorInWorld(Vector& bodyVector, FRAME_CONVENTION fc) const {
            bodyVector = GetQuaternion().Rotate<Vector>(bodyVector, fc);
            return bodyVector;
        }

        /// Project a vector given in world reference frame, to the body reference frame
        /// \tparam Vector type of the vector defined in FrVector.h
        /// \param worldVector vector given in world reference frame
        /// \param fc frame convention (NED/NWU)
        /// \return vector in body reference frame
        template <class Vector>
        Vector ProjectVectorInBody(const Vector& worldVector, FRAME_CONVENTION fc) const {
            return GetQuaternion().GetInverse().Rotate<Vector>(worldVector, fc);
        }

        /// Project in place a vector given in world reference frame, to the body reference frame
        /// \tparam Vector type of the vector defined in FrVector.h
        /// \param worldVector vector given in world reference frame
        /// \param fc frame convention (NED/NWU)
        /// \return vector in body reference frame
        template <class Vector>
        Vector& ProjectVectorInBody(Vector& worldVector, FRAME_CONVENTION fc) const {
            worldVector = GetQuaternion().GetInverse().Rotate<Vector>(worldVector, fc);
            return worldVector;
        }


        // Projection of generalized vectors defined in FrVector.h


        /// Project a generalized vector given in body reference frame, to the world reference frame
        /// \tparam Vector type of the generalized vector defined in FrVector.h
        /// \param bodyVector vector given in body reference frame
        /// \param fc frame convention (NED/NWU)
        /// \return vector in world reference frame
        template <class Vector>
        Vector ProjectGenerallizedVectorInWorld(const Vector& bodyVector, FRAME_CONVENTION fc) const {
            return GetQuaternion().Rotate<Vector>(bodyVector, fc);
        }

        /// Project in place a generalized vector given in body reference frame, to the world reference frame
        /// \tparam Vector type of the generalized vector defined in FrVector.h
        /// \param bodyVector vector given in body reference frame
        /// \param fc frame convention (NED/NWU)
        /// \return vector in world reference frame
        template <class Vector>
        Vector& ProjectGenerallizedVectorInWorld(Vector& bodyVector, FRAME_CONVENTION fc) const {
            bodyVector = GetQuaternion().Rotate<Vector>(bodyVector, fc);
            return bodyVector;
        }

        /// Project a generalized vector given in world reference frame, to the body reference frame
        /// \tparam Vector type of the generalized vector defined in FrVector.h
        /// \param bodyVector vector given in world reference frame
        /// \param fc frame convention (NED/NWU)
        /// \return vector in body reference frame
        template <class Vector>
        Vector ProjectGenerallizedVectorInBody(const Vector& worldVector, FRAME_CONVENTION fc) const {
            return GetQuaternion().GetInverse().Rotate<Vector>(worldVector, fc);
        }

        /// Project in place a generalized vector given in world reference frame, to the body reference frame
        /// \tparam Vector type of the generalized vector defined in FrVector.h
        /// \param bodyVector vector given in world reference frame
        /// \param fc frame convention (NED/NWU)
        /// \return vector in body reference frame
        template <class Vector>
        Vector& ProjectGeneralizedVectorInBody(Vector &worldVector, FRAME_CONVENTION fc) const {
            worldVector = GetQuaternion().GetInverse().Rotate<Vector>(worldVector, fc);
            return worldVector;
        }

        // =============================================================================================================
        // CONSTRAINTS ON DOF
        // =============================================================================================================

        FrBodyDOFMask* GetDOFMask();

    protected:

//        enum FRAME {
//            WORLD,
//            BODY
//        };

        /// Set the COG position in the body reference frame
        /// \param bodyPos COG position in the body reference frame
        /// \param fc frame convention (NED/NWU)
        void SetCOG(const Position& bodyPos, FRAME_CONVENTION fc);

        /// Convert a cartesian position to a geographic position, using the geographic service of FrEnvironment
        /// \param cartPos cartesian position to convert
        /// \param geoCoord geographic position
        /// \param fc frame position (NED/NWU) of the cartesian position
        void CartToGeo(const Position &cartPos, FrGeographicCoord &geoCoord, FRAME_CONVENTION fc) const;

        /// Convert a cartesian position to a geographic position, using the geographic service of FrEnvironment
        /// \param cartPos cartesian position to convert
        /// \param geoCoord geographic position
        /// \param fc frame position (NED/NWU) of the cartesian position
        void CartToGeo(const Position &cartPos, FrGeographicCoord &geoCoord, FRAME_CONVENTION fc);

        /// Convert a cartesian position to a geographic position, using the geographic service of FrEnvironment
        /// \param cartPos cartesian position to convert
        /// \param fc frame position (NED/NWU) of the cartesian position
        /// \return geographic position
        FrGeographicCoord CartToGeo(const Position &cartPos, FRAME_CONVENTION fc) const;

        /// Convert a cartesian position to a geographic position, using the geographic service of FrEnvironment
        /// \param cartPos cartesian position to convert
        /// \param fc frame position (NED/NWU) of the cartesian position
        /// \return geographic position
        FrGeographicCoord CartToGeo(const Position &cartPos, FRAME_CONVENTION fc);

        /// Convert a geographic position to a cartesian position, using the geographic service of FrEnvironment
        /// \param geoCoord geographic position to convert
        /// \param cartPos cartesian position
        /// \param fc frame position (NED/NWU) of the cartesian position
        void GeoToCart(const FrGeographicCoord& geoCoord, Position& cartPos, FRAME_CONVENTION fc);

        /// Convert a geographic position to a cartesian position, using the geographic service of FrEnvironment
        /// \param geoCoord geographic position to convert
        /// \param cartPos cartesian position
        /// \param fc frame position (NED/NWU) of the cartesian position
        void GeoToCart(const FrGeographicCoord& geoCoord, Position& cartPos, FRAME_CONVENTION fc) const;

        /// Convert a geographic position to a cartesian position, using the geographic service of FrEnvironment
        /// \param geoCoord geographic position to convert
        /// \param fc frame position (NED/NWU) of the cartesian position
        /// \return cartPos cartesian position
        Position GeoToCart(const FrGeographicCoord& geoCoord, FRAME_CONVENTION fc) const;

        /// Convert a geographic position to a cartesian position, using the geographic service of FrEnvironment
        /// \param geoCoord geographic position to convert
        /// \param fc frame position (NED/NWU) of the cartesian position
        /// \return cartPos cartesian position
        Position GeoToCart(const FrGeographicCoord& geoCoord, FRAME_CONVENTION fc);

        /// Get the shared pointer to the chronoBody attribute
        /// \return shared pointer to the chronoBody attribute
        std::shared_ptr<internal::FrBodyBase> GetChronoBody();

        /// Get the chronoBody attribute pointer
        /// \return Pointer to the chronoBody attribute
        internal::FrBodyBase* GetChronoItem_ptr() const override;

        void InitializeLockedDOF();

        // Friends of FrBody : they can have access to chrono internals
        friend void makeItBox(std::shared_ptr<FrBody>, double, double, double, double);
        friend void makeItCylinder(std::shared_ptr<FrBody>, double, double, double);
        friend void makeItSphere(std::shared_ptr<FrBody>, double, double);


        friend FrNode::FrNode(FrBody*);
        friend class FrLinkBase;


    public:

        void SetupInitial();

        /// Body initialization method
        void Initialize() override;

        /// Method called at the send of a time step. Logging may be used here
        void StepFinalize() override;

        //FIXME : FrBody doit-il dériver de FrPhysicsItem ?
        /// Body update method
        virtual void Update();

        // Linear iterators on external forces
        using ForceIter = ForceContainer::iterator;
        using ConstForceIter = ForceContainer::const_iterator;

        ForceIter       force_begin();
        ConstForceIter  force_begin() const;

        ForceIter       force_end();
        ConstForceIter  force_end() const;

        // Linear iterators on nodes
        using NodeIter = NodeContainer::iterator;
        using ConstNodeIter = NodeContainer::const_iterator;

        NodeIter       node_begin();
        ConstNodeIter  node_begin() const;

        NodeIter       node_end();
        ConstNodeIter  node_end() const;



        // friend declarations
        // This one is made for the FrOffshoreSystem to be able to add the embedded chrono object into the embedded
        // chrono system (ChSystem)
        friend void FrOffshoreSystem::AddBody(std::shared_ptr<frydom::FrBody>);
        friend void FrOffshoreSystem::RemoveBody(std::shared_ptr<frydom::FrBody>);

        friend int internal::FrAddedMassBase::GetBodyOffset(FrBody* body) const;
        friend int internal::FrVariablesAddedMassBase::GetBodyOffset(FrBody* body) const ;
        friend void internal::FrVariablesAddedMassBase::SetVariables(FrBody *body, chrono::ChMatrix<double> &result,
                                                                     int offset) const;
        friend void internal::FrAddedMassBase::SetVariables(FrBody *body, chrono::ChMatrix<double> &qb, int offset) const;
        friend chrono::ChMatrix<double> internal::FrVariablesAddedMassBase::GetVariablesFb(FrBody *body) const;
        friend chrono::ChMatrix<double> internal::FrVariablesAddedMassBase::GetVariablesQb(FrBody *body) const;

        // ##CC test hydrodynamic interaction
        friend int internal::FrAddedMassBase2::GetBodyOffset(FrBody* body) const;
        friend internal::FrAddedMassBase2::FrAddedMassBase2(FrRadiationModel *radiationModel);
        // ##CC

    };


}  // end namespace frydom

#endif //FRYDOM_FRBODY_H
