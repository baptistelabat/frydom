//
// Created by frongere on 21/06/17.
//

#ifndef FRYDOM_FRBODY_H
#define FRYDOM_FRBODY_H

#include "chrono/physics/ChBodyAuxRef.h"

#include "hermes/hermes.h"

#include "FrObject.h"
#include "FrOffshoreSystem.h"
#include "FrVector.h"
#include "FrGeographic.h"
#include "FrForce.h"
#include "FrEulerAngles.h" // TODO : devrait disparaitre

#include "frydom/mesh/FrTriangleMeshConnected.h"

#include "FrInertia.h"
#include "FrForce.h"
#include "FrColors.h"
#include "FrNode.h"

#include "frydom/environment/FrFluidType.h"


namespace frydom {

    class FrNode;

    class FrBody : public chrono::ChBodyAuxRef,
                   public std::enable_shared_from_this<FrBody>,
                   public FrObject
    {  // TODO: voir a supprimer cet heritage...

    protected:
        std::vector<std::shared_ptr<FrForce>> external_force_list;
        std::shared_ptr<FrTriangleMeshConnected> m_visu_mesh;
        hermes::Message m_bodyMsg;
        bool is_log = true;

        // ##CC : fix pour logger les angles de rotation via hermes
        // TODO : ameliorer hermes pour permettre le log de variable non définis en attributs
        chrono::ChVector<double> m_angles_rotation;
        chrono::ChVector<double> m_angles_speed; ///< local coordinate system
        // ##CC

    public:

        FrBody() {
            SetLogNameAndDescription("Body_msg", "Message for a body");
        }

        std::shared_ptr<FrBody> GetSharedPtr() {
            return shared_from_this();
        }

        void SetAbsPos(const chrono::ChVector<double>& REFabsPos) {
            auto frame = chrono::ChFrame<double>();
            frame.SetPos(REFabsPos);
            SetFrame_REF_to_abs(frame);
        }

        /// Get the body absolute position (this of its reference point)
        chrono::ChVector<> GetPosition(FRAME_CONVENTION frame = NWU) const{
            switch (frame) {
                case NWU:
                    return GetPos();
                case NED:
//                    return internal::swap_NED_NWU(GetPos());
                    auto pos = GetPos();
                    pos[1] = -pos[1];
                    pos[2] = -pos[2];
                    return pos;
//                    return internal::swap_NED_NWU(GetPos());
            }
        }

        /// Get the body orientation
        chrono::ChVector<> GetOrientation(FRAME_CONVENTION frame= NWU) const{
            // TODO
        }

        /// Get the body velocity
        chrono::ChVector<> GetVelocity(FRAME_CONVENTION frame= NWU) const{
            switch (frame) {
                case NWU:
                    return GetPos_dt();
                case NED:
                    auto vel = GetPos_dt();
                    vel[1] = -vel[1];
                    vel[2] = -vel[2];
                    return vel;
//                    return internal::swap_NED_NWU(GetPos_dt());
            }
        }

        /// Get the body angular velocity
        chrono::ChVector<> GetAngularVelocity(FRAME_CONVENTION frame= NWU) {
            //TODO
        }

        void AddNode(std::shared_ptr<FrNode> node);
        // TODO: implementer aussi les removeMarker etc...

        std::shared_ptr<FrNode> CreateNode();
        std::shared_ptr<FrNode> CreateNode(const chrono::ChVector<double> relpos);

//        std::shared_ptr<FrBody> CreateNodeDynamic();
//        std::shared_ptr<FrBody> CreateNodeDynamic(const chrono::ChVector<double> relpos);

        /// Set the position of the COG with respect to the body's reference frame (in local coordinates)
        void SetCOG(const chrono::ChVector<double>& COGRelPos) {
            auto COGFrame = chrono::ChFrame<double>();

            COGFrame.SetPos(COGRelPos);
            SetFrame_COG_to_REF(COGFrame);
        }

        const chrono::ChVector<double> GetAbsCOG() const {
            auto COGFrame = GetFrame_COG_to_abs();
            return COGFrame.GetPos();
        }

        const chrono::ChVector<double> GetRelCOG() const {
            auto COGFrame = GetFrame_COG_to_REF();
            return COGFrame.GetPos();
        }

        /// Set the mesh used for visualization from a mesh shared instance
        void SetVisuMesh(std::shared_ptr<FrTriangleMeshConnected> mesh);

        /// Set the mesh used for visualization from a wavefront obj file
        void SetVisuMesh(std::string obj_filename);

        virtual void Initialize() override {
            // TODO: mettre l'initialisation de message dans une methode privee qu'on appelle ici...

            // Initializing forces
            for (int iforce=0; iforce<forcelist.size(); iforce++) {
                auto force = dynamic_cast<FrForce*>(forcelist[iforce].get());
                if (force) {
                    force->Initialize();
                }
            }

            if (is_log) {
                InitializeLog();
            }
        }

        /// Define the name and the description of the log message of the body
        virtual void SetLogNameAndDescription(std::string name="ShipLog",
                                              std::string description="Log of the body position and force at COG") {
            m_bodyMsg.SetNameAndDescription(name, description);
            is_log = true;
        }

        /// Deactivate the generation of log from the body
        virtual void DeactivateLog() { is_log = false; }

        /// Definition of the field to log
        virtual void SetLogDefault() {

            // Initializing message
            if (m_bodyMsg.GetName() == "") {
                m_bodyMsg.SetNameAndDescription(
                        fmt::format("Body_{}", GetUUID()),
                        "Message of a body");
            }

            m_bodyMsg.AddCSVSerializer();
            //m_bodyMsg.AddPrintSerializer();

            // Adding fields
            m_bodyMsg.AddField<double>("time", "s", "Current time of the simulation", &ChTime);

            m_bodyMsg.AddField<double>("x", "m", "x position of the body reference frame origin", &coord.pos.x());
            m_bodyMsg.AddField<double>("y", "m", "y position of the body reference frame origin", &coord.pos.y());
            m_bodyMsg.AddField<double>("z", "m", "z position of the body reference frame origin", &coord.pos.z());

            m_bodyMsg.AddField<double>("vx", "m/s", "velocity of the body along x-axis", &coord_dt.pos.x());
            m_bodyMsg.AddField<double>("vy", "m/s", "velocity of the body along x-axis", &coord_dt.pos.y());
            m_bodyMsg.AddField<double>("vz", "m/s", "velocity of the body along x-axis", &coord_dt.pos.z());

            m_bodyMsg.AddField<double>("rx", "rad", "euler angle along the x-direction (body ref frame)", &m_angles_rotation.x());
            m_bodyMsg.AddField<double>("ry", "rad", "euler angle along the y-direction (body ref frame)", &m_angles_rotation.y());
            m_bodyMsg.AddField<double>("rz", "rad", "euler angle along the z-direction (body ref frame)", &m_angles_rotation.z());

            m_bodyMsg.AddField<double>("vrx", "rad/s", "angular speed around x-axis (expressed in local coords)", &m_angles_speed.x());
            m_bodyMsg.AddField<double>("vry", "rad/s", "angular speed around y-axis (expressed in local coords)", &m_angles_speed.y());
            m_bodyMsg.AddField<double>("vrz", "rad/s", "angular speed around z-axis (expressed in local coords)", &m_angles_speed.z());

            m_bodyMsg.AddField<double>("Xbody_FX", "N", "force acting on the body at COG (in absolute reference frame)", &Xforce.x());
            m_bodyMsg.AddField<double>("Xbody_FY", "N", "force acting on the body at COG (in absolute reference frame)", &Xforce.y());
            m_bodyMsg.AddField<double>("Xbody_FZ", "N", "force acting on the body at COG (in absolute reference frame)", &Xforce.z());

            m_bodyMsg.AddField<double>("Xbody_MX", "N.m", "moment acting on the body at COG (in body reference frame)", &Xtorque.x());
            m_bodyMsg.AddField<double>("Xbody_MY", "N.m", "moment acting on the body at COG (in body reference frame)", &Xtorque.y());
            m_bodyMsg.AddField<double>("Xbody_MZ", "N.m", "moment acting on the body at COG (in body reference frame)", &Xtorque.z());


            for (auto force: forcelist) {
                auto dforce = dynamic_cast<FrForce *>(force.get());
                m_bodyMsg.AddField<hermes::Message>("force", "-", "external force on a body", dforce->GetLog());
            }
        }

        virtual void AddMessageLog(std::shared_ptr<FrForce> dforce) {
            m_bodyMsg.AddField<hermes::Message>("force", "-", "external force on a body", dforce->GetLog());
            //dforce->DeactivateLog();
        }

        virtual void AddMessageLog(FrForce* dforce) {
            m_bodyMsg.AddField<hermes::Message>("force", "-", "external force on a body", dforce->GetLog());
            //dforce->DeactivateLog();
        }

        /// Initialize log file and serialize body field
        virtual void InitializeLog() {
            m_bodyMsg.Initialize();
            m_bodyMsg.Send();
        }

        /// Return the object message of the body
        hermes::Message& Log() { return m_bodyMsg; }

        virtual void UpdateLog() {
            m_bodyMsg.Serialize();
            m_bodyMsg.Send();
        }

        virtual void StepFinalize() override {

            m_angles_rotation = internal::quat_to_euler(GetRot());    // FIXME : ceci est un fixe pour permettre de logger l'angle de rotation
            m_angles_speed = GetWvel_loc();

            for (auto& iforce: forcelist) {
                auto force = dynamic_cast<FrForce*>(iforce.get());
                if (force) {
                    force->StepFinalize();
                }
            }

            m_bodyMsg.Serialize();
            m_bodyMsg.Send();
        }

    };

















    /// REFACTORING ------>>>>>>>>>>>>>>

    class FrQuaternion_;
    class FrBody_;


    namespace internal {

        /// Base class inheriting from chrono ChBodyAuxRef
        /// This class must be used by external FRyDoM users. It is used in composition rule along with the FrBody_ FRyDoM class
        struct _FrBodyBase : public chrono::ChBodyAuxRef {  // TODO : encapsuler dans le namespace internal

            FrBody_ *m_frydomBody;

            explicit _FrBodyBase(FrBody_ *body);

            void SetupInitial() override;

            void Update(bool update_assets) override;

        };

    }  // end namespace internal


    // Forward declarations
//    class FrForce_;
    class FrFrame_;
    class FrRotation_;
//    class FrNode_;
    class FrOffshoreSystem_;

    /// Main class for a FRyDoM rigid body
    class FrBody_ : public FrObject {

    protected:

        std::shared_ptr<internal::_FrBodyBase> m_chronoBody;  ///< Embedded Chrono body Object

        FrOffshoreSystem_* m_system;                ///< Pointer to the FrOffshoreSystem where the body has been registered

        using ForceContainer = std::vector<std::shared_ptr<FrForce_>>;
        ForceContainer m_externalForces;            ///< Container of the external forces acting on body


        using CONTACT_TYPE = FrOffshoreSystem_::SYSTEM_TYPE;
        CONTACT_TYPE m_contactType = CONTACT_TYPE::SMOOTH_CONTACT; ///< The contact method that has to be consistent with that of the FrOffshoreSystem


    public:

        /// Default constructor
        FrBody_();

        /// Get the FrOffshoreSystem where the body has been registered
        FrOffshoreSystem_* GetSystem();

        /// Set the body name
        void SetName(const char name[]);

        /// Make the body fixed
        void SetBodyFixed(bool state);




        ////// NOUVEAU


        // =============================================================================================================
        // PRINCIPAL INERTIAL PARAMETERS
        // =============================================================================================================

        /// Get the body mass in kg
        double GetMass() const;

        /// Set the body mass in kg
        void SetMass(double mass);

        /// Set the COG position in the body reference frame
        void SetCOG(const Position& bodyPos, FRAME_CONVENTION fc);

        /// Get the COG position in the body reference frame
        Position GetCOG(FRAME_CONVENTION fc) const;

        /// Get the inertia parameters as a FrInertiaTensor_ object
        // TODO : gerer la frame convention !
        FrInertiaTensor_ GetInertiaParams() const; // TODO : voir pour une methode renvoyant une reference non const

        /// Set the inertia parameters as a FrInertiaTensor_ object
        void SetInertiaParams(const FrInertiaTensor_& inertia);

        /// Set the principal inertia parameters given as coefficients expressed in coeffsFrame that can be different
        /// from the local COG position cogPosition (expressed in body reference coordinate system)
        void SetInertiaParams(double mass,
                              double Ixx, double Iyy, double Izz,
                              double Ixy, double Ixz, double Iyz,
                              const FrFrame_& coeffsFrame,
                              const Position& cogPosition,
                              FRAME_CONVENTION fc);

        /// Set the inertia parameters given in cogFrame relative to
        void SetInertiaParams(double mass,
                              double Ixx, double Iyy, double Izz,
                              double Ixy, double Ixz, double Iyz,
                              const FrFrame_& cogFrame,
                              FRAME_CONVENTION fc);

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
        void SetContactMethod(CONTACT_TYPE contactType);

        /// Get the contact method of this body
        CONTACT_TYPE GetContactType() const;

        /// Set the collide mode. If true, a collision shape must be set and the body will participate in physical
        /// collision with other physical collision enabled items
        void SetCollide(bool isColliding);

        // TODO : ajouter de quoi definir des shapes de collision !!!

        // =============================================================================================================
        // VISUAL ASSETS
        // =============================================================================================================
//        void AssetActive() // TODO

        /// Add a box shape to the body with its dimensions defined in absolute coordinates. Dimensions in meters
        void AddBoxShape(double xSize, double ySize, double zSize);  // TODO : definir plutot les dimensions dans le repere local du corps...

        /// Add a cylinder shape to the body with its dimensions defined in ???? Dimensions in meters
        void AddCylinderShape(double radius, double height);  // FIXME : travailler la possibilite de definir un axe... dans le repere local du corps

        /// Add a sphere shape to the body. Dimensions in meters.
        void AddSphereShape(double radius);  // TODO : permettre de definir un centre en coords locales du corps

        // =============================================================================================================
        // SPEED LIMITATIONS TO STABILIZE SIMULATIONS
        // =============================================================================================================

        /// Enable the maximum speed limits in both linear and angular speed (beyond this limit it will be clamped).
        /// This is useful in virtual reality and real-time simulations, because
        /// it reduces the risk of bad collision detection.
        /// The realism is limited, but the simulation is more stable.
        void ActivateSpeedLimits(bool activate);

        /// Set the maximum linear speed (beyond this limit it will be clamped). In m/s
        /// This is useful in virtual reality and real-time simulations, because
        /// it reduces the risk of bad collision detection.
        /// This speed limit is active only if you set  SetLimitSpeed(true);
        void SetMaxSpeed(double maxSpeed);

        /// Set the maximum angular speed (beyond this limit it will be clamped). In rad/s
        /// This is useful in virtual reality and real-time simulations, because
        /// it reduces the risk of bad collision detection.
        /// This speed limit is active only if you set  SetLimitSpeed(true);
        void SetMaxRotationSpeed(double wMax);

        /// [DEBUGGING MODE] Remove the gravity by adding a anti-gravity. This is a debugging method and should not be
        /// used in projects
        void RemoveGravity(bool val);


        // =============================================================================================================
        // FORCES
        // =============================================================================================================

        /// Add an external force to the body
        void AddExternalForce(std::shared_ptr<FrForce_> force);

        /// Remove an external force to the body
        void RemoveExternalForce(std::shared_ptr<FrForce_> force);

        /// Remove all forces from the body
        void RemoveAllForces();


        // =============================================================================================================
        // NODES
        // =============================================================================================================

        /// Get a new node attached to the body given a frame defined with respect to the body reference frame
        std::shared_ptr<FrNode_> NewNode(const FrFrame_& localFrame);

        /// Get a new node attached to the body given a position of the node expressed into the body reference frame
        std::shared_ptr<FrNode_> NewNode(const Position& localPosition);

        /// Get a new node attached to the body given a position of the node expressed into the body reference frame
        std::shared_ptr<FrNode_> NewNode(double x, double y, double z);


        // TODO : permettre de definir un frame a l'aide des parametres de Denavit-Hartenberg modifies ?? --> dans FrFrame_ !


        // =============================================================================================================
        // Asset
        // =============================================================================================================

        /// Add a mesh as an asset for visualization given a WaveFront .obj file name
        void AddMeshAsset(std::string obj_filename);

        /// Add a mesh as an asset for visualization given a FrTriangleMeshConnected mesh object
        void AddMeshAsset(std::shared_ptr<FrTriangleMeshConnected> mesh);

        /// Set the mesh color in visualization given a color id
        void SetColor(NAMED_COLOR colorName);

        /// Set the mesh color in visualization given a FrColor object
        void SetColor(const FrColor& color);

        // =============================================================================================================
        // POSITIONS
        // =============================================================================================================

        // Position of the body reference frame in world -->

        /// Get the position in world frame of the origin of the body reference frame
        Position GetPosition(FRAME_CONVENTION fc) const;

        /// Set the position in world frame of the origin of the body reference frame
        /// Note that it moves the entire body along with its nodes and other attached elements to the body (nodes...)
        /// which are updated
        void SetPosition(const Position& worldPos, FRAME_CONVENTION fc);


        /// Get the rotation object that represents the orientation of the body reference frame in the world
        FrRotation_ GetRotation() const;

        /// Set the orientation of the body reference frame in world using a rotation object
        /// Note that it moves the entire body along with its nodes and other attached elements to the body (nodes...)
        /// which are updated
        void SetRotation(const FrRotation_& rotation);

        /// Get the quaternion object that represents the orientation of the body reference frame in the world
        FrQuaternion_ GetQuaternion() const;

        /// Set the orientation of the body reference frame in world using a quaterion object
        /// Note that it moves the entire body along with its nodes and other attached elements to the body (nodes...)
        /// which are updated
        void SetRotation(const FrQuaternion_& quaternion);

        //TODO : ajouter ici toutes les methodes portant sur d'autres representations de la rotation



        /// Get the body reference frame expressed in the world
        FrFrame_ GetFrame() const;

        /// Set the body reference frame expressed in the world frame
        /// Note that it moves the entire body along with its nodes and other attached elements to the body (nodes...)
        /// which are updated
        void SetFrame(const FrFrame_& worldFrame);

        /// Get a frame object whose origin is located at a body point expressed in BODY frame and orientation is that
        /// of the body reference frame
        FrFrame_ GetFrameAtPoint(const Position& bodyPoint, FRAME_CONVENTION fc);

        /// Get a frame object whose origin is locate at COG and orientation is that of the body reference frame
        FrFrame_ GetFrameAtCOG(FRAME_CONVENTION fc);


        /// Get the position in world frame of a body fixed point whose position is given in body reference frame
        Position GetPointPositionInWorld(const Position& bodyPos, FRAME_CONVENTION fc) const;

        /// Get the position in body reference frame of a body fixed point whose position is given in world frame
        Position GetPointPositionInBody(const Position& worldPos, FRAME_CONVENTION fc) const;

        /// Get the body COG position in world frame (coordinates are expressed in world frame)
        Position GetCOGPositionInWorld(FRAME_CONVENTION fc) const;



        // FIXME : est ce que ces methodes de setPointPosition ne devraient pas etre plutot du movePointPosition
        // Le pb est qu'on implicite la rotation...
        /// Set the position in world of a body fixed point whose position is defined wrt body reference frame
        /// Note that it moves the entire body along with its nodes and other attached elements to the body (nodes...)
        /// which are updated
        void SetPositionOfBodyPoint(const Position &bodyPoint, const Position &worldPos, FRAME_CONVENTION fc);

        /// Translate the body along a translation vector whose coordinates are given in the world frame
        /// Note that it moves the entire body along with its nodes and other attached elements to the body (nodes...)
        /// which are updated
        void TranslateInWorld(const Position& worldTranslation, FRAME_CONVENTION fc);

        /// Translate the body along a translation vector whose coordinates are given in the body frame
        /// Note that it moves the entire body along with its nodes and other attached elements to the body (nodes...)
        /// which are updated
        void TranslateInBody(const Position& bodyTranslation, FRAME_CONVENTION fc);


        /// Rotate the body with respect to its current orientation in world using a rotation object
        /// Note that it moves the entire body along with its nodes and other attached elements to the body (nodes...)
        /// which are updated
        void Rotate(const FrRotation_& relRotation);

        /// Rotate the body with respect to its current orientation in world using a quaternion object
        /// Note that it moves the entire body along with its nodes and other attached elements to the body (nodes...)
        /// which are updated
        void Rotate(const FrQuaternion_& relQuaternion);


        // FIXME : reflechir de nouveau a ce que sont les eux methodes precedentes... on tourne autour de quoi ?
        // Possible que ca n'ait pas de sens...
        void RotateAroundPointInWorld(const FrRotation_& rot, const Position& worldPos, FRAME_CONVENTION fc);

        void RotateAroundPointInBody(const FrRotation_& rot, const Position& bodyPos, FRAME_CONVENTION fc);

        void RotateAroundCOG(const FrRotation_& rot, FRAME_CONVENTION fc);


        void RotateAroundPointInWorld(const FrQuaternion_& rot, const Position& worldPos, FRAME_CONVENTION fc);

        void RotateAroundPointInBody(const FrQuaternion_& rot, const Position& bodyPos, FRAME_CONVENTION fc);

        void RotateAroundCOG(const FrQuaternion_& rot, FRAME_CONVENTION fc);




        // TODO : ajouter aussi les RotateX, RotateAxisAngle, RotateEuler...


//        /// Set the velocity of the body reference frame with a vector expressed in WORLD frame
//        void SetVelocityInWorld(const Velocity& worldVel, FRAME_CONVENTION fc);
//
//        /// Set the velocity of the body reference frame with a vector expressed in BODY frame
//        void SetVelocityInBody(const Velocity& bodyVel, FRAME_CONVENTION fc);
//
        /// Get the velocity of the body reference frame with a vector expressed in WORLD frame
        Velocity GetVelocityInWorld(FRAME_CONVENTION fc) const;

        /// Get the velocity of the body reference frame with a vector expressed in BODY frame
        Velocity GetVelocityInBody(FRAME_CONVENTION fc) const;



        /// Set the velocity of the body COG with a vector expressed in WORLD frame
        void SetCOGVelocityInWorld(const Velocity& worldVel, FRAME_CONVENTION fc);

        /// Set the velocity of the body COG with a vector expressed in BODY frame
        void SetCOGVelocityInBody(const Velocity& bodyVel, FRAME_CONVENTION fc);

        /// Get the velocity of the body COG with a vector expressed in WORLD frame
        Velocity GetCOGVelocityInWorld(FRAME_CONVENTION fc) const;

        /// Get the velocity of the body COG with a vector expressed in BODY frame
        Velocity GetCOGVelocityInBody(FRAME_CONVENTION fc) const;




//        /// Set the acceleration of the body reference frame with a vector expressed in WORLD frame
//        void SetAccelerationInWorld(const Velocity& worldVel, FRAME_CONVENTION fc);
//
//        /// Set the acceleration of the body reference frame with a vector expressed in BODY frame
//        void SetAccelerationInBody(const Velocity& bodyVel, FRAME_CONVENTION fc);
//
//        /// Get the acceleration of the body reference frame with a vector expressed in WORLD frame
//        void GetAccelerationInWorld(const Velocity& worldVel, FRAME_CONVENTION fc);
//
//        /// Get the acceleration of the body reference frame with a vector expressed in BODY frame
//        void GetAccelerationInBody(const Velocity& bodyVel, FRAME_CONVENTION fc);




        /// Set the acceleration of the body COG with a vector expressed in WORLD frame
        void SetCOGAccelerationInWorld(const Acceleration& worldAcc, FRAME_CONVENTION fc);

        /// Set the acceleration of the body COG with a vector expressed in BODY frame
        void SetCOGAccelerationInBody(const Acceleration& bodyAcc, FRAME_CONVENTION fc);

        /// Get the acceleration of the body COG with a vector expressed in WORLD frame
        Acceleration GetCOGAccelerationInWorld(FRAME_CONVENTION fc) const;

        /// Get the acceleration of the body COG with a vector expressed in BODY frame
        Acceleration GetCOGAccelerationInBody(FRAME_CONVENTION fc) const;



        /// Set the body angular velocity from a vector expressed in WORLD frame
        void SetAngularVelocityInWorld(const AngularVelocity& worldAngVel, FRAME_CONVENTION fc);

        /// Set the body angular velocity from a vector expressed in BODY frame
        void SetAngularVelocityInBody(const AngularVelocity& bodyAngVel, FRAME_CONVENTION fc);


        /// Get the body angular velocity from a vector expressed in WORLD frame
        AngularVelocity GetAngularVelocityInWorld(FRAME_CONVENTION fc) const;

        /// Get the body angular velocity from a vector expressed in BODY frame
        AngularVelocity GetAngularVelocityInBody(FRAME_CONVENTION fc) const;




        /// Set the body angular acceleration from a vector expressed in WORLD frame
        void SetAngularAccelerationInWorld(const AngularAcceleration& worldAngAcc, FRAME_CONVENTION fc);

        /// Set the body angular acceleration from a vector expressed in BODY frame
        void SetAngularAccelerationInBody(const AngularAcceleration& bodyAngAcc, FRAME_CONVENTION fc);


        /// Get the body angular acceleration from a vector expressed in WORLD frame
        AngularAcceleration GetAngularAccelerationInWorld(FRAME_CONVENTION fc) const;

        /// Get the body angular acceleration from a vector expressed in BODY frame
        AngularAcceleration GetAngularAccelerationInBody(FRAME_CONVENTION fc) const;



        /// Get the velocity expressed in world frame of a body fixed point whose coordinates are given in world frame
        Velocity GetVelocityInWorldAtPointInWorld(const Position& worldPoint, FRAME_CONVENTION fc) const;

        /// Get the velocity expressed in world frame of a body fixed point whose coordinates are given in body frame
        Velocity GetVelocityInWorldAtPointInBody(const Position& bodyPoint, FRAME_CONVENTION fc) const;

        /// Get the velocity expressed in body frame of a body fixed point whose coordinates are given in world frame
        Velocity GetVelocityInBodyAtPointInWorld(const Position& worldPoint, FRAME_CONVENTION fc) const;

        /// Get the velocity expressed in body frame of a body fixed point whose coordinates are given in body frame
        Velocity GetVelocityInBodyAtPointInBody(const Position& bodyPoint, FRAME_CONVENTION fc) const;



        /// Get the acceleration expressed in world frame of a body fixed point whose coordinates are given in world frame
        Acceleration GetAccelerationInWorldAtPointInWorld(const Position& worldPoint, FRAME_CONVENTION fc) const;

        /// Get the acceleration expressed in world frame of a body fixed point whose coordinates are given in body frame
        Acceleration GetAccelerationInWorldAtPointInBody(const Position& bodyPoint, FRAME_CONVENTION fc) const;

        /// Get the acceleration expressed in body frame of a body fixed point whose coordinates are given in world frame
        Acceleration GetAccelerationInBodyAtPointInWorld(const Position& worldPoint, FRAME_CONVENTION fc) const;

        /// Get the acceleration expressed in body frame of a body fixed point whose coordinates are given in body frame
        Acceleration GetAccelerationInBodyAtPointInBody(const Position& bodyPoint, FRAME_CONVENTION fc) const;



        /// Set the velocity expressed in WORLD frame of a body fixed point whose coordinates are given in WORLD frame
        /// along with the angular velocity expressed in WORLD frame so that the velocity state is totally defined
        void SetGeneralizedVelocityInWorldAtPointInWorld(const Position& worldPoint,
                const Velocity& worldVel, const AngularVelocity& worldAngVel, FRAME_CONVENTION fc);

        /// Set the velocity expressed in WORLD frame of a body fixed point whose coordinates are given in BODY frame
        /// along with the angular velocity expressed in WORLD frame so that the velocity state is totally defined
        void SetGeneralizedVelocityInWorldAtPointInBody(const Position& bodyPoint,
                const Velocity& worldVel, const AngularVelocity& worldAngVel, FRAME_CONVENTION fc);

        /// Set the velocity expressed in BODY frame of a body fixed point whose coordinates are given in WORLD frame
        /// along with the angular velocity expressed in BODY frame so that the velocity state is totally defined
        void SetGeneralizedVelocityInBodyAtPointInWorld(const Position& worldPoint,
                const Velocity& bodyVel, const AngularVelocity& bodyAngVel, FRAME_CONVENTION fc);

        /// Set the velocity expressed in BODY frame of a body fixed point whose coordinates are given in BODY frame
        /// along with the angular velocity expressed in BODY frame so that the velocity state is totally defined
        void SetGeneralizedVelocityInBodyAtPointInBody(const Position& bodyPoint,
                const Velocity& bodyVel, const AngularVelocity& bodyAngVel, FRAME_CONVENTION fc);

        /// Set the velocity expressed in WORLD frame of a body fixed point whose coordinates are given in WORLD frame
        /// along with the angular velocity expressed in WORLD frame so that the acceleration state is totally defined
        void SetGeneralizedAccelerationInWorldAtPointInWorld(const Position& worldPoint,
                const Acceleration& worldAcc, const AngularAcceleration& worldAngVel, FRAME_CONVENTION fc);

        /// Set the velocity expressed in WORLD frame of a body fixed point whose coordinates are given in WORLD frame
        /// along with the angular velocity expressed in WORLD frame so that the acceleration state is totally defined
        void SetGeneralizedAccelerationInWorldAtPointInBody(const Position& bodyPoint,
                const Acceleration& worldAcc, const AngularAcceleration& worldAngAcc, FRAME_CONVENTION fc);

        /// Set the velocity expressed in BODY frame of a body fixed point whose coordinates are given in WORLD frame
        /// along with the angular velocity expressed in BODY frame so that the acceleration state is totally defined
        void SetGeneralizedAccelerationInBodyAtPointInWorld(const Position& worldPoint,
                const Acceleration& bodyAcc, const AngularAcceleration& bodyAngAcc, FRAME_CONVENTION fc);

        /// Set the velocity expressed in BODY frame of a body fixed point whose coordinates are given in BODY frame
        /// along with the angular velocity expressed in BODY frame so that the acceleration state is totally defined
        void SetGeneralizedAccelerationInBodyAtPointInBody(const Position& bodyPoint,
                const Acceleration& bodyAcc, const AngularAcceleration& bodyAngAcc, FRAME_CONVENTION fc);


        // =============================================================================================================
        // PROJECTIONS
        // =============================================================================================================

        template <class Vector>
        Vector ProjectVectorInWorld(const Vector& bodyVector, FRAME_CONVENTION fc) const {
            return GetQuaternion().Rotate<Vector>(bodyVector, fc);
        }

        template <class Vector>
        Vector& ProjectVectorInWorld(Vector& bodyVector, FRAME_CONVENTION fc) const {
            bodyVector = GetQuaternion().Rotate<Vector>(bodyVector, fc);
            return bodyVector;
        }

        template <class Vector>
        Vector ProjectVectorInBody(const Vector& worldVector, FRAME_CONVENTION fc) const {
            return GetQuaternion().GetInverse().Rotate<Vector>(worldVector, fc);
        }

        template <class Vector>
        Vector& ProjectVectorInBody(Vector& worldVector, FRAME_CONVENTION fc) const {
            worldVector = GetQuaternion().GetInverse().Rotate<Vector>(worldVector, fc);
            return worldVector;
        }

        // =============================================================================================================
        // CONSTRAINTS ON DOF
        // =============================================================================================================

        // Motion constraints  : FIXME : experimental !!!!
        void ConstainDOF(bool cx, bool cy, bool cz, bool crx, bool cry, bool crz);

        void ConstrainInVx(double Vx);


    protected:

        enum FRAME {
            WORLD,
            BODY
        };

        void _SetPointPosition(const Position& point, FRAME pointFrame, const Position& pos, FRAME posFrame, FRAME_CONVENTION fc);

        void _Translate(const Position& translation, FRAME translationFrame, FRAME_CONVENTION fc);

        void _SetVelocityAtPoint(const Position& point, FRAME pointFrame,
                                 const Velocity& vel, FRAME velFrame,
                                 const AngularVelocity& angVel, FRAME angVelFrame,
                                 FRAME_CONVENTION fc);


        // TODO : voir si on a besoin que ce bloc soit protected...
        std::shared_ptr<chrono::ChBody> GetChronoBody() {
            return m_chronoBody;
        }

        // Friends of FrBody_ : they can have access to chrono internals
        friend void makeItBox(std::shared_ptr<FrBody_>, double, double, double, double);
        friend void makeItCylinder(std::shared_ptr<FrBody_>, double, double, double);
        friend void makeItSphere(std::shared_ptr<FrBody_>, double, double);

        friend FrNode_::FrNode_(FrBody_*);

    public:

        /// Body initialization method
        void Initialize() override;

        /// Method called at the send of a time step. Logging may be used here
        void StepFinalize() override;

        /// Body update method
        void Update();


        // Linear iteraotrs on external forces
        using ForceIter = ForceContainer::iterator;
        using ConstForceIter = ForceContainer::const_iterator;

        ForceIter       force_begin();
        ConstForceIter  force_begin() const;

        ForceIter       force_end();
        ConstForceIter  force_end() const;



        // friend declarations
        // This one is made for the FrOffshoreSystem to be able to add the embedded chrono object into the embedded
        // chrono system (ChSystem)
        friend void FrOffshoreSystem_::AddBody(std::shared_ptr<frydom::FrBody_>);


    };


}  // end namespace frydom

#endif //FRYDOM_FRBODY_H
