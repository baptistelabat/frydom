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


        // =============================================================================================================
        // PRINCIPAL INERTIAL PARAMETERS
        // =============================================================================================================

        /// Get the body mass in kg
        double GetMass() const;

        /// Get the COG position in body reference coordinate system
        Position GetCOGLocalPosition(FRAME_CONVENTION fc) const;

        /// Set the COG position in body reference coordinate system
        void SetCOGLocalPosition(double x, double y, double z, bool transportInertia, FRAME_CONVENTION fc);

        /// Set the COG position in body reference coordinate system
        void SetCOGLocalPosition(const Position& position, bool transportInertia, FRAME_CONVENTION fc);

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
        // COG POSITION
        // =============================================================================================================

        /// Set the absolute position of the body COG
        void SetCOGAbsPosition(double x, double y, double z, FRAME_CONVENTION fc);

        /// Set the absolute position of the body COG
        void SetCOGAbsPosition(Position position, FRAME_CONVENTION fc);

        /// Get the absolute position of the body COG
        void GetCOGAbsPosition(double& x, double& y, double& z, FRAME_CONVENTION fc) const;

        /// Get the absolute position of the body COG
        Position GetCOGAbsPosition(FRAME_CONVENTION fc) const;


        // =============================================================================================================
        // BODY POSITION (reference frame)
        // =============================================================================================================

        /// Set the absolute position of the body reference frame origin
        void SetAbsPosition(double x, double y, double z, FRAME_CONVENTION fc);

        /// Set the absolute position of the body reference frame origin
        void SetAbsPosition(const Position& position, FRAME_CONVENTION fc);

        /// Get the absolute position of the body reference frame origin
        Position GetAbsPosition(FRAME_CONVENTION fc) const;

        /// get the absolute position of the body reference frame origin
        void GetAbsPosition(Position &position, FRAME_CONVENTION fc) const;

        /// get the absolute position of the body reference frame origin
        void GetAbsPosition(double &x, double &y, double &z, FRAME_CONVENTION fc) const;


        // =============================================================================================================
        // POSITION OF ANY BODY FIXED POINT
        // =============================================================================================================

        /// Get the absolute position of a point defined with respect to the body local reference frame
        Position GetAbsPositionOfLocalPoint(double x, double y, double z, FRAME_CONVENTION fc) const;

        /// Get the absolute position of a point defined with respect to the body local reference frame
        Position GetAbsPositionOfLocalPoint(const Position& localPos, FRAME_CONVENTION fc) const;

        /// Get the position of an absolute point with respect to the body reference frame
        Position GetLocalPositionOfAbsPoint(double x, double y, double z, FRAME_CONVENTION fc) const;

        /// Get the position of an absolute point with respect to the body reference frame
        Position GetLocalPositionOfAbsPoint(const Position& absPos, FRAME_CONVENTION fc) const;


        // =============================================================================================================
        // About rotations
        // =============================================================================================================

        /// Get the rotation of the body with respect to the absolute frame
        FrRotation_ GetAbsRotation() const;

        /// Set the rotation of the body with respect to the absolute frame
        void SetAbsRotation(const FrRotation_& rotation);

        /// Get the quaternion of the body with respect to the absolute frame
        FrQuaternion_ GetAbsQuaternion() const;

        /// Set the rotation of the body with respect to the absolute frame given a quaternion
        void SetAbsRotation(const FrQuaternion_& quaternion);

        /// Get the euler angles given an axis sequence of the rotation of the body with respect to the absolute frame
        /// (angles in radians)
        void GetEulerAngles_RADIANS(double& phi, double& theta, double& psi, EULER_SEQUENCE seq, FRAME_CONVENTION fc) const;

        /// Set the euler angles given an axis sequence of the rotation of the body with respect to the absolute frame
        /// (angles in radians)
        void SetEulerAngles_RADIANS(double phi, double theta, double psi, EULER_SEQUENCE seq, FRAME_CONVENTION fc);

        /// Get the euler angles given an axis sequence of the rotation of the body with respect to the absolute frame
        /// (angles in degrees)
        void GetEulerAngles_DEGREES(double& phi, double& theta, double& psi, EULER_SEQUENCE seq, FRAME_CONVENTION fc) const;

        /// Set the euler angles given an axis sequence of the rotation of the body with respect to the absolute frame
        /// (angles in degrees)
        void SetEulerAngles_DEGREES(double phi, double theta, double psi, EULER_SEQUENCE seq, FRAME_CONVENTION fc);

        /// Get the Cardan angles of the rotation of the body with respect to the absolute frame
        /// (angles in radians)
        void GetCardanAngles_RADIANS(double& phi, double& theta, double& psi, FRAME_CONVENTION fc) const;

        /// Set the Cardan angles of the rotation of the body with respect to the absolute frame
        /// (angles in radians)
        void SetCardanAngles_RADIANS(double phi, double theta, double psi, FRAME_CONVENTION fc);

        /// Get the Cardan angles of the rotation of the body with respect to the absolute frame
        /// (angles in degrees)
        void GetCardanAngles_DEGREES(double& phi, double& theta, double& psi, FRAME_CONVENTION fc) const;

        /// Set the Cardan angles of the rotation of the body with respect to the absolute frame
        /// (angles in degrees)
        void SetCardanAngles_DEGREES(double phi, double theta, double psi, FRAME_CONVENTION fc);

        /// Get the rotation of the body with respect ot the absolute frame in axis angle representation (angle in radians)
        void GetRotationAxisAngle(Direction& axis, double angleRAD, FRAME_CONVENTION fc) const;

        /// Set the rotation of the body with respect ot the absolute frame in axis angle representation (angle in radians)
        void SetRotationAxisAngle(const Direction& axis, double angleRAD, FRAME_CONVENTION fc);

        /// Get the roll angle of the body with respect to the absolute frame (angle in degrees)
        double GetRoll_DEGREES(FRAME_CONVENTION fc) const;

        /// Set the roll angle of the body with respect to the absolute frame (angle in degrees)
        void SetRoll_DEGREES(double roll, FRAME_CONVENTION fc);

        /// Get the pitch angle of the body with respect to the absolute frame (angle in degrees)
        double GetPitch_DEGREES(FRAME_CONVENTION fc) const;

        /// Set the pitch angle of the body with respect to the absolute frame (angle in degrees)
        double SetPitch_DEGREES(double pitch, FRAME_CONVENTION fc);

        /// Get the yaw angle of the body with respect to the absolute frame (angle in degrees)
        double GetYaw_DEGREES(FRAME_CONVENTION fc) const;

        /// Set the yaw angle of the body with respect to the absolute frame (angle in degrees)
        double SetYaw_DEGREES(double yaw, FRAME_CONVENTION fc);

        /// Get the roll angle of the body with respect to the absolute frame (angle in radians)
        double GetRoll_RADIANS(FRAME_CONVENTION fc) const;

        /// Set the roll angle of the body with respect to the absolute frame (angle in radians)
        double SetRoll_RADIANS(double roll, FRAME_CONVENTION fc);

        /// Get the pitch angle of the body with respect to the absolute frame (angle in radians)
        double GetPitch_RADIANS(FRAME_CONVENTION fc) const;

        /// Set the pitch angle of the body with respect to the absolute frame (angle in radians)
        double SetPitch_RADIANS(double pitch, FRAME_CONVENTION fc);

        /// Get the yaw angle of the body with respect to the absolute frame (angle in radians)
        double GetYaw_RADIANS(FRAME_CONVENTION fc) const;

        /// Set the yaw angle of the body with respect to the absolute frame (angle in radians)
        double SetYaw_RADIANS(double yaw, FRAME_CONVENTION fc);

        // =============================================================================================================
        // Projections in frames absolute and body frame of vector quantities
        // =============================================================================================================

        /// Project a vector expressed in absolute coordinate system into the body reference coordinate system
        template <class Vector>
        void ProjectAbsVectorInBodyCoords(Vector& vector, FRAME_CONVENTION fc) const {
            vector = GetAbsQuaternion().Rotate<Vector>(vector, fc);
        }

        /// Project a vector expressed in absolute coordinate system into the body reference coordinate system
        template <class Vector>
        Vector ProjectAbsVectorInBodyCoords(const Vector& vector, FRAME_CONVENTION fc) const {
            return GetAbsQuaternion().Rotate<Vector>(vector, fc);
        }

        /// Project a vector expressed in body reference coordinate system into the absolute reference coordinate system
        // TODO : voir pourquoi on ne peut pas acceder a cette methode...
        template <class Vector>
        void ProjectBodyVectorInAbsCoords(Vector& vector, FRAME_CONVENTION fc) const {
            vector = GetAbsQuaternion().GetInverse().Rotate<Vector>(vector, fc);
        }

        /// Project a vector expressed in body reference coordinate system into the absolute reference coordinate system
        template <class Vector>
        Vector ProjectBodyVectorInAbsCoords(const Vector& vector, FRAME_CONVENTION fc) const {
            return GetAbsQuaternion().GetInverse().Rotate<Vector>(vector, fc);
        }

        // =============================================================================================================
        // Frame
        // =============================================================================================================

        /// Get the reference frame of the body with respect to the absolute frame
        FrFrame_ GetAbsFrame() const;

        /// Set the reference frame of the body with respect to the absolute frame
        void SetAbsFrame(const FrFrame_& absFrame);

        /// Get the transform between an other frame (wrt absolute frame) and the body reference frame (wrt absolute)
        FrFrame_ GetOtherFrameRelativeTransform_WRT_ThisBody(const FrFrame_ &otherFrame, FRAME_CONVENTION fc) const;

        // =============================================================================================================
        // Velocities
        // =============================================================================================================

        /// Set the absolute velocity of the origin of the body reference frame expressed into the absolute frame
        void SetAbsVelocity(double vx, double vy, double vz, FRAME_CONVENTION fc);

        /// Set the absolute velocity of the origin of the body reference frame expressed into the absolute frame
        void SetAbsVelocity(const Velocity& absVel, FRAME_CONVENTION fc);

        /// Get the absolute velocity of the origin of the body reference frame expressed into the absolute frame
        Velocity GetAbsVelocity(FRAME_CONVENTION fc) const;

        /// Get the absolute velocity of the origin of the body reference frame expressed into the absolute frame
        void GetAbsVelocity(Velocity& absVel, FRAME_CONVENTION fc) const;

        /// Get the absolute velocity of the origin of the body reference frame expressed into the absolute frame
        void GetAbsVelocity(double& vx, double& vy, double& vz, FRAME_CONVENTION fc) const;

        /// Set the absolute velocity of the origin of the body reference frame expressed into the body reference frame
        void SetLocalVelocity(double u, double v, double w, FRAME_CONVENTION fc);

        /// Set the absolute velocity of the origin of the body reference frame expressed into the body reference frame
        void SetLocalVelocity(const Velocity& velocity, FRAME_CONVENTION fc);

        /// Get the absolute velocity of the origin of the body reference frame expressed into the body reference frame
        Velocity GetLocalVelocity(FRAME_CONVENTION fc) const;

        /// Get the absolute velocity of the origin of the body reference frame expressed into the body reference frame
        void GetLocalVelocity(Velocity& velocity, FRAME_CONVENTION fc) const;

        /// Get the absolute velocity of the origin of the body reference frame expressed into the body reference frame
        void GetLocalVelocity(double& u, double& v, double& w, FRAME_CONVENTION fc) const;

        /// Get the absolute velocity of a body fixed point expressed in absolute frame and whose position is expressed
        /// into the body reference frame
        Velocity GetAbsVelocityOfLocalPoint(double x, double y, double z, FRAME_CONVENTION fc) const;

        /// Get the absolute velocity of a body fixed point expressed in absolute frame and whose position is expressed
        /// into the body reference frame
        Velocity GetAbsVelocityOfLocalPoint(const Position& locaPos, FRAME_CONVENTION fc) const;

        /// Get the absolute velocity of a body fixed point expressed in body reference frame and whose position is
        /// expressed into the body reference frame
        Velocity GetLocalVelocityOfLocalPoint(double x, double y, double z, FRAME_CONVENTION fc) const;

        /// Get the absolute velocity of a body fixed point expressed in body reference frame and whose position is
        /// expressed into the body reference frame
        Velocity GetLocalVelocityOfLocalPoint(const Position& localPos, FRAME_CONVENTION fc) const;



        // =============================================================================================================
        // Relative velocities in current and wind
        // =============================================================================================================

        // TODO : utiliser l'enum FLUID_TYPE defini dans FrEnvironment
        // On veut la vitesse du vecteur flux relativement au corps en absolu ou en relatif en un point du corps (exprime en reltif ou absolu)
        // Quand on prend un point exprime en coord absolues, c'est une approche eulerienne...
        // Quand on prend un point exprime en coord locale (body), c'est une approche lagrangienne...
        // TODO : Ne doit-on pas plutot nommer Get

        /// Get the absolute relative velocity (expressed in absolute frame) with respect to a fluid stream
        /// (air or water) of a body fixed point whose coordinates are given in absolute coordinates
        Velocity GetAbsRelVelocityInStreamAtAbsPoint(const Position &absPos, FLUID_TYPE ft, FRAME_CONVENTION fc) const;

        /// Get the absolute relative velocity (expressed in absolute frame) with respect to a fluid stream
        /// (air or water) of the body COG
        Velocity GetAbsRelVelocityInStreamAtCOG(FLUID_TYPE ft, FRAME_CONVENTION fc) const; // Voir si on met un enum CURRENT ou WIND

        /// Get the absolute relative velocity (expressed in body coordinate frame) with respect to a fluid stream
        /// (air or water) of the body COG
        Velocity GetLocalRelVelocityInStreamAtCOG(FLUID_TYPE ft, FRAME_CONVENTION fc) const;

        /// Get the absolute velocity (exressed in absolute frame) with respect to a fluid stream
        /// (air or water) of a body fixed point whose coordinates are given in body local reference frame
        Velocity GetAbsRelVelocityInStreamAtLocalPoint(const Position& localPos, FLUID_TYPE ft, FRAME_CONVENTION fc) const;

        /// Get the absolute relative velocity (expressed body reference frame) with respect to a fluid stream
        /// (air or water) of a body fixed point whose coordinates are given in body reference frame
        Velocity GetLocalRelVelocityInStreamAtLocalPoint(const Position& localPos, FLUID_TYPE ft, FRAME_CONVENTION fc) const;

        /// Get the absolute relative velocity (expressed body reference frame) with respect to a fluid stream
        /// (air or water) of a body fixed point whose coordinates are given in absolute frame
        Velocity GetLocalRelVelocityInStreamAtAbsPoint(const Position& absPos, FLUID_TYPE ft, FRAME_CONVENTION fc) const;

        // =============================================================================================================
        // Velocities for COG
        // =============================================================================================================

        /// Set the absolute velocity of the body COG (expressed in absolute frame)
        void SetCOGAbsVelocity(double vx, double vy, double vz, FRAME_CONVENTION fc);

        /// Set the absolute velocity of the body COG (expressed in absolute frame)
        void SetCOGAbsVelocity(const Velocity& velocity, FRAME_CONVENTION fc);

        /// Get the absolute velocity of the body COG (expressed in absolute frame)
        Velocity GetCOGAbsVelocity(FRAME_CONVENTION fc) const;

        /// Get the absolute velocity of the body COG (expressed in absolute frame)
        void GetCOGAbsVelocity(Velocity& velocity, FRAME_CONVENTION fc) const;

        /// Get the absolute velocity of the body COG (expressed in absolute frame)
        void GetCOGAbsVelocity(double& vx, double& vy, double& vz, FRAME_CONVENTION fc) const;

        /// Set the absolute velocity of the body COG (expressed in body reference frame)
        void SetCOGLocalVelocity(double u, double v, double w, FRAME_CONVENTION fc);

        /// Set the absolute velocity of the body COG (expressed in body reference frame)
        void SetCOGLocalVelocity(const Velocity& velocity, FRAME_CONVENTION fc);

        /// Get the absolute velocity of the body COG (expressed in body reference frame)
        Velocity GetCOGLocalVelocity(FRAME_CONVENTION fc) const;

        /// Get the absolute velocity of the body COG (expressed in body reference frame)
        void GetCOGLocalVelocity(Velocity& velocity, FRAME_CONVENTION fc) const;

        /// Get the absolute velocity of the body COG (expressed in body reference frame)
        void GetCOGLocalVelocity(double& u, double& v, double& w, FRAME_CONVENTION fc) const;

        // =============================================================================================================
        // Rotational velocities
        // =============================================================================================================

        /// Set the body absolute angular velocity (expressed in absolute frame)
        void SetAbsRotationalVelocity(double wx, double wy, double wz, FRAME_CONVENTION fc);

        /// Set the body absolute angular velocity (expressed in absolute frame)
        void SetAbsRotationalVelocity(const RotationalVelocity &omega, FRAME_CONVENTION fc);

        /// Get the body absolute angular velocity (expressed in absolute frame)
        RotationalVelocity GetAbsRotationalVelocity(FRAME_CONVENTION fc) const;

        /// Get the body absolute angular velocity (expressed in absolute frame)
        void GetAbsRotationalVelocity(RotationalVelocity &omega, FRAME_CONVENTION fc) const;

        /// Get the body absolute angular velocity (expressed in absolute frame)
        void GetAbsRotationalVelocity(double &wx, double &wy, double &wz, FRAME_CONVENTION fc) const;

        /// Set the body absolute angular velocity (expressed in body reference frame)
        void SetLocalRotationalVelocity(double p, double q, double r, FRAME_CONVENTION fc);

        /// Set the body absolute angular velocity (expressed in body reference frame)
        void SetLocalRotationalVelocity(const RotationalVelocity &omega, FRAME_CONVENTION fc);

        /// Get the body absolute angular velocity (expressed in body reference frame)
        RotationalVelocity GetLocalRotationalVelocity(FRAME_CONVENTION fc) const;

        /// Get the body absolute angular velocity (expressed in body reference frame)
        void GetLocalRotationalVelocity(RotationalVelocity &omega, FRAME_CONVENTION fc) const;

        /// Get the body absolute angular velocity (expressed in body reference frame)
        void GetLocalRotationalVelocity(double &p, double &q, double &r, FRAME_CONVENTION fc) const;


        // =============================================================================================================
        // ACCELERATIONS
        // =============================================================================================================

        /// Set the absolute acceleration of body reference frame (expressed in absolute frame)
        void SetAbsAcceleration(double ax, double ay, double az, FRAME_CONVENTION fc);

        /// Set the absolute acceleration of body reference frame (expressed in absolute frame)
        void SetAbsAcceleration(const Acceleration &acceleration, FRAME_CONVENTION fc);

        /// Get the absolute acceleration of body reference frame (expressed in absolute frame)
        Acceleration GetAbsAcceleration(FRAME_CONVENTION fc) const;

        /// Get the absolute acceleration of body reference frame (expressed in absolute frame)
        void GetAbsAcceleration(Acceleration &acceleration, FRAME_CONVENTION fc) const;

        /// Get the absolute acceleration of body reference frame (expressed in absolute frame)
        void GetAbsAcceleration(double &ax, double &ay, double &az, FRAME_CONVENTION fc) const;


        /// Get the absolute acceleration of body reference frame (expressed in body reference frame)
        void SetLocalAcceleration(double up, double vp, double wp, FRAME_CONVENTION fc);

        /// Set the absolute acceleration of body reference frame (expressed in body reference frame)
        void SetLocalAcceleration(const Acceleration &acceleration, FRAME_CONVENTION fc);

        /// Get the absolute acceleration of body reference frame (expressed in body reference frame)
        Acceleration GetLocalAcceleration(FRAME_CONVENTION fc) const;

        /// Get the absolute acceleration of body reference frame (expressed in body reference frame)
        void GetLocalAcceleration(Acceleration &acceleration, FRAME_CONVENTION fc) const;

        /// Get the absolute acceleration of body reference frame (expressed in body reference frame)
        void GetLocalAcceleration(double &up, double &vp, double &wp, FRAME_CONVENTION fc) const;

        /// Get the absolute acceleration (expressed in absolute frame) of a body fixed point whose coordinates are
        /// given in body reference frame
        Acceleration GetAbsAccelerationOfLocalPoint(double x, double y, double z, FRAME_CONVENTION fc) const;

        /// Get the absolute acceleration (expressed in body reference frame) of a body fixed point whose coordinates are
        /// given in body reference frame
        Acceleration GetLocalAccelerationOfLocalPoint(double x, double y, double z, FRAME_CONVENTION fc) const;

        // =============================================================================================================
        // Accelerations for COG
        // =============================================================================================================

        /// Set the absolute acceleration of body COG (expressed in absolute frame)
        void SetCOGAbsAcceleration(double ax, double ay, double az, FRAME_CONVENTION fc);

        /// Set the absolute acceleration of body COG (expressed in absolute frame)
        void SetCOGAbsAcceleration(const Acceleration& acceleration, FRAME_CONVENTION fc);

        /// Get the absolute acceleration of body COG (expressed in absolute frame)
        Acceleration GetCOGAbsAcceleration(FRAME_CONVENTION fc) const;

        /// Get the absolute acceleration of body COG (expressed in absolute frame)
        void GetCOGAbsAcceleration(Acceleration& acceleration, FRAME_CONVENTION fc) const;

        /// Get the absolute acceleration of body COG (expressed in absolute frame)
        void GetCOGAbsAcceleration(double& ax, double& ay, double& az, FRAME_CONVENTION fc) const;

        /// Set the absolute acceleration of body COG (expressed in body reference frame)
        void SetCOGLocalAcceleration(double up, double vp, double wp, FRAME_CONVENTION fc);

        /// Set the absolute acceleration of body COG (expressed in body reference frame)
        void SetCOGLocalAcceleration(const Acceleration& acceleration, FRAME_CONVENTION fc);

        /// Get the absolute acceleration of body COG (expressed in body reference frame)
        Acceleration GetCOGLocalAcceleration(FRAME_CONVENTION fc) const;

        /// Get the absolute acceleration of body COG (expressed in body reference frame)
        void GetCOGLocalAcceleration(Acceleration& acceleration, FRAME_CONVENTION fc) const;

        /// Get the absolute acceleration of body COG (expressed in body reference frame)
        void GetCOGLocalAcceleration(double& up, double& vp, double& wp, FRAME_CONVENTION fc) const;

        // TODO : voir si on peut avoir des methodes de calcul d'acceleration a des points differents du COG...

        // =============================================================================================================
        // Rotational accelerations
        // =============================================================================================================

        /// Set the absolute angular acceleration of the body (expressed in absolute frame)
        void SetAbsRotationalAcceleration(double wxp, double wyp, double wzp, FRAME_CONVENTION fc);

        /// Set the absolute angular acceleration of the body (expressed in absolute frame)
        void SetAbsRotationalAcceleration(const RotationalAcceleration& omegap, FRAME_CONVENTION fc);

        /// Get the absolute angular acceleration of the body (expressed in absolute frame)
        RotationalAcceleration GetAbsRotationalAcceleration(FRAME_CONVENTION fc) const;

        /// Get the absolute angular acceleration of the body (expressed in absolute frame)
        void GetAbsRotationalAcceleration(RotationalAcceleration& omegap, FRAME_CONVENTION fc) const;

        /// Get the absolute angular acceleration of the body (expressed in absolute frame)
        void GetAbsRotationalAcceleration(double& wxp, double& wyp, double& wzp, FRAME_CONVENTION fc) const;

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



        // Motion constraints  : FIXME : experimental !!!!
        void ConstainDOF(bool cx, bool cy, bool cz, bool crx, bool cry, bool crz);

        void ConstrainInVx(double Vx);



    protected:

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
