//
// Created by frongere on 21/06/17.
//

#ifndef FRYDOM_FRBODY_H
#define FRYDOM_FRBODY_H


#include "FrOffshoreSystem.h"
//
//
#include "chrono/physics/ChBodyAuxRef.h"

#include "FrObject.h"
#include "FrVector.h"
#include "frydom/core/FrGeographic.h"
#include "frydom/core/FrForce.h"
#include "hermes/hermes.h"
#include "frydom/mesh/FrTriangleMeshConnected.h"
#include "frydom/core/FrEulerAngles.h"
#include "FrInertia.h"

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

            m_bodyMsg.AddField<double>("rx", "rad", "euler angle along the x-direction (body ref frame)",
                                       &m_angles_rotation.x());
            m_bodyMsg.AddField<double>("ry", "rad", "euler angle along the y-direction (body ref frame)",
                                       &m_angles_rotation.y());
            m_bodyMsg.AddField<double>("rz", "rad", "euler angle along the z-direction (body ref frame)",
                                       &m_angles_rotation.z());

            m_bodyMsg.AddField<double>("Xbody_FX", "N", "force acting on the body at COG (in absolute reference frame)", &Xforce.x());
            m_bodyMsg.AddField<double>("Xbody_FY", "N", "force acting on the body at COG (in absolute reference frame)", &Xforce.y());
            m_bodyMsg.AddField<double>("Xbody_FZ", "N", "force acting on the body at COG (in absolute reference frame)", &Xforce.z());

            m_bodyMsg.AddField<double>("Xbody_MX", "N.m", "moment acting on the body at COG (in body reference frame)", &Xtorque.x());
            m_bodyMsg.AddField<double>("Xbody_MY", "N.m", "moment acting on the body at COG (in body reference frame)", &Xtorque.y());
            m_bodyMsg.AddField<double>("Xbody_MZ", "N.m", "moment acting on the body at COG (in body reference frame)", &Xtorque.z());


            //for (auto force: forcelist) {
            //    auto dforce = dynamic_cast<FrForce *>(force.get());
            //    m_bodyMsg.AddField<hermes::Message>("force", "-", "external force on a body", dforce->GetLog());
            //}
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

    /// Cette classe n'est la que pour heriter des objets chrono.
    /// Ce n'est pas celle qu'on manipule directement dans frydom en tant qu'utilisateur !!!
    class _FrBodyBase : public chrono::ChBodyAuxRef {

    public:

        _FrBodyBase();

    };



    // Forward declarations
    class FrForce_;
    class FrFrame_;
    class FrRotation_;


    class FrBody_ : public FrObject {

    protected:

        std::shared_ptr<_FrBodyBase> m_chronoBody;  // Chrono objects are always expressed in NWU frame convention


        using ForceContainer = std::vector<std::shared_ptr<FrForce_>>;
        ForceContainer m_externalForces;


        using CONTACT_TYPE = FrOffshoreSystem_::SYSTEM_TYPE;
        CONTACT_TYPE m_contactType = CONTACT_TYPE::SMOOTH_CONTACT;


    public:

        /// Default constructor
        FrBody_();

        void SetName(const char name[]);

        void SetBodyFixed(bool state);


        // TODO: ici, toutes les methodes qu'on veut publiques pour un corps !!!




        // =============================================================================================================
        // Principal inertial parameters
        // =============================================================================================================

        double GetMass() const;

        Position GetCOGLocalPosition(FRAME_CONVENTION fc) const;

        void SetCOGLocalPosition(double x, double y, double z, bool transportInertia, FRAME_CONVENTION fc);

        void SetCOGLocalPosition(const Position& position, bool transportInertia, FRAME_CONVENTION fc);

        FrInertiaTensor_ GetInertiaParams() const;

        void SetInertiaParams(const FrInertiaTensor_& inertia);

        void SetInertiaParams(double mass,
                              double Ixx, double Iyy, double Izz,
                              double Ixy, double Ixz, double Iyz,
                              const FrFrame_& coeffsFrame,
                              const Position& cogPosition,
                              FRAME_CONVENTION fc);

        void SetInertiaParams(double mass,
                              double Ixx, double Iyy, double Izz,
                              double Ixy, double Ixz, double Iyz,
                              const FrFrame_& cogFrame,
                              FRAME_CONVENTION fc);


        // Contact

        void SetSmoothContact();

        void SetNonSmoothContact();

        void SetContactMethod(CONTACT_TYPE contactType);

        CONTACT_TYPE GetContactType() const;

        void SetCollide(bool isColliding);

        // Assets
//        void AssetActive() // TODO

        void AddBoxShape(double xSize, double ySize, double zSize);

        void AddCylinderShape(double radius, double height);

        void AddSphereShape(double radius);


        // Speed limitation to stabilize simulations

        void ActivateSpeedLimits(bool activate);

        void SetMaxSpeed(double maxSpeed);

        void SetMaxRotationSpeed(double wMax);

        void RemoveGravity(bool val);




        /// Methodes validees =======================>>>>>>>>>>>>>>>>>>


        // About COG position

        void SetCOGAbsPosition(double x, double y, double z, FRAME_CONVENTION fc);

        void SetCOGAbsPosition(Position position, FRAME_CONVENTION fc);

        void GetCOGAbsPosition(double& x, double& y, double& z, FRAME_CONVENTION fc) const;

        Position GetCOGAbsPosition(FRAME_CONVENTION fc) const;


        // About body position (the one of its reference frame)

        void SetAbsPosition(double x, double y, double z, FRAME_CONVENTION fc);

        void SetAbsPosition(const Position& position, FRAME_CONVENTION fc);

        Position GetAbsPosition(FRAME_CONVENTION fc) const;

        void GetAbsPosition(Position &position, FRAME_CONVENTION fc) const;

        void GetAbsPosition(double &x, double &y, double &z, FRAME_CONVENTION fc) const;

        Position GetAbsPositionOfLocalPoint(double x, double y, double z, FRAME_CONVENTION fc) const;



        // About rotation

        FrRotation_ GetAbsRotation() const;

        void SetAbsRotation(const FrRotation_& rotation, FRAME_CONVENTION fc);

        FrQuaternion_ GetAbsQuaternion() const;

        void SetAbsRotation(const FrQuaternion_& quaternion, FRAME_CONVENTION fc);

        void GetEulerAngles_RADIANS(double& phi, double& theta, double& psi, EULER_SEQUENCE seq, FRAME_CONVENTION fc) const;

        void SetEulerAngles_RADIANS(double phi, double theta, double psi, EULER_SEQUENCE seq, FRAME_CONVENTION fc);

        void GetEulerAngles_DEGREES(double& phi, double& theta, double& psi, EULER_SEQUENCE seq, FRAME_CONVENTION fc) const;

        void SetEulerAngles_DEGREES(double phi, double theta, double psi, EULER_SEQUENCE seq, FRAME_CONVENTION fc);

        void GetCardanAngles_RADIANS(double& phi, double& theta, double& psi, FRAME_CONVENTION fc) const;

        void SetCardanAngles_RADIANS(double phi, double theta, double psi, FRAME_CONVENTION fc);

        void GetCardanAngles_DEGREES(double& phi, double& theta, double& psi, FRAME_CONVENTION fc) const;

        void SetCardanAngles_DEGREES(double phi, double theta, double psi, FRAME_CONVENTION fc);

        void GetRotationAxisAngle(Direction& axis, double angle, FRAME_CONVENTION fc) const;

        void SetRotationAxisAngle(const Direction& axis, double angleRAD, FRAME_CONVENTION fc);

        double GetRoll_DEGREES(FRAME_CONVENTION fc) const;
        void SetRoll_DEGREES(double roll, FRAME_CONVENTION fc);

        double GetPitch_DEGREES(FRAME_CONVENTION fc) const;
        double SetPitch_DEGREES(double pitch, FRAME_CONVENTION fc);

        double GetYaw_DEGREES(FRAME_CONVENTION fc) const;
        double SetYaw_DEGREES(double yaw, FRAME_CONVENTION fc);

        double GetRoll_RADIANS(FRAME_CONVENTION fc) const;
        double SetRoll_RADIANS(double roll, FRAME_CONVENTION fc);

        double GetPitch_RADIANS(FRAME_CONVENTION fc) const;
        double SetPitch_RADIANS(double pitch, FRAME_CONVENTION fc);

        double GetYaw_RADIANS(FRAME_CONVENTION fc) const;
        double SetYaw_RADIANS(double yaw, FRAME_CONVENTION fc);



        // Frame

        FrFrame_ GetAbsFrame() const;

        FrFrame_ GetOtherFrameRelativeTransform_WRT_ThisBody(const FrFrame_ &otherFrame, FRAME_CONVENTION fc) const;


        // Velocities

        void SetAbsVelocity(double vx, double vy, double vz, FRAME_CONVENTION fc);

        void SetAbsVelocity(const Velocity& velocity, FRAME_CONVENTION fc);

        Velocity GetAbsVelocity(FRAME_CONVENTION fc) const;

        void GetAbsVelocity(Velocity& velocity, FRAME_CONVENTION fc) const;

        void GetAbsVelocity(double& vx, double& vy, double& vz, FRAME_CONVENTION fc) const;



        void SetLocalVelocity(double u, double v, double w, FRAME_CONVENTION fc);

        void SetLocalVelocity(const Velocity& velocity, FRAME_CONVENTION fc);

        Velocity GetLocalVelocity(FRAME_CONVENTION fc) const;

        void GetLocalVelocity(Velocity& velocity, FRAME_CONVENTION fc) const;

        void GetLocalVelocity(double& u, double& v, double& w, FRAME_CONVENTION fc) const;

        Velocity GetAbsVelocityOfLocalPoint(double x, double y, double z, FRAME_CONVENTION fc) const;

        Velocity GetLocalVelocityOfLocalPoint(double x, double y, double z, FRAME_CONVENTION fc) const;


        // Velocities for COG

        void SetCOGAbsVelocity(double vx, double vy, double vz, FRAME_CONVENTION fc);

        void SetCOGAbsVelocity(const Velocity& velocity, FRAME_CONVENTION fc);

        Velocity GetCOGAbsVelocity(FRAME_CONVENTION fc) const;

        void GetCOGAbsVelocity(Velocity& velocity, FRAME_CONVENTION fc) const;

        void GetCOGAbsVelocity(double& vx, double& vy, double& vz, FRAME_CONVENTION fc) const;


        void SetCOGLocalVelocity(double u, double v, double w, FRAME_CONVENTION fc);

        void SetCOGLocalVelocity(const Velocity& velocity, FRAME_CONVENTION fc);

        Velocity GetCOGLocalVelocity(FRAME_CONVENTION fc) const;

        void GetCOGLocalVelocity(Velocity& velocity, FRAME_CONVENTION fc) const;

        void GetCOGLocalVelocity(double& u, double& v, double& w, FRAME_CONVENTION fc) const;


        // Rotational velocities

        void SetAbsRotationalVelocity(double wx, double wy, double wz, FRAME_CONVENTION fc);

        void SetAbsRotationalVelocity(const RotationalVelocity &omega, FRAME_CONVENTION fc);

        RotationalVelocity GetAbsRotationalVelocity(FRAME_CONVENTION fc) const;

        void GetAbsRotationalVelocity(RotationalVelocity &omega, FRAME_CONVENTION fc) const;

        void GetAbsRotationalVelocity(double &wx, double &wy, double &wz, FRAME_CONVENTION fc) const;







        // Accelerations

        void SetAbsAcceleration(double ax, double ay, double az, FRAME_CONVENTION fc);

        void SetAbsAcceleration(const Acceleration &acceleration, FRAME_CONVENTION fc);

        Acceleration GetAbsAcceleration(FRAME_CONVENTION fc) const;

        void GetAbsAcceleration(Acceleration &acceleration, FRAME_CONVENTION fc);

        void GetAbsAcceleration(double &ax, double &ay, double &az, FRAME_CONVENTION fc) const;



        void SetLocalAcceleration(double up, double vp, double wp, FRAME_CONVENTION fc);

        void SetLocalAcceleration(const Acceleration &acceleration, FRAME_CONVENTION fc);

        Acceleration GetLocalAcceleration(FRAME_CONVENTION fc) const;

        void GetLocalAcceleration(Acceleration &acceleration, FRAME_CONVENTION fc);

        void GetLocalAcceleration(double &up, double &vp, double &wp, FRAME_CONVENTION fc) const;

        Acceleration GetAbsAccelerationOfLocalPoint(double x, double y, double z, FRAME_CONVENTION fc) const;

        Acceleration GetLocalAccelerationOfLocalPoint(double x, double y, double z, FRAME_CONVENTION fc) const;


        // Accelerations for COG

        void SetCOGAbsAcceleration(double ax, double ay, double az, FRAME_CONVENTION fc);

        void SetCOGAbsAcceleration(const Acceleration& acceleration, FRAME_CONVENTION fc);

        Acceleration GetCOGAbsAcceleration(FRAME_CONVENTION fc) const;

        void GetCOGAbsAcceleration(Acceleration& acceleration, FRAME_CONVENTION fc);

        void GetCOGAbsAcceleration(double& ax, double& ay, double& az, FRAME_CONVENTION fc) const;


        void SetCOGLocalAcceleration(double up, double vp, double wp, FRAME_CONVENTION fc);

        void SetCOGLocalAcceleration(const Acceleration& acceleration, FRAME_CONVENTION fc);

        Acceleration GetCOGLocalAcceleration(FRAME_CONVENTION fc) const;

        void GetCOGLocalAcceleration(Acceleration& acceleration, FRAME_CONVENTION fc);

        void GetCOGLocalAcceleration(double& up, double& vp, double& wp, FRAME_CONVENTION fc) const;


        // Rotational accelerations

        void SetAbsRotationalAcceleration(double wxp, double wyp, double wzp, FRAME_CONVENTION fc);

        void SetAbsRotationalAcceleration(const RotationalAcceleration& omegap, FRAME_CONVENTION fc);

        RotationalAcceleration GetAbsRotationalAcceleration(FRAME_CONVENTION fc) const;

        void GetAbsRotationalAcceleration(RotationalAcceleration& omegap, FRAME_CONVENTION fc);

        void GetAbsRotationalAcceleration(double& wxp, double& wyp, double& wzp, FRAME_CONVENTION fc) const;











        // About body orientation




        /// =================>>>>>>>>>>>>>>>>>>< Fin methodes validees







        void AddMeshAsset(std::string obj_filename);





    protected:

        std::shared_ptr<chrono::ChBody> GetChronoBody() {
            return m_chronoBody;
        }

        friend void makeItBox(std::shared_ptr<FrBody_>, double, double, double, double);
        friend void makeItCylinder(std::shared_ptr<FrBody_>, double, double, double);
        friend void makeItSphere(std::shared_ptr<FrBody_>, double, double);


    public:

        void Initialize() override;

        void StepFinalize() override;


        // Linear iterators on external forces
        using ForceIter = ForceContainer::iterator;
        using ConstForceIter = ForceContainer::const_iterator;

        ForceIter       force_begin();
        ConstForceIter  force_begin() const;

        ForceIter       force_end();
        ConstForceIter  force_end() const;






//    public:
        friend void FrOffshoreSystem_::AddBody(std::shared_ptr<frydom::FrBody_> body); // Voir a replacer


//        friend void internal::AddBodyToSystem(FrOffshoreSystem* , std::shared_ptr<FrBody>);



    };





}  // end namespace frydom

#endif //FRYDOM_FRBODY_H
