//
// Created by frongere on 21/06/17.
//

#ifndef FRYDOM_FRBODY_H
#define FRYDOM_FRBODY_H

#include "chrono/physics/ChBodyAuxRef.h"

#include "hermes/hermes.h"

#include "frydom/core/common/FrObject.h"
#include "frydom/core/FrOffshoreSystem.h"
#include "frydom/core/math/FrVector.h"
#include "frydom/core/common/FrConvention.h"
#include "frydom/core/force/FrForce.h"
#include "frydom/core/math/FrEulerAngles.h" // TODO : devrait disparaitre

#include "frydom/asset/FrGridAsset.h"

#include "frydom/mesh/FrTriangleMeshConnected.h"

#include "FrInertia.h"
//#include "FrForce.h"
#include "frydom/core/misc/FrColors.h"
#include "frydom/core/common/FrNode.h"

#include "frydom/environment/FrFluidType.h"

// TODO : voir si il n'y a pas moyen de passer ces includes
#include "frydom/hydrodynamics/seakeeping/linear/radiation/FrAddedMassBase.h"
#include "frydom/hydrodynamics/seakeeping/linear/radiation/FrVariablesAddedMassBase.h"


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

















    // REFACTORING ------>>>>>>>>>>>>>>

    class FrUnitQuaternion_;
    class FrBody_;


    namespace internal {

        /// Base class inheriting from chrono ChBodyAuxRef
        /// This class must not be used by external FRyDoM users. It is used in composition rule along with the FrBody_ FRyDoM class
        struct _FrBodyBase : public chrono::ChBodyAuxRef {

            FrBody_ *m_frydomBody;                      ///< pointer to the FrBody containing this bodyBase

            /// Constructor of the bodyBase
            /// \param body body containing this bodyBase
            explicit _FrBodyBase(FrBody_ *body);

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

            ///
            void RemoveAsset(std::shared_ptr<chrono::ChAsset> asset);

        };

    }  // end namespace internal


    // Forward declarations
    class FrFrame_;
    class FrRotation_;
    class FrOffshoreSystem_;
    class FrGeographicCoord;
    class FrAsset;

    /// Main class for a FRyDoM rigid body
    class FrBody_ : public FrObject {

    protected:

        std::shared_ptr<internal::_FrBodyBase> m_chronoBody;  ///< Embedded Chrono body Object

        FrOffshoreSystem_* m_system;                ///< Pointer to the FrOffshoreSystem where the body has been registered

        using ForceContainer = std::vector<std::shared_ptr<FrForce_>>;
        ForceContainer m_externalForces;            ///< Container of the external forces acting on body

        using AssetContainer = std::vector<std::shared_ptr<FrAsset>>;
        AssetContainer m_assets;                    ///< Container of the assets added to the body

        using CONTACT_TYPE = FrOffshoreSystem_::SYSTEM_TYPE;
        CONTACT_TYPE m_contactType = CONTACT_TYPE::SMOOTH_CONTACT; ///< The contact method that has to be consistent with that of the FrOffshoreSystem


    public:

        /// Default constructor
        FrBody_();

        /// Get the FrOffshoreSystem where the body has been registered
        FrOffshoreSystem_* GetSystem() const;

        /// Set the body name
        /// \param name body name
        void SetName(const char name[]);

        /// Make the body fixed
        /// \param state true if body is fixed, false otherwise
        void SetBodyFixed(bool state);


        // =============================================================================================================
        // PRINCIPAL INERTIAL PARAMETERS
        // =============================================================================================================

        /// Get the body mass in kg
        double GetMass() const;

//        /// Set the body mass in kg
//        /// \param mass body mass in kg
//        void SetMass(double mass);
//

        /// Get the inertia parameters as a FrInertiaTensor_ object
        // TODO : gerer la frame convention !
        FrInertiaTensor_ GetInertiaTensor(FRAME_CONVENTION fc) const; // TODO : voir pour une methode renvoyant une reference non const

        /// Set the inertia parameters as a FrInertiaTensor_ object
        void SetInertiaTensor(const FrInertiaTensor_ &inertia);

//        /// Set the principal inertia parameters given as coefficients expressed in coeffsFrame that can be different
//        /// from the local COG position cogPosition (expressed in body reference coordinate system)
//        void SetInertiaParams(double mass,
//                              double Ixx, double Iyy, double Izz,
//                              double Ixy, double Ixz, double Iyz,
//                              const FrFrame_& coeffsFrame,
//                              const Position& cogPosition,
//                              FRAME_CONVENTION fc);
//
//        /// Set the inertia parameters given in cogFrame relative to
//        void SetInertiaParams(double mass,
//                              double Ixx, double Iyy, double Izz,
//                              double Ixy, double Ixz, double Iyz,
//                              const FrFrame_& cogFrame,
//                              FRAME_CONVENTION fc);

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
        void SetCollide(bool isColliding);

        std::shared_ptr<chrono::ChMaterialSurfaceSMC> GetMaterialSurface() {return m_chronoBody->GetMaterialSurfaceSMC();}

        // TODO : ajouter de quoi definir des shapes de collision !!!

        // =============================================================================================================
        // VISUAL ASSETS
        // =============================================================================================================
//        void AssetActive() // TODO

        /// Add a box shape to the body with its dimensions defined in absolute coordinates. Dimensions in meters
        /// \param xSize size of the box along the x absolute coordinates
        /// \param ySize size of the box along the y absolute coordinates
        /// \param zSize size of the box along the z absolute coordinates
        void AddBoxShape(double xSize, double ySize, double zSize);  // TODO : definir plutot les dimensions dans le repere local du corps...

        /// Add a cylinder shape to the body with its dimensions defined in ???? Dimensions in meters
        /// \param radius radius of the cylinder shape.
        /// \param height height of the cylinder shape.
        void AddCylinderShape(double radius, double height);  // FIXME : travailler la possibilite de definir un axe... dans le repere local du corps

        /// Add a sphere shape to the body. Dimensions in meters.
        /// \param radius radius of the sphere shape.
        void AddSphereShape(double radius);  // TODO : permettre de definir un centre en coords locales du corps

        /// Add a mesh as an asset for visualization given a WaveFront .obj file name
        /// \param obj_filename filename of the asset to be added
        void AddMeshAsset(std::string obj_filename);

        /// Add a mesh as an asset for visualization given a FrTriangleMeshConnected mesh object
        /// \param mesh mesh of the asset to be added
        void AddMeshAsset(std::shared_ptr<FrTriangleMeshConnected> mesh);

        void AddAsset(std::shared_ptr<FrAsset> asset);

        /// Set the asset color in visualization given a color id
        /// \param colorName color of the asset
        void SetColor(NAMED_COLOR colorName);

        /// Set the asset color in visualization given a FrColor object
        /// \param color color of the asset
        void SetColor(const FrColor& color);

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
        void AddExternalForce(std::shared_ptr<FrForce_> force);

        /// Remove an external force to the body
        /// \param force force to be removed to the body
        void RemoveExternalForce(std::shared_ptr<FrForce_> force);

        /// Remove all forces from the body
        void RemoveAllForces();


        // =============================================================================================================
        // NODES
        // =============================================================================================================

        /// Generates a new node attached to the body which position and orientation are coincident with the body
        /// reference frame
        /// \return node created
        std::shared_ptr<FrNode_> NewNode();

//        /// Get a new node attached to the body given a frame defined with respect to the body reference frame
//        /// \param nodeFrame frame of the node, given in body reference frame
//        /// \return node created
//        std::shared_ptr<FrNode_> NewNode(const FrFrame_& nodeFrame);
//
//        /// Get a new node attached to the body given a position and a rotation defined with respect to the body
//        /// reference frame
//        /// \param bodyFrame frame of the node, given in body reference frame
//        /// \return node created
//        std::shared_ptr<FrNode_> NewNode(const Position& nodeLocalPosition, const FrRotation_& nodeLocalRotation,
//                                         FRAME_CONVENTION fc);
//
//        /// Get a new node attached to the body given a position of the node expressed into the body reference frame
//        /// \param nodeLocalPosition position of the node, in the body reference frame
//        /// \param fc frame convention (NED/NWU)
//        /// \return node created
//        std::shared_ptr<FrNode_> NewNode(const Position& nodeLocalPosition, FRAME_CONVENTION fc);
//
//        /// Get a new node attached to the body given a position of the node expressed into the body reference frame
//        /// \param x x position of the node in the body reference frame
//        /// \param y y position of the node in the body reference frame
//        /// \param z z position of the node in the body reference frame
//        /// \param fc frame convention (NED/NWU)
//        /// \return node created
//        std::shared_ptr<FrNode_> NewNode(double x, double y, double z, FRAME_CONVENTION fc);

        // TODO : permettre de definir un frame a l'aide des parametres de Denavit-Hartenberg modifies ?? --> dans FrFrame_ !

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
        FrRotation_ GetRotation() const;

        /// Set the orientation of the body reference frame in world using a rotation object
        /// Note that it moves the entire body along with its nodes and other attached elements to the body (nodes...)
        /// which are updated
        /// \param rotation orientation of the body reference frame in world frame
        void SetRotation(const FrRotation_& rotation);

        /// Get the quaternion object that represents the orientation of the body reference frame in the world frame
        /// \return quaternion object that represents the orientation of the body reference frame in the world frame
        FrUnitQuaternion_ GetQuaternion() const;

        /// Set the orientation of the body reference frame in world frame using a quaterion object
        /// Note that it moves the entire body along with its nodes and other attached elements to the body (nodes...)
        /// which are updated
        /// \param quaternion orientation of the body reference frame in world frame
        void SetRotation(const FrUnitQuaternion_& quaternion);

        //TODO : ajouter ici toutes les methodes portant sur d'autres representations de la rotation



        /// Get the body reference frame expressed in the world frame
        /// \return body reference frame expressed in the world frame
        FrFrame_ GetFrame() const;

        /// Set the body reference frame expressed in the world frame
        /// Note that it moves the entire body along with its nodes and other attached elements to the body (nodes...)
        /// which are updated
        /// \param worldFrame body reference frame expressed in the world frame
        void SetFrame(const FrFrame_& worldFrame);

        /// Get a frame object whose origin is located at a body point expressed in BODY frame and orientation is that
        /// of the body reference frame
        /// \param bodyPoint origin of the frame to return
        /// \param fc frame convention (NED/NWU)
        /// \return body frame
        FrFrame_ GetFrameAtPoint(const Position& bodyPoint, FRAME_CONVENTION fc);

        /// Get a frame object whose origin is locate at COG and orientation is that of the body reference frame
        /// \param fc frame convention (NED/NWU)
        /// \return body frame
        FrFrame_ GetFrameAtCOG(FRAME_CONVENTION fc);


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
        void TranslateInWorld(const Position& worldTranslation, FRAME_CONVENTION fc);

        /// Translate the body along a translation vector whose coordinates are given in the body frame
        /// Note that it moves the entire body along with its nodes and other attached elements to the body (nodes...)
        /// which are updated
        /// \param bodyTranslation translation to be applied to the body, expressed in body reference frame
        /// \param fc frame convention (NED/NWU)
        void TranslateInBody(const Position& bodyTranslation, FRAME_CONVENTION fc);


        /// Rotate the body with respect to its current orientation in world using a rotation object
        /// Note that it moves the entire body along with its nodes and other attached elements to the body (nodes...)
        /// which are updated
        /// \param relRotation relative rotation to be applied
        void Rotate(const FrRotation_& relRotation);

        /// Rotate the body with respect to its current orientation in world using a quaternion object
        /// Note that it moves the entire body along with its nodes and other attached elements to the body (nodes...)
        /// which are updated
        /// \param relQuaternion relative rotation, defined with quaternion, to be applied
        void Rotate(const FrUnitQuaternion_& relQuaternion);
        // FIXME : reflechir de nouveau a ce que sont les eux methodes precedentes... on tourne autour de quoi ?
        // Possible que ca n'ait pas de sens...

        /// Rotate the body around a point, given in world reference frame,  with respect to its current orientation
        /// in world using a rotation object.
        /// Note that it moves the entire body along with its nodes and other attached elements to the body (nodes...)
        /// which are updated
        /// \param rot rotation to be applied
        /// \param worldPos point position around which the body is to be rotated, given in world reference frame
        /// \param fc frame convention (NED/NWU)
        void RotateAroundPointInWorld(const FrRotation_& rot, const Position& worldPos, FRAME_CONVENTION fc);

        /// Rotate the body around a point, given in body reference frame,  with respect to its current orientation
        /// in world using a rotation object.
        /// Note that it moves the entire body along with its nodes and other attached elements to the body (nodes...)
        /// which are updated
        /// \param rot rotation to be applied
        /// \param worldPos point position around which the body is to be rotated, given in body reference frame
        /// \param fc frame convention (NED/NWU)
        void RotateAroundPointInBody(const FrRotation_& rot, const Position& bodyPos, FRAME_CONVENTION fc);

        /// Rotate the body around COG with respect to its current orientation in world using a rotation object.
        /// Note that it moves the entire body along with its nodes and other attached elements to the body (nodes...)
        /// which are updated
        /// \param rot rotation to be applied
        /// \param fc frame convention (NED/NWU)
        void RotateAroundCOG(const FrRotation_& rot, FRAME_CONVENTION fc);

        /// Rotate the body around a point, given in world reference frame,  with respect to its current orientation
        /// in world using a quaternion object.
        /// Note that it moves the entire body along with its nodes and other attached elements to the body (nodes...)
        /// which are updated
        /// \param rot rotation to be applied, with a quaternion object
        /// \param worldPos point position around which the body is to be rotated, given in world reference frame
        /// \param fc frame convention (NED/NWU)
        void RotateAroundPointInWorld(const FrUnitQuaternion_& rot, const Position& worldPos, FRAME_CONVENTION fc);

        /// Rotate the body around a point, given in body reference frame,  with respect to its current orientation
        /// in world using a quaternion object.
        /// Note that it moves the entire body along with its nodes and other attached elements to the body (nodes...)
        /// which are updated
        /// \param rot rotation to be applied, with a quaternion object
        /// \param worldPos point position around which the body is to be rotated, given in body reference frame
        /// \param fc frame convention (NED/NWU)
        void RotateAroundPointInBody(const FrUnitQuaternion_& rot, const Position& bodyPos, FRAME_CONVENTION fc);

        /// Rotate the body around COG with respect to its current orientation in world using a quaternion object.
        /// Note that it moves the entire body along with its nodes and other attached elements to the body (nodes...)
        /// which are updated
        /// \param rot rotation to be applied, with a quaternion object
        /// \param fc frame convention (NED/NWU)
        void RotateAroundCOG(const FrUnitQuaternion_& rot, FRAME_CONVENTION fc);




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
        Vector& ProjectGenerallizedVectorInBody(Vector& worldVector, FRAME_CONVENTION fc) const {
            worldVector = GetQuaternion().GetInverse().Rotate<Vector>(worldVector, fc);
            return worldVector;
        }

        // =============================================================================================================
        // CONSTRAINTS ON DOF
        // =============================================================================================================

        // TODO : tenir a jour un masque de degres de liberte bloques...

        // Motion constraints  : FIXME : experimental !!!!
        void ConstainDOF(bool cx, bool cy, bool cz, bool crx, bool cry, bool crz);

//        void ConstrainInVx(double Vx);


    protected:

        enum FRAME {
            WORLD,
            BODY
        };

        /// Set the COG position in the body reference frame
        /// \param bodyPos COG position in the body reference frame
        /// \param fc frame convention (NED/NWU)
        void SetCOG(const Position& bodyPos, FRAME_CONVENTION fc);

//        void _SetPointPosition(const Position& point, FRAME pointFrame, const Position& pos, FRAME posFrame, FRAME_CONVENTION fc);

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


        // Linear iterators on external forces
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
        friend void FrGridAsset::Initialize();

        friend int internal::FrAddedMassBase::GetBodyOffset(FrBody_* body) const;
        friend int internal::FrVariablesAddedMassBase::GetBodyOffset(FrBody_* body) const ;
        //friend void internal::FrVariablesAddedMassBase::Initialize();

    };


}  // end namespace frydom

#endif //FRYDOM_FRBODY_H
