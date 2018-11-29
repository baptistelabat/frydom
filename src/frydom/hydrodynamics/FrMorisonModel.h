//
// Created by camille on 17/04/18.
//

#ifndef FRYDOM_FRMORISONMODEL_H
#define FRYDOM_FRMORISONMODEL_H

/// <<<<<<<<<<<<< Head

#include <vector>
#include <frydom/core/FrPhysicsItem.h>

#include "frydom/core/FrConvention.h"
#include "frydom/core/FrNode.h"
#include "frydom/environment/ocean/freeSurface/waves/FrWaveField.h"
#include "frydom/environment/ocean/freeSurface/waves/FrFlowSensor.h"

/// <<<<<<<<<<<<< Adding header for refactoring

#include "frydom/core/FrForce.h"


namespace frydom {

    class FrMorisonForce;
    class FrHydroBody;
    //class FrFlowSensor;

    // --------------------------------------------------------------------------
    // MORISON ELEMENT
    // --------------------------------------------------------------------------

    class FrMorisonModel {

    protected:
        std::shared_ptr<FrMorisonForce> m_force;    ///< morison force pointer
        bool is_rigid;                              ///< Flag to identify if the structure is moving
        bool is_log = false;                        ///< True if the morison model is logable
        bool m_include_current = false;             ///< if true the current is accounted for the frag force computation

    public:
        /// Update the morison force
        virtual void UpdateState() = 0;

        /// Return the morison force pointer
        virtual FrMorisonForce* GetForce() const { return m_force.get(); }

        /// Return the force applied on the body
        virtual chrono::ChVector<double> GetBodyForce() const = 0;

        /// Return the moment applied on the body
        virtual chrono::ChVector<double> GetBodyTorque() const = 0;

        /// Add force the hydro body
        virtual void AddForce(FrHydroBody* body) = 0;

        /// Define the body to which the morison model is applied
        virtual void SetBody(FrHydroBody* body, bool add_force=false) = 0;

        /// Set the morison model rigid
        void SetRigid(const bool rigid=true) { is_rigid=rigid; }

        virtual void Initialize() = 0;

        /// Return true if the log message is active
        bool LogIsActive() const { return is_log; }

        /// Specified if the current flow must be included into the morison force
        virtual void IncludeCurrent(const bool flag) { m_include_current = flag; }

    };

    // ---------------------------------------------------------------------------
    // SINGLE ELEMENT
    // ---------------------------------------------------------------------------

    class FrSingleElement : public FrMorisonModel {

    private:
        std::shared_ptr<FrNode> m_nodeA;        ///< First node of the element
        std::shared_ptr<FrNode> m_nodeB;        ///< Second node of the element
        chrono::ChFrame<> m_frame;              ///< Frame linked to the morison element (in global coordinate system)
        double m_cd_x;                          ///< Drag coefficient (x_axis local frame)
        double m_cd_y;                          ///< Drag coefficient (y-axis local frame)
        double m_ca_x;                          ///< mass coefficient (x_axis local frame)
        double m_ca_y;                          ///< mass coefficient (y-axis local frame)
        double m_cf;                            ///< friction coefficient (z-axis local frame)
        double m_diameter;                      ///< diameter  of the morison element (m)
        double m_length;                        ///< Length of the morison element (m)
        double m_volume;                        ///< volume of the morison element
        std::unique_ptr<FrFlowSensor> m_flow;   ///< computation of the flow velocity and acceleration
        chrono::ChVector<> m_dir;               ///< unit vector in the axis of the element

    public:
        FrSingleElement();

        /// Constructor from node position and morison parameters
        FrSingleElement(std::shared_ptr<FrNode>& nodeA,
                        std::shared_ptr<FrNode>& nodeB,
                        double diameter, double ca, double cd, double cf);

        FrSingleElement(std::shared_ptr<FrNode>& nodeA,
                        std::shared_ptr<FrNode>& nodeB,
                        double diameter, double ca_x, double ca_y,
                        double cd_x, double cd_y, double cf);

        FrSingleElement(chrono::ChVector<>& posA, chrono::ChVector<>& posB,
                        double diameter, double ca, double cd, double cf);

        FrSingleElement(chrono::ChVector<>& posA, chrono::ChVector<>& posB,
                        double diameter, double ca_x, double ca_y, double cd_x, double cd_y, double cf);

        /// Definition of nodes from reference
        void SetNodes(FrNode& nodeA, FrNode& nodeB);

        /// Pass shared pointer for nodes
        void SetNodes(std::shared_ptr<FrNode>& nodeA, std::shared_ptr<FrNode>& nodeB);

        /// Definition of the added mass coefficient (isotrope)
        void SetAddedMass(const double ca) { SetAddedMass(ca, ca); }

        /// Definition of the added mass coefficient (anisotrope)
        void SetAddedMass(const double ca_x, double ca_y) {
            m_ca_x = ca_x;
            m_ca_y = ca_y;
        }

        /// Definition of the adimentional drag coefficient (isotrope)
        void SetDrag(const double cd) { SetDrag(cd, cd); }

        /// Definition of the adimentional drag coefficient (anisotrope)
        void SetDrag(const double cd_x, const double cd_y) {
            m_cd_x = cd_x;
            m_cd_y = cd_y;
        }


        /// Definition of the friction coefficient (-) (for tangential force component)
        void SetFriction(const double cf) { m_cf = cf; }

        /// Definition of the equivalent cylinder diameter (m)
        void SetDiameter(const double diameter) { m_diameter = diameter; }

        /// Return the equivalent cylinder diameter (m)
        double GetDiameter() const {return m_diameter; }

        /// Add the morison force to the offshore structure
        void AddForce(FrHydroBody* body) override;

        /// Define the body to which the morison model is applied
        void SetBody(FrHydroBody* body, bool add_force=false) override;

        /// Definition of the flow sensor
        void SetFlowSensor(FrHydroBody* body);

        /// Return the flow sensor
        FrFlowSensor* GetFlowSensor() { return m_flow.get(); }

        /// Return the force applied on the body
        chrono::ChVector<double> GetBodyForce() const override;

        /// Return the moment applied to the body
        chrono::ChVector<double> GetBodyTorque() const override;

        // UPDATE

        /// Update position of the element for not fixed structures
        void UpdateFrame();

        /// Update the morison model
        void UpdateState() override;

        /// Initialisation of the morison element
        void Initialize() override;

    private:
        /// Return the water density from environment
        inline double WaterDensity() const;


    };

    // -----------------------------------------------------------------------------
    // COMPOSITE ELEMENT
    // -----------------------------------------------------------------------------

    class FrCompositeElement : public FrMorisonModel {

    protected:
        std::vector<std::unique_ptr<FrMorisonModel>> m_morison;
        double m_cd = 1.;                            ///< Default value for drag coefficient
        double m_ca = 1.;                            ///< Default value for mass coefficient
        double m_cf = 0.;                            ///< Default value for friction coefficient
        double m_diameter = 0.;                      ///< Default value for diameter of the morison element (m)
        bool is_global_force = false;                ///< yes : use resultant force ; no : considerate each force separatly

    public:
        /// Default constructor
        FrCompositeElement();

        /// Define if the resulting force should be used instead of each force separatly
        void SetGlobalForce(const bool global_force) {
            is_global_force = global_force;
        }

        /// Return if the resultant force is used
        bool GetGlobalForce() const { return is_global_force; }

        /// Activation of the resulting force
        void ActivateGlobalForce() { is_global_force = true; }

        /// Add a new element to the morison model
        void AddElement(FrMorisonModel* element) {
            m_morison.push_back(std::unique_ptr<FrMorisonModel>(element));
        }

        /// Add a new element from position (isotrope)
        void AddElement(chrono::ChVector<> posA,
                        chrono::ChVector<> posB,
                        double diameter,
                        double ca,
                        double cd,
                        double cf);

        /// Add a new element from position (anisotrope)
        void AddElement(chrono::ChVector<> posA,
                        chrono::ChVector<> posB,
                        double diameter,
                        double ca_x, double ca_y,
                        double cd_x, double cd_y,
                        double cf);

        /// Add a new element from position using default morison property
        void AddElement(chrono::ChVector<> posA,
                        chrono::ChVector<> posB);

        /// Add a new element with node reference (isotrope)
        void AddElement(std::shared_ptr<FrNode>& nodeA,
                        std::shared_ptr<FrNode>& nodeB,
                        double diameter,
                        double ca,
                        double cd,
                        double cf);

        /// Add a new element with node reference (anisotrope)
        void AddElement(std::shared_ptr<FrNode>& nodeA,
                        std::shared_ptr<FrNode>& nodeB,
                        double diameter,
                        double ca_x, double ca_y,
                        double cd_x, double cd_y,
                        double cf);

        /// Add a new element with node reference using default morison property
        void AddElement(std::shared_ptr<FrNode>& nodeA,
                        std::shared_ptr<FrNode>& nodeB);

        void AddElement(chrono::ChVector<> posA, chrono::ChVector<> posB, const double dL,
                        double diameter, double ca, double cd, double cf);

        void AddElement(chrono::ChVector<> posA, chrono::ChVector<> posB, const double dL,
                        double diameter, double ca_x, double ca_y, double cd_x, double cd_y, double cf);

        /// Add new element with discretization (element size)
        void AddElement(chrono::ChVector<> posA, chrono::ChVector<> posB, const double dL);

        void AddElement(chrono::ChVector<> posA, chrono::ChVector<> posB, const int n,
                        double diameter, double ca, double cd, double cf);

        void AddElement(chrono::ChVector<> posA, chrono::ChVector<> posB, const int n,
                        double diameter, double ca_x, double ca_y, double cd_x, double cd_y, double cf);

        /// Add new element with discretization (number of element)
        void AddElement(chrono::ChVector<> posA, chrono::ChVector<> posB, const int n);

        /// Define the default value for drag coefficient
        void SetDefaultDrag(double cd) { m_cd = cd; }

        /// Get the default value for drag coefficient
        double GetDefaultDrag() const { return m_cd; }

        /// Define the default value for friction coefficient
        void SetDefaultFriction(double cf) { m_cf = cf; }

        /// Get the default value for friction coefficient
        double GetDefaultFriction() const { return m_cf; }

        /// Define the default value for mass coefficient
        void SetDefaultMass(double ca) { m_ca = ca; }

        /// Return the default mass coefficient
        double GetDefaultMass() const { return m_ca; }

        /// Define the default diameter
        void SetDefaultDiameter(double diameter) { m_diameter = diameter; }

        /// Return the default value of the diameter
        double GetDefaultDiameter() const { return m_diameter; }

        /// Set hydro body recursively to the morison model
        void SetBody(FrHydroBody* body, bool add_force=false) override {
            for (auto& element: m_morison) {
                element->SetBody(body, false);
            }
            this->AddForce(body);
        }

        /// Add force recursively to the morison model
        void AddForce(FrHydroBody* body) override;

        /// Compute the resulting force on the body
        chrono::ChVector<double> GetBodyForce() const override {
            chrono::ChVector<double> force(0.);
            for (auto& element : m_morison) {
                force += element->GetBodyForce();
            }
            return force;
        }

        /// Compute the resulting moment to the body
        chrono::ChVector<double> GetBodyTorque() const override {
            chrono::ChVector<double> moment(0.);
            for (auto& element: m_morison) {
                    moment += element->GetBodyTorque();
            }
            return moment;
        }

        void ActivateLog() {
            if (is_global_force) {
                is_log = true;
            } else {
                std::cout << "warning : cannot active log, global force is not active" << std::endl;
                is_log = false;
            }
        }

        void IncludeCurrent(const bool flag) override {
            for (auto& element: m_morison) {
                element->IncludeCurrent(flag);
            }
        }

        /// Update state of the morison element
        void UpdateState() override;

        /// Call initialisation of the element contained in composite element
        void Initialize() override;
    };


















    /// >>>>>>>>>>>>>>>>>>>>>>>>>>>><<< REFACTORING


    // --------------------------------------------------------------------------
    // MORISON ELEMENT
    // --------------------------------------------------------------------------

    class FrMorisonElement_ : public FrPhysicsItem_ {

    protected:
        bool m_fixStructure;
        bool m_includeCurrent;
        std::shared_ptr<FrNode_> m_frame;
        Force m_force;                      ///< Force at COG in world-coordinates
        Torque m_torque;                    ///< Torque at COG in body-coordinates

    public:
        ///
        /// \param fixStructure
        void SetFixStructure(bool fixStructure) { m_fixStructure = fixStructure; }

        ///
        /// \param includeCurrent
        virtual void SetIncludeCurrent(bool includeCurrent) { m_includeCurrent = includeCurrent; }

        void SetFrame(FrBody_* body, Position posA, Position posB, Direction vect = Direction(0., 0., 1.));

        void SetFrame(FrBody_* body, FrFrame_ frame);

        Force GetForceInWorld(FRAME_CONVENTION fc) const;

        Torque GetTorqueInBody() const;

    };


    // --------------------------------------------------------------------------
    // MORISON SINGLE ELEMENT
    // --------------------------------------------------------------------------

    struct MorisonCoeff {
        double x;
        double y;

        MorisonCoeff() { }

        MorisonCoeff(double val) {
            x = val;
            y = val;
        }

        MorisonCoeff& operator=(double val) {
            x = val;
            y = val;
            return *this;
        }
    };

    struct MorisonElementProperty {
        MorisonCoeff cd;
        MorisonCoeff ca;
        double cf;
        double diameter;
        double length;
        double volume;
    };


    class FrMorisonSingleElement_ : public FrMorisonElement_ {


    private:
        std::shared_ptr<FrNode_> m_nodeA;
        std::shared_ptr<FrNode_> m_nodeB;

        MorisonElementProperty m_property;
        bool m_extendedModel = false;


    public:
        FrMorisonSingleElement_(FrBody_* body);

        FrMorisonSingleElement_(std::shared_ptr<FrNode_>& nodeA,
                                std::shared_ptr<FrNode_>& nodeB,
                                double diameter, MorisonCoeff ca, MorisonCoeff cd, double cf,
                                Direction perpendicular = Direction(0., 0., 1.));

        FrMorisonSingleElement_(FrBody_* body, Position posA, Position posB,
                                double diameter, MorisonCoeff ca, MorisonCoeff cd, double cf,
                                Direction perpendicular = Direction(0., 0., 1.));

        FrMorisonSingleElement_(FrBody_* body, FrFrame_ frame, double diameter, double length,
                                MorisonCoeff ca, MorisonCoeff cd, double cf);

        void SetNodes(std::shared_ptr<FrNode_>& nodeA, std::shared_ptr<FrNode_>& nodeB);

        void SetNodes(FrBody_* body, Position posA, Position posB);

        void SetAddedMass(MorisonCoeff ca);

        void SetDragCoeff(MorisonCoeff cd);

        void SetFrictionCoeff(double cf);

        void SetDiameter(double diameter);

        double GetDiameter() const { return m_property.diameter; }

        double GetVolume() const { return m_property.volume; }

        double GetLength() const { return m_property.length; }

        void SetExtendedModel(bool extendedModel) { m_extendedModel = extendedModel; }


        //
        // UPDATE
        //

        void Update(double time) override;

        void Initialize() override;

        void StepFinalize() override;

    private:

        void SetLength(Position posA, Position posB);

        void SetLength(double length) { m_property.length = length; }

        void SetVolume();

        Velocity GetFlowVelocity();

        Acceleration GetFlowAcceleration();

    };

    // --------------------------------------------------------------------------
    // MORISON COMPOSITE ELEMENT
    // --------------------------------------------------------------------------

    class FrMorisonCompositeElement_ : public FrMorisonElement_ {


    protected:

        std::vector<std::unique_ptr<FrMorisonElement_>> m_morison;
        MorisonElementProperty m_property;
        bool m_isGlobal = false;

    public:

        FrMorisonCompositeElement_(FrBody_* body);

        FrMorisonCompositeElement_(FrBody_* body, FrFrame_& frame);

        void SetGlobal(bool isGlobal) { m_isGlobal = isGlobal; }

        bool GetGlobal() const { return m_isGlobal; }

        void AddElement(FrMorisonElement_* model) {
            m_morison.push_back(std::unique_ptr<FrMorisonElement_>(model));
        }

        void AddElement(std::shared_ptr<FrNode_> nodeA, std::shared_ptr<FrNode_> nodeB, double diameter,
                        MorisonCoeff ca, MorisonCoeff cd, double cf,
                        Direction perpendicular = Direction(0., 0, 1.));

        void AddElement(Position posA, Position posB, double diameter,
                        MorisonCoeff ca, MorisonCoeff cd, double cf, unsigned int n =1,
                        Direction perpendicular = Direction(0., 0., 1.));

        void AddElement(FrFrame_ frame, double diameter, double length,
                        MorisonCoeff ca, MorisonCoeff Cd, double cf);

        void SetDragCoeff(MorisonCoeff cd);

        MorisonCoeff GetDragCoeff() const { return m_property.cd; }

        void SetFrictionCoeff(double cf);

        double GetFrictionCoeff() const { return m_property.cf; }

        void SetAddedMass(MorisonCoeff ca);

        MorisonCoeff GetAddedMass() const { return m_property.ca; }

        //
        // UPDATE
        //

        void Update(double time) override;

        void Initialize() override;
    };















}

#endif //FRYDOM_FRMORISONMODEL_H
