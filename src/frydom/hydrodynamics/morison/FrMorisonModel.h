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


#ifndef FRYDOM_FRMORISONMODEL_H
#define FRYDOM_FRMORISONMODEL_H

// <<<<<<<<<<<<< Head

#include <vector>
#include <frydom/core/common/FrPhysicsItem.h>

#include "frydom/core/common/FrConvention.h"
#include "frydom/core/common/FrNode.h"
#include "frydom/environment/ocean/freeSurface/waves/FrWaveField.h"
#include "frydom/environment/ocean/freeSurface/waves/FrFlowSensor.h"

// <<<<<<<<<<<<< Adding header for refactoring

#include "frydom/core/force/FrForce.h"


namespace frydom {

    // Forward declaration
    class FrMorisonCompositeElement_;

    /// Maker for a Morison model : instantiate and return a FrMorisonCompositeElement
    /// \param body body related to the morison model
    /// \return Morison model, as a Morison composite element
    std::shared_ptr<FrMorisonCompositeElement_> make_morison_model(FrBody_ *body);


    // --------------------------------------------------------------------------
    // MORISON ELEMENT
    // --------------------------------------------------------------------------

    /// This class is a base class for morison model with only one single element or composite elements.
    class FrMorisonElement_ {

    protected:
        std::shared_ptr<FrNode_> m_frame;   ///< Frame at the center position of the morison element with z-axis along its direction
        Force m_force;                      ///< Force at COG of the body in world-coordinates
        Torque m_torque;                    ///< Torque at COG of the body in body-coordinates

        bool m_includeCurrent;              ///< Include current flow in morison element model
        bool m_extendedModel = false;                   ///< If true the inertial component of the morison force is used (false by dafault)

    public:
        /// Set the local frame of the morison model from node positions
        /// \param body Body to which the frame is attached
        /// \param posA Position of the first extremity of the morison element
        /// \param posB Position of the second extremity of the morison element
        /// \param vect x-axis of the frame is built such as is perpendicular to the morison element direction and this vector
        void SetFrame(FrBody_* body, Position posA, Position posB, Direction vect = Direction(0., 0., 1.));

        /// Set the local frame of the morison model from another frame
        /// \param body Body to which the frame is attached
        /// \param frame Other frame
        void SetFrame(FrBody_* body, const FrFrame_& frame);

        /// Get the local frame of the morison model
        /// \return Local frame
        FrFrame_ GetFrame() const { return m_frame->GetFrameInWorld(); }

        /// Get the local frame of the morison model as node
        /// \return Node
        std::shared_ptr<FrNode_> GetNode() const { return m_frame; }

        /// Get the force vector at COG in world reference frame
        /// \param fc Frame convention
        /// \return Force vector
        Force GetForceInWorld(FRAME_CONVENTION fc) const;

        /// Get the torque vector at COG in body reference frame
        /// \return Torque vector
        Torque GetTorqueInBody() const;

        /// Include current flow in the morison model
        /// \param includeCurrent Boolean, if true the current is included in morison model
        virtual void SetIncludeCurrent(bool includeCurrent) { m_includeCurrent = includeCurrent; }

        /// Defines if the inertial component with added mass is used (false by default)
        /// \param extendedModel Boolean, if true inertial component is used
        void SetExtendedModel(bool extendedModel) { m_extendedModel = extendedModel; }

        /// Update the force of the morison model
        /// \param time Current time of the simulation from begining (in seconds)
        virtual void Update(double time) = 0;

        /// Initialize the morison model
        virtual void Initialize() = 0;

        /// Method to be applied at the end of each time step
        virtual void StepFinalize() = 0;

    };


    // --------------------------------------------------------------------------
    // MORISON SINGLE ELEMENT
    // --------------------------------------------------------------------------

    /// Morison coefficient structure used to allow isotropic or anisotropic coefficients definition
    /// for the morison model
    struct MorisonCoeff {
        double x;
        double y;

        MorisonCoeff() { }

        MorisonCoeff(double val) {
            x = val;
            y = val;
        }

        MorisonCoeff(double val1, double val2) {
            x = val1;
            y = val2;
        }

        MorisonCoeff& operator=(double val) {
            x = val;
            y = val;
            return *this;
        }

    };

    /// The MorisonElementProperty structure encapsulate the basics property of a morison model
    struct MorisonElementProperty {
        MorisonCoeff cd = 0.;                   ///< Drag coefficient (can be isotropic or anisotropic)
        MorisonCoeff ca = 0.;                   ///< Added mass (can be isotropic ar anisotropic)
        double cf       = 0.;                   ///< Friction coefficient
        double diameter = 0.;                   ///< Diameter of the morison model, in meter
        double length   = 0.;                   ///< Length of the morison model, in meter
        double volume   = 0.;                   ///< Volume of the morison model, in m³
    };

    /// This class defines a morison model.
    /// It can be instanciate when the morison model is composed by only one single element
    /// The pointer to the body must be specified before to create a new morison model with single element
    class FrMorisonSingleElement_ : public FrMorisonElement_ {

    protected:
        std::shared_ptr<FrNode_> m_nodeA;               ///< First extremity node of the morison element
        std::shared_ptr<FrNode_> m_nodeB;               ///< Second extremity node of the morison element

        MorisonElementProperty m_property;              ///< Container of the morison property (diameter, drag coeff...)


    public:
        /// Constructor of a new morison element without property definition
        /// \param body Body to which the morison force is applied
        FrMorisonSingleElement_(FrBody_* body);

        /// Constructor of a new morison element with property
        /// \param nodeA First extremity node of the morison element
        /// \param nodeB Second extremity node of the morison element
        /// \param diameter Diameter, in m
        /// \param ca Added mass coefficient
        /// \param cd Drag coefficient
        /// \param cf Friction coefficient
        /// \param perpendicular x-axis is built such as is perpendicular to the morison element direction and this vector
        FrMorisonSingleElement_(std::shared_ptr<FrNode_>& nodeA,
                                std::shared_ptr<FrNode_>& nodeB,
                                double diameter, MorisonCoeff ca, MorisonCoeff cd, double cf,
                                Direction perpendicular = Direction(0., 0., 1.));

        /// Constructor of a new element with property
        /// \param body Body to which the morison element force is applied
        /// \param posA First extremity position of the morison element
        /// \param posB Second extremity position of the morison element
        /// \param diameter Diameter of the morison element
        /// \param ca Added mass
        /// \param cd Drag coefficient
        /// \param cf Friction coefficient
        /// \param perpendicular x-axis is built such as is perpendicular to the morison element direction and this vector
        FrMorisonSingleElement_(FrBody_* body, Position posA, Position posB,
                                double diameter, MorisonCoeff ca, MorisonCoeff cd, double cf,
                                Direction perpendicular = Direction(0., 0., 1.));

        /// Constructor of a new element with property
        /// \param body Body to which the morison element force is applied
        /// \param frame Frame placed at the middle of the morison element with z-axis defining its direction
        /// \param diameter Diameter of the morison element
        /// \param length Length of the morison element
        /// \param ca Added mass
        /// \param cd Drag coefficients
        /// \param cf Friction coefficient
        FrMorisonSingleElement_(FrBody_* body, FrFrame_ frame, double diameter, double length,
                                MorisonCoeff ca, MorisonCoeff cd, double cf);

        /// Defines nodes at the extremities of the morison element
        /// \param nodeA First extremity node
        /// \param nodeB Second extremity node
        void SetNodes(std::shared_ptr<FrNode_>& nodeA, std::shared_ptr<FrNode_>& nodeB);

        /// Defines nodes from position and body link
        /// \param body Body to which the nodes are linked
        /// \param posA Position of the first node
        /// \param posB Position of the second node
        void SetNodes(FrBody_* body, Position posA, Position posB);

        /// Set the added mass of the morison element (only if extended version is used)
        /// \param ca Added mass coefficients (double of {double, double})
        void SetAddedMass(MorisonCoeff ca);

        /// Set the drag coefficient of the morison element
        /// \param cd Drag coefficient (double or {double, double})
        void SetDragCoeff(MorisonCoeff cd);

        /// Set the firiction coefficient of the morison element (force along the local z-axis)
        /// \param cf Friction coefficient
        void SetFrictionCoeff(double cf);

        /// Set the diameter of the morison element
        /// \param diameter Diameter in meter
        void SetDiameter(double diameter);

        /// Get the diameter of the morison element
        /// \return Diameter in meter
        double GetDiameter() const { return m_property.diameter; }

        /// Get the volume of the morison element
        /// \return Volume in m^3
        double GetVolume() const { return m_property.volume; }

        /// Get the length of the morison element
        /// \return length in meter
        double GetLength() const { return m_property.length; }

        //
        // UPDATE
        //

        /// Update the force of the morison element
        /// \param time Current time of the simulation from begining
        void Update(double time) override;

        /// Initialize the morison model
        void Initialize() override;

        /// Methods to be applied at the end of each time step
        void StepFinalize() override;

    protected:

        /// Set length of the morison element from node positions
        /// \param posA First extremity position of the element
        /// \param posB Second extremity position of the element
        void SetLength(Position posA, Position posB);

        /// Set length of the morison element
        /// \param length Length in meter
        void SetLength(double length) { m_property.length = length; }

        /// Set volume of the morison element
        void SetVolume();

        /// Get the relative flow velocity at frame position in local frame
        /// \return  Flow velocity
        Velocity GetFlowVelocity();

        /// Get the relative flow acceleration at frame position in local frame
        /// \return Flow acceleration
        Acceleration GetFlowAcceleration();
    };

    // --------------------------------------------------------------------------
    // MORISON COMPOSITE ELEMENT
    // --------------------------------------------------------------------------

    /// This class is used to build a composite morison model with multiple morison element.
    /// Composition of composite model can also be set up from this class.
    /// The resultant force and torque are the sum of the force and torque of each morison model component
    /// computed at the center of gravity of the body. The force is expressed in the world coordinates system
    /// and the torque in the body coordinate system.
    class FrMorisonCompositeElement_ : public FrMorisonElement_ {

    protected:
        std::vector<std::unique_ptr<FrMorisonElement_>> m_morison;      ///< morison model components of the composite model
        MorisonElementProperty m_property = MorisonElementProperty();   ///< default element properties

    public:
        /// Constructor of a new composite model of the morison force
        /// \param body Body to which the morison model is applied
        FrMorisonCompositeElement_(FrBody_* body);

        /// Constructor of a new composite model of the morison force with frame local frame
        /// \param body BOdy to which the morison model is applied
        /// \param frame Local frame
        FrMorisonCompositeElement_(FrBody_* body, FrFrame_& frame);

        /// Add a new morison model to the composite model
        /// \param model Morison model (can be single or composite elements)
        void AddElement(FrMorisonElement_* model) {
            m_morison.push_back(std::unique_ptr<FrMorisonElement_>(model));
        }

        /// Add a new single element to the composite model from nodes
        /// \param nodeA Node at the first extremity of the morison element
        /// \param nodeB Node at the second extremity of the morison element
        /// \param diameter Diameter of the morison element
        /// \param ca Added mass coefficient
        /// \param cd Drag coefficient
        /// \param cf Friction coefficient
        /// \param perpendicular x-axis is built such as is perpendicular to the morison element direction and this vector
        void AddElement(std::shared_ptr<FrNode_>& nodeA, std::shared_ptr<FrNode_>& nodeB,
                        double diameter, MorisonCoeff ca,  MorisonCoeff cd, double cf,
                        Direction perpendicular = Direction(0., 0, 1.));

        /// Add a new single element to the composite model from nodes with default morison properties
        /// \param nodeA Node at the first extremity of the morison model
        /// \param nodeB Node at the second extremity of the morison model
        /// \param perpendicular x-axis is built such as is perpendicular to the morison element direction and this vector
        void AddElement(std::shared_ptr<FrNode_>& nodeA, std::shared_ptr<FrNode_>& nodeB,
                        Direction perpendicular = Direction(0., 0, 1.));

        /// Add a new single element to the composite model from positions
        /// \param posA Position of the first extremity of the morison element
        /// \param posB Position of the second extremity of the morison element
        /// \param diameter Diamter of the morison element
        /// \param ca Added mass coefficient
        /// \param cd Drag coefficient
        /// \param cf Friction coefficient
        /// \param n Number of discrete elements along the morison element
        /// \param perpendicular x-axis is built such as is perpendicular to the morison element direction and this vector
        void AddElement(Position posA, Position posB, double diameter,
                        MorisonCoeff ca, MorisonCoeff cd,  double cf, unsigned int n =1,
                        Direction perpendicular = Direction(0., 0., 1.));

        /// Add a new single element to the composite model from position with default morison properties
        /// \param posA Position of the first extremity of the morison element
        /// \param posB Position of the second extremity of the morison element
        /// \param n Number of discrete elements along the morison element
        /// \param perpendicular x-axis is built such as is perpendicular to the morison element direction and this vector
        void AddElement(Position posA, Position posB, unsigned int n =1,
                        Direction perpendicular = Direction(0., 0., 1.));

        /// Add a new single element to the composite model from frame definition
        /// \param frame Local frame of the morison element
        /// \param length Length of the morison element
        /// \param diameter Diameter of the morison element
        /// \param ca Added mass coefficient
        /// \param Cd Drag coefficient
        /// \param cf Friction coefficient
        void AddElement(FrFrame_ frame, double length, double diameter,  MorisonCoeff ca, MorisonCoeff Cd, double cf);

        /// Add a new single element to the composite model from frame definition with default properties
        /// \param frame Local frame of the morison element
        /// \param length Length of the morison element
        void AddElement(FrFrame_ frame, double length);

        /// Set the default drag coefficient for composite element
        /// \param cd Drag coefficient
        void SetDragCoeff(MorisonCoeff cd);

        /// Get the default drag coefficient
        /// \return Drag coefficient
        MorisonCoeff GetDragCoeff() const { return m_property.cd; }

        /// Set the defautl friction coefficient for composite element
        /// \param cf Friction coefficient
        void SetFrictionCoeff(double cf);

        /// Get the default friction coefficient
        /// \return Friction coefficient
        double GetFrictionCoeff() const { return m_property.cf; }

        /// Set the default added mass for composite element
        /// \param ca Added mass
        void SetAddedMass(MorisonCoeff ca);

        /// Set the default diameter for composite element
        /// \param diameter Diameter in meter
        void SetDiameter(double diameter);

        /// Get the default diameter for composite element
        /// \return Diameter in meter
        double GetDiameter() const { return m_property.diameter; }

        /// Get the default added mass coefficient for composite element
        /// \return Added mass
        MorisonCoeff GetAddedMass() const { return m_property.ca; }

        //
        // UPDATE
        //

        /// Update the force of the morison composite model
        /// \param time Current time of the simulation from begining
        void Update(double time) override;

        /// Initialize the morion composite model
        void Initialize() override;

        /// Methods to be applied at the end of each time step of the simulation
        void StepFinalize() override { }
    };

}  // end namespace frydom

#endif //FRYDOM_FRMORISONMODEL_H
