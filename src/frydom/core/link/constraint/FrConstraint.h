//
// Created by lletourn on 22/05/19.
//

#ifndef FRYDOM_FRCONSTRAINT_H
#define FRYDOM_FRCONSTRAINT_H

#include <chrono/physics/ChLinkMate.h>
#include "frydom/core/link/FrLinkBase.h"

#include "frydom/core/common/FrConvention.h"

namespace frydom {

    template<typename OffshoreSystemType>
    class FrCPoint;

    template<typename OffshoreSystemType>
    class FrCAxis;

    template<typename OffshoreSystemType>
    class FrCPlane;

    class Force;

    class Torque;

    class FrFrame;

    /**
     * \class FrConstraint
     * \brief Class to deal with constraints. Derived from FrConstraintBase
     */
    template<typename OffshoreSystemType>
    class FrConstraint : public FrLinkBase<OffshoreSystemType> {

     protected:

      std::shared_ptr<chrono::ChLink> m_chronoConstraint; ///< Chrono object handling the constraint

     public:

      /// Constraint constructor, requires 2 nodes, fixed in their respective body reference frame.
      /// \param node1 first node
      /// \param node2 second node
      /// \param system system in charge of the constraint
      FrConstraint(const std::shared_ptr<FrNode<OffshoreSystemType>> &node1,
                   const std::shared_ptr<FrNode<OffshoreSystemType>> &node2,
                   FrOffshoreSystem<OffshoreSystemType> *system);

      /// Get the constraint reference frame, relatively to the world reference frame
      /// \return constraint reference frame, relatively to the world reference frame
      FrFrame GetConstraintReferenceFrameInWorld() const;

      /// Get the constraint reference frame, relatively to the first body reference frame
      /// \return constraint reference frame, relatively to the first body reference frame
      FrFrame GetConstraintReferenceFrameInBody1() const;

      /// Get the constraint reaction force (Body2 on Body1) in the constraint reference frame
      /// \param fc frame convention (NED/NWU)
      /// \return constraint reaction force (Body2 on Body1)
      Force GetForceInConstraint(FRAME_CONVENTION fc) const;

      /// Get the constraint reaction torque (Body2 on Body1) in the constraint reference frame at its origin
      /// \param fc frame convention (NED/NWU)
      /// \return constraint reaction torque (Body2 on Body1)
      Torque GetTorqueInConstraint(FRAME_CONVENTION fc) const;

      /// Get the constraint reaction force (Body2 on Body1) in the first body reference frame
      /// \param fc frame convention (NED/NWU)
      /// \return constraint reaction force (Body2 on Body1)
      Force GetForceInBody1(FRAME_CONVENTION fc) const;

      /// Get the constraint reaction torque (Body2 on Body1) in the first body reference frame at its COG
      /// \param fc frame convention (NED/NWU)
      /// \return constraint reaction torque (Body2 on Body1)
      Torque GetTorqueInBody1AtCOG(FRAME_CONVENTION fc) const;


      /// Get the constraint reaction force (Body2 on Body1) in the world reference frame
      /// \param fc frame convention (NED/NWU)
      /// \return constraint reaction force (Body2 on Body1)
      Force GetForceInWorld(FRAME_CONVENTION fc) const;

      /// Get the constraint reaction torque (Body2 on Body1) in the world reference frame at constraint reference
      /// frame origin
      /// \param fc frame convention (NED/NWU)
      /// \return constraint reaction torque (Body2 on Body1)
      Torque GetTorqueInWorldAtConstraint(FRAME_CONVENTION fc) const;

      /// Tells if all constraints of this constraint are currently turned on or off by the user.
      bool IsDisabled() const override;;

      /// User can use this to enable/disable all the constraint of the constraint as desired.
      void SetDisabled(bool disabled) override;

      /// Tells if the constraint is currently active, in general,
      /// that is tells if it must be included into the system solver or not.
      /// This method cumulates the effect of various flags (so a constraint may
      /// be not active either because disabled, or broken, or not valid)
      bool IsActive() const override;

     protected:

      /// Add the fields to the Hermes message
      void AddFields() override;

      /// Get the embedded Chrono object
      std::shared_ptr<chrono::ChLink> GetChronoLink() override;

      /// Get the pointer to the chrono constraint
      /// \return pointer to the chrono constraint
      chrono::ChLink *GetChronoItem_ptr() const override;

    };

    //------------------------------------------------------------------------------------------------------------------

    /**
     * \class FrConstraintParallel
     * \brief Class implementing a parallel constraint between two axis. Derived from FrConstraint
     * The constraint is defined based on two FrAxis.
     */
    template<typename OffshoreSystemType>
    class FrConstraintParallel : public FrConstraint<OffshoreSystemType> {

     private:

      const std::shared_ptr<FrCAxis<OffshoreSystemType>> m_axis1;  ///< first axis to be constrained
      const std::shared_ptr<FrCAxis<OffshoreSystemType>> m_axis2;  ///< second axis to be constrained

     public:

      /// Constructor for a parallel constraint between two axis
      /// \param axis1 first axis to be constrained
      /// \param axis2 second axis to be constrained
      /// \param system system to add the constrain
      FrConstraintParallel(const std::shared_ptr<FrCAxis<OffshoreSystemType>> &axis1,
                           const std::shared_ptr<FrCAxis<OffshoreSystemType>> &axis2,
                           FrOffshoreSystem<OffshoreSystemType> *system);

      /// Get the type name of this object
      /// \return type name of this object
      std::string GetTypeName() const override { return "ConstraintParallel"; }

      /// Initialize the constraint
      void Initialize() override;

     protected:

      /// Get the pointer to the chrono constraint
      /// \return pointer to the chrono constraint
      chrono::ChLinkMateParallel *
      GetChronoItem_ptr() const override { return dynamic_cast<chrono::ChLinkMateParallel *>(this->m_chronoConstraint.get()); }

    };

    /// maker for a parallel constraint
    /// \param axis1 first axis
    /// \param axis2 second axis
    /// \param system system to add the constraint
    /// \return parallel constraint
    template<typename OffshoreSystemType>
    std::shared_ptr<FrConstraintParallel<OffshoreSystemType>>
    make_constraint_parallel(
        const std::shared_ptr<FrCAxis<OffshoreSystemType>> &axis1,
        const std::shared_ptr<FrCAxis<OffshoreSystemType>> &axis2,
        FrOffshoreSystem<OffshoreSystemType> *system);


    //------------------------------------------------------------------------------------------------------------------

    /**
     * \class FrConstraintPerpendicular
     * \brief Class implementing a perpendicular constraint between two axis. Derived from FrConstraint
     * The constraint is defined based on two FrAxis.
     */
    template<typename OffshoreSystemType>
    class FrConstraintPerpendicular : public FrConstraint<OffshoreSystemType> {

     private:

      const std::shared_ptr<FrCAxis<OffshoreSystemType>> m_axis1;  ///< first axis to be constrained
      const std::shared_ptr<FrCAxis<OffshoreSystemType>> m_axis2;  ///< second axis to be constrained

     public:

      /// Constructor for a perpendicular constraint between two axis
      /// \param axis1 first axis to be constrained
      /// \param axis2 second axis to be constrained
      /// \param system system to add the constrain
      FrConstraintPerpendicular(const std::shared_ptr<FrCAxis<OffshoreSystemType>> &axis1,
                                const std::shared_ptr<FrCAxis<OffshoreSystemType>> &axis2,
                                FrOffshoreSystem<OffshoreSystemType> *system);

      /// Get the type name of this object
      /// \return type name of this object
      std::string GetTypeName() const override { return "ConstraintPerpendicular"; }

      /// Initialize the constraint
      void Initialize() override;

     protected:

      /// Get the pointer to the chrono constraint
      /// \return pointer to the chrono constraint
      chrono::ChLinkMateOrthogonal *
      GetChronoItem_ptr() const override { return dynamic_cast<chrono::ChLinkMateOrthogonal *>(this->m_chronoConstraint.get()); }

    };

    /// maker for a perpendicular constraint
    /// \param axis1 first axis
    /// \param axis2 second axis
    /// \param system system to add the constraint
    /// \return perpendicular constraint
    template<typename OffshoreSystemType>
    std::shared_ptr<FrConstraintPerpendicular<OffshoreSystemType>>
    make_constraint_perpendicular(
        const std::shared_ptr<FrCAxis<OffshoreSystemType>> &axis1,
        const std::shared_ptr<FrCAxis<OffshoreSystemType>> &axis2,
        FrOffshoreSystem<OffshoreSystemType> *system);


    //------------------------------------------------------------------------------------------------------------------

    /**
     * \class FrConstraintPlaneOnPlane
     * \brief Class implementing a plane on plane constraint. Derived from FrConstraint
     * The constraint is defined based on two FrPlane.
     */
    template<typename OffshoreSystemType>
    class FrConstraintPlaneOnPlane : public FrConstraint<OffshoreSystemType> {

     private:

      const std::shared_ptr<FrCPlane<OffshoreSystemType>> m_plane1;    ///< first plane to constrain
      const std::shared_ptr<FrCPlane<OffshoreSystemType>> m_plane2;    ///< second plane to constrain

     public:

      /// Constructor for a plane on plane constraint
      /// \param plane1 first plane to constrain
      /// \param plane2 second plane to constrain
      /// \param system system to add the constraint
      /// \param flipped if true, the normale of the first plane is flipped
      /// \param distance distance separating the two planes
      FrConstraintPlaneOnPlane(
          const std::shared_ptr<FrCPlane<OffshoreSystemType>> &plane1,
          const std::shared_ptr<FrCPlane<OffshoreSystemType>> &plane2,
          FrOffshoreSystem<OffshoreSystemType> *system,
          bool flipped = false,
          double distance = 0.);

      /// Get the type name of this object
      /// \return type name of this object
      std::string GetTypeName() const override { return "ConstraintPlaneOnPlane"; }

      /// Initialize the constraint
      void Initialize() override;

      /// Set if the normale of the first plan is to be flipped
      /// \param flip true if the normale of the first plan is to be flipped
      void SetFlipped(bool flip);

      /// Set the distance between the two planes
      /// \param distance distance between the two planes, in m
      void SetDistance(double distance);

     protected:

      /// Get the pointer to the chrono constraint
      /// \return pointer to the chrono constraint
      chrono::ChLinkMatePlane *
      GetChronoItem_ptr() const override { return dynamic_cast<chrono::ChLinkMatePlane *>(this->m_chronoConstraint.get()); }

    };

    /// maker for a plane on plane constraint
    /// \param plane1 first plane
    /// \param plane2 second plane
    /// \param system system to add the constraint
    /// \param flipped if true, the normale of the first plan is flipped
    /// \param distance distance separating the two planes
    /// \return plane on plane constraint
    template<typename OffshoreSystemType>
    std::shared_ptr<FrConstraintPlaneOnPlane<OffshoreSystemType>>
    make_constraint_plane_on_plane(
        const std::shared_ptr<FrCPlane<OffshoreSystemType>> &plane1,
        const std::shared_ptr<FrCPlane<OffshoreSystemType>> &plane2,
        FrOffshoreSystem<OffshoreSystemType> *system,
        bool flipped = false,
        double distance = 0.);


    //------------------------------------------------------------------------------------------------------------------

    /**
     * \class FrConstraintPointOnPlane
     * \brief Class implementing a point on plane constraint. Derived from FrConstraint
     * The constraint is defined based on a FrPlane and a FrPoint and a distance in option
     */
    template<typename OffshoreSystemType>
    class FrConstraintPointOnPlane : public FrConstraint<OffshoreSystemType> {

     private:

      const std::shared_ptr<FrCPoint<OffshoreSystemType>> m_point; ///< point constrained to stay on the plane
      const std::shared_ptr<FrCPlane<OffshoreSystemType>> m_plane; ///< plane on which the point is constrained

     public:

      /// Constructor of the point on plane constraint
      /// \param plane plane on which the point is constrained
      /// \param point point constrained to stay on the plane
      /// \param system system to add the constraint
      /// \param distance distance to the plane
      FrConstraintPointOnPlane(const std::shared_ptr<FrCPlane<OffshoreSystemType>> &plane,
                               const std::shared_ptr<FrCPoint<OffshoreSystemType>> &point,
                               FrOffshoreSystem<OffshoreSystemType> *system,
                               double distance = 0.);

      /// Get the type name of this object
      /// \return type name of this object
      std::string GetTypeName() const override { return "ConstraintPointOnPlane"; }

      /// Initialize the constraint
      void Initialize() override;

      /// Set the distance between the point and the plane
      /// \param distance distance between the point and the plane, in m
      void SetDistance(double distance);

     protected:

      /// Get the pointer to the chrono constraint
      /// \return pointer to the chrono constraint
      chrono::ChLinkMateXdistance *
      GetChronoItem_ptr() const override { return dynamic_cast<chrono::ChLinkMateXdistance *>(this->m_chronoConstraint.get()); }

    };

    /// maker for a point on plane constraint
    /// \param plane plane
    /// \param point point
    /// \param system system to add the constraint
    /// \param distance distance to the plane
    /// \return point on plane constraint
    template<typename OffshoreSystemType>
    std::shared_ptr<FrConstraintPointOnPlane<OffshoreSystemType>>
    make_constraint_point_on_plane(
        const std::shared_ptr<FrCPlane<OffshoreSystemType>> &plane,
        const std::shared_ptr<FrCPoint<OffshoreSystemType>> &point,
        FrOffshoreSystem<OffshoreSystemType> *system,
        double distance = 0.);

    //------------------------------------------------------------------------------------------------------------------

    /**
     * \class FrConstraintPointOnLine
     * \brief Class implementing a point on line constraint. Derived from FrConstraint
     * The constraint is defined based on a FrAxis and a FrPoint and a distance in option
     */
    template<typename OffshoreSystemType>
    class FrConstraintPointOnLine : public FrConstraint<OffshoreSystemType> {

     private:

      const std::shared_ptr<FrCPoint<OffshoreSystemType>> m_point; ///< point constrained to stay on the plane
      const std::shared_ptr<FrCAxis<OffshoreSystemType>> m_axis;   ///< axis on which the point is constrained

     public:

      /// Constructor of the point on plane constraint
      /// \param line line on which the point is constrained
      /// \param point point constrained to stay on the line
      /// \param system system to add the constraint
      /// \param distance distance to the line
      FrConstraintPointOnLine(const std::shared_ptr<FrCAxis<OffshoreSystemType>> &line,
                              const std::shared_ptr<FrCPoint<OffshoreSystemType>> &point,
                              FrOffshoreSystem<OffshoreSystemType> *system,
                              double distance = 0.);

      /// Get the type name of this object
      /// \return type name of this object
      std::string GetTypeName() const override { return "ConstraintPointOnLine"; }

      /// Initialize the constraint
      void Initialize() override;

     protected:
      friend class FrNode<OffshoreSystemType>; // To make possible to declare SetMarkers friend in FrNode

      /// Get the pointer to the chrono constraint
      /// \return pointer to the chrono constraint
      chrono::ChLinkLockPointLine *
      GetChronoItem_ptr() const override { return dynamic_cast<chrono::ChLinkLockPointLine *>(this->m_chronoConstraint.get()); }

    };

    /// maker for a point on plane constraint
    /// \param line line on which the point is constrained
    /// \param point point constrained to stay on the line
    /// \param system system to add the constraint
    /// \param distance distance to the line
    /// \return point on plane constraint
    template<typename OffshoreSystemType>
    std::shared_ptr<FrConstraintPointOnLine<OffshoreSystemType>>
    make_constraint_point_on_line(
        const std::shared_ptr<FrCAxis<OffshoreSystemType>> &line,
        const std::shared_ptr<FrCPoint<OffshoreSystemType>> &point,
        FrOffshoreSystem<OffshoreSystemType> *system);

    //------------------------------------------------------------------------------------------------------------------

    /**
     * \class FrConstraintDistanceToAxis
     * \brief Class implementing a distance constraint between a point to an axis. Derived from FrConstraint
     * The constraint is defined based on a FrPoint, an FrAxis around which the point will rotate and a distance.
     */
    template<typename OffshoreSystemType>
    class FrConstraintDistanceToAxis : public FrConstraint<OffshoreSystemType> {

     private:

      const std::shared_ptr<FrCPoint<OffshoreSystemType>> m_point;     ///< point to constraint
      const std::shared_ptr<FrCAxis<OffshoreSystemType>> m_axis;       ///< axis around which constraining the point
      bool m_autoDistance;                        ///< if true, the imposed distance is the distance from the point
      ///< to the axis as given
      double m_distance;                          ///< distance to constraint the point around the axis

     public:

      /// Constructor to a distance constraint from a point to an axis.
      /// \param axis axis around which to constrain the point
      /// \param point point to be constrained
      /// \param system system to add the constraint
      /// \param autoDistance if true, the imposed distance is the distance from the point to the axis, as given
      /// \param distance distance to constraint the point around the axis
      FrConstraintDistanceToAxis(const std::shared_ptr<FrCAxis<OffshoreSystemType>> &axis,
                                 const std::shared_ptr<FrCPoint<OffshoreSystemType>> &point,
                                 FrOffshoreSystem<OffshoreSystemType> *system,
                                 bool autoDistance,
                                 double distance = 0.);

      /// Get the type name of this object
      /// \return type name of this object
      std::string GetTypeName() const override { return "ConstraintDistanceToAxis"; }

      /// Initialize the constraint
      void Initialize() override;

      /// Set the distance between the point and the axis
      /// \param distance distance between the point and the axis, in m
      void SetDistance(double distance);

      /// Get the distance between the point and the axis
      /// \return distance between the point and the axis, in m
      double GetDistance() const;

     protected:

      /// Get the pointer to the chrono constraint
      /// \return pointer to the chrono constraint
      chrono::ChLinkRevoluteSpherical *
      GetChronoItem_ptr() const override { return dynamic_cast<chrono::ChLinkRevoluteSpherical *>(this->m_chronoConstraint.get()); }

    };

    /// maker for a distance constraint between a point to an axis
    /// \param axis axis
    /// \param point point
    /// \param system system to add the constraint
    /// \param autoDistance if true, the imposed distance is the distance between the two given points
    /// \param distance distance to be imposed between the two points
    /// \return distance constraint between a point to an axis
    template<typename OffshoreSystemType>
    std::shared_ptr<FrConstraintDistanceToAxis<OffshoreSystemType>>
    make_constraint_distance_to_axis(
        const std::shared_ptr<FrCAxis<OffshoreSystemType>> &axis,
        const std::shared_ptr<FrCPoint<OffshoreSystemType>> &point,
        FrOffshoreSystem<OffshoreSystemType> *system,
        bool autoDistance,
        double distance = 0.);


    //------------------------------------------------------------------------------------------------------------------

    /**
     * \class FrConstraintDistanceBetweenPoints
     * \brief Class implementing a distance constraint between two points. Derived from FrConstraint
     * The constraint is defined based on two FrPoint and a distance.
     */
    template<typename OffshoreSystemType>
    class FrConstraintDistanceBetweenPoints : public FrConstraint<OffshoreSystemType> {

     private:

      const std::shared_ptr<FrCPoint<OffshoreSystemType>> m_point1;    ///< first point
      const std::shared_ptr<FrCPoint<OffshoreSystemType>> m_point2;    ///< second point
      bool m_autoDistance;                        ///< distance between the points is computed automatically
      double m_distance;                          ///< distance to be constrained between the points

     public:

      /// Constructor of the distance contraint between tow points
      /// \param point1 first point
      /// \param point2 second point
      /// \param system system to add the constrain
      /// \param autoDistance if true, initializes the imposed distance as the distance between the two points
      /// \param distance distance between the two points
      FrConstraintDistanceBetweenPoints(const std::shared_ptr<FrCPoint<OffshoreSystemType>> &point1,
                                        const std::shared_ptr<FrCPoint<OffshoreSystemType>> &point2,
                                        FrOffshoreSystem<OffshoreSystemType> *system,
                                        bool autoDistance = false,
                                        double distance = 0.);

      /// Get the type name of this object
      /// \return type name of this object
      std::string GetTypeName() const override { return "ConstraintDistanceBetweenPoints"; }

      /// Initialize the constraint
      void Initialize() override;

      /// Set the distance between the two points
      /// \param distance distance between the two points, in m
      void SetDistance(double distance);

      /// Get the distance between the two points
      /// \return distance between the two points, in m
      double GetDistance() const;

     protected:

      /// Get the pointer to the chrono constraint
      /// \return pointer to the chrono constraint
      chrono::ChLinkDistance *
      GetChronoItem_ptr() const override { return dynamic_cast<chrono::ChLinkDistance *>(this->m_chronoConstraint.get()); }

    };

    /// maker for a distance constraint between two points
    /// \param point1 first point
    /// \param point2 second point
    /// \param system system to add the constraint
    /// \param autoDistance if true, the imposed distance is the distance between the two given points
    /// \param distance distance to be imposed between the two points
    /// \return distance constraint between two points
    template<typename OffshoreSystemType>
    std::shared_ptr<FrConstraintDistanceBetweenPoints<OffshoreSystemType>> make_constraint_distance_between_points(
        const std::shared_ptr<FrCPoint<OffshoreSystemType>> &point1,
        const std::shared_ptr<FrCPoint<OffshoreSystemType>> &point2,
        FrOffshoreSystem<OffshoreSystemType> *system,
        bool autoDistance,
        double distance = 0.);


} // end namespace frydom
#endif //FRYDOM_FRCONSTRAINT_H
