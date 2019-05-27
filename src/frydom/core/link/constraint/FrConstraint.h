//
// Created by lletourn on 22/05/19.
//

#ifndef FRYDOM_FRCONSTRAINT_H
#define FRYDOM_FRCONSTRAINT_H

#include <chrono/physics/ChLinkMate.h>
#include "frydom/core/link/FrLinkBase.h"

#include "frydom/core/common/FrConvention.h"

namespace frydom {

    class FrPoint;
    class FrAxis;
    class FrPlane;
    class Force;
    class Torque;
    class FrFrame;

    /**
     * \class FrConstraint
     * \brief Class to deal with constraints. Derived from FrLinkBase
     */
    class FrConstraint : public FrLinkBase {

    protected:

        std::shared_ptr<chrono::ChLink> m_chronoConstraint; ///< Chrono object handling the constraint

    public:

        /// Constraint constructor, requires 2 nodes, fixed in their respective body reference frame.
        /// \param node1 first node
        /// \param node2 second node
        /// \param system system in charge of the constraint
        FrConstraint(const std::shared_ptr<FrNode>& node1, const std::shared_ptr<FrNode>& node2, FrOffshoreSystem *system);

        /// Get the link reference frame, relatively to the world reference frame
        /// \return link reference frame, relatively to the world reference frame
        FrFrame GetLinkReferenceFrameInWorld() const;

        /// Get the link reference frame, relatively to the first body reference frame
        /// \return link reference frame, relatively to the first body reference frame
        FrFrame GetLinkReferenceFrameInBody1() const;

        /// Get the constraint reaction force (Body2 on Body1) in the constraint reference frame
        /// \param fc frame convention (NED/NWU)
        /// \return constraint reaction force (Body2 on Body1)
        Force GetForceInLink(FRAME_CONVENTION fc) const;

        /// Get the constraint reaction torque (Body2 on Body1) in the constraint reference frame at its origin
        /// \param fc frame convention (NED/NWU)
        /// \return constraint reaction torque (Body2 on Body1)
        Torque GetTorqueInLink(FRAME_CONVENTION fc) const;

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
        Torque GetTorqueInWorldAtLink(FRAME_CONVENTION fc) const;

        /// Tells if all constraints of this link are currently turned on or off by the user.
        bool IsDisabled() const override;;

        /// User can use this to enable/disable all the constraint of the link as desired.
        void SetDisabled(bool disabled) override;


        /// Tells if the link is currently active, in general,
        /// that is tells if it must be included into the system solver or not.
        /// This method cumulates the effect of various flags (so a link may
        /// be not active either because disabled, or broken, or not valid)
        bool IsActive() const override;

    protected:

        void AddFields() override;


        /// Get the embedded Chrono object
        std::shared_ptr<chrono::ChLink> GetChronoLink() override;

        chrono::ChLink* GetChronoItem_ptr() const override;

    };

    class FrConstraintParallel_ : public FrConstraint {

    private:

        const std::shared_ptr<FrAxis> m_axis1;
        const std::shared_ptr<FrAxis> m_axis2;

    public:

        FrConstraintParallel_(const std::shared_ptr<FrAxis>& axis1, const std::shared_ptr<FrAxis>& axis2, FrOffshoreSystem* system);

        /// Get the type name of this object
        /// \return type name of this object
        std::string GetTypeName() const override { return "ConstraintParallel"; }

        void Initialize() override;

    protected:

        chrono::ChLinkMateParallel* GetChronoItem_ptr() const override { return dynamic_cast<chrono::ChLinkMateParallel*>(m_chronoConstraint.get()); }

    };


    class FrConstraintPerpendicular : public FrConstraint {

    private:

        const std::shared_ptr<FrAxis> m_axis1;
        const std::shared_ptr<FrAxis> m_axis2;

    public:

        FrConstraintPerpendicular(const std::shared_ptr<FrAxis>& axis1, const std::shared_ptr<FrAxis>& axis2, FrOffshoreSystem* system);

        /// Get the type name of this object
        /// \return type name of this object
        std::string GetTypeName() const override { return "ConstraintPerpendicular"; }

        void Initialize() override;

    protected:

        chrono::ChLinkMateOrthogonal* GetChronoItem_ptr() const override { return dynamic_cast<chrono::ChLinkMateOrthogonal*>(m_chronoConstraint.get()); }

    };

    class FrConstraintPlaneOnPlane : public FrConstraint {

    private:

        const std::shared_ptr<FrPlane> m_plane1;
        const std::shared_ptr<FrPlane> m_plane2;

    public:

        FrConstraintPlaneOnPlane(const std::shared_ptr<FrPlane>& plane1, const std::shared_ptr<FrPlane>& plane2, FrOffshoreSystem* system);

        /// Get the type name of this object
        /// \return type name of this object
        std::string GetTypeName() const override { return "ConstraintPlaneOnPlane"; }

        void SetFlipped(bool flip);

        void SetDistance(double distance);

        void Initialize() override;

    protected:

        chrono::ChLinkMatePlane* GetChronoItem_ptr() const override { return dynamic_cast<chrono::ChLinkMatePlane*>(m_chronoConstraint.get()); }

    };


    class FrConstraintPointOnPlane : public FrConstraint {

    private:

        const std::shared_ptr<FrPoint> m_point;
        const std::shared_ptr<FrPlane> m_plane;

    public:

        FrConstraintPointOnPlane(const std::shared_ptr<FrPoint>& point,
                const std::shared_ptr<FrPlane>& plane,
                FrOffshoreSystem* system,
                double distance = 0.);

        /// Get the type name of this object
        /// \return type name of this object
        std::string GetTypeName() const override { return "ConstraintPointOnPlane"; }

        void Initialize() override;

        void SetDistance(double distance);

    protected:

        chrono::ChLinkMateXdistance* GetChronoItem_ptr() const override { return dynamic_cast<chrono::ChLinkMateXdistance*>(m_chronoConstraint.get()); }

    };


    class FrConstraintDistanceToAxis : public FrConstraint {

    private:

        const std::shared_ptr<FrPoint> m_point;
        const std::shared_ptr<FrAxis> m_axis;

        double m_distance;

    public:

        FrConstraintDistanceToAxis(const std::shared_ptr<FrPoint>& point,
                                 const std::shared_ptr<FrAxis>& axis,
                                 FrOffshoreSystem* system,
                                 double distance = 0.);

        /// Get the type name of this object
        /// \return type name of this object
        std::string GetTypeName() const override { return "ConstraintDistanceToAxis"; }

        void Initialize() override;

        void SetDistance(double distance);

        double GetDistance() const;

    protected:

        chrono::ChLinkRevoluteSpherical* GetChronoItem_ptr() const override { return dynamic_cast<chrono::ChLinkRevoluteSpherical*>(m_chronoConstraint.get()); }

    };


    class FrConstraintDistanceBetweenPoints : public FrConstraint {

    private:

        const std::shared_ptr<FrPoint> m_point1;
        const std::shared_ptr<FrPoint> m_point2;

        double m_distance;

    public:

        FrConstraintDistanceBetweenPoints(const std::shared_ptr<FrPoint>& point1,
                                   const std::shared_ptr<FrPoint>& point2,
                                   FrOffshoreSystem* system,
                                   double distance = 0.);

        /// Get the type name of this object
        /// \return type name of this object
        std::string GetTypeName() const override { return "ConstraintDistanceBetweenPoints"; }

        void Initialize() override;

        void SetDistance(double distance);

        double GetDistance() const;

    protected:

        chrono::ChLinkDistance* GetChronoItem_ptr() const override { return dynamic_cast<chrono::ChLinkDistance*>(m_chronoConstraint.get()); }

    };

    std::shared_ptr<FrConstraintDistanceBetweenPoints>
            make_constraint_distance_between_points(const std::shared_ptr<FrPoint>& point1,
                    const std::shared_ptr<FrPoint>& point2, FrOffshoreSystem* system, double distance = 0.);


} // end namespace frydom
#endif //FRYDOM_FRCONSTRAINT_H
