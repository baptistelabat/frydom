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

    class FrConstraint : public FrLinkBase {

    protected:

        std::shared_ptr<chrono::ChLinkMateGeneric> m_chronoConstraint; //TODO passer en ChLink pour avoir acces à ChLinkDistance?

    public:

        FrConstraint(const std::shared_ptr<FrNode>& node1, const std::shared_ptr<FrNode>& node2, FrOffshoreSystem *system);

        Force GetForceInNode2(FRAME_CONVENTION fc) const;
        Torque GetTorqueInNode2(FRAME_CONVENTION fc) const;

        Force GetForceInWorld(FRAME_CONVENTION fc) const;
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


        /// Get the embedded Chrono object
        std::shared_ptr<chrono::ChLink> GetChronoLink() override;

        chrono::ChLinkMateGeneric* GetChronoItem_ptr() const override;

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

        void SetFlipped(bool flip) { GetChronoItem_ptr()->SetFlipped(flip); }

        void SetDistance(double distance) { GetChronoItem_ptr()->SetSeparation(distance); }

        void Initialize() override;

    protected:

        chrono::ChLinkMatePlane* GetChronoItem_ptr() const override { return dynamic_cast<chrono::ChLinkMatePlane*>(m_chronoConstraint.get()); }


    };




} // end namespace frydom
#endif //FRYDOM_FRCONSTRAINT_H
