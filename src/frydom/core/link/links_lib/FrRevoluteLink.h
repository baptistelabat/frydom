//
// Created by frongere on 23/01/19.
//

#ifndef FRYDOM_FRREVOLUTELINK_H
#define FRYDOM_FRREVOLUTELINK_H

#include "FrLink.h"

namespace frydom {


    class FrRevoluteLink : public FrLink_ {

    private:
        double m_stiffness = 0.; ///> Link rotational stiffness (N)
        double m_damping = 0.;   ///> Link rotational damping (Nm/s)

    public:

        FrRevoluteLink(std::shared_ptr<FrNode_> node1, std::shared_ptr<FrNode_> node2, FrOffshoreSystem_* system);

        void SetSpringDamper(double stiffness, double damping);

        void SetRestAngle(double restAngle);

        double GetRestAngle() const;

        const Direction GetLinkAxisInWorld(FRAME_CONVENTION fc) const;

        double GetLinkAngle() const;

        double GetLinkAngularVelocity() const;

        double GetLinkAngularAcceleration() const;

        void Initialize() override;

        void Update(double time) override;

        void StepFinalize() override;


    };

    std::shared_ptr<FrRevoluteLink> make_revolute_link(std::shared_ptr<FrNode_> node1, std::shared_ptr<FrNode_> node2, FrOffshoreSystem_* system);

}  // end namespace frydom

#endif //FRYDOM_FRREVOLUTELINK_H
