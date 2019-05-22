//
// Created by lletourn on 22/05/19.
//

#ifndef FRYDOM_FRCONSTRAINTPARALLEL_H
#define FRYDOM_FRCONSTRAINTPARALLEL_H

#include <chrono/physics/ChLinkMate.h>
#include "frydom/core/link/FrLinkBase.h"

namespace frydom {

    class FrAxis;

    class FrConstraintParallel : public FrLinkBase {

    private:
        std::shared_ptr<chrono::ChLinkMateParallel> m_chronoConstraint;

        FrAxis* m_axis1;
        FrAxis* m_axis2;

    public:

        FrConstraintParallel(FrAxis* axis1, FrAxis* axis2, FrOffshoreSystem* system);

        void Initialize() override;

        Force GetForceInWorld(FRAME_CONVENTION fc) const;

    };

} // end namespace frydom
#endif //FRYDOM_FRCONSTRAINTPARALLEL_H
