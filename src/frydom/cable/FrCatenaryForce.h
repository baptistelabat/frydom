//
// Created by frongere on 08/09/17.
//

#ifndef FRYDOM_FRCATENARYFORCE_H
#define FRYDOM_FRCATENARYFORCE_H

#include "frydom/core/FrForce.h"

namespace frydom {

    class FrCatenaryLine;

    enum line_side {
        LINE_START,
        LINE_END
    };

    class FrCatenaryForce : public FrForce {

    private:
        FrCatenaryLine* m_line; ///< The parent line
        line_side m_line_side;  ///< The side of the line where the tension is applied

    public:

        FrCatenaryForce(FrCatenaryLine* p_line, line_side p_side) : m_line(p_line), m_line_side(p_side) {};

        void UpdateState() override;

    };

}  // end namespace frydom


#endif //FRYDOM_FRCATENARYFORCE_H
