//
// Created by frongere on 10/10/17.
//

#ifndef FRYDOM_FRDYNAMICCABLE_H
#define FRYDOM_FRDYNAMICCABLE_H

#include <memory>

#include "chrono_fea/ChBeamSection.h"
#include "chrono_fea/ChElementBeamANCF.h"

using namespace chrono::fea;

namespace frydom {

    class FrDynamicCable {

    private:
        std::shared_ptr<ChMesh> m_mesh;
        std::shared_ptr<ChBeamSectionCable> m_section;


    public:
        FrDynamicCable() = default;

        // TODO: faire constructeur par copie



        void SetDiameter(const double diameter) {}
        void SetYoungModulus(const double E) {}
        void SetRayleighDamping(const double d) {}

        void SetStartingPoint(chrono::ChVector<double> P0) {}
        void SetEndingPoint(chrono::ChVector<double> P1) {}

        





    };

}

#endif //FRYDOM_FRDYNAMICCABLE_H
