//
// Created by lucas on 23/01/19.
//

#ifndef FRYDOM_FRCATENARYLINEASSET_H
#define FRYDOM_FRCATENARYLINEASSET_H

#include <tuple>
#include <memory>
#include <vector>

namespace chrono {
    class ChLineShape;
}

namespace frydom {

    // Forward declarations:
    class FrCatenaryLine_;


    class FrCatenaryLineAsset_ {

    private:

        FrCatenaryLine_ *m_catenaryLine;

        using Triplet = std::tuple<double, double, std::shared_ptr<chrono::ChLineShape>>;
        std::vector<Triplet> m_elements;

        double m_maxTension = 0;

    public:

        explicit FrCatenaryLineAsset_(FrCatenaryLine_ * line);

        void Update();

        void Initialize();

    private:

        void InitRangeTensionColor();

        static Triplet make_triplet(double s0, double s1, std::shared_ptr<chrono::ChLineShape> lineShape) {
            return std::make_tuple(s0, s1, lineShape);
        }
    };
}//end namespace frydom
#endif //FRYDOM_FRCATENARYLINEASSET_H
