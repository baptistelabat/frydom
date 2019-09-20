//
// Created by lletourn on 06/03/19.
//

#ifndef FRYDOM_FRNODEASSET_H
#define FRYDOM_FRNODEASSET_H

#include "frydom/asset/FrAsset.h"

namespace frydom {

    // Forward declaration
    template <typename OffshoreSystemType>
    class FrNode;


    /**
     * \class FrNodeAsset
     * \brief Class to display a reference frame.
     */
     template <typename OffshoreSystemType>
    class FrNodeAsset : public FrAsset {

    private:

        FrNode<OffshoreSystemType>* m_node;
        double m_CharacteristicLength;

    public:

        explicit FrNodeAsset(FrNode<OffshoreSystemType>* node);

        void SetSize(double size);;

        void Initialize() override;

        void StepFinalize() override;

    };

} // end namespace frydom

#endif //FRYDOM_FRNODEASSET_H
