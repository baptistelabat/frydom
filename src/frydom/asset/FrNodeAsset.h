//
// Created by lletourn on 06/03/19.
//

#ifndef FRYDOM_FRNODEASSET_H
#define FRYDOM_FRNODEASSET_H

#include "frydom/asset/FrAsset.h"

namespace frydom {

  // Forward declaration
  class FrNode;


  /**
   * \class FrNodeAsset
   * \brief Class to display a reference frame.
   */
  class FrNodeAsset : public FrAsset {

   private:

    FrNode *m_node;
    double m_CharacteristicLength;

   public:

    explicit FrNodeAsset(FrNode *node);

    void SetSize(double size);;

    void Initialize() override;

    void StepFinalize() override;

  };

} // end namespace frydom

#endif //FRYDOM_FRNODEASSET_H
