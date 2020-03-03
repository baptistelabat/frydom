// ==========================================================================
// FRyDoM - frydom-ce.org
//
// Copyright (c) Ecole Centrale de Nantes (LHEEA lab.) and D-ICE Engineering.
// All rights reserved.
//
// Use of this source code is governed by a GPLv3 license that can be found
// in the LICENSE file of FRyDoM.
//
// ==========================================================================


#ifndef FRYDOM_FRCATENARYLINEASSET_H
#define FRYDOM_FRCATENARYLINEASSET_H

#include <tuple>
#include <memory>
#include <vector>

#include "frydom/asset/FrAsset.h"
#include "frydom/asset/FrAssetOwner.h"

#include "frydom/core/math/FrVector.h"

// Chrono forward declaration
namespace chrono {
  class ChLineShape;
}

namespace frydom {

  // Forward declarations:
//    class FrCatenaryLine;
  class FrNode;

  class FrCatenaryAssetOwner : public FrAssetOwner {

    bool is_lineAsset = true;       ///< Is the line asset shown
    unsigned int m_nbElements = 40; ///< Numbers of asset elements depicted
    double m_maxTension = 0;            ///< maximum tension for visualization

   public:

    //--------------------------------------------------------------------------------------------------------------
    // Pure virtual methods, used in FrCatenaryAsset

    virtual double GetUnstretchedLength() const = 0;

    virtual Position GetPositionInWorld(const double &s, FRAME_CONVENTION fc) const = 0;

    virtual Force GetTension(const double &s, FRAME_CONVENTION fc) const = 0;

    //--------------------------------------------------------------------------------------------------------------

    void ShowAsset(bool show);

    bool IsAssetShown() const;

    void SetAssetElements(unsigned int nbElements);

    int GetAssetElements() const;

    void SetMaxTension(double max);

    double GetMaxTension() const;

   protected:

    virtual void Initialize();

    virtual void InitMaxTension();

  };


  /**
   * \class FrCatenaryLineAsset
   * \brief Class for a catenary line asset, using chrono::ChLineShape in aggregation
   * Line elements of ChLineShape are updated in position and color (related to the tension) in this class
   */
  class FrCatenaryLineAsset : public FrAsset {

   private:

    FrCatenaryAssetOwner *m_catenaryLine;    ///< Catenary line containing this asset

//        using Triplet = std::tuple<double, double, std::shared_ptr<chrono::ChLineShape>>;
//        std::vector<Triplet> m_elements;    ///< container of elements based on ChLineShape
    using Triplet = std::tuple<double, double, unsigned int>;
    std::vector<Triplet> m_elements;

    double m_maxTension = 0;            ///< max tension cached value for the color visualization

   public:

    /// Catenary line asset constructor
    /// \param line catenary line containing this asset
    explicit FrCatenaryLineAsset(FrCatenaryAssetOwner *line);

    /// Initialize the asset by creating the elements
    void Initialize() override;

    /// Update the state of the asset, at the end of a time step
    void StepFinalize() override;

   private:

    /// Initialize the max tension value for the color visualization
    void InitRangeTensionColor();

    /// Make a tuple, based on the starting and ending lagrangian coordinates and the corresponding element
    /// \param s0 starting lagrangian coordinate of the element
    /// \param s1 ending lagrangiand coordinate of the element
    /// \param index index of the element based on ChLineShape
    /// \return
    static Triplet make_triplet(double s0, double s1, unsigned int index) {
      return std::make_tuple(s0, s1, index);
    }

  };


}  //end namespace frydom

#endif //FRYDOM_FRCATENARYLINEASSET_H
