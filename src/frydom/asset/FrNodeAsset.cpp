//
// Created by lletourn on 06/03/19.
//

#include "FrNodeAsset.h"

#include "chrono/assets/ChGlyphs.h"

#include "frydom/core/common/FrNode.h"


namespace frydom {

    template <typename OffshoreSystemType>
    FrNodeAsset<OffshoreSystemType>::FrNodeAsset(frydom::FrNode<OffshoreSystemType> *node) : m_node(node), m_CharacteristicLength(1.),
                                                               FrAsset() {
    }
    template <typename OffshoreSystemType>
    void FrNodeAsset<OffshoreSystemType>::Initialize() {

        auto glyphAsset = std::make_shared<chrono::ChGlyphs>();
        glyphAsset->SetDrawMode(chrono::ChGlyphs::eCh_GlyphType::GLYPH_COORDSYS);

        chrono::ChCoordsys<double> nodeFrame = internal::FrFrame2ChCoordsys(m_node->GetFrameInBody());

        glyphAsset->SetGlyphCoordsys(0,nodeFrame);

        glyphAsset->SetGlyphsSize(m_CharacteristicLength);  // Ne semble pas avoir d'effet dans Irrlicht

        m_chronoAsset->AddAsset(glyphAsset);
    }
    template <typename OffshoreSystemType>
    void FrNodeAsset<OffshoreSystemType>::StepFinalize() {

        // Get the glyph asset form the AssetLevel
        auto GlyphAsset = dynamic_cast<chrono::ChGlyphs*> (m_chronoAsset->GetAssetN(0).get());

        GlyphAsset->SetGlyphsSize(m_CharacteristicLength);

        // Here, the asset point is automatically following the motion but the force has to be updated
        chrono::ChCoordsys<double> nodeFrame = internal::FrFrame2ChCoordsys(m_node->GetFrameInBody());

        GlyphAsset->SetGlyphCoordsys(0,nodeFrame);

    }
    template <typename OffshoreSystemType>
    void FrNodeAsset<OffshoreSystemType>::SetSize(double size) {
        m_CharacteristicLength = size;
    }

}
