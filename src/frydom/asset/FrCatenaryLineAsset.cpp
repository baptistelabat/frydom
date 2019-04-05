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


#include "FrCatenaryLineAsset.h"

#include "chrono/assets/ChColor.h"
#include "chrono/assets/ChLineShape.h"

#include "frydom/cable/FrCatenaryLine.h"
#include "frydom/core/common/FrNode.h"


namespace frydom{

    FrCatenaryLineAsset::FrCatenaryLineAsset(FrCatenaryLine *line) : m_catenaryLine(line), FrAsset() {}

    void FrCatenaryLineAsset::Initialize() { // TODO : il semble que ChLine soit capable de rendre des lignes courbes

        // Generating line segments
        double ds = m_catenaryLine->GetUnstretchedLength() / m_catenaryLine->GetAssetElements();

        chrono::ChVector<double> p0, p1;
        chrono::ChColor color;
        p0 = internal::Vector3dToChVector(m_catenaryLine->GetStartingNode()->GetPositionInWorld(NWU));

        double s0 = 0.;
        double s1 = ds;
        
        auto index = m_chronoAsset->GetAssets().size();

        while (s1 < m_catenaryLine->GetUnstretchedLength()-ds) {

            p1 = internal::Vector3dToChVector(m_catenaryLine->GetAbsPosition(s1, NWU));
            auto newElement = std::make_shared<chrono::ChLineShape>();
            color = chrono::ChColor::ComputeFalseColor(m_catenaryLine->GetTension(s0, NWU).norm(), 0, m_maxTension, true);

            newElement->SetColor(color);
            newElement->SetLineGeometry(std::make_shared<chrono::geometry::ChLineSegment>(p0, p1));

            m_elements.push_back(make_triplet(s0, s1, index));
            m_chronoAsset->AddAsset(newElement);

            p0 = p1;
            s0 = s1;
            s1 += ds;
            index ++;
        }

        // For the last element
        auto last_ele = m_elements.back();
        if (std::get<1>(last_ele)<m_catenaryLine->GetUnstretchedLength()){
            p1 = internal::Vector3dToChVector(m_catenaryLine->GetEndingNode()->GetPositionInWorld(NWU));
            auto newElement = std::make_shared<chrono::ChLineShape>();
            color = chrono::ChColor::ComputeFalseColor(m_catenaryLine->GetEndingNodeTension(NWU).norm(), 0, m_maxTension, true);

            newElement->SetColor(color);
            newElement->SetLineGeometry(std::make_shared<chrono::geometry::ChLineSegment>(p0, p1));

            m_elements.push_back(make_triplet(s0, s1, index));
            m_chronoAsset->AddAsset(newElement);
        }

        InitRangeTensionColor();
    }

    void FrCatenaryLineAsset::StepFinalize(){

        chrono::ChVector<double> p0, p1;
        double s0, s1;

        p0 = internal::Vector3dToChVector(m_catenaryLine->GetStartingNode()->GetPositionInWorld(NWU));

        for (auto& element : m_elements) {

            s0 = std::get<0>(element);
            s1 = std::get<1>(element);
            auto lineShape = dynamic_cast<chrono::ChLineShape*> (m_chronoAsset->GetAssetN(std::get<2>(element)).get());

            auto lineSegment = std::dynamic_pointer_cast<chrono::geometry::ChLineSegment>(lineShape->GetLineGeometry());

            p1 = internal::Vector3dToChVector(m_catenaryLine->GetAbsPosition(s1, NWU));

            lineSegment->pA = p0;
            lineSegment->pB = p1;

            lineShape->SetColor(
                    chrono::ChColor::ComputeFalseColor(m_catenaryLine->GetTension(s0, NWU).norm(), 0, m_maxTension, false));

            p0 = p1;

        }
    }

    void FrCatenaryLineAsset::InitRangeTensionColor() {
        auto breakingTension = m_catenaryLine->GetBreakingTension();
        if (breakingTension>0){
            m_maxTension = breakingTension;
        }
        else{
            double ds = m_catenaryLine->GetUnstretchedLength()/ m_catenaryLine->GetAssetElements();
            double max = m_catenaryLine->GetTension(0, NWU).norm();
            for (int i=1; i< m_catenaryLine->GetAssetElements(); i++){
                auto LocalTension = m_catenaryLine->GetTension(i*ds, NWU).norm();
                if (LocalTension > max) max = LocalTension;
            }
            m_maxTension = 1.25*max;  // TODO : affiner le critere...
        }

    }

}
