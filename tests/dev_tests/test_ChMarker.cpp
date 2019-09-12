//
// Created by lletourn on 12/09/19.
//

#include "chrono/core/ChLog.h"
#include "chrono/physics/ChMarker.h"
#include "chrono/physics/ChBodyAuxRef.h"


using namespace chrono;

int main() {

    ChBodyAuxRef body;

    body.SetFrame_REF_to_abs(ChFrame<>(ChVector<>(1.,2.,3.)));
    GetLog()<<body.GetFrame_REF_to_abs().GetPos()<<"Frame_REF_to Abs \n";

    body.SetFrame_COG_to_REF(ChFrame<>(ChVector<>(5.,3.,1.)));
    GetLog()<<body.GetFrame_COG_to_abs().GetPos()<<"Frame_COG_to Abs \n";


//    auto marker = std::make_shared<ChMarker>("marker0", &body,ChCoordsys<>(ChVector<>(4.,1.,2.)),ChCoordsys<>(),ChCoordsys<>());
    auto marker = std::make_shared<ChMarker>();
    body.AddMarker(marker);

    marker->Impose_Rel_Coord(ChCoordsys<>(ChVector<>(4.,1.,2.)));

    GetLog()<<marker->GetPos()<<"Marker rel pos \n";
    GetLog()<<marker->GetAbsFrame().GetPos()<<"Marker abs pos \n";


    body.SetFrame_COG_to_REF(ChFrame<>(ChVector<>(8.,4.,2.)));

    GetLog()<<body.GetFrame_COG_to_abs().GetPos()<<"Frame_COG_to Abs \n";
    GetLog()<<marker->GetPos()<<"Marker rel pos \n";
    GetLog()<<marker->GetAbsFrame().GetPos()<<"Marker abs pos \n";

}