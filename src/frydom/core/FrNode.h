//
// Created by frongere on 08/09/17.
//

#ifndef FRYDOM_FRNODE_H
#define FRYDOM_FRNODE_H

#include "chrono/physics/ChMarker.h"
#include "FrBody.h"


namespace frydom {

//    class FrBody;

    class FrNode : public chrono::ChMarker{
//    private:
//        FrBody* Body;

    public:

//        FrBody* GetBody() const { return Body; }

        FrNode() : chrono::ChMarker() {
//            SetMotionType(M_MOTION_EXTERNAL); // Hack to keep the relative position unchanged
        }

//        FrNode(char myname[], FrBody* myBody,
//               chrono::Coordsys myrel_pos, chrono::Coordsys myrel_pos_dt, chrono::Coordsys myrel_pos_dtdt)
//            : chrono::ChMarker(myname, myBody, myrel_pos, myrel_pos_dt, myrel_pos_dtdt) {
//            SetMotionType(M_MOTION_EXTERNAL); // Hack to keep the relative position unchanged
//        }

        chrono::ChVector<double> GetAbsPos() {
            return GetAbsCoord().pos;
        }

//        std::shared_ptr<FrBody> GetSharedBody() {
//            return Body->shared_from_this();
//        }

        chrono::Coordsys GetRelPos() const {
            return GetRest_Coord();
        }


    };  // end class FrNode

}  // end namespace frydom


#endif //FRYDOM_FRNODE_H
