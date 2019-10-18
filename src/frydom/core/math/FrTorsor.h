//
// Created by lletourn on 18/10/19.
//

#ifndef FRYDOM_FRTORSOR_H
#define FRYDOM_FRTORSOR_H

//#include "frydom/core/common/FrFrame.h"
#include "FrVector.h"

namespace frydom {

  class FrTorsor {

   public:

    FrTorsor(const mathutils::Vector3d<double>& resultant, const mathutils::Vector3d<double>& moment,const Position& point, FRAME_CONVENTION fc);

    Position GetPoint(FRAME_CONVENTION fc) const;


   protected:

    mathutils::Vector3d<double> m_resultant;  ///< resultant of the torsor
    mathutils::Vector3d<double> m_moment;     ///< moment of the torsor, expressed at the point
    Position m_point;                         ///< Application point, where the torsor is expressed

    mathutils::Vector3d<double> TransportMomentAtPoint(const Position &newPoint, FRAME_CONVENTION fc) const;

//    FrFrame m_frame;

  };



  class FrGeneralizedForceTorsor : public FrTorsor {

   public:

    FrGeneralizedForceTorsor(const Force& force, const Torque& torque, const Position& point, FRAME_CONVENTION fc);

    Force GetForce() const;

    Torque GetTorqueAtPoint(const Position &point, FRAME_CONVENTION fc) const;

  };



  class FrGeneralizedVelocityTorsor : public FrTorsor {

   public:

    FrGeneralizedVelocityTorsor(const Velocity& linearvelocity, const AngularVelocity& angularVelocity, const Position& point, FRAME_CONVENTION fc);

    Force GetAngularVelocity() const;

    Torque GetLinearVelocityAtPoint(const Position &point, FRAME_CONVENTION fc) const;

  };

} //end namespace frydom

#endif //FRYDOM_FRTORSOR_H
