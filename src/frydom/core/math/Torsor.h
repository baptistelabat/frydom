//
// Created by lletourn on 18/10/19.
//

#ifndef FRYDOM_TORSOR_H
#define FRYDOM_TORSOR_H

//#include "frydom/core/common/FrFrame.h"
#include "FrVector.h"

namespace frydom {

  class Torsor {

   public:

    Torsor(const mathutils::Vector3d<double> &resultant, const mathutils::Vector3d<double> &moment,
           const Position &point, FRAME_CONVENTION fc);

    Position GetPoint(FRAME_CONVENTION fc) const;


   protected:

    void
    Set(const mathutils::Vector3d<double> &resultant, const mathutils::Vector3d<double> &moment, const Position &point,
        FRAME_CONVENTION fc);

    mathutils::Vector3d<double> m_resultant;  ///< resultant of the torsor
    mathutils::Vector3d<double> m_moment;     ///< moment of the torsor, expressed at the point
    Position m_point;                         ///< Application point, where the torsor is expressed

    mathutils::Vector3d<double> TransportMomentAtPoint(const Position &newPoint, FRAME_CONVENTION fc) const;

//    FrFrame m_frame;

   private:

    friend std::ostream &operator<<(std::ostream &os, const Torsor &torsor);

    std::ostream &cout(std::ostream &os) const;
  };


  class GeneralizedForceTorsor : public Torsor {

   public:

    GeneralizedForceTorsor(const Force &force, const Torque &torque, const Position &point, FRAME_CONVENTION fc);

    void Set(const Force &force, const Torque &torque, const Position &point, FRAME_CONVENTION fc);

    Force GetForce() const;

    Torque GetTorqueAtPoint(const Position &point, FRAME_CONVENTION fc) const;

  };


  class GeneralizedVelocityTorsor : public Torsor {

   public:

    GeneralizedVelocityTorsor(const Velocity &linearvelocity, const AngularVelocity &angularVelocity,
                              const Position &point, FRAME_CONVENTION fc);

    void Set(const Velocity &linearvelocity, const AngularVelocity &angularVelocity, const Position &point,
             FRAME_CONVENTION fc);

    Force GetAngularVelocity() const;

    Torque GetLinearVelocityAtPoint(const Position &point, FRAME_CONVENTION fc) const;

  };

} //end namespace frydom

#endif //FRYDOM_TORSOR_H
