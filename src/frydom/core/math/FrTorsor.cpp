//
// Created by lletourn on 18/10/19.
//

#include "FrTorsor.h"

namespace frydom {


  FrTorsor::FrTorsor(const mathutils::Vector3d<double> &resultant, const mathutils::Vector3d<double> &moment,
                     const Position &point, FRAME_CONVENTION fc) :
      m_resultant(resultant), m_moment(moment), m_point(point){
    if (IsNED(fc)) {
      internal::SwapFrameConvention(m_resultant);
      internal::SwapFrameConvention(m_moment);
      internal::SwapFrameConvention(m_point);
    }
  }

  mathutils::Vector3d<double> FrTorsor::TransportMomentAtPoint(const Position &newPoint, FRAME_CONVENTION fc) const {
    Position tempPos = newPoint;
    if(IsNED(fc)) internal::SwapFrameConvention(tempPos);
    Position newToOld = m_point - tempPos;
    mathutils::Vector3d<double> tempMoment = m_moment + newToOld.cross(m_resultant);
    if (IsNED(fc)) internal::SwapFrameConvention(tempMoment);
    return tempMoment;
  }

  Position FrTorsor::GetPoint(FRAME_CONVENTION fc) const {
    Position tempPos = m_point;
    if(IsNED(fc)) internal::SwapFrameConvention(tempPos);
    return tempPos;
  }





  // FrGeneralizedForceTorsor

  FrGeneralizedForceTorsor::FrGeneralizedForceTorsor(const Force &force, const Torque &torque, const Position &point, FRAME_CONVENTION fc) :
  FrTorsor(force, torque, point, fc) {  }

  Force FrGeneralizedForceTorsor::GetForce() const {
    return m_resultant;
  }

  Torque FrGeneralizedForceTorsor::GetTorqueAtPoint(const Position &point, FRAME_CONVENTION fc) const {
    return TransportMomentAtPoint(point, fc);
  }





  // FrGeneralizedVelocityTorsor

  FrGeneralizedVelocityTorsor::FrGeneralizedVelocityTorsor(const Velocity &linearvelocity,
                                                           const AngularVelocity &angularVelocity,
                                                           const Position &point, FRAME_CONVENTION fc) :
                                                           FrTorsor(angularVelocity, linearvelocity, point, fc){

  }

  Force FrGeneralizedVelocityTorsor::GetAngularVelocity() const {
    return m_resultant;
  }

  Torque FrGeneralizedVelocityTorsor::GetLinearVelocityAtPoint(const Position &point, FRAME_CONVENTION fc) const {
    return TransportMomentAtPoint(point, fc);
  }
} //end namespace frydom