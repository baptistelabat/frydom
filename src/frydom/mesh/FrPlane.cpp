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

#include "FrPlane.h"

namespace frydom {

  namespace geom {

    FrPlane::FrPlane(const Position &origin, const Direction &normal, FRAME_CONVENTION fc) : m_origin(origin),
                                                                                             m_normal(normal) {
      assert(std::abs(1. - m_normal.norm()) < 1E-8);
      if (IsNED(fc)) {
        internal::SwapFrameConvention(m_origin);
        internal::SwapFrameConvention(m_normal);
      }
      BuildFrame();
    }

    FrPlane::FrPlane(const std::vector<Position> &cloudPoint, FRAME_CONVENTION fc) {

      double a, b, c;
      a = b = c = 0;

      Position G;
      for (auto &point : cloudPoint) {
        G += point;
      }
      G /= cloudPoint.size();
      m_origin = G;

      m_normal.SetNull();

      for (int i = 0; i < cloudPoint.size() - 1; i++) {
        Position vec1 = G - cloudPoint[i];
        m_normal += vec1.cross(G - cloudPoint[i + 1]);
      }
      m_normal.normalize();

      if (IsNED(fc)) {
        internal::SwapFrameConvention(m_origin);
        internal::SwapFrameConvention(m_normal);
      }
      BuildFrame();

    }

    void FrPlane::SetOrigin(const Position &origin, FRAME_CONVENTION fc) {
      m_origin = origin;
      if (IsNED(fc)) internal::SwapFrameConvention(m_origin);
      BuildFrame();
    }

    Position FrPlane::GetOrigin(FRAME_CONVENTION fc) const {
      Position origin = m_origin;
      if (IsNED(fc)) internal::SwapFrameConvention(origin);
      return origin;
    }

    void FrPlane::GetOrigin(Position &origin, FRAME_CONVENTION fc) const {
      origin = GetOrigin(fc);
    }

    void FrPlane::SetNormal(const Direction &normal, FRAME_CONVENTION fc) {
      m_normal = normal;
      if (IsNED(fc)) internal::SwapFrameConvention(m_normal);
      BuildFrame();
    }

    Direction FrPlane::GetNormal(FRAME_CONVENTION fc) const {
      Position normal = m_normal;
      if (IsNED(fc)) internal::SwapFrameConvention(normal);
      return normal;
    }

    void FrPlane::GetNormal(Direction &normal, FRAME_CONVENTION fc) const {
      normal = GetNormal(fc);
    }

    FrFrame FrPlane::GetFrame() const {
      return m_frame;
    }

    double FrPlane::GetDistanceToPoint(Position PointInWorld, FRAME_CONVENTION fc) const {
      return std::abs(GetSignedDistanceToPoint(PointInWorld, fc));
    }


    double FrPlane::GetSignedDistanceToPoint(Position PointInWorld, FRAME_CONVENTION fc) const {

      Position vector = PointInWorld - GetOrigin(fc);

      return vector.dot(GetNormal(fc));
    }

    Position FrPlane::GetIntersectionWithLine(Position P0, Position P1, FRAME_CONVENTION fc) const {
      auto normale = GetNormal(fc);

      // check if P0P1 is parallel to the plan / or P0P1 null
      Direction line = (P1 - P0);
      assert(line.norm() > 1E-16);
      assert(line.dot(normale) != 0);

      // P0O
      Direction vector = GetOrigin(fc) - P0;

      // s_i
      double s = vector.dot(normale) / line.dot(normale);

      // P_i
      return P0 + (P1 - P0) * s;
    }

    Position FrPlane::GetClosestPointOnPlane(Position PointInWorld, FRAME_CONVENTION fc) const {
      return GetIntersectionWithLine(PointInWorld, PointInWorld + GetNormal(fc), fc);
    }

    void FrPlane::BuildFrame() {

//            m_frame.SetPosition(GetIntersectionWithLine(Position(0,0,0),Position(0,0,1),NWU), NWU);

      m_frame.SetPosition(GetClosestPointOnPlane(Position(), NWU), NWU);

      Direction z = m_normal;

      Direction x = m_normal.cross(Direction(0, -1, 0));
      x.normalize();

      if (x.norm() < 1E-8) x = m_normal.cross(Direction(1, 0, 0));
      Direction y = z.cross(x);
      y.normalize();

      Matrix33<double> matrix;
      matrix << x.Getux(), y.Getux(), z.Getux(),
          x.Getuy(), y.Getuy(), z.Getuy(),
          x.Getuz(), y.Getuz(), z.Getuz();

      FrUnitQuaternion quat;
      quat.Set(matrix, NWU);
//            quat.Set(matrix.transpose(), NWU);

      m_frame.SetRotation(quat);

    }

  } //end namespace frydom::mesh

}// end namespace frydom
