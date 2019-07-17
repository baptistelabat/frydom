//
// Created by lletourn on 16/07/19.
//

#include "FrPlane.h"

namespace frydom {

    namespace geom {

        FrPlane::FrPlane(const Position &origin, const Direction &normal, FRAME_CONVENTION fc) :  m_origin(origin), m_normal(normal) {
            if (IsNED(fc)) {
                internal::SwapFrameConvention(m_origin);
                internal::SwapFrameConvention(m_normal);
            }
        }

        FrPlane::FrPlane(const std::vector<Position> &cloudPoint, FRAME_CONVENTION fc) {

            double a, b, c;
            a = b = c = 0;

            Position G;
            for (auto& point : cloudPoint) {
                G += point;
            }
            G /= cloudPoint.size();
            m_origin = G;

            Position prePoint = G;
            for (auto& point : cloudPoint){
                a += (prePoint.GetY() - point.GetY()) * (prePoint.GetZ() + point.GetZ());
                b += (prePoint.GetZ() - point.GetZ()) * (prePoint.GetX() + point.GetX());
                c += (prePoint.GetX() - point.GetX()) * (prePoint.GetY() + point.GetY());
            }
//            d = - a*G.GetX() - b*G.GetY() - c*G.GetZ();

            m_normal = {a, b, c};
            m_normal.normalize();

            if (IsNED(fc)) {
                internal::SwapFrameConvention(m_origin);
                internal::SwapFrameConvention(m_normal);
            }

        }

        Position FrPlane::GetOrigin(FRAME_CONVENTION fc) const {
            Position origin = m_origin;
            if (IsNED(fc)) internal::SwapFrameConvention(origin);
            return origin;
        }

        void FrPlane::GetOrigin(Position origin, FRAME_CONVENTION fc) const {
            origin = GetOrigin(fc);
        }

        Direction FrPlane::GetNormal(FRAME_CONVENTION fc) const {
            Position normal = m_normal;
            if (IsNED(fc)) internal::SwapFrameConvention(normal);
            return normal;
        }

        void FrPlane::GetNormal(Direction normal, FRAME_CONVENTION fc) const {
            normal = GetNormal(fc);
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
            Direction line = (P1 - P0); assert(line.norm()>1E-16);
            assert(line.dot(normale)!=0);

            // P0O
            Direction vector = GetOrigin(fc) - P0;

            // s_i
            double s = vector.dot(normale) / line.dot(normale);

            // P_i
            return P0 + (P1-P0) * s;
        }

        Position FrPlane::GetClosestPointOnPlane(Position PointInWorld, FRAME_CONVENTION fc) const {
            return GetIntersectionWithLine(PointInWorld, PointInWorld+GetNormal(fc), fc);
        }

    } //end namespace frydom::mesh

}// end namespace frydom