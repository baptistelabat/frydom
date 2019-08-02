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

#ifndef FRYDOM_FRPLANE_H
#define FRYDOM_FRPLANE_H

#include <memory>

#include "frydom/core/math/FrVector.h"
#include "frydom/core/common/FrFrame.h"

namespace frydom{

    namespace geom {

        /**
         * Class defining geometrically a plane, based on a point, considered as the plane origin and a normal vector
         */
        class FrPlane {

        public:

            /// Constructor for a plane based on a point on the plane, considered as the origin and a normal vector
            /// \param origin point on the plane
            /// \param normal normal vector to the plane
            /// \param fc frame convention (NED/NWU)
            FrPlane(const Position& origin, const Direction& normal, FRAME_CONVENTION fc);

            /// Constructor for a plane based on a cloud of points, supposed to be planar
            /// \param cloudPoints cloud of points, supposed planar
            /// \param fc frame convention (NED/NWU)
            FrPlane(const std::vector<Position> &cloudPoints, FRAME_CONVENTION fc);

            /// Set the origin of the plane (a point on the plane)
            /// \param origin plane origin
            /// \param fc frame convention (NED/NWU)
            void SetOrigin(const Position& origin, FRAME_CONVENTION fc);

            /// Get the point defined as the plane origin
            /// \param fc frame convention (NED/NWU)
            /// \return position of the origin
            Position GetOrigin(FRAME_CONVENTION fc) const;

            /// Get the point defined as the plane origin
            /// \param origin position of the origin
            /// \param fc frame convention (NED/NWU)
            void GetOrigin(Position &origin, FRAME_CONVENTION fc) const;

            /// Set the normal vector to the plane
            /// \param normal normal vector
            /// \param fc frame convention (NED/NWU)
            void SetNormal(const Direction &normal, FRAME_CONVENTION fc);

            /// Get the normal vector to the plane
            /// \param fc frame convention (NED/NWU)
            /// \return normal vector
            Direction GetNormal(FRAME_CONVENTION fc) const;

            /// Get the normal vector to the plane
            /// \param normal normal vector
            /// \param fc frame convention (NED/NWU)
            void GetNormal(Direction &normal, FRAME_CONVENTION fc) const;

            /// Get the plane reference frame, built from the origin and normal vector
            /// \return plane reference frame
            FrFrame GetFrame() const;

            /// Get the distance from the plane to a point given in the world reference frame
            /// \param PointInWorld position of a given point
            /// \param fc frame convention (NED/NWU)
            /// \return distance from the plane to the point
            double GetDistanceToPoint(Position PointInWorld, FRAME_CONVENTION fc) const;

            /// Get the signed distance from the plane to a point given in the world reference frame
            /// \param PointInWorld position of a given point
            /// \param fc frame convention (NED/NWU)
            /// \return signed distance from the plane to the point
            double GetSignedDistanceToPoint(Position PointInWorld, FRAME_CONVENTION fc) const;

            /// Get the intersection point of the plane and a line, defined by two points, given in the world reference frame
            /// \param P0 first point on the line
            /// \param P1 second point on the line
            /// \param fc frame convention (NED/NWU)
            /// \return position of the intersection point
            Position GetIntersectionWithLine(Position P0, Position P1, FRAME_CONVENTION fc) const;

            /// Get the closest point on the plane of a point given in the word reference frame
            /// \param PointInWorld given point
            /// \param fc frame convention (NED/NWU)
            /// \return position of the closest point
            Position GetClosestPointOnPlane(Position PointInWorld, FRAME_CONVENTION fc) const;


        private:

            /// Build the plane reference frame, based on the plane origin and normal vector. Directions of tangential vectors are arbitrary
            void BuildFrame();

            Position m_origin;          ///< plane origin
            Direction m_normal;         ///< plane normal
            FrFrame m_frame;            ///< plane reference frame

        };

    } //end namespace frydom::mesh

}// end namespace frydom
#endif //FRYDOM_FRPLANE_H
