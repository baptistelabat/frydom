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

#ifndef FRYDOM_DICE_MESHCLIPPER_H
#define FRYDOM_DICE_MESHCLIPPER_H

#include <memory>
#include "FrMesh.h"
#include "frydom/environment/ocean/freeSurface/FrFreeSurface.h"
#include "frydom/core/body/FrBody.h"
#include "MathUtils/MathUtils.h"

namespace frydom {

    namespace mesh {

        /**
        * \class FrClippingSurface
        * \brief Class for dealing with the clipping incident wave field.
        */
        class FrClippingSurface {

        protected:
            double m_meanHeight = 0.;
            double m_ThresholdDichotomy = 1e-4;

            Position m_bodyPosition = {0.,0.,0.};

        public:

            /// Constructor.
            FrClippingSurface() = default;

            /// Constructor and initialization of the mean height.
            explicit FrClippingSurface(double meanHeight);

            /// This function sets the mean height of the incident wave field.
            void SetMeanHeight(const double &meanHeight);

            /// Set the body position in world reference frame, for correction in ClippingWaveSurface::GetDistance
            /// \param bodyPos
            void SetBodyPosition(Position bodyPos);

            /// This function gives the distance to the incident wave field.
            virtual double GetDistance(const FrMesh::Point &point) const = 0;

            /// This function gives the intersection node position between an edge and an incident wave field.
            virtual FrMesh::Point GetIntersection(const FrMesh::Point &p0, const FrMesh::Point &p1) = 0;

        };

        /**
        * \class FrClippingPlane
        * \brief Class used when the clipping incident wave field is a HORIZONTAL plane.
        */
        class FrClippingPlane : public FrClippingSurface {

        public:

            /// Constructor.
            FrClippingPlane() = default;

            /// Constructor.
            explicit FrClippingPlane(double meanHeight);

            /// This function gives the distance to the plane.
            inline double GetDistance(const FrMesh::Point &point) const override {

                // This function gives the distance between the node and the clipping plane.

                /*
                 * TODO: idee, lorsqu'on fait une requete de distance des vertex, on peut deja effectuer des projections !!
                 * Du coup c'est fait en preliminaire avant les decoupes. Ca permet a priori d'epargner pas mal de clip et de
                 * facettes pourries en plus. Si on a projete, la distance est nulle et on renvoie 0...
                 * Si on a pas projete, la distance est on nulle et on classifie le vertex above ou under.
                 * La classification est faite par la methode ClassifyVertex !! elle ne doit pas etre faite ici
                 * Par contre, il semblerait qu'on soit oblige de passer le maillage en argument :/ afin de pouvoir
                */

                // TODO: projeter !

                // It is useless because it does not depend on the horizontal position of the mesh.

                return point[2] - m_meanHeight;
            }

            /// This function gives the intersection node position between an edge and the plane.
            FrMesh::Point GetIntersection(const FrMesh::Point &p0, const FrMesh::Point &p1) override;

        };

        /**
        * \class FrClippingWavesSurface
        * \brief Class for clipping a mesh by an arbitrary incident wave field.
        */
        class FrClippingWaveSurface : public FrClippingSurface {

        private:

            /// FreeSurface.
            FrFreeSurface* m_freesurface;

        public:

            /// Constructor.
            FrClippingWaveSurface() = default;

            /// Constructor.
            explicit FrClippingWaveSurface(const double &meanHeight, FrFreeSurface* FreeSurface);

            /// This function gives the distance to the incident wave.
            double GetDistance(const FrMesh::Point &point) const override;

            /// This function performs a bisection method to track the intersection node.
            FrMesh::Point GetIntersection(const FrMesh::Point &p0, const FrMesh::Point &p1) override;
        };

        class FrMeshClipper {

        private:

            /// Initial mesh.
            FrMesh* m_mesh;

            /// Clipping surface, by default the plane z = 0.
            //TODO: INCORRECT
            std::unique_ptr<FrClippingSurface> m_clippingSurface;

            double m_Threshold = 1e-4;
            double m_ProjectionThresholdRatio = 1 / 4.;

            /// Vector to store the faces which are on and/or above the incident free surface and have to be deleted.
            std::vector<FrMesh::FaceHandle> c_FacesToDelete;

            /// Vector to store the faces which need to be clipped.
            std::vector<FrMesh::FaceHandle *> c_FacesToUpdate;


        public:

            FrMeshClipper();

            /// This function gives the clipping surface.
            FrClippingSurface* GetClippingSurface();

            /// This function initializes the MeshClipper object from an input mesh and performs the clipping.
            void Apply(FrMesh* mesh);

            void SetEps(double eps);

            void SetProjectionThresholdRatio(double projectionThresholdRatio);

            /// This function sets a clipping plane.
            void SetPlaneClippingSurface(const double &meanHeight = 0.);

            /// This function sets a clipping wave surface.
            void SetWaveClippingSurface(const double &meanHeight, FrFreeSurface *FreeSurface);

        private:

            void Initialize();

            void Clear();

            void ClassifyVertices();

            VertexPosition ClassifyVertex(const FrMesh::VertexHandle &vh) const;

            FacePositionType ClassifyFace(const FrMesh::FaceHandle &fh);

            void Clip();

            void UpdateModifiedFaceProperties(FaceHandle fh);

            void ProcessFace(const FrMesh::FaceHandle &fh);

            bool HasProjection(const FrMesh::FaceHandle &fh);

            void ProcessHalfEdge(FrMesh::HalfedgeHandle heh);

            void FlagFaceToBeDeleted(const FrMesh::FaceHandle &fh);

            void FlagVertexAdjacentFacesToBeDeleted(const FrMesh::VertexHandle &vh);

            bool IsEdgeCrossing(const FrMesh::EdgeHandle &eh);

            bool IsHalfEdgeCrossing(const FrMesh::HalfedgeHandle &heh);

            bool IsHalfEdgeDownCrossing(const FrMesh::HalfedgeHandle &heh);

            bool IsHalfEdgeUpCrossing(const FrMesh::HalfedgeHandle &heh);

            FrMesh::HalfedgeHandle FindUpcrossingHalfEdge(const FrMesh::FaceHandle &fh);

            FrMesh::HalfedgeHandle FindDowncrossingHalfEdge(const FrMesh::FaceHandle &fh);

            FrMesh::VertexHandle InsertIntersectionVertex(const FrMesh::HalfedgeHandle &heh);

            double GetVertexDistanceToSurface(const FrMesh::VertexHandle &vh) const;

            void ApplyFaceDeletion();

            void Finalize();

//            /// This function gives the position in the body frame of a node in the mesh frame.
//            FrMesh::Point GetNodePositionInBody(FrMesh::Point point) const;


        };

    }  // end namespace mesh
}  // end namespace frydom


#endif //FRYDOM_DICE_MESHCLIPPER_H
