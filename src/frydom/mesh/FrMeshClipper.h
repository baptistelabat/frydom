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
        * \class ClippingSurface
        * \brief Class for dealing with the clipping incident wave field.
        */
        class ClippingSurface {

        protected:
            double m_meanHeight = 0.;
            double m_ThresholdDichotomy = 1e-4;

            Position m_bodyPosition = {0.,0.,0.};

        public:

            /// Constructor.
            ClippingSurface() = default;

            /// Constructor and initialization of the mean height.
            explicit ClippingSurface(double meanHeight);

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
        * \class ClippingPlane
        * \brief Class used when the clipping incident wave field is a HORIZONTAL plane.
        */
        class ClippingPlane : public ClippingSurface {

        public:

            /// Constructor.
            ClippingPlane() = default;

            /// Constructor.
            explicit ClippingPlane(double meanHeight);

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
        * \class ClippingWavesSurface
        * \brief Class for clipping a mesh by an arbitrary incident wave field.
        */
        class ClippingWaveSurface : public ClippingSurface {

        private:

            /// FreeSurface.
            FrFreeSurface* m_freesurface;

        public:

            /// Constructor.
            ClippingWaveSurface() = default;

            /// Constructor.
            explicit ClippingWaveSurface(const double &meanHeight, FrFreeSurface* FreeSurface);

            /// This function gives the distance to the incident wave.
            double GetDistance(const FrMesh::Point &point) const override;

            /// This function performs a bisection method to track the intersection node.
            FrMesh::Point GetIntersection(const FrMesh::Point &p0, const FrMesh::Point &p1) override;
        };

        class MeshClipper {

        private:

            /// Initial mesh.
            FrMesh* m_mesh;

            /// Clipping surface, by default the plane z = 0.
            //TODO: INCORRECT
            std::unique_ptr<ClippingSurface> m_clippingSurface;

            double m_Threshold = 1e-4;
            double m_ProjectionThresholdRatio = 1 / 4.;

            /// Vector to store the faces which are on and/or above the incident free surface and have to be deleted.
            std::vector<FrMesh::FaceHandle> c_FacesToDelete;

            /// Vector to store the faces which need to be clipped.
            std::vector<FrMesh::FaceHandle *> c_FacesToUpdate;


        public:

            MeshClipper();

            /// This function gives the clipping surface.
            ClippingSurface* GetClippingSurface();

            /// This function initializes the MeshClipper object from an input mesh and performs the clipping.
            void Apply(FrMesh* mesh);

            void SetEps(double eps) { m_Threshold = eps; }

            void SetProjectionThresholdRatio(double projectionThresholdRatio);

            /// This function sets a clipping plane.
            void SetPlaneClippingSurface(const double &meanHeight = 0.);

            /// This function sets a clipping wave surface.
            void SetWaveClippingSurface(const double &meanHeight, FrFreeSurface *FreeSurface);

//            void UpdateMeshPositionInWorld();

        private:

            void Initialize();

            void Clear();

            void ClassifyVertices();

            inline VertexPosition ClassifyVertex(const FrMesh::VertexHandle &vh) const {
                double distance = GetVertexDistanceToSurface(vh);

                // This function computes the distance wrt the incident wave and classifies the nodes.

                // TODO: On peut projeter le vertex si distance est petit (si plus petit que meanEdgeLength * m_threshold)

                VertexPosition vPos;

                if (fabs(distance) < m_Threshold) {
                    vPos = VP_ON_SURFACE;
                } else if (distance > 0.) {
                    vPos = VP_ABOVE_SURFACE;
                } else {
                    vPos = VP_UNDER_SURFACE;
                }

                return vPos;
            }

            inline FacePositionType ClassifyFace(const FrMesh::FaceHandle &fh) { // TODO: renvoyer le resultat !!

                /// This function classfies faces wrt the incident wave field.

                FacePositionType fPos;
                unsigned int nbAbove, nbUnder;
                nbAbove = nbUnder = 0;

                // Counting the number of vertices above and under the plane.
                FrMesh::FaceVertexIter fv_iter = m_mesh->fv_iter(fh);
                for (; fv_iter.is_valid(); ++fv_iter) {

                    if (m_mesh->data(*fv_iter).IsAbove()) {
                        ++nbAbove;
                    }
                    if (m_mesh->data(*fv_iter).IsUnder()) {
                        ++nbUnder;
                    }
                }

                // Face under or on the free surface.
                if (nbAbove == 0) {
                    if (nbUnder == 0) {
                        fPos = FPT_030;
                    } else if (nbUnder == 1) {
                        fPos = FPT_021;
                    } else if (nbUnder == 2) {
                        fPos = FPT_012;
                    } else {
                        fPos = FPT_003;
                    }
                    // Face crossing the free surface.
                } else if (nbAbove == 1) {
                    if (nbUnder == 0) {
                        fPos = FPT_120;
                    } else if (nbUnder == 1) {
                        fPos = FPT_111;
                    } else {
                        fPos = FPT_102;
                    }
                    // Face crossing the free surface.
                } else if (nbAbove == 2) {
                    if (nbUnder == 0) {
                        fPos = FPT_210;
                    } else {
                        fPos = FPT_201;
                    }
                    // Face above the free surface.
                } else {
                    fPos = FPT_300;
                }

                return fPos;
            }

            void Clip();

            void UpdateModifiedFaceProperties(FaceHandle fh);

            inline void ProcessFace(const FrMesh::FaceHandle &fh) {

                /// This function performs the clipping or not of a single face.

                // Not clipping a face several time // FIXME: ne resout pas le pb...

                FrMesh::FaceHandle opp_fh;
                FrMesh::HalfedgeHandle heh, heh_down, heh_up;
                FrMesh::VertexHandle vh;
                FrMesh::FaceEdgeIter fe_iter;
                FrMesh::FaceHandle fh_tmp;

                // Classification of faces wrt the incident wave field.
                FacePositionType fPos = ClassifyFace(fh);

                switch (fPos) { // Pourquoi ne pas faire tout ici ?
                    case FPT_003:
                        // Totally wet, we keep the face
                        break;
                    case FPT_012:
                        // Totally wet, we keep the face
                        break;
                    case FPT_021:
                        // Totally wet, we keep the face
                        break;
                    case FPT_030:
                        // Face on the free surface.
                        FlagFaceToBeDeleted(fh);
                        break;
                    case FPT_102:
//                    c_FacesToUpdate.emplace_back(new FaceHandle(const_cast<FaceHandle&>(fh)));

                        // If HasProjection = True, some vertices are projected to the intersection nodes and no new face or vertex is added.

                        if (HasProjection(fh)) {
                            // The function ProcessFase is run again to update the classfication?
                            ProcessFace(fh);
                        } else {

                            // Clipping the panel, creation of new nodes on the intersection curve and new faces.
                            // TODO: avoir une methode qui a la fois ajouter le vertex et plitte l'edge...

                            // Unique half edge oriented downward the intersection curve.
                            heh_down = FindDowncrossingHalfEdge(fh);

                            // Unique half edge oriented upward the intersection curve.
                            heh_up = FindUpcrossingHalfEdge(fh);

                            // Clipping of the upward and downward harfedges, creation of new vertices and faces and deletion of the useless ones.
                            ProcessHalfEdge(heh_down);
                            ProcessHalfEdge(heh_up);

                        }
//                    UpdateModifiedFaceProperties(fh);
                        break;

                    case FPT_111:
//                    c_FacesToUpdate.emplace_back(new FaceHandle(const_cast<FaceHandle&>(fh)));

                        if (HasProjection(fh)) {
                            // The function ProcessFase is run again to update the classfication?
                            ProcessFace(fh);
                        } else {
                            fe_iter = m_mesh->fe_iter(fh);
                            for (; fe_iter.is_valid(); ++fe_iter) {
                                if (IsEdgeCrossing(*fe_iter)) {
                                    break;
                                }
                            }

                            ProcessHalfEdge(m_mesh->halfedge_handle(*fe_iter, 0));
                        }
//                    UpdateModifiedFaceProperties(fh);
                        break;

                    case FPT_120:
                        // Face above and on the free surface.
                        FlagFaceToBeDeleted(fh);
                        break;

                    case FPT_201:
//                    c_FacesToUpdate.emplace_back(new FaceHandle(const_cast<FaceHandle&>(fh)));

                        if (HasProjection(fh)) {
                            // The function ProcessFase is run again to update the classfication?
                            ProcessFace(fh);
                        } else {

                            // TODO: avoir une methode qui a la fois ajouter le vertex et plitte l'edge...
                            heh_down = FindDowncrossingHalfEdge(fh);
                            heh_up = FindUpcrossingHalfEdge(fh);

                            ProcessHalfEdge(heh_down);
                            ProcessHalfEdge(heh_up);
                        }
//                    UpdateModifiedFaceProperties(fh);
                        break;

                    case FPT_210:
                        // Face above and on the free surface.
                        FlagFaceToBeDeleted(fh);
                        break;

                    case FPT_300:
                        // Face above the free surface.
                        FlagFaceToBeDeleted(fh);
                        break;

                    case FPT_UNDEFINED:
                        // TODO: throw exception
                        break;

                }
            }

            bool HasProjection(const FrMesh::FaceHandle &fh);

            inline void ProcessHalfEdge(FrMesh::HalfedgeHandle heh) {

                /// This function performs the clipping of a halfedge, the computation of the intersection node, the creation of new panels and the deletion of useless panels and vertices.

                FrMesh::VertexHandle vh;  // FIXME: au final, on va juste effectuer l'intersection vu qu'on va projeter les

                // Intersection node.
                vh = InsertIntersectionVertex(heh);

                // Clipping, updating of the mesh, deletion of useless panels and vertices.
                m_mesh->split(m_mesh->edge_handle(heh), vh);

                // Updating the faces to delete.
                FlagVertexAdjacentFacesToBeDeleted(vh);

            }

            inline void FlagFaceToBeDeleted(const FrMesh::FaceHandle &fh) {

                /// This function adds a face to the vector which stores the faces to delete.

                c_FacesToDelete.push_back(fh);
            }

            inline void FlagVertexAdjacentFacesToBeDeleted(const FrMesh::VertexHandle &vh) {

                FrMesh::VertexFaceIter vf_iter = m_mesh->vf_iter(vh);
                FacePositionType fPos;
                for (; vf_iter.is_valid(); ++vf_iter) {
                    fPos = ClassifyFace(*vf_iter);
                    if (fPos == FPT_120 || fPos == FPT_210 || fPos == FPT_030) {
                        FlagFaceToBeDeleted(*vf_iter);
                    }
                }
            }

            inline bool IsEdgeCrossing(const FrMesh::EdgeHandle &eh) {

                /// This function checks if an edge crossed the free surface or not.

                FrMesh::HalfedgeHandle heh = m_mesh->halfedge_handle(eh, 0);
                double dz_0 = GetVertexDistanceToSurface(m_mesh->from_vertex_handle(heh));
                double dz_1 = GetVertexDistanceToSurface(m_mesh->to_vertex_handle(heh));
                double prod = dz_0 * dz_1;
                bool out;

                if(fabs(dz_0) < m_Threshold || fabs(dz_1) < m_Threshold){
                    out = false;
                }
                else{
                    // If prof is negative, the two vertices are not on the same side of the free surface.
                    out = (prod < 0.);
                }

                return out;
            }

            inline bool IsHalfEdgeCrossing(const FrMesh::HalfedgeHandle &heh) {

                /// This function checks if a halfedge crossed the free surface or not.

                return IsEdgeCrossing(m_mesh->edge_handle(heh));
            }

            inline bool IsHalfEdgeDownCrossing(const FrMesh::HalfedgeHandle &heh) {

                /// This function checks if a halfedge crossed the free surface downwardly.

                double dz_from = GetVertexDistanceToSurface(m_mesh->from_vertex_handle(heh));
                double dz_to = GetVertexDistanceToSurface(m_mesh->to_vertex_handle(heh));
                bool out;
                if (fabs(dz_from) < m_Threshold || fabs(dz_to) < m_Threshold) {
                    out = false;
                } else {
                    out = (dz_from > 0. && dz_to < 0.);
                }
                return out;
            }

            inline bool IsHalfEdgeUpCrossing(const FrMesh::HalfedgeHandle &heh) {

                /// This function checks if a halfedge crossed the free surface upwardly.

                double dz_from = GetVertexDistanceToSurface(m_mesh->from_vertex_handle(heh));
                double dz_to = GetVertexDistanceToSurface(m_mesh->to_vertex_handle(heh));
                bool out;
                if (fabs(dz_from) < m_Threshold || fabs(dz_to) < m_Threshold) {
                    out = false;
                } else {
                    out = (dz_from < 0. && dz_to > 0.);
                }
                return out;
            }

            inline FrMesh::HalfedgeHandle FindUpcrossingHalfEdge(const FrMesh::FaceHandle &fh) {

                /// This function tracks the halfedge which crosses the free surface upwardly.

                // TODO: throw error if no upcrossing halfedge is found
                FrMesh::HalfedgeHandle heh = m_mesh->halfedge_handle(fh);
                while (!IsHalfEdgeUpCrossing(heh)) {
                    heh = m_mesh->next_halfedge_handle(heh);
                }
                return heh;
            }

            inline FrMesh::HalfedgeHandle FindDowncrossingHalfEdge(const FrMesh::FaceHandle &fh) {

                /// This function tracks the halfedge which crosses the free surface downwardly.

                // TODO: throw error if no downcrossing halfedge is found
                FrMesh::HalfedgeHandle heh = m_mesh->halfedge_handle(fh);
                unsigned int i = 0;
                while (!IsHalfEdgeDownCrossing(heh) && i < 2) { // TODO: abandonner le i pour le garde fou... --> erreur
                    i++;
                    heh = m_mesh->next_halfedge_handle(heh);
                }
                return heh;
            }

            inline FrMesh::VertexHandle InsertIntersectionVertex(const FrMesh::HalfedgeHandle &heh) {

                /// This function adds an intersection node of an edge as a new vertex of the mesh.

                FrMesh::Point p0 = m_mesh->point(m_mesh->from_vertex_handle(heh));
                FrMesh::Point p1 = m_mesh->point(m_mesh->to_vertex_handle(heh));

                FrMesh::Point p_intersection = m_clippingSurface->GetIntersection(
                        m_mesh->point(m_mesh->from_vertex_handle(heh)),
                        m_mesh->point(m_mesh->to_vertex_handle(heh))
                );

                FrMesh::VertexHandle vh = m_mesh->add_vertex(p_intersection);
                m_mesh->data(vh).SetOn(); // Vertex has been built on the clipping surface

                return vh;
            }

            inline double GetVertexDistanceToSurface(
                    const FrMesh::VertexHandle &vh) const {

                /// This function gives the distance of a node to the incident wave field.

                return m_clippingSurface->GetDistance(m_mesh->point(vh));

            }

            void ApplyFaceDeletion();

            void Finalize();

//            /// This function gives the position in the body frame of a node in the mesh frame.
//            FrMesh::Point GetNodePositionInBody(FrMesh::Point point) const;


        };

    }  // end namespace mesh
}  // end namespace frydom


#endif //FRYDOM_DICE_MESHCLIPPER_H
