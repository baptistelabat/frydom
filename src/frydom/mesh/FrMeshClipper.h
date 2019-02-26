//
// Created by frongere on 16/05/18.
//

#ifndef FRYDOM_DICE_MESHCLIPPER_H
#define FRYDOM_DICE_MESHCLIPPER_H

#include <memory>
#include "FrMesh.h"

namespace frydom {

    namespace mesh {

        class ClippingSurface {

        protected:
            double m_meanHeight = 0.;

        public:

            ClippingSurface() = default;

            explicit ClippingSurface(const double meanHeight) : m_meanHeight(meanHeight) {}

            void SetMeanHeight(const double &meanHeight) { m_meanHeight = meanHeight; }

            virtual double GetElevation(const double &x, const double &y) const = 0;

            virtual double GetDistance(const FrMesh::Point &point) const = 0;

            virtual FrMesh::Point GetIntersection(const FrMesh::Point &p0, const FrMesh::Point &p1) = 0;

        };

        class ClippingPlane : public ClippingSurface {

        public:

            ClippingPlane() = default;

            explicit ClippingPlane(const double meanHeight) : ClippingSurface(meanHeight) {}

            double GetElevation(const double &x, const double &y) const override {
                return m_meanHeight;
            }

            inline virtual double GetDistance(const FrMesh::Point &point) const {

                /*
                 * TODO: idee, lorsqu'on fait ue requete de distance des vertex, on peut deja effectuer des projections !!
                 * Du coup c'est fait en preliminaire avant les decoupes. Ca permet a priori d'epargner pas mal de clip et de
                 * facettes pourries en plus. Si on a projete, la distance est nulle et on renvoie 0...
                 * Si on a pas projete, la distance est on nulle et on classifie le vertex above ou under.
                 * La classification est faite par la methode ClassifyVertex !! elle ne doit pas etre faite ici
                 * Par contre, il semblerait qu'on soit oblige de passer le maillage en argument :/ afin de pouvoir
                */

                // TODO: projeter !

                return point[2] - m_meanHeight;
            }

            FrMesh::Point GetIntersection(const FrMesh::Point &p0, const FrMesh::Point &p1) override {
                double t = (p0[2] - m_meanHeight) / (p0[2] - p1[2]);
                return p0 * (1 - t) + p1 * t;
            }
        };

        class ClippingAiryWavesSurface : ClippingSurface {

        private:
            // TODO Garder le FrOffshoreSystem en pointeur pour acceder a l'evironnement

        public:

            ClippingAiryWavesSurface() = default;

            explicit ClippingAiryWavesSurface(const double &meanHeight) : ClippingSurface(meanHeight) {}

            double GetElevation(const double &x, const double &y) const override {
                // TODO
                return 0.;
            }

            inline virtual double GetDistance(const FrMesh::Point &point) const {
                // TODO
                return 0.;
            }

            FrMesh::Point GetIntersection(const FrMesh::Point &p0, const FrMesh::Point &p1) override {

            }

        };


        class MeshClipper {  // Pour le moment, on coupe par un plan

        private:

            FrMesh m_InitMesh;
            std::shared_ptr<ClippingSurface> m_clippingSurface = std::make_shared<ClippingPlane>(0.);

            double m_Threshold = 1e-4;
//        double m_Threshold = 0.5;
            double m_ProjectionThresholdRatio = 1 / 4.;

            std::vector<FrMesh::FaceHandle> c_FacesToDelete;
            std::vector<FrMesh::FaceHandle *> c_FacesToUpdate;

        public:

            MeshClipper() = default;

            std::shared_ptr<ClippingSurface> GetClippingSurface() { return m_clippingSurface; }

            FrMesh &operator()(const FrMesh &mesh) {

                Clear();
                m_InitMesh = FrMesh(mesh);
                Initialize();
                Clip();
                Finalize();

                return m_InitMesh;  // TODO: renvoyer un etat...
            }

            void SetEps(double eps) { m_Threshold = eps; }

            void SetProjectionThresholdRatio(double projectionThresholdRatio) {
                m_ProjectionThresholdRatio = projectionThresholdRatio;
            }

            void SetPlaneClippingSurface(const double &meanHeight = 0.) {
                m_clippingSurface = std::make_unique<ClippingPlane>(meanHeight);
            }

        private:

            void Initialize() {
                c_FacesToDelete.reserve(m_InitMesh.n_faces()); // Worst case (fully under the clipping surface)
                c_FacesToUpdate.clear();
                ClassifyVertices();
            }

            void Clear() {
                m_InitMesh.clear();
            }

            void ClassifyVertices() {
                // Iterating on vertices to get their place wrt to plane
                VertexPosition vPos;
                VertexHandle vh;
                for (FrMesh::VertexIter vh_iter = m_InitMesh.vertices_begin();
                     vh_iter != m_InitMesh.vertices_end(); ++vh_iter) {
                    vh = *vh_iter;
                    vPos = ClassifyVertex(vh);
                    m_InitMesh.data(vh).SetPositionType(vPos);
                }
            }

            inline VertexPosition ClassifyVertex(const FrMesh::VertexHandle &vh) const {
                double distance = GetVertexDistanceToSurface(vh);

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

                FacePositionType fPos;
                unsigned int nbAbove, nbUnder;
                nbAbove = nbUnder = 0;

                // Counting the number of vertices above and under the plane
                FrMesh::FaceVertexIter fv_iter = m_InitMesh.fv_iter(fh);
                for (; fv_iter.is_valid(); ++fv_iter) {

                    if (m_InitMesh.data(*fv_iter).IsAbove()) {
                        ++nbAbove;
                    }
                    if (m_InitMesh.data(*fv_iter).IsUnder()) {
                        ++nbUnder;
                    }
                }

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

                } else if (nbAbove == 1) {
                    if (nbUnder == 0) {
                        fPos = FPT_120;
                    } else if (nbUnder == 1) {
                        fPos = FPT_111;
                    } else {
                        fPos = FPT_102;
                    }

                } else if (nbAbove == 2) {
                    if (nbUnder == 0) {
                        fPos = FPT_210;
                    } else {
                        fPos = FPT_201;
                    }

                } else {
                    fPos = FPT_300;
                }

                return fPos;
            }

            void Clip() {
                for (FrMesh::FaceIter fh_iter = m_InitMesh.faces_begin(); fh_iter != m_InitMesh.faces_end(); ++fh_iter) {
                    ProcessFace(*fh_iter);
                }
            }

            void UpdateModifiedFaceProperties(FaceHandle fh) {
                FrMesh::FFIter ff_iter = m_InitMesh.ff_iter(fh);
                for (; ff_iter.is_valid(); ++ff_iter) {
                    m_InitMesh.update_normal(*ff_iter);
                    m_InitMesh.CalcFacePolynomialIntegrals(*ff_iter);
                }
            }

            inline void ProcessFace(const FrMesh::FaceHandle &fh) {

                // Not clipping a face several time // FIXME: ne resout pas le pb...

                FrMesh::FaceHandle opp_fh;
                FrMesh::HalfedgeHandle heh, heh_down, heh_up;
                FrMesh::VertexHandle vh;
                FrMesh::FaceEdgeIter fe_iter;
                FrMesh::FaceHandle fh_tmp;

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
                        FlagFaceToBeDeleted(fh);
                        break;
                    case FPT_102:
//                    c_FacesToUpdate.emplace_back(new FaceHandle(const_cast<FaceHandle&>(fh)));

                        if (HasProjection(fh)) {  // TODO faire pareil sur 111 et 201
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

                    case FPT_111:
//                    c_FacesToUpdate.emplace_back(new FaceHandle(const_cast<FaceHandle&>(fh)));


                        if (HasProjection(fh)) {  // TODO faire pareil sur 111 et 201
                            ProcessFace(fh);
                        } else {

                            fe_iter = m_InitMesh.fe_iter(fh);
                            for (; fe_iter.is_valid(); ++fe_iter) {
                                if (IsEdgeCrossing(*fe_iter)) {
                                    break;
                                }
                            }

                            ProcessHalfEdge(m_InitMesh.halfedge_handle(*fe_iter, 0));
                        }
//                    UpdateModifiedFaceProperties(fh);
                        break;

                    case FPT_120:
                        FlagFaceToBeDeleted(fh);
                        break;

                    case FPT_201:
//                    c_FacesToUpdate.emplace_back(new FaceHandle(const_cast<FaceHandle&>(fh)));


                        if (HasProjection(fh)) {  // TODO faire pareil sur 111 et 201
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
                        FlagFaceToBeDeleted(fh);
                        break;

                    case FPT_300:
                        FlagFaceToBeDeleted(fh);
                        break;

                    case FPT_UNDEFINED:
                        // TODO: throw exception
                        break;

                }
            }

            bool HasProjection(const FrMesh::FaceHandle &fh) {

                bool anyVertexProjected = false;
                double dist, edge_length;
                double distMin = 1e10;

                FrMesh::HalfedgeHandle oheh;
                FrMesh::VertexHandle vh;
                FrMesh::Point P0, P1, Pi, Pi_final;
                FrMesh::VertexOHalfedgeIter voh_iter;

                // Iterating on face vertices
                FrMesh::FaceVertexIter fv_iter = m_InitMesh.fv_iter(fh);
                for (; fv_iter.is_valid(); ++fv_iter) {
                    vh = *fv_iter;
                    P0 = m_InitMesh.point(vh);

                    // Iterating on outgoing halfedges to get the shortest edge path
                    voh_iter = m_InitMesh.voh_iter(vh);
                    for (; voh_iter.is_valid(); ++voh_iter) {
                        oheh = *voh_iter;

                        // Is the halfedge clipping the surface ?
                        if (IsHalfEdgeCrossing(oheh)) {

                            P1 = m_InitMesh.point(m_InitMesh.to_vertex_handle(oheh));
                            // Get the intersection
                            Pi = m_clippingSurface->GetIntersection(P0, P1);

                            dist = (Pi - P0).norm();

                            edge_length = (P1 - P0).norm(); // TODO: utiliser le precalcul...

                            if (dist < edge_length * m_ProjectionThresholdRatio) {
                                if (dist < distMin) {
                                    Pi_final = Pi;
                                    distMin = dist;
                                }
                                anyVertexProjected = true;
                            }
                        }
                    }

                    if (anyVertexProjected) {
                        // Placing vh on Pi
                        m_InitMesh.point(vh) = Pi_final;
                        m_InitMesh.data(vh).SetOn();
                        break; // We anyVertexProjected only one vertex per face at a time as other will be processed by adjacent faces
                    }
                }

                return anyVertexProjected;
            }

            inline void ProcessHalfEdge(FrMesh::HalfedgeHandle heh) {
                FrMesh::VertexHandle vh;  // FIXME: au final, on va juste effectuer l'intersection vu qu'on va projeter les
                vh = InsertIntersectionVertex(heh);
                m_InitMesh.split(m_InitMesh.edge_handle(heh), vh);
                FlagVertexAdjacentFacesToBeDeleted(vh);
            }

            inline void FlagFaceToBeDeleted(const FrMesh::FaceHandle &fh) {
                c_FacesToDelete.push_back(fh);
            }

            inline void FlagVertexAdjacentFacesToBeDeleted(const FrMesh::VertexHandle &vh) {
                FrMesh::VertexFaceIter vf_iter = m_InitMesh.vf_iter(vh);
                FacePositionType fPos;
                for (; vf_iter.is_valid(); ++vf_iter) {
                    fPos = ClassifyFace(*vf_iter);
                    if (fPos == FPT_120 || fPos == FPT_210 || fPos == FPT_030) {
                        FlagFaceToBeDeleted(*vf_iter);
                    }
                }
            }

            inline bool IsEdgeCrossing(const FrMesh::EdgeHandle &eh) {
                // FIXME: ne porte que sur un plan z=0...
                FrMesh::HalfedgeHandle heh = m_InitMesh.halfedge_handle(eh, 0);
                double dz_0 = GetVertexDistanceToSurface(m_InitMesh.from_vertex_handle(heh));
                double dz_1 = GetVertexDistanceToSurface(m_InitMesh.to_vertex_handle(heh));
                double prod = dz_0 * dz_1;
                bool out;
                if (fabs(prod) < m_Threshold) {
                    out = false;
                } else {
                    out = (prod < 0.);
                }
                return out;
            }

            inline bool IsHalfEdgeCrossing(const FrMesh::HalfedgeHandle &heh) {
                return IsEdgeCrossing(m_InitMesh.edge_handle(heh));
            }

            inline bool IsHalfEdgeDownCrossing(const FrMesh::HalfedgeHandle &heh) {
                // FIXME: ne porte que sur un plan z=0...
                double dz_from = GetVertexDistanceToSurface(m_InitMesh.from_vertex_handle(heh));
                double dz_to = GetVertexDistanceToSurface(m_InitMesh.to_vertex_handle(heh));
                bool out;
                if (fabs(dz_from) < m_Threshold || fabs(dz_to) < m_Threshold) {
                    out = false;
                } else {
                    out = (dz_from > 0. && dz_to < 0.);
                }
                return out;
            }

            inline bool IsHalfEdgeUpCrossing(const FrMesh::HalfedgeHandle &heh) {
                // FIXME: ne porte que sur un plan z=0...
                double dz_from = GetVertexDistanceToSurface(m_InitMesh.from_vertex_handle(heh));
                double dz_to = GetVertexDistanceToSurface(m_InitMesh.to_vertex_handle(heh));
                bool out;
                if (fabs(dz_from) < m_Threshold || fabs(dz_to) < m_Threshold) {
                    out = false;
                } else {
                    out = (dz_from < 0. && dz_to > 0.);
                }
                return out;
            }

            inline FrMesh::HalfedgeHandle FindUpcrossingHalfEdge(const FrMesh::FaceHandle &fh) {
                // TODO: throw error if no upcrossing halfedge is found
                FrMesh::HalfedgeHandle heh = m_InitMesh.halfedge_handle(fh);
                while (!IsHalfEdgeUpCrossing(heh)) {
                    heh = m_InitMesh.next_halfedge_handle(heh);
                }
                return heh;
            }

            inline FrMesh::HalfedgeHandle FindDowncrossingHalfEdge(const FrMesh::FaceHandle &fh) {
                // TODO: throw error if no downcrossing halfedge is found
                FrMesh::HalfedgeHandle heh = m_InitMesh.halfedge_handle(fh);
                unsigned int i = 0;
                while (!IsHalfEdgeDownCrossing(heh) && i < 2) { // TODO: abandonner le i pour le garde fou... --> erreur
                    i++;
                    heh = m_InitMesh.next_halfedge_handle(heh);
                }
                return heh;
            }

            inline FrMesh::VertexHandle InsertIntersectionVertex(const FrMesh::HalfedgeHandle &heh) {
                FrMesh::Point p0 = m_InitMesh.point(m_InitMesh.from_vertex_handle(heh));
                FrMesh::Point p1 = m_InitMesh.point(m_InitMesh.to_vertex_handle(heh));

                FrMesh::Point p_intersection = m_clippingSurface->GetIntersection(
                        m_InitMesh.point(m_InitMesh.from_vertex_handle(heh)),
                        m_InitMesh.point(m_InitMesh.to_vertex_handle(heh))
                );

                FrMesh::VertexHandle vh = m_InitMesh.add_vertex(p_intersection);
                m_InitMesh.data(vh).SetOn(); // Vertex has been built on the clipping surface
                return vh;
            }

            inline double GetVertexDistanceToSurface(
                    const FrMesh::VertexHandle &vh) const { // FIXME: ca doit venir de m_clippingSurface
                return m_clippingSurface->GetDistance(m_InitMesh.point(vh));
            }

            void ApplyFaceDeletion() { // TODO: mettre en prive
                for (FrMesh::FaceHandle fh : c_FacesToDelete) {
                    if (!m_InitMesh.status(fh).deleted()) {
                        m_InitMesh.delete_face(fh);
                    }
                }
                c_FacesToDelete.clear();
            }

            void Finalize() {
                ApplyFaceDeletion();
                std::vector<VertexHandle *> vh_to_update;  // FIXME: ne fonctionne pas, il faut que ces vecteurs soient initialises avec des elements a tracker
                std::vector<HalfedgeHandle *> hh_to_update;
//            std::vector<FaceHandle*> fh_to_update;

                m_InitMesh.garbage_collection(vh_to_update, hh_to_update, c_FacesToUpdate);

                m_InitMesh.UpdateAllProperties(); // FIXME: il ne faudrait mettre a jour que les pptes de facettes ayant ete mofifiees ou nouvellement crees

                // We have to update the modified and new faces properties
//            DMesh::FaceFaceIter ff_iter;
//            for (FaceHandle* fh_ptr : c_FacesToUpdate) {
//                if (!fh_ptr->is_valid()) continue;
//
//                m_InitMesh.update_normal(*fh_ptr);
//                // TODO: faire l'update aussi des centres !!
//                m_InitMesh.CalcFacePolynomialIntegrals(*fh_ptr);
//
//                // Updating the surrounding faces too
//                ff_iter = m_InitMesh.ff_iter(*fh_ptr);
//                for (; ff_iter.is_valid(); ++ff_iter) {
//                    m_InitMesh.update_normal(*ff_iter);
//                    m_InitMesh.CalcFacePolynomialIntegrals(*ff_iter);
//                }
//
//
//                // Updating also the surrounding faces
//            }
//
//            // Updating the modified and new vertices properties
//            for (VertexHandle* vh_ptr : vh_to_update) {
//                m_InitMesh.update_normal(*vh_ptr);
//            }


            }

        };

    }  // end namespace mesh
}  // end namespace frydom


#endif //FRYDOM_DICE_MESHCLIPPER_H
