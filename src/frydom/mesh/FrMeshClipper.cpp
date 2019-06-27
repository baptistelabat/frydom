//
// Created by frongere on 16/05/18.
//

#include "FrMeshClipper.h"

#include "frydom/core/common/FrGeometrical.h"

#include "frydom/environment/ocean/freeSurface/FrFreeSurface.h"
#include "frydom/environment/ocean/freeSurface/tidal/FrTidalModel.h"

namespace frydom {
    
    namespace mesh {

        void FrClippingSurface::SetBodyPosition(Position bodyPos) {
            m_bodyPosition = bodyPos;
        }

        // -----------------------------------------------------------------------------------------------------------------

        VectorT<double, 3>
        FrClippingPlane::GetIntersection(const VectorT<double, 3> &p0, const VectorT<double, 3> &p1) {
            // Find the abscissa on [P0P1] of the intersection with the plan :
            // P_i = P(s_i) = P0 + s_i * P0P1
            // s_i = n . P0O / n . P0P1
            // O being a point on the plane

            Position P0 = {p0[0],p0[1],p0[2]};
            Position P1 = {p1[0],p1[1],p1[2]};

            // Application of the horizontal translation.
            auto BodyPos = m_bodyPosition; BodyPos.GetZ() = 0.;

            P0 += BodyPos;
            P1 += BodyPos;

            auto n = m_plane->GetNormaleInWorld(NWU);

            // check if P0P1 is parallel to the plan / or P0P1 null
            Direction line = (P1 - P0); assert(line.norm()>1E-16);
            assert(line.dot(n)!=0);

            // P0O
            Direction vector = m_plane->GetOriginInWorld(NWU) - P0;

            // s_i
            double s = vector.dot(n) / line.dot(n);

            // P_i
            Position Intersection = P0 + (P1-P0) * s;

            return {Intersection.GetX(),Intersection.GetY(),Intersection.GetZ()};

        }

        double FrClippingPlane::GetDistance(const FrMesh::Point &point) const {

            // This function gives the distance between the node and the clipping plane.

            auto PointInWorld = OpenMeshPointToVector3d<Position>(point);

            // Application of the horizontal translation.
            auto BodyPos = m_bodyPosition; BodyPos.GetZ() = 0.;
            const Position temp = PointInWorld + BodyPos;

            Position vector = temp - m_plane->GetOriginInWorld(NWU);

            return vector.dot(m_plane->GetNormaleInWorld(NWU));

        }

        // -----------------------------------------------------------------------------------------------------------------

        VectorT<double, 3>
        FrClippingWaveSurface::GetIntersection(const VectorT<double, 3> &p0, const VectorT<double, 3> &p1) {

            // The good position of the mesh is automatically found by using the function GetDistance.
            // The position of Pout depends on the positions of p0 and p1 so no transformation is required.

            FrMesh::Point Pout = {0., 0., 0.};

            // Initializations.
            double z0 = GetDistance(p0); // Distance between p0 and Eta_I.
            if (fabs(z0) <= m_ThresholdDichotomy) return p0;

            double z1 = GetDistance(p1); // Distance between p1 and Eta_I.
            if (fabs(z1) <= m_ThresholdDichotomy) return p1;

            int n = 0; // Number of iterations in the dichotomy loop.
            double s0 = 0.; // Curvilinear abscissa of p0;
            double s1 = 1.; // Curvilinear abscissa of p1;

            // Dichotomy loop.
            while (n <= 1000) { // Maximum number of iterations in the dichotomy loop.
                auto s = 0.5 * (s0 + s1);
                auto P = s * p1 + (1 - s) * p0; // Equation of the straight line between p0 and p1.
                auto deltaz = GetDistance(P);
                if ((deltaz > 0. && z1 > 0.) || (deltaz <= 0. && z1 <= 0.)) {
                    s1 = s;
                } else {
                    s0 = s;
                }
                if (fabs(deltaz) <= m_ThresholdDichotomy) return P;
                n++;
            }

            //If dichotomy fails to converge
            std::cout << "GetIntersection: dichotomy method cannot find the intersection point:" << std::endl;
            std::cout << "p0 = " << p0[0] << " " << p0[1] << " " << p0[2] << std::endl;
            std::cout << "p1 = " << p1[0] << " " << p1[1] << " " << p1[2] << std::endl;
            std::cout << "n = " << n << std::endl;
            std::cout << "Pout = " << Pout[0] << " " << Pout[1] << " " << Pout[2] << std::endl;

            return Pout;
        }

        double FrClippingWaveSurface::GetDistance(const VectorT<double, 3> &point) const {

            auto NodeInWorld = OpenMeshPointToVector3d<Position>(point);

            // Application of the horizontal translation.
            auto BodyPos = m_bodyPosition;
            BodyPos.GetZ() = 0.;
            const Position temp = NodeInWorld + BodyPos;

            return temp.GetZ() - m_freeSurface->GetPosition(temp, NWU);

        }



        // -----------------------------------------------------------------------------------------------------------------

        FrClippingSurface *FrMeshClipper::GetClippingSurface() {
            return m_clippingSurface.get();
        }

        void FrMeshClipper::Apply(FrMesh *mesh) {

            // Storage of the input mesh file.
            m_mesh = mesh;

            // Partition of the mesh.
            Initialize();

            // Clipping.
            Clip();

            // Clipped mesh cleaning (deletion of faces, update of every property, etc.).
            Finalize();
        }

        void FrMeshClipper::SetProjectionThresholdRatio(double projectionThresholdRatio) {
            m_ProjectionThresholdRatio = projectionThresholdRatio;
        }

        void FrMeshClipper::SetEps(double eps) {
            m_Threshold = eps;
        }

        void FrMeshClipper::SetClippingSurface(std::shared_ptr<FrClippingSurface> clippingSurface) {
            m_clippingSurface = clippingSurface;
        }

        void FrMeshClipper::Initialize() {

            // This function initializes the clipper.

            // Vector to store the faces to delete.
            c_FacesToDelete.reserve(
                    m_mesh->n_faces()); // Worst case (fully under the clipping surface) PYW: above the clipping surface?

            // Vector to store the faces to update.
            c_FacesToUpdate.clear();

            // Partition of the vertices of the mesh.
            ClassifyVertices();
        }

        void FrMeshClipper::ClassifyVertices() {

            // This function classify the vertices wrt the clipping surface.

            // Iterating on vertices to get their place wrt to plane

            // Loop over the vertices.
            for (auto vh : m_mesh->vertices()) {

                // Computation of the distance to the plane and classification of the nodes.
                auto vPos = ClassifyVertex(vh);

                // Storage of the partition.
                m_mesh->data(vh).SetPositionType(vPos);

            }
        }

        void FrMeshClipper::Clear() {
            m_mesh->clear();
        }

        void FrMeshClipper::Clip() {

            // This function performs the clipping of the mesh wrt the incident wave field.

            // Loop over faces.
            for (FrMesh::FaceIter fh_iter = m_mesh->faces_begin(); fh_iter != m_mesh->faces_end(); ++fh_iter) {
                ProcessFace(*fh_iter);
//        for (auto face : m_mesh->faces()) { // can't be used here; dunno why...
//            ProcessFace(face);
            }
        }

        void FrMeshClipper::UpdateModifiedFaceProperties(FaceHandle fh) {
            FrMesh::FFIter ff_iter = m_mesh->ff_iter(fh);
            for (; ff_iter.is_valid(); ++ff_iter) {
                m_mesh->update_normal(*ff_iter);
                m_mesh->CalcFacePolynomialIntegrals(*ff_iter);
            }
        }

        bool FrMeshClipper::HasProjection(const FaceHandle &fh) {

            /// This function performs the clipping of a panel with the incoming wave field.

            bool anyVertexProjected = false; // Initialization.
            double dist, edge_length;
            double distMin = 1e10;

            FrMesh::HalfedgeHandle oheh;
            FrMesh::VertexHandle vh; // Vertex.
            FrMesh::Point P0, P1, Pi, Pi_final; // Vertices.
            FrMesh::VertexOHalfedgeIter voh_iter; // Vertex outgoing halfedge iterator.

            // Iterating on vertices of the face to clip.
            FrMesh::FaceVertexIter fv_iter = m_mesh->fv_iter(fh);

            for (; fv_iter.is_valid(); ++fv_iter) {

                // First vertex of the edge.
                vh = *fv_iter;
                P0 = m_mesh->point(vh);

                // Iterating on outgoing halfedges to get the shortest edge path.
                // A vertex has several outgoing halfedges, pointing to all its neighbooring vertices.
                voh_iter = m_mesh->voh_iter(vh);
                for (; voh_iter.is_valid(); ++voh_iter) {

                    // Hafledge.
                    oheh = *voh_iter;

                    // Is the halfedge clipping the surface?
                    if (IsHalfEdgeCrossing(oheh)) {

                        // Second vertex of the edge.
                        P1 = m_mesh->point(m_mesh->to_vertex_handle(oheh));

                        // Get the intersection node position.

                        Pi = m_clippingSurface->GetIntersection(P0, P1);

                        dist = (Pi - P0).norm();
                        edge_length = (P1 - P0).norm(); // TODO: utiliser le precalcul...

                        // If the intersection node is too close from the vertex vh, it will be projected.
                        if (dist < edge_length * m_ProjectionThresholdRatio) {
                            if (dist < distMin) {

                                // Storage of the intersection node which is the closest of the vertex vh.
                                Pi_final = Pi;
                                distMin = dist;
                            }
                            anyVertexProjected = true;
                        }
                    }
                }

                // If anyVertexProjected is true, the vertex vh will be projected to one of its neighboring vertex.
                // The choice is made based on the minimum distance to them, by using distMin.
                // If the vertex is projected, no new vertex will be created.
                if (anyVertexProjected) {
                    // Placing vh on Pi.
                    m_mesh->point(vh) = Pi_final;
                    m_mesh->data(vh).SetOn();
                    break; // We anyVertexProjected only one vertex per face at a time as other will be processed by adjacent faces
                }
            }

            return anyVertexProjected;
        }

        VertexPosition FrMeshClipper::ClassifyVertex(const VertexHandle &vh) const {
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

        FacePositionType
        FrMeshClipper::ClassifyFace(const FaceHandle &fh) { // TODO: renvoyer le resultat !!

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

        void FrMeshClipper::ProcessFace(const FaceHandle &fh) {

            /// This function performs the clipping or not of a single face.

            // Not clipping a face several time // FIXME: ne resout pas le pb...

            FrMesh::HalfedgeHandle heh_down, heh_up;
            FrMesh::FaceEdgeIter fe_iter;

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

        void FrMeshClipper::ProcessHalfEdge(HalfedgeHandle heh) {

            /// This function performs the clipping of a halfedge, the computation of the intersection node, the creation of new panels and the deletion of useless panels and vertices.

            FrMesh::VertexHandle vh;  // FIXME: au final, on va juste effectuer l'intersection vu qu'on va projeter les

            // Intersection node.
            vh = InsertIntersectionVertex(heh);

            // Clipping, updating of the mesh, deletion of useless panels and vertices.
            m_mesh->split(m_mesh->edge_handle(heh), vh);

            // Updating the faces to delete.
            FlagVertexAdjacentFacesToBeDeleted(vh);

        }

        void FrMeshClipper::FlagFaceToBeDeleted(const FaceHandle &fh) {

            /// This function adds a face to the vector which stores the faces to delete.

            c_FacesToDelete.push_back(fh);
        }

        void FrMeshClipper::FlagVertexAdjacentFacesToBeDeleted(const VertexHandle &vh) {

            FrMesh::VertexFaceIter vf_iter = m_mesh->vf_iter(vh);
            FacePositionType fPos;
            for (; vf_iter.is_valid(); ++vf_iter) {
                fPos = ClassifyFace(*vf_iter);
                if (fPos == FPT_120 || fPos == FPT_210 || fPos == FPT_030) {
                    FlagFaceToBeDeleted(*vf_iter);
                }
            }
        }

        bool FrMeshClipper::IsEdgeCrossing(const EdgeHandle &eh) {

            /// This function checks if an edge crossed the free surface or not.

            FrMesh::HalfedgeHandle heh = m_mesh->halfedge_handle(eh, 0);
            double dz_0 = GetVertexDistanceToSurface(m_mesh->from_vertex_handle(heh));
            double dz_1 = GetVertexDistanceToSurface(m_mesh->to_vertex_handle(heh));
            double prod = dz_0 * dz_1;
            bool out;

            if (fabs(dz_0) < m_Threshold || fabs(dz_1) < m_Threshold) {
                out = false;
            } else {
                // If prof is negative, the two vertices are not on the same side of the free surface.
                out = (prod < 0.);
            }

            return out;
        }

        bool FrMeshClipper::IsHalfEdgeCrossing(const HalfedgeHandle &heh) {

            /// This function checks if a halfedge crossed the free surface or not.

            return IsEdgeCrossing(m_mesh->edge_handle(heh));
        }

        bool FrMeshClipper::IsHalfEdgeDownCrossing(const HalfedgeHandle &heh) {

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

        bool FrMeshClipper::IsHalfEdgeUpCrossing(const HalfedgeHandle &heh) {

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

        HalfedgeHandle FrMeshClipper::FindUpcrossingHalfEdge(const FaceHandle &fh) {

            /// This function tracks the halfedge which crosses the free surface upwardly.

            // TODO: throw error if no upcrossing halfedge is found
            FrMesh::HalfedgeHandle heh = m_mesh->halfedge_handle(fh);
            while (!IsHalfEdgeUpCrossing(heh)) {
                heh = m_mesh->next_halfedge_handle(heh);
            }
            return heh;
        }

        HalfedgeHandle FrMeshClipper::FindDowncrossingHalfEdge(const FaceHandle &fh) {

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

        VertexHandle FrMeshClipper::InsertIntersectionVertex(const HalfedgeHandle &heh) {

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

        double FrMeshClipper::GetVertexDistanceToSurface(const VertexHandle &vh) const {

            /// This function gives the distance of a node to the incident wave field.

            return m_clippingSurface->GetDistance(m_mesh->point(vh));

        }

        void FrMeshClipper::ApplyFaceDeletion() {
            for (FrMesh::FaceHandle fh : c_FacesToDelete) {
                if (!m_mesh->status(fh).deleted()) {
                    m_mesh->delete_face(fh);
                }
            }
            c_FacesToDelete.clear();
        }

        void FrMeshClipper::Finalize() {

            /// This function cleans the clipped mesh (deletion of faces, update of every property, etc.).

            ApplyFaceDeletion();
            std::vector<VertexHandle *> vh_to_update;  // FIXME: ne fonctionne pas, il faut que ces vecteurs soient initialises avec des elements a tracker
            std::vector<HalfedgeHandle *> hh_to_update;
//            std::vector<FaceHandle*> fh_to_update;

            m_mesh->garbage_collection(vh_to_update, hh_to_update, c_FacesToUpdate);

            m_mesh->UpdateAllProperties(); // FIXME: il ne faudrait mettre a jour que les pptes de facettes ayant ete mofifiees ou nouvellement crees

            // We have to update the modified and new faces properties
//            DMesh::FaceFaceIter ff_iter;
//            for (FaceHandle* fh_ptr : c_FacesToUpdate) {
//                if (!fh_ptr->is_valid()) continue;
//
//                m_mesh->update_normal(*fh_ptr);
//                // TODO: faire l'update aussi des centres !!
//                m_mesh->CalcFacePolynomialIntegrals(*fh_ptr);
//
//                // Updating the surrounding faces too
//                ff_iter = m_mesh->ff_iter(*fh_ptr);
//                for (; ff_iter.is_valid(); ++ff_iter) {
//                    m_mesh->update_normal(*ff_iter);
//                    m_mesh->CalcFacePolynomialIntegrals(*ff_iter);
//                }
//
//
//                // Updating also the surrounding faces
//            }
//
//            // Updating the modified and new vertices properties
//            for (VertexHandle* vh_ptr : vh_to_update) {
//                m_mesh->update_normal(*vh_ptr);
//            }


        }

    } //end namespace mesh

}; // end namespace frydom