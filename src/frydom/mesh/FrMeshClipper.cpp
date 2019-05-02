//
// Created by frongere on 16/05/18.
//

#include "FrMeshClipper.h"


namespace frydom {


    mesh::ClippingSurface::ClippingSurface(double meanHeight) : m_meanHeight(meanHeight) {}

    void mesh::ClippingSurface::SetMeanHeight(const double &meanHeight) { m_meanHeight = meanHeight; }

    VectorT<double, 3> mesh::ClippingSurface::GetNodePositionInWorld(VectorT<double, 3> point) const {

        mathutils::Vector3d<double> NodeInWorldWithoutTranslation;
        NodeInWorldWithoutTranslation[0] = point[0];
        NodeInWorldWithoutTranslation[1] = point[1];
        NodeInWorldWithoutTranslation[2] = point[2];

        // Application of the translation.
        Position NodeInWorldFrame;
        Position BodyPos = m_bodyPosition;
        NodeInWorldFrame[0] = BodyPos[0] + NodeInWorldWithoutTranslation[0]; // x.
        NodeInWorldFrame[1] = BodyPos[1] + NodeInWorldWithoutTranslation[1]; // y.
        NodeInWorldFrame[2] = NodeInWorldWithoutTranslation[2]; // z.

        FrMesh::Point Pout;
        Pout[0] = NodeInWorldFrame[0];
        Pout[1] = NodeInWorldFrame[1];
        Pout[2] = NodeInWorldFrame[2];

        return Pout;

    }

    void mesh::ClippingSurface::SetBodyPosition(Position bodyPos) {
            m_bodyPosition = bodyPos;
    }

    // -----------------------------------------------------------------------------------------------------------------

    mesh::ClippingPlane::ClippingPlane(double meanHeight) : ClippingSurface(meanHeight) {}

    double mesh::ClippingPlane::GetElevation(const double &x, const double &y) const {

        return m_meanHeight;
    }

    VectorT<double, 3> mesh::ClippingPlane::GetIntersection(const VectorT<double, 3> &p0, const VectorT<double, 3> &p1) {

        // It is useless because it does not depend on the horizontal position of the mesh.

        double t = (p0[2] - m_meanHeight) / (p0[2] - p1[2]);
        return p0 * (1 - t) + p1 * t;

    }

    // -----------------------------------------------------------------------------------------------------------------

    mesh::ClippingWaveSurface::ClippingWaveSurface(const double &meanHeight, FrFreeSurface *FreeSurface) : ClippingSurface(meanHeight) {
        m_freesurface = FreeSurface;
    }

    double mesh::ClippingWaveSurface::GetElevation(const double &x, const double &y) const {

        return m_freesurface->GetPosition(x,y,NWU);
    }

    VectorT<double, 3>
    mesh::ClippingWaveSurface::GetIntersection(const VectorT<double, 3> &p0, const VectorT<double, 3> &p1) {

        // The good position of the mesh is automatically found by using the function GetDistance.
        // The position of Pout depends on the positions of p0 and p1 so no transformation is required.

        FrMesh::Point Pout;
        Pout[0] = 0;
        Pout[1] = 0;
        Pout[2] = 0;

        // Initializations.
        double z0 = GetDistance(p0); // Distance between p0 and Eta_I.
        double z1 = GetDistance(p1); // Distance between p1 and Eta_I.
        double d = std::min(fabs(z0),fabs(z1)); // Initialization of the distance beteen the intersection point and Eta_I.
        int n = 0; // Number of iterations in the dichotomy loop.
        int nmax = 1000; // Maximum number of itarations in the dichotomy loop.
        double s0 = 0.; // Curvilinear abscissa of p0;
        double s1 = 1.; // Curvilinear abscissa of p1;
        double s = 0;
        FrMesh::Point P;
        P[0] = 0;
        P[1] = 0;
        P[2] = 0;
        double deltaz;

        // Dichotomy loop.
        while((d > m_ThresholdDichotomy) && (n <= nmax)){
            n = n + 1;
            s = 0.5*(s0+s1);
            P = s*p1 + (1-s)*p0; // Equation of the straight line between p0 and p1.
            deltaz = GetDistance(P);
            if((deltaz > 0. && z1 > 0.) || (deltaz <= 0. && z1 <= 0.)){
                s1 = s;
            }
            else{
                s0 = s;
            }
            d = fabs(deltaz);
        }

        //Result.
        if(d <= m_ThresholdDichotomy){
            if(n > 0){
                // The dichotomy loop was used.
                Pout = P;
            }
            else if(n == 0){
                // The dichotomy loop was not used.
                if(std::abs(z0) <= std::abs(z1)){
                    Pout = p0;
                }
                else{
                    Pout = p1;
                }
            }
        }
        else{
            std::cout << "GetIntersection: dichotomy method cannot find the intersection point:" << std::endl;
            std::cout << "p0 = " << p0[0] << " " << p0[1] << " " << p0[2] << std::endl;
            std::cout << "p1 = " << p1[0] << " " << p1[1] << " " << p1[2] << std::endl;
            std::cout << "d = " << d << std::endl;
            std::cout << "n = " << n << std::endl;
            if(n != 0){
                std::cout << "Pout = " << Pout[0] << " " << Pout[1] << " " << Pout[2] << std::endl;
            }
        }

        return Pout;
    }



    // -----------------------------------------------------------------------------------------------------------------

    std::shared_ptr<mesh::ClippingSurface> mesh::MeshClipper::GetClippingSurface() { return m_clippingSurface; }

    void mesh::MeshClipper::Apply(mesh::FrMesh* mesh) {

        // Storage of the input mesh file.
        m_mesh = mesh;

        // Partition of the mesh.
        Initialize();

        // Clipping.
        Clip();

        // Clipped mesh cleaning (deletion of faces, update of every property, etc.).
        Finalize();
    }

    void mesh::MeshClipper::SetProjectionThresholdRatio(double projectionThresholdRatio) {
        m_ProjectionThresholdRatio = projectionThresholdRatio;
    }

    void mesh::MeshClipper::SetPlaneClippingSurface(const double &meanHeight) {

        m_clippingSurface = std::make_shared<ClippingPlane>(meanHeight);

    }

    void mesh::MeshClipper::SetWaveClippingSurface(const double &meanHeight, FrFreeSurface *FreeSurface) {

        m_clippingSurface = std::make_shared<ClippingWaveSurface>(meanHeight,FreeSurface);

    }

    void mesh::MeshClipper::Initialize() {

        // This function initializes the clipper.

        // Vector to store the faces to delete.
        c_FacesToDelete.reserve(m_mesh->n_faces()); // Worst case (fully under the clipping surface) PYW: above the clipping surface?

        // Vector to store the faces to update.
        c_FacesToUpdate.clear();

        // Partition of the vertices of the mesh.
        ClassifyVertices();
    }

    void mesh::MeshClipper::ClassifyVertices() {

        // This function classify the vertices wrt the clipping surface.

        // Iterating on vertices to get their place wrt to plane
        VertexPosition vPos;
        VertexHandle vh;

        // Loop over the vertices.
        for (FrMesh::VertexIter vh_iter = m_mesh->vertices_begin();
             vh_iter != m_mesh->vertices_end(); ++vh_iter) {

            vh = *vh_iter;

            // Computation of the distance to the plane and classification of the nodes.
            vPos = ClassifyVertex(vh);

            // Storage of the partition.
            m_mesh->data(vh).SetPositionType(vPos);

        }
    }

    void mesh::MeshClipper::Clear() {
        m_mesh->clear();
    }

    void mesh::MeshClipper::Clip() {

        /// This function performs the clipping of the mesh wrt the incident wave field.

        // Loop over faces.
        for (FrMesh::FaceIter fh_iter = m_mesh->faces_begin(); fh_iter != m_mesh->faces_end(); ++fh_iter) {
            ProcessFace(*fh_iter);
        }
    }

    void mesh::MeshClipper::UpdateModifiedFaceProperties(FaceHandle fh) {
        FrMesh::FFIter ff_iter = m_mesh->ff_iter(fh);
        for (; ff_iter.is_valid(); ++ff_iter) {
            m_mesh->update_normal(*ff_iter);
            m_mesh->CalcFacePolynomialIntegrals(*ff_iter);
        }
    }

    bool mesh::MeshClipper::HasProjection(const FaceHandle &fh) {

        /// This function peforms the clipping of a panel with the incoming wave field.

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

    void mesh::MeshClipper::ApplyFaceDeletion() {
        for (FrMesh::FaceHandle fh : c_FacesToDelete) {
            if (!m_mesh->status(fh).deleted()) {
                m_mesh->delete_face(fh);
            }
        }
        c_FacesToDelete.clear();
    }

    void mesh::MeshClipper::Finalize() {

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

};