//
// Created by frongere on 16/05/18.
//

#include "FrMeshClipper.h"


namespace frydom {


    mesh::ClippingSurface::ClippingSurface(double meanHeight) : m_meanHeight(meanHeight) {}

    void mesh::ClippingSurface::SetMeanHeight(const double &meanHeight) { m_meanHeight = meanHeight; }

    void
    mesh::ClippingSurface::SetMeshOffsetRotation(const Position &Offset, const mathutils::Matrix33<double> &Rotation) {
        m_MeshOffset = Offset;
        m_Rotation = Rotation;
    }

    void mesh::ClippingSurface::SetBody(FrBody *body) {
        m_body = body;
    }

    VectorT<double, 3> mesh::ClippingSurface::GetNodePositionInWorld(VectorT<double, 3> point) const {

        mathutils::Vector3d<double> NodeInWorldWithoutTranslation;
        NodeInWorldWithoutTranslation[0] = point[0];
        NodeInWorldWithoutTranslation[1] = point[1];
        NodeInWorldWithoutTranslation[2] = point[2];

        // Application of the translation.
        Position NodeInWorldFrame;
        Position BodyPos = m_body->GetPosition(NWU);
        NodeInWorldFrame[0] = BodyPos[0] + NodeInWorldWithoutTranslation[0]; // x.
        NodeInWorldFrame[1] = BodyPos[1] + NodeInWorldWithoutTranslation[1]; // y.
        NodeInWorldFrame[2] = NodeInWorldWithoutTranslation[2]; // z.

        FrMesh::Point Pout;
        Pout[0] = NodeInWorldFrame[0];
        Pout[1] = NodeInWorldFrame[1];
        Pout[2] = NodeInWorldFrame[2];

        return Pout;

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


    void mesh::MeshClipper::SetMeshOffsetRotation(const Position &Offset, const mathutils::Matrix33<double> &Rotation) {
        m_clippingSurface->SetMeshOffsetRotation(Offset,Rotation);
        m_MeshOffset = Offset;
        m_Rotation = Rotation;
    }

    std::shared_ptr<mesh::ClippingSurface> mesh::MeshClipper::GetClippingSurface() { return m_clippingSurface; }

    mesh::FrMesh &mesh::MeshClipper::Apply(const mesh::FrMesh &mesh) {

        // Cleaning of the initial mesh structure.
        Clear();

        // Storage of the input mesh file.
        m_InitMesh = FrMesh(mesh);

        // Transport of the mesh from the mesh frame to the body frame, then applies the rotation of mesh in the world frame.
        UpdateMeshPositionInWorld();

        // Partition of the mesh.
        Initialize();

        // Clipping.
        Clip();

        // Clipped mesh cleaning (deletion of faces, update of every property, etc.).
        Finalize();

        return m_InitMesh;  // TODO: renvoyer un etat...
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

    void mesh::MeshClipper::SetBody(FrBody *body) {
        m_clippingSurface->SetBody(body);
        m_body = body;
    }

    void mesh::MeshClipper::UpdateMeshPositionInWorld() {

        // This function transports the mesh from the mesh frame to the body frame, then applies the rotation of mesh in the world frame.

        // Iterating on vertices to get their place wrt to plane.
        VertexHandle vh;
        Position NodeInBody, NodeInWorld;
        Position BodyPos = m_body->GetPosition(NWU);

        // Loop over the vertices.
        for (FrMesh::VertexIter vh_iter = m_InitMesh.vertices_begin();
             vh_iter != m_InitMesh.vertices_end(); ++vh_iter) {

            vh = *vh_iter;

            // From the mesh frame to the body frame.
            m_InitMesh.point(vh) = GetNodePositionInBody(m_InitMesh.point(vh));

            NodeInBody[0] = m_InitMesh.point(vh)[0];
            NodeInBody[1] = m_InitMesh.point(vh)[1];
            NodeInBody[2] = m_InitMesh.point(vh)[2];

            // Rotation from the body frame to the world frame (just the rotation and the vertical translation, not the horizontal translation of the mesh at the good position in the world mesh).
            // The horizontal translation is not done to avoid numerical errors.
            NodeInWorld = m_body->ProjectVectorInWorld<Position>(NodeInBody, NWU);

            // Vertical translation.
            NodeInWorld[2] = NodeInWorld[2] + BodyPos[2]; // x.

            m_InitMesh.point(vh)[0] = NodeInWorld[0];
            m_InitMesh.point(vh)[1] = NodeInWorld[1];
            m_InitMesh.point(vh)[2] = NodeInWorld[2];

        }

    }

    void mesh::MeshClipper::Initialize() {

        // This function initializes the clipper.

        // Vector to store the faces to delete.
        c_FacesToDelete.reserve(m_InitMesh.n_faces()); // Worst case (fully under the clipping surface) PYW: above the clipping surface?

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
        for (FrMesh::VertexIter vh_iter = m_InitMesh.vertices_begin();
             vh_iter != m_InitMesh.vertices_end(); ++vh_iter) {

            vh = *vh_iter;

            // Computation of the distance to the plane and classification of the nodes.
            vPos = ClassifyVertex(vh);

            // Storage of the partition.
            m_InitMesh.data(vh).SetPositionType(vPos);

        }
    }

    void mesh::MeshClipper::Clear() {
        m_InitMesh.clear();
    }

    void mesh::MeshClipper::Clip() {

        /// This function performs the clipping of the mesh wrt the incident wave field.

        // Loop over faces.
        for (FrMesh::FaceIter fh_iter = m_InitMesh.faces_begin(); fh_iter != m_InitMesh.faces_end(); ++fh_iter) {
            ProcessFace(*fh_iter);
        }
    }

    void mesh::MeshClipper::UpdateModifiedFaceProperties(FaceHandle fh) {
        FrMesh::FFIter ff_iter = m_InitMesh.ff_iter(fh);
        for (; ff_iter.is_valid(); ++ff_iter) {
            m_InitMesh.update_normal(*ff_iter);
            m_InitMesh.CalcFacePolynomialIntegrals(*ff_iter);
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
        FrMesh::FaceVertexIter fv_iter = m_InitMesh.fv_iter(fh);

        for (; fv_iter.is_valid(); ++fv_iter) {

            // First vertex of the edge.
            vh = *fv_iter;
            P0 = m_InitMesh.point(vh);

            // Iterating on outgoing halfedges to get the shortest edge path.
            // A vertex has several outgoing halfedges, pointing to all its neighbooring vertices.
            voh_iter = m_InitMesh.voh_iter(vh);
            for (; voh_iter.is_valid(); ++voh_iter) {

                // Hafledge.
                oheh = *voh_iter;

                // Is the halfedge clipping the surface?
                if (IsHalfEdgeCrossing(oheh)) {

                    // Second vertex of the edge.
                    P1 = m_InitMesh.point(m_InitMesh.to_vertex_handle(oheh));

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
                m_InitMesh.point(vh) = Pi_final;
                m_InitMesh.data(vh).SetOn();
                break; // We anyVertexProjected only one vertex per face at a time as other will be processed by adjacent faces
            }
        }

        return anyVertexProjected;
    }

    void mesh::MeshClipper::ApplyFaceDeletion() {
        for (FrMesh::FaceHandle fh : c_FacesToDelete) {
            if (!m_InitMesh.status(fh).deleted()) {
                m_InitMesh.delete_face(fh);
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

    VectorT<double, 3> mesh::MeshClipper::GetNodePositionInBody(VectorT<double, 3> point) const {

        // From the mesh frame to the body frame: OmP = ObOm + bRm*OmP.
        mathutils::Vector3d<double> NodeInMeshFrameVect;
        NodeInMeshFrameVect[0] = point[0];
        NodeInMeshFrameVect[1] = point[1];
        NodeInMeshFrameVect[2] = point[2];

        mathutils::Vector3d<double> TmpVect;
        TmpVect = m_Rotation*NodeInMeshFrameVect;

        Position NodeInBodyFrame;
        NodeInBodyFrame[0] = m_MeshOffset[0] + TmpVect[0];
        NodeInBodyFrame[1] = m_MeshOffset[1] + TmpVect[1];
        NodeInBodyFrame[2] = m_MeshOffset[2] + TmpVect[2];

        // Position -> point.
        FrMesh::Point Pout;
        Pout[0] = NodeInBodyFrame[0];
        Pout[1] = NodeInBodyFrame[1];
        Pout[2] = NodeInBodyFrame[2];

        return Pout;

    }
};