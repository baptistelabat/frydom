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

            /// Body.
            FrBody* m_body;

            /// Mesh frame offset in the body frame.
            Position m_MeshOffset;

            /// Rotation of the mesh frame compared to the body frame.
            mathutils::Matrix33<double> m_Rotation;

        public:

            /// Constructor.
            ClippingSurface() = default;

            /// Constructor and initialization of the mean height.
            explicit ClippingSurface(const double meanHeight) : m_meanHeight(meanHeight) {}

            /// This function sets the mean height of the incident wave field.
            void SetMeanHeight(const double &meanHeight) { m_meanHeight = meanHeight; }

            /// This function gives the wave elevation at a point.
            virtual double GetElevation(const double &x, const double &y) const = 0;

            /// This function gives the distance to the incident wave field.
            virtual double GetDistance(const FrMesh::Point &point) const = 0;

            /// This function gives the intersection node position between an edge and an incident wave field.
            virtual FrMesh::Point GetIntersection(const FrMesh::Point &p0, const FrMesh::Point &p1) = 0;

            /// This function sets the offset of the mesh frame in the body frame.
            void SetMeshOffsetRotation(const Position Offset, const mathutils::Matrix33<double> Rotation){
                m_MeshOffset = Offset;
                m_Rotation = Rotation;
            }

            /// This function gives the body.
            void SetBody(FrBody* body){
                m_body = body;
            }

            /// This function gives the position in the world frame at the good position (with horizontal translation) of a node in the world frame without horizontal translation.
            FrMesh::Point GetNodePositionInWorld(FrMesh::Point point) const{

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
            explicit ClippingPlane(const double meanHeight) : ClippingSurface(meanHeight) {}

            /// This function gives the elevation of the plane.
            double GetElevation(const double &x, const double &y) const override {

                return m_meanHeight;
            }

            /// This function gives the distance to the plane.
            inline virtual double GetDistance(const FrMesh::Point &point) const {

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
            FrMesh::Point GetIntersection(const FrMesh::Point &p0, const FrMesh::Point &p1) override {

                // It is useless because it does not depend on the horizontal position of the mesh.

                double t = (p0[2] - m_meanHeight) / (p0[2] - p1[2]);
                return p0 * (1 - t) + p1 * t;

            }

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
            explicit ClippingWaveSurface(const double &meanHeight, FrFreeSurface* FreeSurface) : ClippingSurface(meanHeight) {
                m_freesurface = FreeSurface;
            }

            /// This function gives the wave elevation of the incident wave field.
            double GetElevation(const double &x, const double &y) const override {

                return m_freesurface->GetPosition(x,y,NWU);
            }

            /// This function gives the distance to the incident wave.
            inline virtual double GetDistance(const FrMesh::Point &point) const {

                // It is necessary to add the horizontal translation to the mesh because of the wave elevation which depends on the node position in the world frame.
                FrMesh::Point PointInWorld = GetNodePositionInWorld(point);
                return PointInWorld[2] - GetElevation(PointInWorld[0],PointInWorld[1]);

            }

            /// This function performs a bisection method to track the intersection node.
            FrMesh::Point GetIntersection(const FrMesh::Point &p0, const FrMesh::Point &p1) override {

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
        };

        class MeshClipper {

        private:

            /// Initial mesh.
            FrMesh m_InitMesh;

            /// Clipping surface, by default the plane z = 0.
            std::shared_ptr<ClippingSurface> m_clippingSurface = std::make_shared<ClippingPlane>(0.);

            double m_Threshold = 1e-4;
            double m_ProjectionThresholdRatio = 1 / 4.;

            /// Vector to store the faces which are on and/or above the incident free surface and have to be deleted.
            std::vector<FrMesh::FaceHandle> c_FacesToDelete;

            /// Vector to store the faces which need to be clipped.
            std::vector<FrMesh::FaceHandle *> c_FacesToUpdate;

            /// Body.
            FrBody* m_body;

            /// Mesh frame offset in the body frame.
            Position m_MeshOffset;

            /// Rotation of the mesh frame compared to the body frame.
            mathutils::Matrix33<double> m_Rotation;


        public:

            MeshClipper() = default;

            /// This function gives the clipping surface.
            std::shared_ptr<ClippingSurface> GetClippingSurface() { return m_clippingSurface; }

            /// This function initializes the MeshClipper object from an input mesh and performs the clipping.
            FrMesh & Apply(const FrMesh &mesh) {

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

            void SetEps(double eps) { m_Threshold = eps; }

            void SetProjectionThresholdRatio(double projectionThresholdRatio) {
                m_ProjectionThresholdRatio = projectionThresholdRatio;
            }

            /// This function sets a clipping plane.
            void SetPlaneClippingSurface(const double &meanHeight = 0.) {

                m_clippingSurface = std::make_shared<ClippingPlane>(meanHeight);

            }

            /// This function sets a clipping wave surface.
            void SetWaveClippingSurface(const double &meanHeight, FrFreeSurface *FreeSurface) {

                m_clippingSurface = std::make_shared<ClippingWaveSurface>(meanHeight,FreeSurface);

            }

            /// This function sets the offset and the rotation matrix of the mesh frame in the body frame in the clipping surface object.
            void SetMeshOffsetRotation(const Position Offset, const mathutils::Matrix33<double> Rotation){
                m_clippingSurface->SetMeshOffsetRotation(Offset,Rotation);
                m_MeshOffset = Offset;
                m_Rotation = Rotation;
            }

            /// This function sets the body in the clipping surface object.
            void SetBody(FrBody* body){
                m_clippingSurface->SetBody(body);
                m_body = body;
            }

            void UpdateMeshPositionInWorld(){

                // This function transports the mesh from the mesh frame to the body frame, then applies the rotation of mesh in the world frame.

                // Iterating on vertices to get their place wrt to plane
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

        private:

            void Initialize() {

                // This function initializes the clipper.

                // Vector to store the faces to delete.
                c_FacesToDelete.reserve(m_InitMesh.n_faces()); // Worst case (fully under the clipping surface) PYW: above the clipping surface?

                // Vector to store the faces to update.
                c_FacesToUpdate.clear();

                // Partition of the vertices of the mesh.
                ClassifyVertices();
            }

            void Clear() {
                m_InitMesh.clear();
            }

            void ClassifyVertices() {

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
                FrMesh::FaceVertexIter fv_iter = m_InitMesh.fv_iter(fh);
                for (; fv_iter.is_valid(); ++fv_iter) {

                    if (m_InitMesh.data(*fv_iter).IsAbove()) {
                        ++nbAbove;
                    }
                    if (m_InitMesh.data(*fv_iter).IsUnder()) {
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

            void Clip() {

                /// This function performs the clipping of the mesh wrt the incident wave field.

                // Loop over faces.
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

            bool HasProjection(const FrMesh::FaceHandle &fh) {

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

            inline void ProcessHalfEdge(FrMesh::HalfedgeHandle heh) {

                /// This function performs the clipping of a halfedge, the computation of the intersection node, the creation of new panels and the deletion of useless panels and vertices.

                FrMesh::VertexHandle vh;  // FIXME: au final, on va juste effectuer l'intersection vu qu'on va projeter les

                // Intersection node.
                vh = InsertIntersectionVertex(heh);

                // Clipping, updating of the mesh, deletion of useless panels and vertices.
                m_InitMesh.split(m_InitMesh.edge_handle(heh), vh);

                // Updating the faces to delete.
                FlagVertexAdjacentFacesToBeDeleted(vh);

            }

            inline void FlagFaceToBeDeleted(const FrMesh::FaceHandle &fh) {

                /// This function adds a face to the vector which stores the faces to delete.

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

                /// This function checks if an edge crossed the free surface or not.

                FrMesh::HalfedgeHandle heh = m_InitMesh.halfedge_handle(eh, 0);
                double dz_0 = GetVertexDistanceToSurface(m_InitMesh.from_vertex_handle(heh));
                double dz_1 = GetVertexDistanceToSurface(m_InitMesh.to_vertex_handle(heh));
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

                return IsEdgeCrossing(m_InitMesh.edge_handle(heh));
            }

            inline bool IsHalfEdgeDownCrossing(const FrMesh::HalfedgeHandle &heh) {

                /// This function checks if a halfedge crossed the free surface downwardly.

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

                /// This function checks if a halfedge crossed the free surface upwardly.

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

                /// This function tracks the halfedge which crosses the free surface upwardly.

                // TODO: throw error if no upcrossing halfedge is found
                FrMesh::HalfedgeHandle heh = m_InitMesh.halfedge_handle(fh);
                while (!IsHalfEdgeUpCrossing(heh)) {
                    heh = m_InitMesh.next_halfedge_handle(heh);
                }
                return heh;
            }

            inline FrMesh::HalfedgeHandle FindDowncrossingHalfEdge(const FrMesh::FaceHandle &fh) {

                /// This function tracks the halfedge which crosses the free surface downwardly.

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

                /// This function adds an intersection node of an edge as a new vertex of the mesh.

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
                    const FrMesh::VertexHandle &vh) const {

                /// This function gives the distance of a node to the incident wave field.

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

            /// This function gives the position in the body frame of a node in the mesh frame.
            FrMesh::Point GetNodePositionInBody(FrMesh::Point point) const{

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

    }  // end namespace mesh
}  // end namespace frydom


#endif //FRYDOM_DICE_MESHCLIPPER_H
