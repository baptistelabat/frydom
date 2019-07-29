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

#include "FrMesh.h"
#include "FrPlane.h"
#include "FrMeshClipper.h"
#include "FrPolygon.h"
#include "frydom/core/body/FrInertiaTensor.h"
#include "frydom/core/link/constraint/FrCGeometrical.h"

namespace frydom {
    namespace mesh {


//        FrInertiaTensor CalcPlainInertiaProperties(const FrMesh &mesh, const double density) {
//
//            auto mass = density * mesh.GetVolume();
//            auto COG = mesh.GetCOG();
//
//            double Ixx, Iyy, Izz, Ixy, Ixz, Iyz;
//
//            double intV_x2 = mesh.GetMeshedSurfaceIntegral(POLY_X2);
//            double intV_y2 = mesh.GetMeshedSurfaceIntegral(POLY_Y2);
//            double intV_z2 = mesh.GetMeshedSurfaceIntegral(POLY_Z2);
//
//            Ixx = density * (intV_y2 + intV_z2);
//            Iyy = density * (intV_x2 + intV_z2);
//            Izz = density * (intV_x2 + intV_y2);
//
//            Iyz = -density * mesh.GetMeshedSurfaceIntegral(POLY_YZ);
//            Ixz = -density * mesh.GetMeshedSurfaceIntegral(POLY_XZ);
//            Ixy = -density * mesh.GetMeshedSurfaceIntegral(POLY_XY);
//
//            return FrInertiaTensor(mass, Ixx, Iyy, Izz, Ixy, Ixz, Iyz, FrFrame(), COG, NWU);
//        }
//
//
//        FrInertiaTensor CalcPlainEqInertiaProperties(const FrMesh &mesh, double mass) {
//            return CalcPlainInertiaProperties(mesh, mass/mesh.GetVolume());
//        }
//
//
//        FrInertiaTensor CalcShellInertiaProperties(const FrMesh &mesh, double density, double thickness) {
//
//
//            double area = mesh.GetArea();
//
//            double re = density * thickness;
//
//            auto mass = re * area;
//
//            auto COG = mesh.GetCOG();
//
//            double Intx2, Inty2, Intz2, Intyz, Intxz, Intxy;
//            Intx2 = Inty2 = Intz2 = Intyz = Intxz = Intxy = 0.;
//            for (FrMesh::FaceIter f_iter = mesh.faces_begin(); f_iter != mesh.faces_end(); ++f_iter) {
//
//                Intx2 += mesh.data(*f_iter).GetSurfaceIntegral(POLY_X2);
//                Inty2 += mesh.data(*f_iter).GetSurfaceIntegral(POLY_Y2);
//                Intz2 += mesh.data(*f_iter).GetSurfaceIntegral(POLY_Z2);
//                Intyz += mesh.data(*f_iter).GetSurfaceIntegral(POLY_YZ);
//                Intxz += mesh.data(*f_iter).GetSurfaceIntegral(POLY_XZ);
//                Intxy += mesh.data(*f_iter).GetSurfaceIntegral(POLY_XY);
//
//            }
//
//            double Ixx, Iyy, Izz, Ixy, Ixz, Iyz;
//
//            // The inertia tensor is expressed at the mesh origin (not COG)
//            Ixx = re * (Inty2 + Intz2);
//            Iyy = re * (Intx2 + Intz2);
//            Izz = re * (Intx2 + Inty2);
//
//            Iyz = -re * Intyz;
//            Ixz = -re * Intxz;
//            Ixy = -re * Intxy;
//
//            return FrInertiaTensor(mass, Ixx, Iyy, Izz, Ixy, Ixz, Iyz, FrFrame(), COG, NWU);
//        }
//
//        FrInertiaTensor CalcShellEqInertiaProperties(const FrMesh &mesh, double mass, double thickness) {
//            CalcShellInertiaProperties(mesh, mass/(mesh.GetArea()*thickness), thickness);
//        }
//
//        double GetMeshedSurfaceIntegral(const FrMesh &mesh, int iNormal, IntegrandType type) {
//
//            double val = 0.;
//            for (auto f_iter = mesh.faces_begin(); f_iter != mesh.faces_end(); ++f_iter) {
//                auto n = mesh.normal(*f_iter);
//                val += n[iNormal] * mesh.data(*f_iter).GetSurfaceIntegral(type);
//            }
//
//            return val;
//
//        }
//
//        double CalcVolumeIntegrals(FrMesh &mesh, IntegrandType type) {
//
//            int in = 0;
//            double alpha = 0.;
//            IntegrandType surfaceIntegralIntegrandType = UNDEFINED_INTEGRAND;
//
//            double value;
//
//            auto polygonSet = mesh.GetBoundaryPolygonSet();
//
//            std::vector<Position> contour;
//            for (auto& polygon:polygonSet){
//                for (const auto& heh:polygon) {
//                    FrMesh::Point P0;
//                    P0 = mesh.point(mesh.from_vertex_handle(heh));
//                    contour.push_back(OpenMeshPointToVector3d<Position>(P0));
//                }
//            }
//
//            geom::FrPlane plane(contour, NWU);
//
//            auto normal = plane.GetNormal(NWU);
//
//            switch (type) {
//                case POLY_1:
//
//
//
//                    value = GetMeshedSurfaceIntegral(mesh,0,POLY_X);
//                    value += normal[0] * mesh.GetBoundaryPolygonsSurfaceIntegral(POLY_X);
//
//                    break;
//                case POLY_X:
//                    in = 0;
//                    alpha = 0.5;
//                    surfaceIntegralIntegrandType = POLY_X2;
//                    break;
//                case POLY_Y:
//                    in = 1;
//                    alpha = 0.5;
//                    surfaceIntegralIntegrandType = POLY_Y2;
//                    break;
//                case POLY_Z:
//                    in = 2;
//                    alpha = 0.5;
//                    surfaceIntegralIntegrandType = POLY_Z2;
//                    break;
//                case POLY_X2:
//                    in = 0;
//                    alpha = 1. / 3.;
//                    surfaceIntegralIntegrandType = POLY_X3;
//                    break;
//                case POLY_Y2:
//                    in = 1;
//                    alpha = 1. / 3.;
//                    surfaceIntegralIntegrandType = POLY_Y3;
//                    break;
//                case POLY_Z2:
//                    in = 2;
//                    alpha = 1. / 3.;
//                    surfaceIntegralIntegrandType = POLY_Z3;
//                    break;
//                case POLY_XY:
//                    in = 0;
//                    alpha = 0.5;
//                    surfaceIntegralIntegrandType = POLY_X2Y;
//                    break;
//                case POLY_YZ:
//                    in = 1;
//                    alpha = 0.5;
//                    surfaceIntegralIntegrandType = POLY_Y2Z;
//                    break;
//                case POLY_XZ:
//                    in = 2;
//                    alpha = 0.5;
//                    surfaceIntegralIntegrandType = POLY_Z2X;
//                    break;
//                default:
//                    std::cerr << "No volume integral can be computed for integrand of type"
//                              << type << std::endl;
//            }
//
////            double val = 0.;
////            for (auto f_iter = mesh.faces_begin(); f_iter != mesh.faces_end(); ++f_iter) {
////                auto n = mesh.normal(*f_iter);
////                val += n[in] * mesh.data(*f_iter).GetSurfaceIntegral(surfaceIntegralIntegrandType);
////            }
////
////            // Volume integration of 1 may be obtained by 3 surface integration in x, y, z. We use the mean of the three.
////            if (type == POLY_1) {
////                double valy, valz; valy = valz = 0;
////                for (auto f_iter = mesh.faces_begin(); f_iter != mesh.faces_end(); ++f_iter) {
////                    auto n = mesh.normal(*f_iter);
////                    valy += n[1] * mesh.data(*f_iter).GetSurfaceIntegral(POLY_Y);
////                    valz += n[2] * mesh.data(*f_iter).GetSurfaceIntegral(POLY_Z);
////                }
////                assert((val-valy)/val<1E-5);assert((val-valz)/val<1E-5);
//////                val /= 3.;
////            }
////
////            val *= alpha;
//
//            return value;
//        }

        FrMesh::FrMesh(std::string meshfile) {

            // Constructor of the class.

            Load(std::move(meshfile));
        }

        void FrMesh::Load(std::string meshfile) {

            // This function loads the mesh file.

            if (!IO::read_mesh(*this, meshfile)) {
                std::cerr << "Meshfile " << meshfile << " could not be read\n";
                exit(1);
            }
            UpdateAllProperties();
        }

        void FrMesh::CreateBox(double Lx, double Ly, double Lz) {

            // generate vertices
            FrMesh::VertexHandle vhandle[8];
            vhandle[0] = add_vertex(FrMesh::Point(-Lx, -Ly,  Lz)*.5);
            vhandle[1] = add_vertex(FrMesh::Point( Lx, -Ly,  Lz)*.5);
            vhandle[2] = add_vertex(FrMesh::Point( Lx,  Ly,  Lz)*.5);
            vhandle[3] = add_vertex(FrMesh::Point(-Lx,  Ly,  Lz)*.5);
            vhandle[4] = add_vertex(FrMesh::Point(-Lx, -Ly, -Lz)*.5);
            vhandle[5] = add_vertex(FrMesh::Point( Lx, -Ly, -Lz)*.5);
            vhandle[6] = add_vertex(FrMesh::Point( Lx,  Ly, -Lz)*.5);
            vhandle[7] = add_vertex(FrMesh::Point(-Lx,  Ly, -Lz)*.5);

            // generate (triangular) faces
            std::vector<FrMesh::VertexHandle>  face_vhandles;

            face_vhandles.clear();
            face_vhandles.push_back(vhandle[0]);
            face_vhandles.push_back(vhandle[1]);
            face_vhandles.push_back(vhandle[2]);
            add_face(face_vhandles);

            face_vhandles.clear();
            face_vhandles.push_back(vhandle[2]);
            face_vhandles.push_back(vhandle[3]);
            face_vhandles.push_back(vhandle[0]);
            add_face(face_vhandles);

            face_vhandles.clear();
            face_vhandles.push_back(vhandle[0]);
            face_vhandles.push_back(vhandle[4]);
            face_vhandles.push_back(vhandle[1]);
            add_face(face_vhandles);

            face_vhandles.clear();
            face_vhandles.push_back(vhandle[1]);
            face_vhandles.push_back(vhandle[4]);
            face_vhandles.push_back(vhandle[5]);
            add_face(face_vhandles);

            face_vhandles.clear();
            face_vhandles.push_back(vhandle[1]);
            face_vhandles.push_back(vhandle[5]);
            face_vhandles.push_back(vhandle[2]);
            add_face(face_vhandles);

            face_vhandles.clear();
            face_vhandles.push_back(vhandle[2]);
            face_vhandles.push_back(vhandle[5]);
            face_vhandles.push_back(vhandle[6]);
            add_face(face_vhandles);

            face_vhandles.clear();
            face_vhandles.push_back(vhandle[2]);
            face_vhandles.push_back(vhandle[6]);
            face_vhandles.push_back(vhandle[3]);
            add_face(face_vhandles);

            face_vhandles.clear();
            face_vhandles.push_back(vhandle[3]);
            face_vhandles.push_back(vhandle[6]);
            face_vhandles.push_back(vhandle[7]);
            add_face(face_vhandles);

            face_vhandles.clear();
            face_vhandles.push_back(vhandle[3]);
            face_vhandles.push_back(vhandle[7]);
            face_vhandles.push_back(vhandle[0]);
            add_face(face_vhandles);

            face_vhandles.clear();
            face_vhandles.push_back(vhandle[0]);
            face_vhandles.push_back(vhandle[7]);
            face_vhandles.push_back(vhandle[4]);
            add_face(face_vhandles);

            face_vhandles.clear();
            face_vhandles.push_back(vhandle[6]);
            face_vhandles.push_back(vhandle[5]);
            face_vhandles.push_back(vhandle[4]);
            add_face(face_vhandles);

            face_vhandles.clear();
            face_vhandles.push_back(vhandle[7]);
            face_vhandles.push_back(vhandle[6]);
            face_vhandles.push_back(vhandle[4]);
            add_face(face_vhandles);

            UpdateAllProperties();

        }

        void FrMesh::Translate(const VectorT<double, 3> t) {

            // This function translates the mesh.

            Point p;
            for (VertexIter v_iter = vertices_begin(); v_iter != vertices_end(); ++v_iter) {
                point(*v_iter) += t;
            }
            UpdateAllProperties();
        }

        void FrMesh::Rotate(const mathutils::Matrix33<double>& Rot_matrix) {

            if (!Rot_matrix.IsIdentity()) {
                // Update the positions of every node.
                mathutils::Vector3d<double> Node_position;
                for (VertexIter v_iter = vertices_begin(); v_iter != vertices_end(); ++v_iter) {

                    // x = R*x (made with the same data structure).
                    auto Node = OpenMeshPointToVector3d<Position>(point(*v_iter));
                    Node = Rot_matrix*Node;
                    point(*v_iter) = Vector3dToOpenMeshPoint(Node);

                }
            }
            UpdateAllProperties();

        }

        void FrMesh::Rotate(double phi, double theta, double psi) {

            // This function rotates the mesh.

            // Rotation matrix.
            double cphi = std::cos(phi);
            double sphi = std::sin(phi);
            double ctheta = std::cos(theta);
            double stheta = std::sin(theta);
            double cpsi = std::cos(psi);
            double spsi = std::sin(psi);

            mathutils::Matrix33<double> Rot_matrix;
            Rot_matrix.at(0, 0) = ctheta * cpsi;
            Rot_matrix.at(0, 1) = sphi * stheta * cpsi - cphi * spsi;
            Rot_matrix.at(0, 2) = cphi * stheta * cpsi + sphi * spsi;
            Rot_matrix.at(1, 0) = ctheta * spsi;
            Rot_matrix.at(1, 1) = sphi * stheta * spsi + cphi * cpsi;
            Rot_matrix.at(1, 2) = cphi * stheta * spsi - sphi * cpsi;
            Rot_matrix.at(2, 0) = -stheta;
            Rot_matrix.at(2, 1) = ctheta * sphi;
            Rot_matrix.at(2, 2) = ctheta * cphi;

            if (!Rot_matrix.IsIdentity()) {
                // Update the positions of every node.
                mathutils::Vector3d<double> Node_position;
                for (VertexIter v_iter = vertices_begin(); v_iter != vertices_end(); ++v_iter) {

                    // x = R*x (made with the same data structure).
                    auto Node = OpenMeshPointToVector3d<Position>(point(*v_iter));
                    Node = Rot_matrix*Node;
                    point(*v_iter) = Vector3dToOpenMeshPoint(Node);

                }
            }
            UpdateAllProperties();

        }

        void FrMesh::Write(std::string meshfile) const {
            if (!IO::write_mesh(*this, meshfile)) {
                std::cerr << "Meshfile " << meshfile << " could not be written\n";
                exit(1);
            }
        }

        void FrMesh::WriteInc(std::string meshfile, int i) {
            m_writer.Reinit(i);
            m_writer.SetFileBase(meshfile);
            m_writer(*this);
        }

        void FrMesh::WriteInc() {
            m_writer(*this);
        }

        void FrMesh::UpdateAllProperties() {

            // This function updates all properties of faces and vertices (normals, centroids, surface integrals).

            // Computation of normal vectors and centroids.
            UpdateBaseProperties();

            // Computation of surface polynomial integrals.
            UpdateFacesPolynomialIntegrals();
        }

        void FrMesh::UpdateBaseProperties() {

            // This function computes the normal vectors everywhere and the centroid of faces.

            // Computation of the normal vectors of faces, vertices and half edges.
            update_normals();  // Update normals for both faces and vertices

            // Update face's center properties.
            Point center;
            for (FaceIter f_iter = faces_begin(); f_iter != faces_end(); ++f_iter) {
                data(*f_iter).SetCenter(calc_face_centroid(*f_iter));
            }

        }

        void FrMesh::UpdateFacesPolynomialIntegrals() {

            // This function updates the computations of the polynomial surface integrals.

            for (FaceIter f_iter = faces_begin(); f_iter != faces_end(); ++f_iter) {
                CalcFacePolynomialIntegrals(*f_iter);
            }
        }

        void FrMesh::CalcFacePolynomialIntegrals(const FrMesh::FaceHandle &fh) {  // TODO: mettre en private

            // This function computes the polynomial surface integrals over the faces.

            typedef Vec3d Point;

            Point P0, P1, P2;
            Point t0, t1, t2;
            Point f1, f2, f3;
            Point g0, g1, g2;
            Point e1, e2, cp;

            double delta;

            // Getting one half-edge handle of the current face
            auto heh = halfedge_handle(fh);

            // Getting the origin vertex of heh
            P0 = point(from_vertex_handle(heh));

            heh = next_halfedge_handle(heh);
            P1 = point(from_vertex_handle(heh));

            heh = next_halfedge_handle(heh);
            P2 = point(from_vertex_handle(heh));

            e1 = P1 - P0;
            e2 = P2 - P0;
            cp = cross(e1, e2);
            delta = cp.norm();

            // factorized terms (optimization terms :) )
            t0 = P0 + P1;
            f1 = t0 + P2;
            t1 = P0 * P0;
            t2 = t1 + P1 * t0;
            f2 = t2 + P2 * f1;
            f3 = P0 * t1 + P1 * t2 + P2 * f2;
            g0 = f2 + P0 * (f1 + P0);
            g1 = f2 + P1 * (f1 + P1);
            g2 = f2 + P2 * (f1 + P2);

            // My Extended Eberly's Formulas.
            // Surface integrals are transformed into contour integrals.
            data(fh).SetSurfaceIntegral(POLY_1, delta / 2.);

            data(fh).SetSurfaceIntegral(POLY_X, delta * f1[0] / 6.);
            data(fh).SetSurfaceIntegral(POLY_Y, delta * f1[1] / 6.);
            data(fh).SetSurfaceIntegral(POLY_Z, delta * f1[2] / 6.);

            data(fh).SetSurfaceIntegral(POLY_YZ, delta * (6. * P0[1] * P0[2]
                                                          + 3. * (P1[1] * P1[2] + P2[1] * P2[2])
                                                          - P0[1] * f1[2] - P0[2] * f1[1]) / 12.);
            data(fh).SetSurfaceIntegral(POLY_XZ, delta * (6. * P0[0] * P0[2]
                                                          + 3. * (P1[0] * P1[2] + P2[0] * P2[2])
                                                          - P0[0] * f1[2] - P0[2] * f1[0]) / 12.);
            data(fh).SetSurfaceIntegral(POLY_XY, delta * (6. * P0[0] * P0[1]
                                                          + 3. * (P1[0] * P1[1] + P2[0] * P2[1])
                                                          - P0[0] * f1[1] - P0[1] * f1[0]) / 12.);

            data(fh).SetSurfaceIntegral(POLY_X2, delta * f2[0] / 12.);
            data(fh).SetSurfaceIntegral(POLY_Y2, delta * f2[1] / 12.);
            data(fh).SetSurfaceIntegral(POLY_Z2, delta * f2[2] / 12.);

            data(fh).SetSurfaceIntegral(POLY_X3, delta * f3[0] / 20.);
            data(fh).SetSurfaceIntegral(POLY_Y3, delta * f3[1] / 20.);
            data(fh).SetSurfaceIntegral(POLY_Z3, delta * f3[2] / 20.);

            data(fh).SetSurfaceIntegral(POLY_X2Y, delta * (P0[1] * g0[0] + P1[1] * g1[0] + P2[1] * g2[0]) / 60.);
            data(fh).SetSurfaceIntegral(POLY_Y2Z, delta * (P0[2] * g0[1] + P1[2] * g1[1] + P2[2] * g2[1]) / 60.);
            data(fh).SetSurfaceIntegral(POLY_Z2X, delta * (P0[0] * g0[2] + P1[0] * g1[2] + P2[0] * g2[2]) / 60.);

        }

        FrAABoundingBox FrMesh::GetBoundingBox() const {
            FrAABoundingBox bbox;

            Point p = point(*vertices_begin());

            bbox.xmin = fmin(bbox.xmin, p[0]);
            bbox.xmax = fmax(bbox.xmax, p[0]);
            bbox.ymin = fmin(bbox.ymin, p[1]);
            bbox.ymax = fmax(bbox.ymax, p[1]);
            bbox.zmin = fmin(bbox.zmin, p[2]);
            bbox.zmax = fmax(bbox.zmax, p[2]);

            for (VertexIter v_iter=vertices_begin(); v_iter != vertices_end(); ++v_iter) {
                p = point(*v_iter);
                bbox.xmin = fmin(bbox.xmin, p[0]);
                bbox.xmax = fmax(bbox.xmax, p[0]);
                bbox.ymin = fmin(bbox.ymin, p[1]);
                bbox.ymax = fmax(bbox.ymax, p[1]);
                bbox.zmin = fmin(bbox.zmin, p[2]);
                bbox.zmax = fmax(bbox.zmax, p[2]);
            }
            return bbox;
        }

        const double FrMesh::GetArea(const FaceHandle &fh) const {
            return data(fh).GetSurfaceIntegral(POLY_1);
        }

        const double FrMesh::GetArea() {

            double area = 0;

            for (FaceIter fh = faces_begin(); fh != faces_end(); ++fh) {
                area += GetArea(*fh);
            }

            for (auto& polygon:GetBoundaryPolygonSet()) {
                area += polygon.GetArea();
            }

            return area;

        }

        const double FrMesh::GetVolume() {

            auto volume = GetMeshedSurfaceIntegral(2,POLY_Z);

            auto polygonSet = GetBoundaryPolygonSet();

            // Boundary polygon contribution
            for (auto& polygon : polygonSet) {

                auto plane = polygon.GetPlane();
                auto frame = plane.GetFrame();

                double e = frame.GetPosition(NWU).norm();
                double wc = plane.GetNormal(NWU).Getuz();
                double R13 = frame.GetRotation().GetInverseRotationMatrix().at(0,2);
                double R23 = frame.GetRotation().GetInverseRotationMatrix().at(1,2);

                volume += wc * (e*wc*polygon.GetSurfaceIntegral(POLY_1)
                                + R13 * polygon.GetSurfaceIntegral(POLY_X)
                                + R23 * polygon.GetSurfaceIntegral(POLY_Y));

            }

            return volume;
        }

        const Position FrMesh::GetCOG() {

            auto Sux2 = GetMeshedSurfaceIntegral(0,POLY_X2);
            auto Svy2 = GetMeshedSurfaceIntegral(1,POLY_Y2);
            auto Swz2 = GetMeshedSurfaceIntegral(2,POLY_Z2);

            auto Inv2Volume = 0.5 / GetVolume();

            auto polygonSet = GetBoundaryPolygonSet();

            Position G(Sux2 * Inv2Volume, Svy2 * Inv2Volume, Swz2 * Inv2Volume);

            Position Gcorr; Gcorr.SetNull();

            for (auto& polygon : polygonSet) {

                auto plane = polygon.GetPlane();
                auto frame = plane.GetFrame();

                auto e = frame.GetPosition(NWU).norm();
                auto normal = plane.GetNormal(NWU);
                auto R = frame.GetRotation().GetInverseRotationMatrix();
//                auto R = frame.GetRotation().GetRotationMatrix();

                auto Sc = polygon.GetSurfaceIntegral(POLY_1);
                auto mux = polygon.GetSurfaceIntegral(POLY_X);
                auto muy = polygon.GetSurfaceIntegral(POLY_Y);
                auto muxy = polygon.GetSurfaceIntegral(POLY_XY);
                auto mux2 = polygon.GetSurfaceIntegral(POLY_X2);
                auto muy2 = polygon.GetSurfaceIntegral(POLY_Y2);

                Gcorr.GetX() =  R.at(0,0)*R.at(0,0) * mux2 + R.at(1,0)*R.at(1,0) * muy2 + e*e * normal.Getux()* normal.Getux() * Sc
                                + 2 * R.at(0,0)*R.at(1,0) * muxy + 2 * e* normal.Getux() * (R.at(0,0)* mux + R.at(1,0) * muy);

                Gcorr.GetY() =  R.at(0,1)*R.at(0,1) * mux2 + R.at(1,1)*R.at(1,1) * muy2 + e*e * normal.Getuy()* normal.Getuy() * Sc
                                + 2 * R.at(0,1)*R.at(1,1) * muxy + 2 * e* normal.Getuy() * (R.at(0,1)* mux + R.at(1,1) * muy);

                Gcorr.GetZ() =  R.at(0,2)*R.at(0,2) * mux2 + R.at(1,2)*R.at(1,2) * muy2 + e*e * normal.Getuz()* normal.Getuz() * Sc
                                + 2 * R.at(0,2)*R.at(1,2) * muxy + 2 * e* normal.Getuz() * (R.at(0,2)* mux + R.at(1,2) * muy);


                Gcorr = Inv2Volume * Gcorr.cwiseProduct(normal);
            }

            G += Gcorr;

            return G;
        }

        bool FrMesh::HasBoundaries() const {  // FIXME: si le maillage est non conforme mais hermetique, HasBoudaries() renvoie true et donc IsWatertight() false, c'est un faux négatif...
            for (FaceIter fh = faces_begin(); fh != faces_end(); ++fh) {
                if (is_boundary(*fh)) {
                    return true;
                }
            }
            return false;
        }

        bool FrMesh::IsWatertight() const { // FIXME: plutot utiliser l'hydrostatique (plonger le corps dans l'eau) et recuperer la resultante pour verifier qu'elle est verticale positive vers le haut.
            // Du coup par deduction, on pourra dire avec HasBoundaries() si le maillage est non conforme (hermetique mais avec frontieres).
            return !HasBoundaries();
        }

        HalfedgeHandle FrMesh::FindFirstUntaggedBoundaryHalfedge() const {  // TODO: placer en prive
            HalfedgeHandle heh;
            for (HalfedgeIter he_iter = halfedges_begin(); he_iter != halfedges_end(); ++he_iter) {
                heh = *he_iter;
                if (is_boundary(heh) && !status(heh).tagged()) {
                    return heh;
                }
            }
            return HalfedgeHandle(-1);
        }

        double FrMesh::GetMeshedSurfaceIntegral(int iNormal, IntegrandType type) {

            double val = 0.;
            for (FaceIter f_iter = faces_begin(); f_iter != faces_end(); ++f_iter) {
                auto n = normal(*f_iter);
                val += n[iNormal] * data(*f_iter).GetSurfaceIntegral(type);
            }

            return val;
        }


        double FrMesh::GetMeshedSurfaceIntegral(IntegrandType type) {

            double val = 0.;
            for (FaceIter f_iter = faces_begin(); f_iter != faces_end(); ++f_iter) {
                auto n = normal(*f_iter);
                val += data(*f_iter).GetSurfaceIntegral(type);
            }

            return val;
        }


        PolygonSet FrMesh::GetBoundaryPolygonSet() { // FIXME: devrait etre const...
            if (!m_polygonSet.IsValid()) {
                CalcBoundaryPolygonSet();
            }
            return m_polygonSet.Get();
        }

        void FrMesh::CalcBoundaryPolygonSet() {

            //  TODO: faire des tests pour verifier que le polygone frontiere suive bien la surface
            // on peut avoir une methode qui accepte une Clipping Surface et qui verifie qu'on a bien une intersection
            // avec la surface de decoupe.

            PolygonSet polygonSet;

            // buffer to keep track of the visited halfedges to reinit the tag after it is being used
            std::vector<HalfedgeHandle> tagged_halfedges;

            HalfedgeHandle heh_init, heh;

            heh_init = FindFirstUntaggedBoundaryHalfedge();
            while (heh_init.idx() != -1) {  // TODO : voir s'il n'y a pas de methode heh_init.is_valid()
                Polygon polygon;
                std::vector<Position> vertexList;

                polygon.push_back(heh_init);
                status(heh_init).set_tagged(true);
                tagged_halfedges.push_back(heh_init);

                auto vertex = OpenMeshPointToVector3d<Position>(point(from_vertex_handle(heh_init)));
                vertexList.push_back(vertex);
                vertex = OpenMeshPointToVector3d<Position>(point(to_vertex_handle(heh_init)));
                vertexList.push_back(vertex);

                // Circulating over the boundary from heh_init until the polygon is closed
                heh = next_halfedge_handle(heh_init);
                while (heh != heh_init) {
                    polygon.push_back(heh);
                    status(heh).set_tagged(true);
                    tagged_halfedges.push_back(heh);
                    vertex = OpenMeshPointToVector3d<Position>(point(to_vertex_handle(heh)));
                    vertexList.push_back(vertex);
                    heh = next_halfedge_handle(heh);
                }

                polygonSet.push_back(FrPolygon(vertexList,NWU));

                heh_init = FindFirstUntaggedBoundaryHalfedge();

            }

            // Removing tags
            for (HalfedgeHandle heh_ptr : tagged_halfedges) {
                status(heh_ptr).set_tagged(false);
            }

            m_polygonSet = polygonSet;
            
        }

        bool FrMesh::CheckBoundaryPolygon(FrClippingPlane *plane) {

            auto polygonSet = GetBoundaryPolygonSet();
            bool valid = !polygonSet.empty();

            for (auto& polygon : polygonSet) {
//                valid &= polygon.CheckBoundaryPolygon(plane);
            }

            return valid;
        }


    }  // end namespace mesh

}  // end namespace frydom