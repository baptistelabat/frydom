//
// Created by frongere on 15/05/18.
//

#include "FrMesh.h"
#include "FrPlane.h"
#include "FrMeshClipper.h"
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
//            double intV_x2 = mesh.GetMeshSurfaceIntegral(POLY_X2);
//            double intV_y2 = mesh.GetMeshSurfaceIntegral(POLY_Y2);
//            double intV_z2 = mesh.GetMeshSurfaceIntegral(POLY_Z2);
//
//            Ixx = density * (intV_y2 + intV_z2);
//            Iyy = density * (intV_x2 + intV_z2);
//            Izz = density * (intV_x2 + intV_y2);
//
//            Iyz = -density * mesh.GetMeshSurfaceIntegral(POLY_YZ);
//            Ixz = -density * mesh.GetMeshSurfaceIntegral(POLY_XZ);
//            Ixy = -density * mesh.GetMeshSurfaceIntegral(POLY_XY);
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
//        double CalcMeshSurfaceIntegrals(const FrMesh &mesh, int iNormal, IntegrandType type) {
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
//                    value = CalcMeshSurfaceIntegrals(mesh,0,POLY_X);
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

        void FrMesh::Rotate(double phi, double theta, double psi) {

            // This function rotates the mesh.

            // Rotation matrix.
            double Norm_angles = std::sqrt(phi*phi + theta*theta + psi*psi);
            double nx = phi / Norm_angles;
            double ny = theta / Norm_angles;
            double nz = psi / Norm_angles;
            double nxny = nx*ny;
            double nxnz = nx*nz;
            double nynz = ny*nz;
            double nx2 = nx*nx;
            double ny2 = ny*ny;
            double nz2 = nz*nz;
            double ctheta = std::cos(Norm_angles);
            double stheta = std::sin(Norm_angles);

            mathutils::Matrix33<double> Rot_matrix;

            if(Norm_angles == 0){
                Rot_matrix.SetIdentity();
            }
            else{

                mathutils::Matrix33<double> Identity;
                Identity.SetIdentity();

                mathutils::Matrix33<double> Nsym;
                Nsym << nx2, nxny, nxnz,
                        nxny, ny2, nynz,
                        nxnz, nynz, nz2;

                mathutils::Matrix33<double> Nnosym;
                Nnosym << 0., -nz, ny,
                        nz, 0., -nx,
                        -ny, nx, 0.;

                Rot_matrix = ctheta*Identity + (1-ctheta)*Nsym + stheta*Nnosym;

                // Update the positions of every node.
                mathutils::Vector3d<double> Node_position;
                for (VertexIter v_iter = vertices_begin(); v_iter != vertices_end(); ++v_iter) {

                    // x = R*x (made with the same data structure).
                    Node_position[0] = point(*v_iter)[0];
                    Node_position[1] = point(*v_iter)[1];
                    Node_position[2] = point(*v_iter)[2];
                    Node_position = Rot_matrix*Node_position;
                    point(*v_iter)[0] = Node_position[0];
                    point(*v_iter)[1] = Node_position[1];
                    point(*v_iter)[2] = Node_position[2];
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

        BoundingBox FrMesh::GetBoundingBox() const {
            BoundingBox bbox;

            Point p;
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

//        const double FrMesh::GetArea() const {
//            if (!c_meshArea.IsValid()) {
//
//                double area = 0.;
//                for (FaceIter f_iter = faces_begin(); f_iter != faces_end(); ++f_iter) {
//                    area += GetArea(*f_iter);
//                }
//                c_meshArea = area;
//            }
//
//            return c_meshArea;
//        }

        const double FrMesh::GetArea(const FaceHandle &fh) const {
            return data(fh).GetSurfaceIntegral(POLY_1);
        }

//        const double FrMesh::GetVolume() const {
//            return GetMeshSurfaceIntegral(POLY_1);
//        }
//
//        const Position FrMesh::GetCOG() const {
//
//            double xb, yb, zb;
//            xb = yb = zb = 0.;
//
//            mesh::FrMesh::Normal Normal;
//
//            for (mesh::FrMesh::FaceIter f_iter = faces_begin(); f_iter != faces_end(); ++f_iter) {
//                Normal = normal(*f_iter);
//                xb += Normal[0] * data(*f_iter).GetSurfaceIntegral(mesh::POLY_X2);
//                yb += Normal[1] * data(*f_iter).GetSurfaceIntegral(mesh::POLY_Y2);
//                zb += Normal[2] * data(*f_iter).GetSurfaceIntegral(mesh::POLY_Z2);
//            }
//
//            auto volume = GetVolume();
//
//            xb /= 2. * volume;
//            yb /= 2. * volume;
//            zb /= 2. * volume; // FIXME: si on prend une cote de surface de clip non nulle, il faut ajouter la quantite ze**2 * Sf
//
//            return {xb,yb,zb};
//
//        }
//
//        const Position FrMesh::GetCOG(FrClippingPlane* plane) {
//
//            auto COG = GetCOG();
//
//            if (!CheckBoundaryPolygon(plane)) return COG;
//
//            double ze = plane->GetPlane()->GetNormaleInWorld(NWU).dot(plane->GetPlane()->GetOriginInWorld(NWU));
//
//            return COG + ze * plane->GetPlane()->GetNormaleInWorld(NWU);
//
//        }

//        const FrInertiaTensor FrMesh::GetPlainInertiaTensorAtCOG(double density) const {
//
//            auto volume = GetVolume();
//            auto mass = volume * density;
//
//            auto COG = GetCOG();
//
//            double Ixx, Iyy, Izz, Ixy, Ixz, Iyz;
//            Ixx = Iyy = Izz= Ixy= Ixz= Iyz = 0.;
//
//            mesh::FrMesh::Normal Normal;
//
//            for (mesh::FrMesh::FaceIter f_iter = faces_begin(); f_iter != faces_end(); ++f_iter) {
//                Normal = normal(*f_iter);
//                Ixx += Normal[1] * data(*f_iter).GetSurfaceIntegral(mesh::POLY_Y3) + Normal[2] * data(*f_iter).GetSurfaceIntegral(mesh::POLY_Z3);
//                Iyy += Normal[0] * data(*f_iter).GetSurfaceIntegral(mesh::POLY_X3) + Normal[2] * data(*f_iter).GetSurfaceIntegral(mesh::POLY_Z3);
//                Izz += Normal[0] * data(*f_iter).GetSurfaceIntegral(mesh::POLY_X3) + Normal[1] * data(*f_iter).GetSurfaceIntegral(mesh::POLY_Y3);
//                Ixy += Normal[0] * data(*f_iter).GetSurfaceIntegral(mesh::POLY_X2Y);
//                Ixz += Normal[2] * data(*f_iter).GetSurfaceIntegral(mesh::POLY_Z2X);
//                Iyz += Normal[1] * data(*f_iter).GetSurfaceIntegral(mesh::POLY_Y2Z);
//            }
//
//            Ixx *= density/3.;
//            Iyy *= density/3.;
//            Izz *= density/3.;
//            Ixy *= -density/2.;
//            Ixz *= -density/2.;
//            Iyz *= -density/2.;
//
//            return FrInertiaTensor(mass, Ixx, Iyy, Izz, Ixy, Ixz, Iyz, COG, NWU);
//        }
//
//        const FrInertiaTensor FrMesh::GetPlainEqInertiaTensorAtCOG(double mass) const {
//
//            return GetPlainInertiaTensorAtCOG(mass/GetVolume());
//
//        }
//
//        const FrInertiaTensor FrMesh::GetShellInertiaTensorAtCOG(double density, double thickness) const {
//
//            auto area = GetArea();
//            auto mass = area * thickness * density;
//
//            auto COG = GetShellCOG();
//            auto COGTest = GetCOG();
//
//
//            double Ixx, Iyy, Izz, Ixy, Ixz, Iyz;
//            Ixx = Iyy = Izz= Ixy= Ixz= Iyz = 0.;
//
//            for (mesh::FrMesh::FaceIter f_iter = faces_begin(); f_iter != faces_end(); ++f_iter) {
//                Ixx += data(*f_iter).GetSurfaceIntegral(mesh::POLY_Y2) + data(*f_iter).GetSurfaceIntegral(mesh::POLY_Z2);
//                Iyy += data(*f_iter).GetSurfaceIntegral(mesh::POLY_X2) + data(*f_iter).GetSurfaceIntegral(mesh::POLY_Z2);
//                Izz += data(*f_iter).GetSurfaceIntegral(mesh::POLY_X2) + data(*f_iter).GetSurfaceIntegral(mesh::POLY_Y2);
//                Ixy += data(*f_iter).GetSurfaceIntegral(mesh::POLY_YZ);
//                Ixz += data(*f_iter).GetSurfaceIntegral(mesh::POLY_XZ);
//                Iyz += data(*f_iter).GetSurfaceIntegral(mesh::POLY_XY);
//            }
//
//            Ixx *= density * thickness;
//            Iyy *= density * thickness;
//            Izz *= density * thickness;
//            Ixy *= -density * thickness;
//            Ixz *= -density * thickness;
//            Iyz *= -density * thickness;
//
//            return FrInertiaTensor(mass, Ixx, Iyy, Izz, Ixy, Ixz, Iyz, COG, NWU);
//        }
//
//        const FrInertiaTensor FrMesh::GetShellEqInertiaTensorAtCOG(double mass, double thickness) const {
//
//            GetShellInertiaTensorAtCOG(GetArea()* thickness / mass, thickness);
//
//        }

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

        double FrMesh::GetBoundaryPolygonsSurfaceIntegral(IntegrandType type) {

            // This function gives the value of a surface integral over the waterline area.

            if (!c_polygonSurfaceIntegrals.IsValid()) {
                UpdateBoundariesSurfacePolynomialIntegrals();
            }
            return c_polygonSurfaceIntegrals.Get().GetSurfaceIntegral(type);
        }

        void FrMesh::UpdateBoundariesSurfacePolynomialIntegrals() { // TODO: mettre en prive

            if (!m_polygonSet.IsValid()) {
                CalcBoundaryPolygonSet();
            }

            double Int1, IntX, IntY, IntXY, IntX2, IntY2;
            Int1 = IntX = IntY = IntXY = IntX2 = IntY2 = 0;

            PolygonSet polygonSet = m_polygonSet.Get();

            FrMesh::Point P0, P1;
            double x0, x1, y0, y1;
            double dx, dy, px, py, a, b;

            for (auto& polygon : polygonSet) {

                P0 = point(from_vertex_handle(polygon[0]));

                for (const HalfedgeHandle &heh : polygon) {

                    P1 = point(to_vertex_handle(heh));

                    x0 = P0[0];
                    y0 = P0[1];

                    x1 = P1[0];
                    y1 = P1[1];

                    dx = x1 - x0;
                    dy = y1 - y0;
                    px = x0 + x1;
                    py = y0 + y1;
                    a = x0 * x0 + x1 * x1;
                    b = y0 * y0 + y1 * y1;

                    Int1 += dy * px;
                    IntX += dy * (px * px - x0 * x1);
                    IntY += dx * (py * py - y0 * y1);
//                    IntXY += dy * (py * a + 2 * px * (x0 * y0 + x1 * y1));
                    IntXY += dy * (py * px*px + y0*x0*x0 + y1*x1*x1);
                    IntX2 += dy * a * px;
                    IntY2 += dx * b * py;

                    P0 = P1;

                }

            }

            Int1  /= 2.;
            IntX  /= 6.;
            IntY  /= -6.;
            IntXY /= 24.;
            IntX2 /= 12.;
            IntY2 /= -12.;

            c_polygonSurfaceIntegrals.Set(BoundaryPolygonSurfaceIntegrals(Int1, IntX, IntY, IntXY, IntX2, IntY2));
        }

        double FrMesh::CalcMeshSurfaceIntegrals(int iNormal, double alpha, IntegrandType type) {

            double val = 0.;
            for (FaceIter f_iter = faces_begin(); f_iter != faces_end(); ++f_iter) {
                auto n = normal(*f_iter);
                val += n[iNormal] * data(*f_iter).GetSurfaceIntegral(type);
            }

            return val * alpha;
        }



        PolygonSet FrMesh::GetBoundaryPolygonSet() { // FIXME: devrait etre const...
            if (!m_polygonSet.IsValid()) {
                CalcBoundaryPolygonSet();
            }
            return m_polygonSet;
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

                polygon.push_back(heh_init);
                status(heh_init).set_tagged(true);
                tagged_halfedges.push_back(heh_init);

                // Circulating over the boundary from heh_init until the polygon is closed
                heh = next_halfedge_handle(heh_init);
                while (heh != heh_init) {
                    polygon.push_back(heh);
                    status(heh).set_tagged(true);
                    tagged_halfedges.push_back(heh);
                    heh = next_halfedge_handle(heh);
                }

                polygonSet.push_back(polygon);
                // Updating polygon properties
//                    polygon.UpdateIntegrals();

                heh_init = FindFirstUntaggedBoundaryHalfedge();

            }

            // Removing tags
            for (HalfedgeHandle heh_ptr : tagged_halfedges) {
                status(heh_ptr).set_tagged(false);
            }

            m_polygonSet = polygonSet;
//            m_polygoneSet.Get() = polygonSet;
        }

        bool FrMesh::CheckBoundaryPolygon(FrClippingPlane *plane) {

            auto polygonSet = GetBoundaryPolygonSet();
            bool valid = !polygonSet.empty();

            for (auto& polygon : polygonSet) {

                auto distance = plane->GetDistance(point(from_vertex_handle(polygon[0])));
                valid &= (distance<1E-8);

            }

            return valid;
        }

//        std::string InertiaTensor::ReportString() const {
//            return fmt::format(
//                    "Inertias expressed at ({}\t{}\t{}):\n\tIxx = {}\n\tIyy = {}\n\tIzz = {}\n\tIyz = {}\n\tIxz = {}\n\tIxy = {}\n",
//                    calcPoint[0], calcPoint[1], calcPoint[2], Ixx, Iyy, Izz, Iyz, Ixz, Ixy);
//        }
//
//        void InertiaTensor::Transport(FrMesh::Point A) {
//            // TODO : mettre en place les procedures de transport auto via Konig Huygens
//
//        }
//
//        const mathutils::MatrixMN<double> InertiaTensor::GetTensorMatrix() const {  // TODO: faire une matrix 33 dans mathutils
//            mathutils::MatrixMN<double> inertiaMatrix(3, 3);
//            inertiaMatrix(0, 0) = Ixx;
//            inertiaMatrix(0, 1) = inertiaMatrix(1, 0) = Ixy;
//            inertiaMatrix(0, 2) = inertiaMatrix(2, 0) = Ixz;
//            inertiaMatrix(1, 1) = Iyy;
//            inertiaMatrix(1, 2) = inertiaMatrix(2, 1) = Iyz;
//            inertiaMatrix(2, 2) = Izz;
//            return inertiaMatrix;
//        }
//
//        std::string InertialProperties::ReportString() const {
//
//            fmt::MemoryWriter mw;
//
//            mw << fmt::format("Mass     : {} tons\n", m_mass / 1e3);
//            mw << fmt::format("COG      : {}\t{}\t{}\n", m_cog[0], m_cog[1], m_cog[2]);
//            mw << m_inertiaTensor.ReportString();
//
//            return mw.str();
//
//        }
    }  // end namespace mesh
}  // end namespace frydom