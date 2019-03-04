//
// Created by frongere on 15/05/18.
//

#include "FrMesh.h"

namespace frydom {
    namespace mesh {


        InertialProperties CalcPlainInertiaProperties(const FrMesh &mesh, const double density) {

//            assert(mesh.IsWatertight());

            InertialProperties inertialProperties;

            inertialProperties.m_mass = density * mesh.GetVolume();
            inertialProperties.m_cog = OpenMeshPointToMathUtilsVector3d(mesh.GetCOG());

            double intV_x2 = mesh.GetVolumeIntegral(POLY_X2);
            double intV_y2 = mesh.GetVolumeIntegral(POLY_Y2);
            double intV_z2 = mesh.GetVolumeIntegral(POLY_Z2);

            inertialProperties.m_inertiaTensor.Ixx = density * (intV_y2 + intV_z2);
            inertialProperties.m_inertiaTensor.Iyy = density * (intV_x2 + intV_z2);
            inertialProperties.m_inertiaTensor.Izz = density * (intV_x2 + intV_y2);

            inertialProperties.m_inertiaTensor.Iyz = -density * mesh.GetVolumeIntegral(POLY_YZ);
            inertialProperties.m_inertiaTensor.Ixz = -density * mesh.GetVolumeIntegral(POLY_XZ);
            inertialProperties.m_inertiaTensor.Ixy = -density * mesh.GetVolumeIntegral(POLY_XY);

            // FIXME : appliquer Huygens pour transporter ces coefficients en G !!!!!!!!!!!! (c'est fait pour l'inertie coque !!)

            return inertialProperties;
        }


        InertialProperties CalcShellInertiaProperties(const FrMesh &mesh, double rho, double thickness) {

            InertialProperties inertialProperties;

            double area = mesh.GetArea();

            double re = rho * thickness;

            inertialProperties.m_mass = re * area;

            double xg, yg, zg;
            xg = yg = zg = 0.;
            double Intx2, Inty2, Intz2, Intyz, Intxz, Intxy;
            Intx2 = Inty2 = Intz2 = Intyz = Intxz = Intxy = 0.;
            for (FrMesh::FaceIter f_iter = mesh.faces_begin(); f_iter != mesh.faces_end(); ++f_iter) {

                xg += mesh.data(*f_iter).GetSurfaceIntegral(POLY_X);
                yg += mesh.data(*f_iter).GetSurfaceIntegral(POLY_Y);
                zg += mesh.data(*f_iter).GetSurfaceIntegral(POLY_Z);

                Intx2 += mesh.data(*f_iter).GetSurfaceIntegral(POLY_X2);
                Inty2 += mesh.data(*f_iter).GetSurfaceIntegral(POLY_Y2);
                Intz2 += mesh.data(*f_iter).GetSurfaceIntegral(POLY_Z2);
                Intyz += mesh.data(*f_iter).GetSurfaceIntegral(POLY_YZ);
                Intxz += mesh.data(*f_iter).GetSurfaceIntegral(POLY_XZ);
                Intxy += mesh.data(*f_iter).GetSurfaceIntegral(POLY_XY);

            }
            xg /= area;
            yg /= area;
            zg /= area;

            inertialProperties.m_cog = {xg, yg, zg};

            // The inertia tensor is expressed at the mesh origin (not COG)
            inertialProperties.m_inertiaTensor.Ixx = re * (Inty2 + Intz2);
            inertialProperties.m_inertiaTensor.Iyy = re * (Intx2 + Intz2);
            inertialProperties.m_inertiaTensor.Izz = re * (Intx2 + Inty2);

            inertialProperties.m_inertiaTensor.Iyz = -re * Intyz;
            inertialProperties.m_inertiaTensor.Ixz = -re * Intxz;
            inertialProperties.m_inertiaTensor.Ixy = -re * Intxy;

            // Transporting from mesh origin to COG (Generalized Huygens theorem)
            double m = inertialProperties.m_mass;
            double xg2 = xg * xg;
            double yg2 = yg * yg;
            double zg2 = zg * zg;

            inertialProperties.m_inertiaTensor.Ixx -= m * (yg2 + zg2);
            inertialProperties.m_inertiaTensor.Iyy -= m * (xg2 + zg2);
            inertialProperties.m_inertiaTensor.Izz -= m * (xg2 + yg2);

            inertialProperties.m_inertiaTensor.Iyz -= -m * yg * zg;
            inertialProperties.m_inertiaTensor.Ixz -= -m * xg * zg;
            inertialProperties.m_inertiaTensor.Ixy -= -m * xg * yg;

            inertialProperties.m_inertiaTensor.calcPoint = inertialProperties.m_cog;


            return inertialProperties;
        }

        template<typename Scalar=double>
        mathutils::Vector3d<Scalar> OpenMeshPointToMathUtilsVector3d(const mesh::FrMesh::Point &point) {
            mathutils::Vector3d<Scalar> vector(point[0], point[1], point[2]);
        }

        void meshutils::IncrementalMeshWriter::operator()(const FrMesh &mesh) {
            Write(mesh);
        }

        void meshutils::IncrementalMeshWriter::Write(const FrMesh &mesh) {
            mesh.Write(GetFilename());
            m_counter++;
        }

        void meshutils::IncrementalMeshWriter::SetFileBase(std::string base) {
            m_meshFileBase = base;
        }

        void meshutils::IncrementalMeshWriter::SetFileType(std::string fileType) {
            m_extension = fileType;
        }

        void meshutils::IncrementalMeshWriter::Reinit(int i) {
            m_counter = i;
        }

        void meshutils::IncrementalMeshWriter::Reinit() {
            m_counter = 0;
        }

        std::string meshutils::IncrementalMeshWriter::GetFilename() const {
            return m_meshFileBase + std::to_string(m_counter) + m_extension;
        }

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
            double ctheta = std::cos(theta);
            double stheta = std::sin(theta);

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
                for (VertexIter v_iter = vertices_begin(); v_iter != vertices_end(); ++v_iter) {

                    // x = R*x (made with the same data structure).
                    mathutils::Vector3d<double> Node_position;
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

            double int_x, int_y, int_z;
            double int_yz, int_xz, int_xy;
            double int_x2, int_y2, int_z2;
            double int_x3, int_y3, int_z3;
            double int_x2y, int_y2z, int_z2x;

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

        double FrMesh::GetVolumeIntegral(IntegrandType type) const {
            // TODO : mettre en cache avec DCache
            // TODO: faire un check que les integrales de surface ont ete calculees

            int in = 0;
            double alpha = 0.;
            IntegrandType surfaceIntegralIntegrandType = UNDEFINED_INTEGRAND;

            switch (type) {
                case POLY_1:
                    in = 0; // faire moyenne ?
                    surfaceIntegralIntegrandType = POLY_X;
                    alpha = 1.;
                    break;
                case POLY_X:
                    in = 1;
                    alpha = 0.5;
                    surfaceIntegralIntegrandType = POLY_X2;
                    break;
                case POLY_Y:
                    in = 1;
                    alpha = 0.5;
                    surfaceIntegralIntegrandType = POLY_Y2;
                    break;
                case POLY_Z:
                    in = 2;
                    alpha = 0.5;
                    surfaceIntegralIntegrandType = POLY_Z2;
                    break;
                case POLY_X2:
                    in = 0;
                    alpha = 1. / 3.;
                    surfaceIntegralIntegrandType = POLY_X3;
                    break;
                case POLY_Y2:
                    in = 1;
                    alpha = 1. / 3.;
                    surfaceIntegralIntegrandType = POLY_Y3;
                    break;
                case POLY_Z2:
                    in = 2;
                    alpha = 1. / 3.;
                    surfaceIntegralIntegrandType = POLY_Z3;
                    break;
                case POLY_XY:
                    in = 0;
                    alpha = 0.5;
                    surfaceIntegralIntegrandType = POLY_X2Y;
                    break;
                case POLY_YZ:
                    in = 1;
                    alpha = 0.5;
                    surfaceIntegralIntegrandType = POLY_Y2Z;
                    break;
                case POLY_XZ:
                    in = 2;
                    alpha = 0.5;
                    surfaceIntegralIntegrandType = POLY_Z2X;
                    break;
                default:
                    std::cerr << "No volume integral can be computed for integrand of type"
                              << type << std::endl;
            }

            Normal n;
            double val = 0.;
            for (FaceIter f_iter = faces_begin(); f_iter != faces_end(); ++f_iter) {
                n = normal(*f_iter);
                val += n[in] * data(*f_iter).GetSurfaceIntegral(surfaceIntegralIntegrandType);
            }

            // Volume integration of 1 may be obtained by 3 surface integration in x, y, z. We use the mean of the three.
            if (type == POLY_1) {
                for (FaceIter f_iter = faces_begin(); f_iter != faces_end(); ++f_iter) {
                    n = normal(*f_iter);
                    val += n[1] * data(*f_iter).GetSurfaceIntegral(POLY_Y);
                    val += n[2] * data(*f_iter).GetSurfaceIntegral(POLY_Z);
                }
                val /= 3.;
            }

            val *= alpha;

            return val;
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

        const double FrMesh::GetArea() const {
            if (!c_meshArea.IsValid()) {

                double area = 0.;
                for (FaceIter f_iter = faces_begin(); f_iter != faces_end(); ++f_iter) {
                    area += GetArea(*f_iter);
                }
                c_meshArea = area;
            }

            return c_meshArea;
        }

        const double FrMesh::GetArea(const FaceHandle &fh) const {
            return data(fh).GetSurfaceIntegral(POLY_1);
        }

        const double FrMesh::GetVolume() const {
            return GetVolumeIntegral(POLY_1);
        }

        const VectorT<double, 3> FrMesh::GetCOG() const {
            Point cog;
            auto volume = GetVolume();
            cog[0] = GetVolumeIntegral(POLY_X) / volume;
            cog[1] = GetVolumeIntegral(POLY_Y) / volume;
            cog[2] = GetVolumeIntegral(POLY_Z) / volume;

            return cog;
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

            BoundaryPolygonSurfaceIntegrals integrals;

            PolygonSet polygonSet = m_polygonSet.Get();

            FrMesh::Point P0, P1;
            double x0, x1, y0, y1;
            double dx, dy, px, py, a, b;
            for (auto& polygon : polygonSet) {

                P0 = point(from_vertex_handle(polygon[0]));
                for (const HalfedgeHandle& heh : polygon) {
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

                    integrals.m_Int_1  += dy * px;
                    integrals.m_Int_x  += dy * (px * px - x0 * x1);
                    integrals.m_Int_y  += dx * (py * py - y0 * y1);
                    integrals.m_Int_xy += dy * (py * a + 2 * px * (x0 * y0 + x1 * y1));
                    integrals.m_Int_x2 += dy * a * px;
                    integrals.m_Int_y2 += dx * b * py;

                    P0 = P1;

                }

                integrals.m_Int_1  /= 2.;
                integrals.m_Int_x  /= 6.;
                integrals.m_Int_y  /= -6.;
                integrals.m_Int_xy /= 24.;
                integrals.m_Int_x2 /= 12.;
                integrals.m_Int_y2 /= -12.;

            }
            c_polygonSurfaceIntegrals = integrals;
        }

        FrMesh::PolygonSet FrMesh::GetBoundaryPolygonSet() { // FIXME: devrait etre const...
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
        }

        std::string InertiaTensor::ReportString() const {
            return fmt::format(
                    "Inertias expressed at ({}\t{}\t{}):\n\tIxx = {}\n\tIyy = {}\n\tIzz = {}\n\tIyz = {}\n\tIxz = {}\n\tIxy = {}\n",
                    calcPoint[0], calcPoint[1], calcPoint[2], Ixx, Iyy, Izz, Iyz, Ixz, Ixy);
        }

        void InertiaTensor::Transport(FrMesh::Point A) {
            // TODO : mettre en place les procedures de transport auto via Konig Huygens

        }

        const mathutils::MatrixMN<double> InertiaTensor::GetTensorMatrix() const {  // TODO: faire une matrix 33 dans mathutils
            mathutils::MatrixMN<double> inertiaMatrix(3, 3);
            inertiaMatrix(0, 0) = Ixx;
            inertiaMatrix(0, 1) = inertiaMatrix(1, 0) = Ixy;
            inertiaMatrix(0, 2) = inertiaMatrix(2, 0) = Ixz;
            inertiaMatrix(1, 1) = Iyy;
            inertiaMatrix(1, 2) = inertiaMatrix(2, 1) = Iyz;
            inertiaMatrix(2, 2) = Izz;
            return inertiaMatrix;
        }

        std::string InertialProperties::ReportString() const {

            fmt::MemoryWriter mw;

            mw << fmt::format("Mass     : {} tons\n", m_mass / 1e3);
            mw << fmt::format("COG      : {}\t{}\t{}\n", m_cog[0], m_cog[1], m_cog[2]);
            mw << m_inertiaTensor.ReportString();

            return mw.str();

        }
    }  // end namespace mesh
}  // end namespace frydom