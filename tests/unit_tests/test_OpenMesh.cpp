//
// Created by frongere on 15/05/18.
//

//#include "frydom/frydom_dice.h"

#include "frydom/mesh/FrMeshClipper.h"
#include "frydom/mesh/FrMesh.h"
#include "frydom/mesh/FrHydrostaticsProperties.h"

using namespace frydom::mesh;
using namespace frydom;

int main() {

    FrMesh mesh("BARGE.obj");

//    std::cout << "Mesh is watertight = " << mesh.IsWatertight() << std::endl;

//    std::cout << mesh.data(*mesh.faces_begin()).GetSurfaceIntegral(POLY_1) << std::endl;

    MeshClipper clipper;
    FrMesh mesh_clipped = clipper(mesh);


//    std::cout << "Waterplane Area = " << mesh_clipped.GetBoundaryPolygonsSurfaceIntegral(POLY_1) << std::endl;


//    std::cout << mesh_clipped.data(*mesh.faces_begin()).GetSurfaceIntegral(POLY_1) << std::endl;

//    mesh_clipped.UpdateAllProperties();

//    std::cout << mesh_clipped.GetVolume();

    auto inertia = CalcPlainInertiaProperties(mesh_clipped, 1030);
    std::cout << "Full inertia : \n" << inertia.ReportString() << std::endl;


//    auto inertia = CalcShellInertiaProperties(mesh, 7850, 0.02);
//    std::cout << "Clip inertia : \n" << inertia.ReportString() << std::endl;


//    inertia = CalcShellInertiaProperties(mesh_clipped, 7850, 0.02, false);
//    std::cout << "\nShell inertia at O: \n" << inertia.ReportString();
//
//    inertia = CalcShellInertiaProperties(mesh_clipped, 7850, 0.02, true);
//    std::cout << "\nShell inertia at G (Huygens): \n" << inertia.ReportString();
//
//
//    mesh_clipped.Translate(-inertia.m_cog);
//    inertia = CalcShellInertiaProperties(mesh_clipped, 7850, 0.02);
//    std::cout << "\nShell inertia at G (translate) : \n" << inertia.ReportString();




//    mesh_clipped.Write("clip.obj");
//    auto inertiaProps = CalcShellInertiaProperties(mesh_clipped, 1030, 0.02);

    FrHydrostaticsProperties hsp(1030, 9.81);  // TODO: enlever le s de hydrostatics
    mathutils::Vector3d<double> cog(46.372, 0, 7.3);
//    hsp.Load(mesh_clipped, inertiaProps.m_cog);
    hsp.Load(mesh_clipped, cog);
    std::cout << hsp.GetReport() << std::endl;

//    std::cout << hsp.GetHydrostaticMatrix() << std::endl;

//    mesh_clipped.UpdateBoundaries();

//    for (auto boundary : boundaries) {
//        for (DMesh::Point p : boundary) {
//            std::cout << p << "\n";
//        }
//    }

//    for (DMesh::FaceIter f_iter = mesh_clipped.faces_begin(); f_iter != mesh_clipped.faces_end(); ++f_iter) {
//        std::cout << mesh_clipped.data(*f_iter).Int_x2() << "\n";
//    }



    return 0;

}