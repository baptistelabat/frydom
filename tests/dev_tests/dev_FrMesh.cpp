//
// Created by lletourn on 05/07/19.
//


#include "frydom/frydom.h"

using namespace frydom;

int main(int argc, char* argv[]) {

    // Define the frame convention (NWU for North-West-Up or NED for North-East-Down)
    FRAME_CONVENTION fc = NWU;

    // Define the wave direction convention (GOTO or COMEFROM), can be used also for current and wind direction definition.
    DIRECTION_CONVENTION dc = GOTO;

    // Create an offshore system, it contains all physical objects : bodies, links, but also environment components
    FrOffshoreSystem system;
    system.SetName("Mesh");

    // create the mesh
    mesh::FrMesh mesh;

    mesh.CreateBox(10,10,10);

    auto node = system.GetWorldBody()->NewNode();
    node->TranslateInWorld(0, 0, 5, NWU);
    auto plane = std::make_shared<FrCPlane>(node);

    auto clippingSurface = std::make_shared<mesh::FrClippingPlane>(plane);

    auto clipper = std::make_unique<mesh::FrMeshClipper>();
    clipper->SetClippingSurface(clippingSurface);

    clipper->Apply(&mesh);

    mesh.Write("clippedMesh.obj");

    std::cout<<mesh.GetVolume()<<std::endl;

    auto COG = mesh.GetCOG();
//    auto COGcorr = mesh.GetCOG(clippingSurface.get());

    std::cout<<"COG : "<<COG.GetX()<<","<<COG.GetY()<<","<<COG.GetZ()<<std::endl;
//    std::cout<<"COG corr : "<<COGcorr.GetX()<<","<<COGcorr.GetY()<<","<<COGcorr.GetZ()<<std::endl;




}