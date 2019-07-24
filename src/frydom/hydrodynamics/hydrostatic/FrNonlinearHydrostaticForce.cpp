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

#include "FrNonlinearHydrostaticForce.h"

#include "frydom/core/body/FrBody.h"

#include "frydom/mesh/FrMesh.h"
#include "frydom/mesh/FrHydroMesh.h"

#include "frydom/environment/FrEnvironment.h"
#include "frydom/environment/ocean/FrOcean.h"
#include "frydom/environment/ocean/freeSurface/FrFreeSurface.h"
#include "frydom/environment/ocean/freeSurface/tidal/FrTidalModel.h"

namespace frydom {

    FrNonlinearHydrostaticForce::FrNonlinearHydrostaticForce(const std::shared_ptr<FrHydroMesh> &HydroMesh) {
        m_hydroMesh = HydroMesh;
    }

    void FrNonlinearHydrostaticForce::AddFields(){

        FrForce::AddFields();

        // This function initializes the logger for the nonlinear hydrostatic loads by giving the position of the center of buoyancy in the body frame.

        m_message->AddField<Eigen::Matrix<double, 3, 1>>
                ("CenterOfBuoyancyInBody","m", fmt::format("Center of buoyancy in body reference frame in {}", GetLogFrameConvention()),
                 [this]() {return GetCenterOfBuoyancyInBody(GetLogFrameConvention());});
        m_message->AddField<Eigen::Matrix<double, 3, 1>>
                ("CenterOfBuoyancyInWorld","m", fmt::format("Center of buoyancy in world reference frame in {}", GetLogFrameConvention()),
                 [this]() {return GetCenterOfBuoyancyInWorld(GetLogFrameConvention());});
        m_message->AddField<Eigen::Matrix<double, 3, 1>>
                ("Test","m", "Test",
                 [this]() {return Test();});
        m_message->AddField<Eigen::Matrix<double, 3, 1>>
                ("HydrostaticTorqueInWorld","Nm", "HydrostaticTorqueInWorld",
                 [this]() {return GetHydrostaticTorqueInWorld(GetLogFrameConvention());});

//        m_message->AddField<Eigen::Matrix<double, 3, 1>>
//                ("ForceInWorld","N", fmt::format("Hydrostatic force, at CoB, in world reference frame in {}", GetLogFrameConvention()),
//                 [this]() {return GetHydrostaticForceInWorld(GetLogFrameConvention());});
//        m_message->AddField<Eigen::Matrix<double, 3, 1>>
//                ("ForceInBody","N", fmt::format("Hydrostatic force, at CoB, in body reference frame in {}", GetLogFrameConvention()),
//                 [this]() {return GetHydrostaticForceInBody(GetLogFrameConvention());});

    }

    void FrNonlinearHydrostaticForce::Compute(double time) {

        Position meshPos = m_body->GetPosition(NWU); meshPos.GetX() = 0; meshPos.GetY() = 0;

        // This function computes the nonlinear hydrostatic loads.
//        SetForceTorqueInWorldAtPointInWorld(GetHydrostaticForceInWorld(NWU), GetHydrostaticTorqueInWorld(NWU), meshPos, NWU);
        SetForceInWorldAtPointInWorld(GetHydrostaticForceInWorld(NWU), GetCenterOfBuoyancyInWorld(NWU), NWU);
//        SetForceInWorldAtPointInWorld(GetHydrostaticForceInWorld(NWU), Test(), NWU);

    }


    Position FrNonlinearHydrostaticForce::GetCenterOfBuoyancyInBody(FRAME_CONVENTION fc){
        return m_body->GetPointPositionInBody(GetCenterOfBuoyancyInWorld(fc),fc);
    }

    Position FrNonlinearHydrostaticForce::Test() {

//        auto clippedMesh = &(m_hydroMesh->GetClippedMesh());
////        clippedMesh->Write("TorqueMesh.obj");
//
//        std::vector<double> Ms1, Ms2, Ms3;
//
//        // Loop over the faces.
//        for (auto f_iter = clippedMesh->faces_begin(); f_iter != clippedMesh->faces_end(); ++f_iter) {
//
//            // Normal.
//            auto normal = clippedMesh->normal(*f_iter);
//
//            double xz = clippedMesh->data(*f_iter).GetSurfaceIntegral(mesh::POLY_XZ)* m_body->GetSystem()->GetGravityAcceleration() * m_body->GetSystem()->GetEnvironment()->GetFluidDensity(WATER);
//            double yz = clippedMesh->data(*f_iter).GetSurfaceIntegral(mesh::POLY_YZ)* m_body->GetSystem()->GetGravityAcceleration() * m_body->GetSystem()->GetEnvironment()->GetFluidDensity(WATER);
//            double zz = clippedMesh->data(*f_iter).GetSurfaceIntegral(mesh::POLY_Z2)* m_body->GetSystem()->GetGravityAcceleration() * m_body->GetSystem()->GetEnvironment()->GetFluidDensity(WATER);
//
//            Ms1.push_back(yz*normal[2]); Ms1.push_back(- zz*normal[1]);
//            Ms2.push_back(zz*normal[0]); Ms2.push_back(- xz*normal[2]);
//            Ms3.push_back(xz*normal[1]); Ms3.push_back(- yz*normal[0]);
//
//        }
//
//        Torque Test;
//        Test.GetMx() = NeumaierSum(Ms1);
//        Test.GetMy() = NeumaierSum(Ms2);
//        Test.GetMz() = NeumaierSum(Ms3);
//
//        return Test;




        auto Fhs = GetHydrostaticForceInWorld(NWU);

        auto Mhs = GetHydrostaticTorqueInWorld(NWU);

        auto absPos = Mhs.norm() / Fhs.norm();

        auto dir = Fhs.cross(Mhs); dir.normalize();

        return absPos * dir;

//        return {-Mhs.GetMy() / Fhs.GetFz(), Mhs.GetMx() / Fhs.GetFz(), 0.};



//        auto clippedMesh = &(m_hydroMesh->GetClippedMesh());
//
//        Position COB = {};
//
//        auto F = clippedMesh->GetMeshedSurfaceIntegral(mesh::POLY_Z);
//
//        // Loop over the faces.
//        for (auto f_iter = clippedMesh->faces_begin(); f_iter != clippedMesh->faces_end(); ++f_iter) {
//
//            auto normal = clippedMesh->normal(*f_iter);
//
//            auto xz = clippedMesh->data(*f_iter).GetSurfaceIntegral(mesh::POLY_XZ);
//            auto yz = clippedMesh->data(*f_iter).GetSurfaceIntegral(mesh::POLY_YZ);
//            auto zz = clippedMesh->data(*f_iter).GetSurfaceIntegral(mesh::POLY_Z2);
//
////            COB.GetX() += xz * normal[0];
////            COB.GetY() += yz * normal[1];
////            COB.GetZ() += zz * normal[2];
//            COB.GetX() += xz;
//            COB.GetY() += yz;
//            COB.GetZ() += zz;
//
//        }
//
////        COB *= m_body->GetSystem()->GetGravityAcceleration() * m_body->GetSystem()->GetEnvironment()->GetFluidDensity(WATER);
//
////        COB.GetX() /= Fhs.GetFx();
////        COB.GetY() /= Fhs.GetFy();
////        COB.GetZ() /= Fhs.GetFz();
//
//        COB /= 2.*F;
//
//        return COB;

    }

    // Not working for clipped mesh with wave plane
    Position FrNonlinearHydrostaticForce::GetCenterOfBuoyancyInWorld(FRAME_CONVENTION fc) {

//        auto Fhs = GetHydrostaticForceInWorld(fc);
//
//        auto clippedMesh = &(m_hydroMesh->GetClippedMesh());
//
//        Position COB = {};
//
//        auto F = clippedMesh->GetMeshedSurfaceIntegral(mesh::POLY_Z);
//
//        // Loop over the faces.
//        for (auto f_iter = clippedMesh->faces_begin(); f_iter != clippedMesh->faces_end(); ++f_iter) {
//
//            auto normal = clippedMesh->normal(*f_iter);
//
//            auto xz = clippedMesh->data(*f_iter).GetSurfaceIntegral(mesh::POLY_XZ);
//            auto yz = clippedMesh->data(*f_iter).GetSurfaceIntegral(mesh::POLY_YZ);
//            auto zz = clippedMesh->data(*f_iter).GetSurfaceIntegral(mesh::POLY_Z2);
//
////            COB.GetX() += xz * normal[0];
////            COB.GetY() += yz * normal[1];
////            COB.GetZ() += zz * normal[2];
//            COB.GetX() += xz;
//            COB.GetY() += yz;
//            COB.GetZ() += zz;
//
//        }
//
////        COB *= m_body->GetSystem()->GetGravityAcceleration() * m_body->GetSystem()->GetEnvironment()->GetFluidDensity(WATER);
//
////        COB.GetX() /= Fhs.GetFx();
////        COB.GetY() /= Fhs.GetFy();
////        COB.GetZ() /= Fhs.GetFz();
//
//        COB /= 2.*F;
//
//        return COB;

//        std::cout<<"COB =           ("<<COB.GetX()<<","<<COB.GetY()<<","<<COB.GetZ()<<")"<<std::endl;


//        auto Fh = GetHydrostaticForceInWorld(fc).GetFz();
//
//        auto Mh = GetHydrostaticTorqueInWorld(fc);
//
//
//        Torque Ms = {0., 0., 0.};
//
//        auto clippedMesh = &(m_hydroMesh->GetClippedMesh());
//
//        // Loop over the faces.
//        for (auto f_iter = clippedMesh->faces_begin(); f_iter != clippedMesh->faces_end(); ++f_iter) {
//
//            // Normal.
//            auto normal = clippedMesh->normal(*f_iter);
//
//            double xx = clippedMesh->data(*f_iter).GetSurfaceIntegral(mesh::POLY_X2);
//            double xy = clippedMesh->data(*f_iter).GetSurfaceIntegral(mesh::POLY_XY);
//            double xz = clippedMesh->data(*f_iter).GetSurfaceIntegral(mesh::POLY_XZ);
//
//            // Hydrostatic torque without the term rho*g.
//            Ms[0] += xy*normal[2] - xz*normal[1];
//            Ms[1] += xz*normal[0] - xx*normal[2];
//            Ms[2] += xx*normal[1] - xy*normal[0];
//
//        }
//
//        // Multiplication by rho*g
//        Ms *= m_body->GetSystem()->GetGravityAcceleration() * m_body->GetSystem()->GetEnvironment()->GetFluidDensity(WATER);
//
////        assert(- Ms.GetMz() - Mh.GetMx() < 1E-5);
//
//        Position Test(-Mh.GetMy() / Fh, Mh.GetMx() / Fh, -Ms.GetMy() / Fh);
//        //--------------------------------------------------------------------------------------------------------------
//
//
//        Position COB = {0,0,0};
//        // Loop over the faces.
//        for (auto f_iter = clippedMesh->faces_begin(); f_iter != clippedMesh->faces_end(); ++f_iter) {
//
//            // Normal.
//            auto normal = clippedMesh->normal(*f_iter);
//
//            double x2 = clippedMesh->data(*f_iter).GetSurfaceIntegral(mesh::POLY_X2);
//            double y2 = clippedMesh->data(*f_iter).GetSurfaceIntegral(mesh::POLY_Y2);
//            double z2 = clippedMesh->data(*f_iter).GetSurfaceIntegral(mesh::POLY_Z2);
//
//            COB += (x2 + y2 +z2) * Position(normal[0],normal[1],normal[2]);
//
//        }
//
//        auto volume = clippedMesh->GetVolume();
//        COB /= 2*volume;
        
        
        //--------------------------------------------------------------------------------------------------------------

        // clipped mesh is expressed in the world reference frame, but its horizontal position is centered around (0.,0.)
        auto CoBInWorld = m_hydroMesh->GetClippedMesh().GetCOG();

        // Addition of the horizontal position of the body
        auto bodyPos = m_body->GetPosition(NWU); bodyPos.GetZ() = 0.;
        CoBInWorld += bodyPos;

        if (IsNED(fc)) internal::SwapFrameConvention<Position>(CoBInWorld);
//        std::cout<<"CoBInWorld =    ("<<CoBInWorld.GetX()<<","<<CoBInWorld.GetY()<<","<<CoBInWorld.GetZ()<<")"<<std::endl;

        return CoBInWorld;
    }

//    Position FrNonlinearHydrostaticForce::GetCenterOfBuoyancyInWorld(FRAME_CONVENTION fc) {
//
//        // clipped mesh is expressed in the world reference frame, but its horizontal position is centered around (0.,0.)
//        auto CoBInWorld = m_hydroMesh->GetClippedMesh().GetCOG();
//
//        // Addition of the horizontal position of the body
//        auto bodyPos = m_body->GetPosition(NWU); bodyPos.GetZ() = 0.;
//        CoBInWorld += bodyPos;
//
//        if (IsNED(fc)) internal::SwapFrameConvention<Position>(CoBInWorld);
//
//        return CoBInWorld;
//    }

    Force FrNonlinearHydrostaticForce::GetHydrostaticForceInBody(FRAME_CONVENTION fc) {
        return m_body->ProjectVectorInBody(GetHydrostaticForceInWorld(fc),fc);
    }

    Force FrNonlinearHydrostaticForce::GetHydrostaticForceInWorld(FRAME_CONVENTION fc) {
        // This function performs the hydrostatic pressure integration.

        Force hydrostaticForce = {0.,0.,0.};
        
        auto clippedMesh = &(m_hydroMesh->GetClippedMesh());

        // Loop over the faces.
        for (auto f_iter = clippedMesh->faces_begin(); f_iter != clippedMesh->faces_end(); ++f_iter) {

            // Normal.
            auto normal = clippedMesh->normal(*f_iter);

            // Pressure*Area.
            auto pressure = clippedMesh->data(*f_iter).GetSurfaceIntegral(mesh::POLY_Z);

            // Hydrostatic force without the term rho*g.
            hydrostaticForce[0] += pressure*normal[0];
            hydrostaticForce[1] += pressure*normal[1];
            hydrostaticForce[2] += pressure*normal[2];

        }

        // Multiplication by rho*g
        hydrostaticForce *= m_body->GetSystem()->GetGravityAcceleration() * m_body->GetSystem()->GetEnvironment()->GetFluidDensity(WATER);

        if (IsNED(fc)) internal::SwapFrameConvention<Position>(hydrostaticForce);

        return hydrostaticForce;
    }

    Torque FrNonlinearHydrostaticForce::GetHydrostaticTorqueInWorld(FRAME_CONVENTION fc) {

        Torque Ms = {0., 0., 0.};

        auto clippedMesh = &(m_hydroMesh->GetClippedMesh());
//        clippedMesh->Write("TorqueMesh.obj");

        // Loop over the faces.
        for (auto f_iter = clippedMesh->faces_begin(); f_iter != clippedMesh->faces_end(); ++f_iter) {

            // Normal.
            auto normal = clippedMesh->normal(*f_iter);

            double xz = clippedMesh->data(*f_iter).GetSurfaceIntegral(mesh::POLY_XZ);
            double yz = clippedMesh->data(*f_iter).GetSurfaceIntegral(mesh::POLY_YZ);
            double zz = clippedMesh->data(*f_iter).GetSurfaceIntegral(mesh::POLY_Z2);

            // Hydrostatic torque without the term rho*g.
            Ms[0] += yz*normal[2] - zz*normal[1];
            Ms[1] += zz*normal[0] - xz*normal[2];
            Ms[2] += xz*normal[1] - yz*normal[0];

        }

        auto COG = clippedMesh->GetCOG();

        // Multiplication by rho*g
        Ms *= m_body->GetSystem()->GetGravityAcceleration() * m_body->GetSystem()->GetEnvironment()->GetFluidDensity(WATER);

        if (IsNED(fc)) internal::SwapFrameConvention<Position>(Ms);

        return Ms;

    }

    Torque FrNonlinearHydrostaticForce::GetHydrostaticTorqueInBody(FRAME_CONVENTION fc) {
        return m_body->ProjectVectorInBody(GetHydrostaticTorqueInWorld(fc),fc);
    }

    std::shared_ptr<FrNonlinearHydrostaticForce>
    make_nonlinear_hydrostatic_force(const std::shared_ptr<FrBody>& body, const std::shared_ptr<FrHydroMesh>& HydroMesh){

        // This function creates a (fully or weakly) nonlinear hydrostatic force object.

        // Construction of the (fully or weakly) nonlinear hydrostatic force object.
        auto forceHst = std::make_shared<FrNonlinearHydrostaticForce>(HydroMesh);

        // Add the (fully or weakly) nonlinear hydrostatic force object as an external force to the body.
        body->AddExternalForce(forceHst);

        return forceHst;
    }


    double NeumaierSum(std::vector<double> vector) {

        double sum = 0.;
        double compensation = 0.;

        for (auto& in:vector) {

            auto t = sum + in;

            if (std::abs(sum) >= std::abs(in)){
                compensation += (sum - t) + in;
            }
            else {
                compensation += (in - t) + sum;
            }

            sum = t;
        }

        return sum + compensation;
    }


}  // end namespace frydom