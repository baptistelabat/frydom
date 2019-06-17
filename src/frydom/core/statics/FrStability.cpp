//
// Created by lletourn on 14/06/19.
//

#include "FrStability.h"
#include "frydom/core/common/FrRotation.h"
#include "frydom/core/body/FrBody.h"
#include "frydom/core/body/FrInertiaTensor.h"
#include "frydom/environment/FrEnvironment.h"
#include "frydom/hydrodynamics/hydrostatic/FrNonlinearHydrostaticForce.h"

#include "frydom/core/link/links_lib/FrLinksLibInc.h"

namespace frydom {


    FrStability::FrStability(const std::shared_ptr<FrBody> &body, const std::shared_ptr<FrNonlinearHydrostaticForce>& force) :
    m_body(body), m_tempForce(force), m_tempInertia(body->GetInertiaTensor()) {
    }

    void FrStability::SetRotations(const std::vector<FrRotation> &rotations) {
        m_rotations = rotations;
    }

    void FrStability::AddRotation(const FrRotation &rotation) {
        m_rotations.push_back(rotation);
    }

    void FrStability::CleanRotation() {
        m_rotations.clear();
    }


    void FrStability::ComputeGZ(const FrInertiaTensor& inertia) {

        FRAME_CONVENTION fc = NWU;

        auto bodyOrigFrame = m_body->GetFrame();

        auto COGPos = inertia.GetCOGPosition(fc);

        m_body->SetInertiaTensor(inertia);


        auto CBNode = m_body->NewNode();
        CBNode->SetPositionInWorld(COGPos, fc);

        auto worldNode = system()->GetWorldBody()->NewNode();
        worldNode->SetPositionInWorld(COGPos, fc);
        auto constraint = make_prismatic_link(worldNode, CBNode, system());


        // Static equilibrium
        m_body->SetFixedInWorld(true);

        system()->Initialize();
        system()->DoAssembly();

        m_body->SetFixedInWorld(false);

        system()->SolveStaticWithRelaxation();

        system()->RemoveLink(constraint);

        for (const auto &rotation : m_rotations) {

            m_body->RotateAroundPointInBody(rotation, COGPos, fc);

            auto CBNode = m_body->NewNode();
            CBNode->SetPositionInWorld(COGPos, fc);

            auto worldNode = system()->GetWorldBody()->NewNode();
            worldNode->SetPositionInWorld(COGPos, fc);
            auto constraint = make_prismatic_link(worldNode, CBNode, system());

            // Vertical static equilibrium of the inclined assembly
            m_body->SetFixedInWorld(true);

            system()->Initialize();
            system()->DoAssembly();

            m_body->SetFixedInWorld(false);

            system()->SolveStaticWithRelaxation();

            m_GZ.push_back(m_tempForce->GetTorqueNormAtCOG()/m_tempForce->GetForceNorm());

            std::cout << "Couple de redressement at COG = " << m_tempForce->GetTorqueInWorldAtCOG(fc) << std::endl;
            std::cout << "GZ at COG = " << m_tempForce->GetTorqueNormAtCOG()/m_tempForce->GetForceNorm() << std::endl;
            std::cout << "B = " << m_tempForce->GetCenterOfBuoyancyInWorld(fc) << std::endl;

//            auto torqueAtAssemblyCOG = m_tempForce->GetTorqueInWorldAtPointInBody(refPos,fc);
//            std::cout << "Couple de redressement at assembly COG = " << torqueAtAssemblyCOG << std::endl;
//            std::cout << "GZ at assembly COG = " << torqueAtAssemblyCOG.norm()/m_tempForce->GetForceNorm() << std::endl;

            m_body->SetFrame(bodyOrigFrame);

            // Static equilibrium
            m_body->SetFixedInWorld(true);

            system()->Initialize();
            system()->DoAssembly();

            m_body->SetFixedInWorld(false);

            system()->RemoveLink(constraint);

        }

        m_body->SetInertiaTensor(m_tempInertia);

    }

    FrOffshoreSystem *FrStability::system() {
        return m_body->GetSystem();
    }

    void FrStability::WriteResults(const std::string &filename) {

        std::fstream outfile;
        outfile.open(filename + ".csv", std::fstream::out);

        outfile<<"phi;theta;psi;GZ;RightingTorque;"<<std::endl;
        outfile<<"deg;deg;deg;m;Nm;"<<std::endl;

        double phi, theta, psi;
        double mass = m_body->GetInertiaTensor().GetMass();
        for (unsigned int i = 0; i<m_rotations.size(); i++) {
            m_rotations.at(i).GetCardanAngles_DEGREES(phi, theta, psi, NWU);
            outfile<<phi<<";"<<theta<<";"<<psi<<";"<<m_GZ.at(i)<<";"<<m_GZ.at(i) * mass * system()->GetEnvironment()->GetGravityAcceleration() <<";"<<std::endl;
        }

        outfile.close();

    }

} // end namespace frydom
