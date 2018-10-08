//
// Created by frongere on 05/10/18.
//

#include <frydom/core/FrException.h>
#include "FrCatway.h"
#include "catenary/Catenary.h"

#include "frydom/core/FrNode.h"
#include "frydom/core/FrForce.h"
#include "frydom/core/FrBody.h"


namespace frydom {


    FrCatway::FrCatway(double youngModulus, double diameter, double linearDensity, double length,
                       unsigned int nbElt, std::shared_ptr<FrNode_> startNode, std::shared_ptr<FrNode_> endNode) {

        auto props = catenary::make_properties(youngModulus, MU_PI * pow(0.5*diameter, 2), linearDensity, true);

        // Creating catenary nodes associated to FrNodes
        m_startNode = std::move(startNode);
        m_endNode   = std::move(endNode);

        // Creating the line
        m_startCatNode = catenary::make_node();
        m_endCatNode   = catenary::make_node();
        m_catLine = std::make_unique<catenary::CatenaryLine>(props, m_startCatNode, m_endCatNode, length);

        // Discretizing the line
//        m_catLine->Discretize(nbElt);

    }

    FrCatway::~FrCatway() {}

    void FrCatway::Update() {

        // Updating catway's node position from frydom nodes
        m_startCatNode->SetPosition(m_startNode->GetAbsPosition());
        m_endCatNode->SetPosition(m_endNode->GetAbsPosition());

        // Solving catenary line
        m_catLine->Solve();

        // Updating forces
        auto startTension = (Force)m_catLine->GetStartNodeTension();
        auto endTension   = (Force)m_catLine->GetEndNodeTension();

        // TODO : terminer



    }

    void FrCatway::Initialize() {

        // Initializing catenary nodes positions
        m_startCatNode->SetPosition(m_startNode->GetAbsPosition());
        m_startCatNode->SetFixed(true);
        m_endCatNode->SetPosition(m_endNode->GetAbsPosition());
        m_endCatNode->SetFixed(true);

//        std::cout << m_endCatNode << std::endl;

        m_catLine->Initialize();


        // Creating forces on attached bodies
        m_startForce = std::make_shared<FrCatForce>(this, m_startNode);
        m_endForce   = std::make_shared<FrCatForce>(this, m_endNode);

        // Adding forces to body
        m_startNode->GetBody()->AddExternalForce(m_startForce);
        m_endNode->GetBody()->AddExternalForce(m_endForce);

    }

    void FrCatway::StepFinalize() {
        // TODO
    }

    Force FrCatway::GetTension(const double s) const {
        return (Force)m_catLine->GetTension(s);
    }

    Force FrCatway::GetStartNodeTension() const {
        return (Force)m_catLine->GetStartNodeTension();
    }

    Force FrCatway::GetEndNodeTension() const {
        return (Force)m_catLine->GetEndNodeTension();
    }

    Position FrCatway::GetAbsPosition(const double s) const {
        return (Position)m_catLine->GetAbsPosition(s);
    }










    FrCatForce::FrCatForce(frydom::FrCatway *catenaryLine, std::shared_ptr<FrNode_> node) :
            FrForce_(std::move(node)), m_catenaryLine(catenaryLine) {
    }

    void FrCatForce::Update(double time) {
        // TODO
        // Forces are updated directly by catenary lines

        // Getting tensions
        Force tension;

        switch (m_side) {
            case START:
                tension = m_catenaryLine->GetStartNodeTension();
                break;
            case END:
                tension = m_catenaryLine->GetEndNodeTension();
                break;
            default:
                throw FrException("Unknown cable SIDE");
        }

        // On update les 2 forces ...
        // TODO : terminer






    }

    void FrCatForce::Initialize() {
        // TODO  : rien a faire ?
    }

    void FrCatForce::StepFinalize() {
        // TODO : Logging..
    }

    void FrCatForce::SetAbsTension(const Force& tension) {
        // TODO
        // Ici, on veut projeter sur le repere du corps et transporter le moment resultant en G...








    }





}  // end namespace frydom
