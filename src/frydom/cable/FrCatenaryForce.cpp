//
// Created by frongere on 08/09/17.
//

#include "chrono/core/ChVector.h"


#include "FrCatenaryForce.h"
#include "FrCatenaryLine.h"


#include "frydom/core/FrBody.h"


namespace frydom {


    void FrCatenaryForce::UpdateState() {

        //m_line->Update(ChTime); // FIXME : ici la mise à jour du cable est faite deux fois.
                                // TODO : separer la mise à jour de la force de celui du cable

        //m_line->solve();  // FIXME: ne pas le faire pour chacune des forces !! C'est juste un essai

        chrono::ChVector<double> relpos;
        switch (m_line_side) {
            case LINE_START:
                force = m_line->getStartingNodeTension();
                relpos = m_line->GetStartingNode()->GetPos();
                break;
            case LINE_END:
                force = -m_line->GetEndingNodeTension(); // Be carefull of the - sign
                relpos = m_line->GetEndingNode()->GetPos();
                break;
        }

//        std::cout << (m_line->GetPosEndingNode() - m_line->GetPosStartingNode()).Length() << "\t" << m_line->GetCableLength() << std::endl;

        // FIXME: Calculer le moment par rapport au point de reference du corps ?
//        Body->Dir_World2Body(force);
        moment = relpos.Cross(Body->Dir_World2Body(force));
//        moment.SetNullRotation();
    }
















    //>>>>>>>>>>>>>>>>>>>>>>>>>>>>REFACTO
    void FrCatenaryForce_::Update(double time) {
        Position relpos;
        Force ForceInWorld;

        // Get the line tension from the corresponding node
        switch (m_line_side) {
            case LINE_START:
                ForceInWorld = m_line->getStartingNodeTension(NWU);
                relpos = m_line->GetStartingNode()->GetNodePositionInBody(NWU);
                break;

            case LINE_END:
                ForceInWorld = m_line->GetEndingNodeTension(NWU);
                relpos = m_line->GetEndingNode()->GetNodePositionInBody(NWU);
                break;
        }
        // Set the tension in the world reference frame and NWU frame convention
        m_chronoForce->SetForceInWorldNWU(ForceInWorld);

        // Set the torque in body reference frame and NWU frame convention
        // FIXME: Calculer le moment par rapport au point de reference du corps ?
        m_chronoForce->SetTorqueInBodyNWU(relpos.cross(m_body->ProjectVectorInBody(ForceInWorld,NWU)));

    }


}  // end namespace frydom