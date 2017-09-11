//
// Created by frongere on 08/09/17.
//

#include "FrCatenaryForce.h"
#include "FrCatenaryLine.h"


namespace frydom {


    void FrCatenaryForce::UpdateState() {

        m_line->solve();  // FIXME: ne pas le faire pour chacune des forces !! C'est juste un essai

        switch (m_line_side) {
            case LINE_START:
                force = m_line->get_starting_node_tension();
                break;
            case LINE_END:
                force = - m_line->get_ending_node_tension(); // Be carefull of the - sign
                break;
        }
        std::cout << force[0] << "\t" << force[1] << "\t" << m_line->get_cable_length() << std::endl;

        // FIXME: Calculer le moment par rapport au point de reference du corps ?
        moment.SetNull();
    }
}  // end namespace frydom