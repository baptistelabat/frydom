//
// Created by frongere on 24/08/18.
//

#include "FrLink.h"


namespace frydom {

    double FrRotationMotorRegulator::SetpointCallback(const double t) {

        double error = m_angleOrder - m_motor->GetMotorRotPeriodic();


        std::cout << "ERROR AT t = " << t << " : " << error << std::endl;


        return m_controller->Get_Out(error, t);
    }




    double FrPositionShapeTanh::SetpointCallback(const double t) {
        if (t > m_t0 + c_T) {
            return m_positionOrder;
        } else {
            return 0.5 * c_dy * (1. + tanh(c_a * (t - m_t0 - 0.5 * c_T)));
        }
    }

    void FrPositionShapeTanh::SetPositionOrder(const double pos) {

        // Get the current time
        m_t0 = m_motor->GetChTime();

        m_positionOrder = pos;

        c_y0 = m_motor->GetMotorRotPeriodic();

        // FIXME : ici il faut travailler a rendre la chose robuste par rapport aux angles
        c_dy = m_positionOrder - c_y0;

        c_a = 2. * m_maxVelocity / c_dy;

        c_T = (c_dy / m_maxVelocity) * atanh( 1. - 2 * (c_y0 + m_eps) / c_dy);


    }

}  // end namespace frydom
