// =============================================================================
// PROJECT FRyDoM
//
// Copyright (c) 2017 Ecole Centrale de Nantes
// All right reserved.
//
//
// =============================================================================
// Authors: Francois Rongere
// =============================================================================
//
// Base for marine current modeling
//
// =============================================================================

#include <math.h>

#include "chrono/core/ChVector.h"

#include "FrCurrent.h"
#include "../../core/FrOffshoreSystem.h"

// TODO: placer ces macros dans le header de FrOffshoreSystem !!! C'est d'utilite publique...
#define M_ONE_MILE 1852.                        ///> NUMBER OF METER IN ONE NAUTICAL MILE
#define M_ONE_MINUTE 60.                        ///> NUMBER OF SECONDS IN ONE MINUTE
#define M_ONE_HOUR (M_ONE_MINUTE*60.)             ///> NUMBER OF SECONDS IN ONE HOUR
#define M_KNOT (M_ONE_MILE/M_ONE_HOUR)            ///> Conversion coeff knot -> m/s

#define M_DEG M_PI/180.                         ///> Conversion DEG->RAD

namespace frydom {
    namespace environment {

        FrCurrent::FrCurrent() : m_velocity_vector(chrono::VNULL) {}

        FrCurrent::FrCurrent(chrono::ChVector<> const velocity_vector, FrFrame frame) {
            m_velocity_vector.SetNull();
            m_velocity_vector = velocity_vector;

            if (frame == NED) {
                m_velocity_vector = swap_NED_NWU(m_velocity_vector);
            }
        }

        FrCurrent::FrCurrent(double const angle,
                             double const velocity,
                             FrAngleUnit angleUnit,
                             FrSpeedUnit speedUnit,
                             FrFrame frame) {

            double vel;
            double ang;

            if (speedUnit == KNOT) {
                // Converting in m/s
                vel = velocity * M_KNOT;
            } else {
                vel = velocity;
            }

            if (angleUnit == DEG) {
                // Converting in RAD
                ang = angle * M_DEG;
            } else {
                ang = angle;
            }

            if (frame == NED) {
                // Expressing the angle in the E frame
                ang = -ang;
            }

            m_velocity_vector.SetNull();

            m_velocity_vector.x() = -vel * cos(ang);
            m_velocity_vector.y() = -vel * sin(ang);

        }

        FrCurrent::FrCurrent(chrono::ChVector<> const udir,
                             double const velocity,
                             FrSpeedUnit speedUnit,
                             FrFrame frame) {

            if (speedUnit == KNOT) {
                // Converting into m/s
                m_velocity_vector = velocity * M_KNOT * udir;
            } else {
                m_velocity_vector = velocity * udir;
            }

            if (frame == NED) {
                // Converting direction to E frame
                m_velocity_vector = NED2NWU(m_velocity_vector);
            }

        }

        double FrCurrent::getVelocity(FrSpeedUnit speedUnit) {
            double vx = m_velocity_vector.x();
            double vy = m_velocity_vector.y();
            double vz = m_velocity_vector.z();

            double vel_ms = sqrt(vx * vx + vy * vy + vz * vz);

            if (speedUnit == KNOT) {
                return vel_ms / M_KNOT;
            } else {
                return vel_ms;
            }
        }

        void FrCurrent::setVelocity(double const velocity, FrSpeedUnit speedUnit) {

            // Retrieving the current direction
            auto dir = getDirection(NWU);
            m_velocity_vector.SetNull();

            // Converting velocity into M/S unit if needed
            if (speedUnit == KNOT) {
                m_velocity_vector = velocity * M_KNOT * dir;
            } else {
                m_velocity_vector = velocity * dir;
            }

        }

        chrono::ChVector<> FrCurrent::getDirection(FrFrame frame) {
            auto vel = getVelocity(MS);

            auto udir = m_velocity_vector / vel;  // Expressed in NWU

            if (frame == NED) {
                // Expressing the direction into NED frame
                udir = NWU2NED(udir);
                return udir;
            } else {
                return udir;
            }
        }

        void FrCurrent::setDirection(double const angle, FrAngleUnit angleUnit, FrFrame frame) {

            // Getting current module
            auto velocity = getVelocity(MS);

            // Converting angle into radians if needed
            double ang;
            if (angleUnit == DEG) {
                ang = angle * M_DEG;
            } else {
                ang = angle;
            }

            // Building direction in E FRAME
            m_velocity_vector.SetNull();

            m_velocity_vector.x() = velocity * cos(angle);
            m_velocity_vector.y() = velocity * sin(angle);

            if (frame == NED) {
                m_velocity_vector = swap_NED_NWU(m_velocity_vector);
            }

        }

        void FrCurrent::setDirection(chrono::ChVector<> const unit_vector, FrFrame frame) {
            // Getting current module
            auto velocity = getVelocity(MS);

            m_velocity_vector.SetNull();
            m_velocity_vector = velocity * unit_vector;

            if (frame == NED) {
                m_velocity_vector = swap_NED_NWU(m_velocity_vector);
            }

        }

        void FrCurrent::Initialize(chrono::ChVector<> const velocity_vector,
                                   FrFrame frame) {
            // TODO: double if
            m_velocity_vector = velocity_vector;
            if (frame == NED) {
                m_velocity_vector = swap_NED_NWU(m_velocity_vector);
            }
        }

        void FrCurrent::Initialize(double const angle,
                                   double const velocity,
                                   FrAngleUnit angleUnit,
                                   FrSpeedUnit speedUnit,
                                   FrFrame frame) {
            double ang;
            if (angleUnit == DEG) {
                ang = angle * M_DEG;
            } else {
                ang = angle;
            }

            double vel;
            if (speedUnit == KNOT) {
                vel = velocity * M_KNOT;
            } else {
                vel = velocity;
            }

            auto velocity_vector = chrono::ChVector<>(cos(ang), sin(ang), 0);

            Initialize(velocity_vector, frame);

        }

        void FrCurrent::Initialize(chrono::ChVector<> const unit_direction,
                                   double const velocity,
                                   FrSpeedUnit speedUnit,
                                   FrFrame frame) {

            if (speedUnit == KNOT) {
                Initialize(velocity * M_KNOT * unit_direction, frame);
            } else {
                Initialize(velocity * unit_direction, frame);
            }

        }

        void FrCurrent::get(chrono::ChVector<> &velocity_vector,
                            FrFrame frame) {
            // FIXME : tout foutu !!
            if (frame == NED) {
                velocity_vector = NWU2NED(m_velocity_vector);
            } else {
                velocity_vector = m_velocity_vector;
            }
        }

        void FrCurrent::get(chrono::ChVector<> &unit_vector,
                            double &velocity,
                            FrFrame frame,
                            FrSpeedUnit speedUnit) {

            unit_vector = getDirection(frame);
            velocity = getVelocity(speedUnit);

        }

        chrono::ChVector<double> FrCurrent::GetVelocityVector(FrCurrent::FrFrame frame) {
            if (frame == NED) {
                return NWU2NED(m_velocity_vector);
            } else {
                return m_velocity_vector;
            }
        }

    }  // end namespace environment
}  // end namespace frydom
