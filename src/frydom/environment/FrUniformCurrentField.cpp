//
// Created by camille on 27/02/18.
//

#include "FrUniformCurrentField.h"


namespace frydom {

    void FrUniformCurrentField::Set(chrono::ChVector<> const velocity_vector,
                         FRAME_CONVENTION frame, FrDirectionConvention convention) {

        auto current_vector = velocity_vector;

        // Place the vector in NWU frame
        if (frame == NED) {
            current_vector = internal::swap_NED_NWU(current_vector);
        }

        if (convention == COMEFROM) {
            current_vector = -current_vector;
        }

        m_currentVector = current_vector;

    }

    void FrUniformCurrentField::Set(double const angle,
                         double const velocity,
                         ANGLE_UNIT angleUnit,
                         SPEED_UNIT speedUnit,
                         FRAME_CONVENTION frame,
                         FrDirectionConvention convention) {

        auto alpha = angle;
        if (angleUnit == DEG) {
            alpha = alpha * DEG2RAD;
        }

        // Ensuring a velocity unit as M/S
        auto magn = convert_velocity_unit(velocity, speedUnit, MS);

        // Building the unit vector from angle
        chrono::ChVector<double> current_vector(
                magn * cos(alpha),
                magn * sin(alpha),
                0.);

        // Managing the convention
        if (convention == COMEFROM) {
            current_vector = -current_vector;
        }

        // Managing conversion into NWU
        if (frame == NED) {
            current_vector = internal::swap_NED_NWU(current_vector);
        }

        m_currentVector = current_vector;

    }

    void FrUniformCurrentField::Set(chrono::ChVector<> const udir,
                         double const velocity,
                         SPEED_UNIT speedUnit,
                         FRAME_CONVENTION frame,
                         FrDirectionConvention convention) {

        // TODO: Ensure that the vector is a unit vector

        // Building the current_vector
        auto current_vector = convert_velocity_unit(velocity, speedUnit, MS) * udir;

        // Place the vector in NWU frame
        if (frame == NED) {
            current_vector = internal::swap_NED_NWU(current_vector);
        }

        if (convention == COMEFROM) {
            current_vector = -current_vector;
        }

        m_currentVector = current_vector;

    }

    void FrUniformCurrentField::Update(double Time) {
//        std::cout << "Updating current model" << std::endl;
    }

    chrono::ChVector<> FrUniformCurrentField::GetFluxVector(FRAME_CONVENTION frame) {
        switch (frame) {
            case NED:
                return internal::swap_NED_NWU(m_currentVector);
            case NWU:
                return m_currentVector;
        }
    }

    chrono::ChVector<> FrUniformCurrentField::GetComeFromVector(FRAME_CONVENTION frame) {
        return -FrUniformCurrentField::GetFluxVector(frame);
    }

    chrono::ChVector<> FrUniformCurrentField::GetGoToVector(FRAME_CONVENTION frame) {
        return FrUniformCurrentField::GetFluxVector(frame);
    }

    double FrUniformCurrentField::GetAngle(FrDirectionConvention convention, FRAME_CONVENTION frame, ANGLE_UNIT angleUnit) {

        chrono::ChVector<> current_vector;
        switch (convention) {
            case COMEFROM:
                current_vector = GetComeFromVector(frame);
                break;
            case GOTO:
                current_vector = GetGoToVector(frame);
                break;
        }

        double angle = atan2(current_vector.y(), current_vector.x());

        switch (angleUnit) {
            case RAD:
                return angle;
            case DEG:
                return angle * RAD2DEG;
        }

    }

    double FrUniformCurrentField::GetMagnitude(SPEED_UNIT speedUnit) {
        return convert_velocity_unit(m_currentVector.Length(), MS, speedUnit);
    }

    double FrUniformCurrentField::GetMagnitude2() {
        // TODO
    }

    void FrUniformCurrentField::Set(const chrono::ChVector<>& unitDirection, double magnitude,
                        FRAME_CONVENTION frame, FrDirectionConvention directionConvention, SPEED_UNIT speedUnit) {

        auto uDirection = unitDirection;
        uDirection /= unitDirection.Length();

        if (frame == NED) {
            uDirection = internal::swap_NED_NWU(uDirection);
        }

        if (directionConvention == COMEFROM) {
            uDirection = - uDirection;
        }

        auto vel = magnitude;

        if (speedUnit != MS) {
            vel = convert_velocity_unit(vel, speedUnit, MS);
        }

        // Building the current vector
        m_currentVector = uDirection * vel;

    }



}