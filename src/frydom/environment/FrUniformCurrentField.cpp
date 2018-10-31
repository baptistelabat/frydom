//
// Created by camille on 27/02/18.
//

#include "FrUniformCurrentField.h"


namespace frydom {

    void FrUniformCurrentField::Set(chrono::ChVector<> const velocity_vector,
                         FRAME_CONVENTION frame, DIRECTION_CONVENTION convention) {

        auto current_vector = velocity_vector;

        // Place the vector in NWU frame
        if (frame == NED) {
            current_vector[1] = -current_vector[1];
            current_vector[2] = -current_vector[2];
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
                         DIRECTION_CONVENTION convention) {

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
            current_vector[1] = -current_vector[1];
            current_vector[2] = -current_vector[2];
        }

        m_currentVector = current_vector;

    }

    void FrUniformCurrentField::Set(chrono::ChVector<> const udir,
                         double const velocity,
                         SPEED_UNIT speedUnit,
                         FRAME_CONVENTION frame,
                         DIRECTION_CONVENTION convention) {

        // TODO: Ensure that the vector is a unit vector

        // Building the current_vector
        auto current_vector = convert_velocity_unit(velocity, speedUnit, MS) * udir;

        // Place the vector in NWU frame
        if (frame == NED) {
            current_vector[1] = -current_vector[1];
            current_vector[2] = -current_vector[2];
        }

        if (convention == COMEFROM) {
            current_vector = -current_vector;
        }

        m_currentVector = current_vector;

    }

    void FrUniformCurrentField::Update(double Time) {
//        std::cout << "Updating current model" << std::endl;
    }

    chrono::ChVector<double> FrUniformCurrentField::GetFluxVector(FRAME_CONVENTION frame) {
        switch (frame) {
            case NED:
                m_currentVector[1] = -m_currentVector[1];
                m_currentVector[2] = -m_currentVector[2];
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

    double FrUniformCurrentField::GetAngle(DIRECTION_CONVENTION convention, FRAME_CONVENTION frame, ANGLE_UNIT angleUnit) {

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
                        FRAME_CONVENTION frame, DIRECTION_CONVENTION directionConvention, SPEED_UNIT speedUnit) {

        auto uDirection = unitDirection;
        uDirection /= unitDirection.Length();

        if (frame == NED) {
            uDirection[1] = -uDirection[1];
            uDirection[2] = -uDirection[2];
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















    ///////// REFACTORING ---------->>>>>>>>>>>>>


    void FrUniformCurrentField_::Set(const Velocity& velocity, FRAME_CONVENTION fc, DIRECTION_CONVENTION dc){

        auto velTmp = velocity;

        if (IsNED(fc)) {
            internal::SwapFrameConvention<Velocity>(velTmp);
        }

        if (IsCOMEFROM(dc)) {
            velTmp = -velTmp;
        }

        m_fluxVectorNWU = velTmp;
    }

    void FrUniformCurrentField_::Set(double angle, double  magnitude,
                                     ANGLE_UNIT angleUnit, SPEED_UNIT speedUnit, FRAME_CONVENTION fc, DIRECTION_CONVENTION dc) {

        magnitude = mathutils::convert_velocity_unit(magnitude, speedUnit, MS); // Convert in M/S

        if (angleUnit == DEG) angle *= DEG2RAD; // Convert in RAD

        if (IsNED(fc)) angle = -angle; // Express in NWU

        if (IsCOMEFROM(dc)) angle += MU_PI; // Express in GOTO

        m_fluxVectorNWU << magnitude * cos(angle), magnitude * sin(angle), 0.;

    }

    void FrUniformCurrentField_::SetNorth(double magnitude, SPEED_UNIT speed_unit, DIRECTION_CONVENTION dc) {
        magnitude = mathutils::convert_velocity_unit(magnitude, speed_unit, MS);
        m_fluxVectorNWU = magnitude * internal::NORTH;
        if (IsCOMEFROM(dc)) m_fluxVectorNWU = -m_fluxVectorNWU;
    }

    void FrUniformCurrentField_::SetNorthEast(double magnitude, SPEED_UNIT speed_unit, DIRECTION_CONVENTION dc) {
        magnitude = mathutils::convert_velocity_unit(magnitude, speed_unit, MS);
        m_fluxVectorNWU = magnitude * internal::NORTH_EAST;
        if (IsCOMEFROM(dc)) m_fluxVectorNWU = -m_fluxVectorNWU;
    }

    void FrUniformCurrentField_::SetEast(double magnitude, SPEED_UNIT speed_unit, DIRECTION_CONVENTION dc) {
        magnitude = mathutils::convert_velocity_unit(magnitude, speed_unit, MS);
        m_fluxVectorNWU = magnitude * internal::EAST;
        if (IsCOMEFROM(dc)) m_fluxVectorNWU = -m_fluxVectorNWU;
    }

    void FrUniformCurrentField_::SetSouthEast(double magnitude, SPEED_UNIT speed_unit, DIRECTION_CONVENTION dc) {
        magnitude = mathutils::convert_velocity_unit(magnitude, speed_unit, MS);
        m_fluxVectorNWU = magnitude * internal::SOUTH_EAST;
        if (IsCOMEFROM(dc)) m_fluxVectorNWU = -m_fluxVectorNWU;
    }

    void FrUniformCurrentField_::SetSouth(double magnitude, SPEED_UNIT speed_unit, DIRECTION_CONVENTION dc) {
        magnitude = mathutils::convert_velocity_unit(magnitude, speed_unit, MS);
        m_fluxVectorNWU = magnitude * internal::SOUTH;
        if (IsCOMEFROM(dc)) m_fluxVectorNWU = -m_fluxVectorNWU;
    }

    void FrUniformCurrentField_::SetSouthWest(double magnitude, SPEED_UNIT speed_unit, DIRECTION_CONVENTION dc) {
        magnitude = mathutils::convert_velocity_unit(magnitude, speed_unit, MS);
        m_fluxVectorNWU = magnitude * internal::SOUTH_WEST;
        if (IsCOMEFROM(dc)) m_fluxVectorNWU = -m_fluxVectorNWU;
    }

    void FrUniformCurrentField_::SetWest(double magnitude, SPEED_UNIT speed_unit, DIRECTION_CONVENTION dc) {
        magnitude = mathutils::convert_velocity_unit(magnitude, speed_unit, MS);
        m_fluxVectorNWU = magnitude * internal::WEST;
        if (IsCOMEFROM(dc)) m_fluxVectorNWU = -m_fluxVectorNWU;
    }

    void FrUniformCurrentField_::SetNorthWest(double magnitude, SPEED_UNIT speed_unit, DIRECTION_CONVENTION dc) {
        magnitude = mathutils::convert_velocity_unit(magnitude, speed_unit, MS);
        m_fluxVectorNWU = magnitude * internal::NORTH_WEST;
        if (IsCOMEFROM(dc)) m_fluxVectorNWU = -m_fluxVectorNWU;
    }

    void FrUniformCurrentField_::Update(double Time) {}

    Velocity FrUniformCurrentField_::GetAbsFluxVelocity(FRAME_CONVENTION fc) {
        auto velocity = m_fluxVectorNWU;
        if (IsNED(fc)) internal::SwapFrameConvention<Velocity>(velocity);
        return velocity;
    }

    double FrUniformCurrentField_::GetAngle(DIRECTION_CONVENTION dc, FRAME_CONVENTION fc, ANGLE_UNIT angleUnit) {

        auto velocity = m_fluxVectorNWU;
        if (IsCOMEFROM(dc)) velocity = -velocity;
        double angle = atan2(velocity.GetVy(), velocity.GetVx());
        if (angleUnit == DEG) angle *= RAD2DEG;
        if (IsNED(fc)) angle = -angle;
        return angle;

    }

    double FrUniformCurrentField_::GetMagnitude(SPEED_UNIT speedUnit) {
        return mathutils::convert_velocity_unit(m_fluxVectorNWU.norm(), MS, speedUnit);
    }

    void FrUniformCurrentField_::Initialize() {}

    void FrUniformCurrentField_::StepFinalize() {}


}  // end namespace frydom