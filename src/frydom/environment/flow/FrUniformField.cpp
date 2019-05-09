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


#include "FrUniformField.h"


namespace frydom {

    void FrUniformField::Set(const Velocity& velocity, FRAME_CONVENTION fc, DIRECTION_CONVENTION dc){

        auto velTmp = velocity;

        if (IsNED(fc)) {
            internal::SwapFrameConvention<Velocity>(velTmp);
        }

        if (IsCOMEFROM(dc)) {
            velTmp = -velTmp;
        }

        m_fluxVectorNWU = velTmp;
    }

    void FrUniformField::Set(double angle, double  magnitude,
                                     ANGLE_UNIT angleUnit, SPEED_UNIT speedUnit, FRAME_CONVENTION fc, DIRECTION_CONVENTION dc) {

        magnitude = mathutils::convert_velocity_unit(magnitude, speedUnit, mathutils::MS); // Convert in M/S

        if (angleUnit == mathutils::DEG) angle *= DEG2RAD; // Convert in RAD

        if (IsNED(fc)) angle = -angle; // Express in NWU

        if (IsCOMEFROM(dc)) angle += MU_PI; // Express in GOTO

        m_fluxVectorNWU << magnitude * cos(angle), magnitude * sin(angle), 0.;

    }

    void FrUniformField::Set(std::function<Velocity(FRAME_CONVENTION)> direction, double magnitude,
                             SPEED_UNIT speed_unit, DIRECTION_CONVENTION dc) {
        magnitude = mathutils::convert_velocity_unit(magnitude, speed_unit, mathutils::MS);
        m_fluxVectorNWU = direction(NWU) * magnitude;
        if (IsCOMEFROM(dc)) m_fluxVectorNWU = -m_fluxVectorNWU;
    }

    void FrUniformField::SetNorth(double magnitude, SPEED_UNIT speed_unit, DIRECTION_CONVENTION dc) {
        this->Set(NORTH, magnitude, speed_unit, dc);
    }

    void FrUniformField::SetNorthEast(double magnitude, SPEED_UNIT speed_unit, DIRECTION_CONVENTION dc) {
        this->Set(NORTH_EAST, magnitude, speed_unit, dc);
    }

    void FrUniformField::SetEast(double magnitude, SPEED_UNIT speed_unit, DIRECTION_CONVENTION dc) {
        this->Set(EAST, magnitude, speed_unit, dc);
    }

    void FrUniformField::SetSouthEast(double magnitude, SPEED_UNIT speed_unit, DIRECTION_CONVENTION dc) {
        this->Set(SOUTH_EAST, magnitude, speed_unit, dc);
    }

    void FrUniformField::SetSouth(double magnitude, SPEED_UNIT speed_unit, DIRECTION_CONVENTION dc) {
        this->Set(SOUTH, magnitude, speed_unit, dc);
    }

    void FrUniformField::SetSouthWest(double magnitude, SPEED_UNIT speed_unit, DIRECTION_CONVENTION dc) {
        this->Set(SOUTH_WEST, magnitude, speed_unit, dc);
    }

    void FrUniformField::SetWest(double magnitude, SPEED_UNIT speed_unit, DIRECTION_CONVENTION dc) {
        this->Set(WEST, magnitude, speed_unit, dc);
    }

    void FrUniformField::SetNorthWest(double magnitude, SPEED_UNIT speed_unit, DIRECTION_CONVENTION dc) {
        this->Set(NORTH_WEST, magnitude, speed_unit, dc);
    }

    void FrUniformField::Update(double time) {
        // TODO : permettre une variation temporelle...
    }

    Velocity FrUniformField::GetFluxVelocityInWorld(const Position& worldPos, FRAME_CONVENTION fc) const {
        auto velocity = m_fluxVectorNWU;
        if (IsNED(fc)) internal::SwapFrameConvention<Velocity>(velocity);
        return velocity;
    }

    void FrUniformField::Initialize() {}


}  // end namespace frydom
