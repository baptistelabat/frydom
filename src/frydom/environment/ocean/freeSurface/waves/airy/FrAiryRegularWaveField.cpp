//
// Created by Lucas Letournel on 21/11/18.
//

#include "FrAiryRegularWaveField.h"

#include "frydom/environment/ocean/freeSurface/FrFreeSurface.h"
#include "frydom/environment/FrEnvironment.h"
#include "frydom/environment/ocean/FrOcean_.h"
#include "frydom/environment/ocean/seabed/FrSeabed.h"
#include "frydom/environment/ocean/freeSurface/waves/FrWaveDispersionRelation.h"
#include "frydom/environment/ocean/freeSurface/waves/FrKinematicStretching.h"
//#include "../FrWaveDispersionRelation.h"

namespace frydom {

    FrAiryRegularWaveField::FrAiryRegularWaveField(FrFreeSurface_* freeSurface) : FrWaveField_(freeSurface) {

        m_waveRamp = std::make_shared<FrRamp>();
        m_waveRamp->Initialize();

        m_verticalFactor = std::make_unique<FrKinematicStretching_>();
        m_verticalFactor->SetInfDepth(m_infinite_depth);

    }

    void FrAiryRegularWaveField::SetWaveHeight(double height) { m_height = height;}

    double FrAiryRegularWaveField::GetWaveHeight() const {return m_height;}

    void FrAiryRegularWaveField::SetWavePeriod(double period, FREQUENCY_UNIT unit) {

        // Set the wave period in seconds
        m_period = convert_frequency(period, unit, S);

        // Set the wave pulsation from the wave period, in rad/s
        m_omega = S2RADS(m_period);

        // Set the wave number, using the wave dispersion relation
        auto waterHeight = m_freeSurface->GetMeanHeight() - m_freeSurface->GetOcean()->GetSeabed()->GetDepth();
        auto gravityAcceleration = m_freeSurface->GetOcean()->GetEnvironment()->GetGravityAcceleration();
        m_k = SolveWaveDispersionRelation(waterHeight, m_omega, gravityAcceleration);

    }

    double FrAiryRegularWaveField::GetWavePeriod(FREQUENCY_UNIT unit) const { return convert_frequency(m_period ,S, unit);}


    void FrAiryRegularWaveField::SetDirection(double dirAngle, ANGLE_UNIT unit, FRAME_CONVENTION fc,
                                              DIRECTION_CONVENTION dc) {
        // The wave direction angle is used internally with the convention NWU, GOTO, and RAD unit.
        m_dirAngle = dirAngle;
        if (unit == DEG) m_dirAngle *= DEG2RAD;
        if (IsNED(fc)) m_dirAngle = - m_dirAngle;
        if (IsCOMEFROM(dc)) m_dirAngle -= MU_PI;

        mathutils::Normalize_0_2PI(m_dirAngle);

        m_direction = Direction(cos(m_dirAngle), sin(m_dirAngle), 0.);

    }

    void FrAiryRegularWaveField::SetDirection(const Direction& direction, FRAME_CONVENTION fc, DIRECTION_CONVENTION dc) {
        assert(mathutils::IsClose(direction.Getuz(),0.));
        double dirAngle = atan2(direction.Getuy(),direction.Getux());
        SetDirection(dirAngle, RAD, fc, dc);
    }

    double
    FrAiryRegularWaveField::GetDirectionAngle(ANGLE_UNIT unit, FRAME_CONVENTION fc, DIRECTION_CONVENTION dc) const {
        double dirAngle = m_dirAngle;
        if (IsNED(fc)) dirAngle = - dirAngle;
        if (IsCOMEFROM(dc)) dirAngle -= MU_PI;
        if (unit == DEG) dirAngle *= RAD2DEG;

        return mathutils::Normalize_0_360(dirAngle);
    }

    Direction FrAiryRegularWaveField::GetDirection(FRAME_CONVENTION fc, DIRECTION_CONVENTION dc) const {
        auto dirAngle = GetDirectionAngle(RAD, fc, dc);
        return {cos(m_dirAngle), sin(m_dirAngle), 0.};
    }

    double FrAiryRegularWaveField::GetWaveLength() const {return 2.*M_PI/m_k;}







    Complex FrAiryRegularWaveField::GetComplexElevation(double x, double y) const {
        // eta = Re(A * exp(j.k.[x.cos(theta) + y.sin(theta)]) * exp(-j.omega.t)

        double kdir = x*cos(m_dirAngle) + y*sin(m_dirAngle);
        double time = GetTime();
        return m_height * exp(JJ*(m_k*kdir - m_omega * time));
    }

    double FrAiryRegularWaveField::GetElevation(double x, double y) const {
        return std::real( GetComplexElevation(x, y));
    }

    std::vector<std::vector<double>>
    FrAiryRegularWaveField::GetElevation(const std::vector<double> &xVect, const std::vector<double> &yVect) const {
        auto nx = xVect.size();
        auto ny = yVect.size();

        std::vector<std::vector<double>> elevations;
        std::vector<double> elev;

        elevations.reserve(nx);
        elev.reserve(ny);

        for (unsigned int ix=0; ix<nx; ++ix) {
            elev.clear();
            for (unsigned int iy=0; iy<ny; ++iy) {
                elev.push_back(GetElevation(xVect[ix], yVect[iy]));
            }
            elevations.push_back(elev);
        }
        return elevations;
    }

    Velocity FrAiryRegularWaveField::GetVelocity(double x, double y, double z) const {
        // Vx = DPhi/Dx = D( - d(eta)/dt )/Dx
        // Vx =


        auto cmplxElevations = GetComplexElevation(x, y);


        return Velocity();
    }

//    void FrAiryRegularWaveField::Update(double time) {
//        FrWaveField_::Update(time);
//    }

    Acceleration FrAiryRegularWaveField::GetAcceleration(double x, double y, double z) const {
        return Acceleration();
    }

    std::vector<std::vector<std::vector<Velocity>>>
    FrAiryRegularWaveField::GetVelocityGrid(const std::vector<double> &xvect, const std::vector<double> &yvect,
                                            const std::vector<double> &zvect) const {
        return std::vector<std::vector<std::vector<Velocity>>>();
    }



}