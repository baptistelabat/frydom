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
        m_depth = m_freeSurface->GetOcean()->GetSeabed()->GetDepth();

        m_waveRamp = std::make_shared<FrRamp>();
        m_waveRamp->Initialize();

        m_verticalFactor = std::make_unique<FrKinematicStretching_>();
        m_verticalFactor->SetInfDepth(m_infinite_depth);

    }

    FrAiryRegularWaveField::FrAiryRegularWaveField(FrFreeSurface_* freeSurface, double waveHeight, double wavePeriod,
    double waveDirAngle, ANGLE_UNIT unit, FRAME_CONVENTION fc, DIRECTION_CONVENTION dc) : FrWaveField_(freeSurface) {

        m_depth = m_freeSurface->GetOcean()->GetSeabed()->GetDepth();

        m_waveRamp = std::make_shared<FrRamp>();
        m_waveRamp->Initialize();

        m_verticalFactor = std::make_unique<FrKinematicStretching_>();
        m_verticalFactor->SetInfDepth(m_infinite_depth);

        SetWaveHeight(waveHeight);
        SetWavePeriod(wavePeriod);
        SetDirection(waveDirAngle, unit, fc ,dc);
    }

    void FrAiryRegularWaveField::SetWaveHeight(double height) { m_height = height;}

    double FrAiryRegularWaveField::GetWaveHeight() const {return m_height;}

    void FrAiryRegularWaveField::SetWavePeriod(double period, FREQUENCY_UNIT unit) {

        // Set the wave period in seconds
        m_period = convert_frequency(period, unit, S);

        // Set the wave pulsation from the wave period, in rad/s
        m_omega = S2RADS(m_period);

        // Set the wave number, using the wave dispersion relation
        auto waterHeight = m_freeSurface->GetMeanHeight() - m_depth;
        auto gravityAcceleration = m_freeSurface->GetOcean()->GetEnvironment()->GetGravityAcceleration();
        m_k = SolveWaveDispersionRelation(waterHeight, m_omega, gravityAcceleration);

        m_infinite_depth = 3. * 2. * M_PI / m_k < m_depth;

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

    void FrAiryRegularWaveField::SetStretching(FrStretchingType type) {
        switch (type) {
            case NO_STRETCHING:
                m_verticalFactor = std::make_unique<FrKinematicStretching_>();
                break;
            case VERTICAL:
                m_verticalFactor = std::make_unique<FrKinStretchingVertical_>();
                break;
            case EXTRAPOLATE:
                m_verticalFactor = std::make_unique<FrKinStretchingExtrapol_>();
                break;
            case WHEELER:
                m_verticalFactor = std::make_unique<FrKinStretchingWheeler_>(this);
                break;
            case CHAKRABARTI:
                m_verticalFactor = std::make_unique<FrKinStretchingChakrabarti_>(this);
            case DELTA:
                m_verticalFactor = std::make_unique<FrKinStretchingDelta_>(this);
            default:
                m_verticalFactor = std::make_unique<FrKinematicStretching_>();
                break;
        }
        m_verticalFactor->SetInfDepth(m_infinite_depth);
    }






    Complex FrAiryRegularWaveField::GetComplexElevation(double x, double y) const {
        // eta = Re(A * exp(j.k.[x.cos(theta) + y.sin(theta)]) * exp(-j.omega.t)

        double kdir = x*cos(m_dirAngle) + y*sin(m_dirAngle);
        double time = GetTime();
        return m_height * exp(JJ*(m_k*kdir - m_omega * time));
    }

    double FrAiryRegularWaveField::GetElevation(double x, double y) const {
        return std::imag( GetComplexElevation(x, y));
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

    mathutils::Vector3d<Complex> FrAiryRegularWaveField::GetComplexVelocity(double x, double y, double z) const {
        auto Vtemp = m_omega * GetComplexElevation(x, y) * m_verticalFactor->Eval(x,y,z,m_k,m_depth);

        auto Vx = cos(m_dirAngle) * Vtemp;
        auto Vy = sin(m_dirAngle) * Vtemp;
        auto Vz = -JJ * m_omega / m_k * GetComplexElevation(x, y) * m_verticalFactor->EvalDZ(x,y,z,m_k,m_depth);

        return {Vx,Vy,Vz};
    }

    Velocity FrAiryRegularWaveField::GetVelocity(double x, double y, double z) const {
        auto cplxVel = GetComplexVelocity(x, y, z);
        return {std::imag(cplxVel.x()),std::imag(cplxVel.y()),std::imag(cplxVel.z())};

//        auto Vtemp = m_omega *  std::imag(GetComplexElevation(x, y)) * m_verticalFactor->Eval(x,y,z,m_k,m_depth);
//
//        auto Vx = cos(m_dirAngle) * Vtemp;
//        auto Vy = sin(m_dirAngle) * Vtemp;
//        auto Vz = m_omega / m_k *  std::real(GetComplexElevation(x, y)) * m_verticalFactor->EvalDZ(x,y,z,m_k,m_depth);
//
//        return {Vx,Vy,Vz};
    }

//    void FrAiryRegularWaveField::Update(double time) {
//        FrWaveField_::Update(time);
//    }

    Acceleration FrAiryRegularWaveField::GetAcceleration(double x, double y, double z) const {
        auto cplxVel = GetComplexVelocity(x, y, z);
        auto cplxAcc = -JJ * m_omega * cplxVel;
        return {std::imag(cplxAcc.x()),std::imag(cplxAcc.y()),std::imag(cplxAcc.z())};

//        auto Atemp = -m_omega * m_omega *  std::real(GetComplexElevation(x, y)) * m_verticalFactor->Eval(x,y,z,m_k,m_depth);
//
//        auto Ax = cos(m_dirAngle) * Atemp;
//        auto Ay = sin(m_dirAngle) * Atemp;
//        auto Az = -m_omega * m_omega / m_k * std::imag(GetComplexElevation(x, y)) * m_verticalFactor->EvalDZ(x,y,z,m_k,m_depth);
//
//        return {Ax,Ay,Az};
    }

    std::vector<std::vector<std::vector<Velocity>>>
    FrAiryRegularWaveField::GetVelocityGrid(const std::vector<double> &xvect, const std::vector<double> &yvect,
                                            const std::vector<double> &zvect) const {
        return std::vector<std::vector<std::vector<Velocity>>>();
    }


}