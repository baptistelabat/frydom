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


#include "FrInertiaTensor.h"

#include "chrono/core/ChMatrixDynamic.h"
#include "frydom/core/math/FrMatrix.h"


namespace frydom {


    FrInertiaTensor::FrInertiaTensor(double mass,
                                       double Ixx, double Iyy, double Izz,
                                       double Ixy, double Ixz, double Iyz,
                                       const FrFrame& coeffsFrame,
                                       const Position& cogPosition,
                                       FRAME_CONVENTION fc) {

        Position cogPosTmp = cogPosition;
        if (IsNED(fc)) {
            internal::SwapInertiaFrameConvention(Ixx, Iyy, Izz, Ixy, Ixz, Iyz); // Convert to NWU
            internal::SwapFrameConvention<Position>(cogPosTmp);
        }

        auto rot_rp = coeffsFrame.GetRotation().GetRotationMatrix();

        m_inertiaAtCOG << Ixx, Ixy, Ixz,
                          Ixy, Iyy, Iyz,
                          Ixz, Iyz, Izz;

        m_inertiaAtCOG = rot_rp * m_inertiaAtCOG * rot_rp.transpose();
        Position PG = cogPosTmp - coeffsFrame.GetPosition(NWU);
        m_inertiaAtCOG -= GetPointMassInertiaMatrix(mass, PG); // Avoir un GetPointMassInertia
        m_cogPosition = cogPosTmp;
        m_mass = mass;

    }

    FrInertiaTensor::FrInertiaTensor(double mass, double Ixx, double Iyy, double Izz, double Ixy, double Ixz,
                                       double Iyz, const Position& cogPos, FRAME_CONVENTION fc) {

        m_inertiaAtCOG << Ixx, Ixy, Ixz,
                          Ixy, Iyy, Iyz,
                          Ixz, Iyz, Izz;

        m_cogPosition = cogPos;
        if (IsNED(fc)) internal::SwapFrameConvention(m_cogPosition);

        m_mass = mass;

    }

    double FrInertiaTensor::GetMass() const {
        return m_mass;
    }

    const Position FrInertiaTensor::GetCOGPosition(FRAME_CONVENTION fc) const {
        auto cogPos = m_cogPosition;
        if (IsNED(fc)) internal::SwapFrameConvention<Position>(cogPos);
        return cogPos;
    }

    void
    FrInertiaTensor::GetInertiaCoeffsAtCOG(double &Ixx, double &Iyy, double &Izz, double &Ixy, double &Ixz, double &Iyz,
                                           FRAME_CONVENTION fc) const {
        SplitMatrix33IntoCoeffs(m_inertiaAtCOG, Ixx, Ixy, Ixz, Ixy, Iyy, Iyz, Ixz, Iyz, Izz);

        if (IsNED(fc)) internal::SwapInertiaFrameConvention(Ixx, Iyy, Izz, Ixy, Ixz, Iyz);
    }

    void FrInertiaTensor::GetInertiaCoeffsAtFrame(double &Ixx, double &Iyy, double &Izz,
                                 double &Ixy, double &Ixz, double &Iyz,
                                 const FrFrame& frame,
                                 FRAME_CONVENTION fc) const {

        auto rot_rp = frame.GetRotation().GetInverseRotationMatrix();

        InertiaMatrix tempInertia = m_inertiaAtCOG;
        Position PG = m_cogPosition - frame.GetPosition(NWU);
        tempInertia += GetPointMassInertiaMatrix(m_mass, PG);


        tempInertia = rot_rp * tempInertia * rot_rp.transpose();

        SplitMatrix33IntoCoeffs(tempInertia, Ixx, Ixy, Ixz, Ixy, Iyy, Iyz, Ixz, Iyz, Izz);

        if (IsNED(fc)) {
            internal::SwapInertiaFrameConvention(Ixx, Iyy, Izz, Ixy, Ixz, Iyz); // Convert to NWU
        }
    }

    FrInertiaTensor::InertiaMatrix FrInertiaTensor::GetPointMassInertiaMatrix(double mass,
                                                                                const frydom::Position &cogPos) {

        // TODO : voir a implementer la version alternative avec le outer product ?? -> annexe technique Innosea
        double a = cogPos[0];
        double b = cogPos[1];
        double c = cogPos[2];
        double a2 = a*a;
        double b2 = b*b;
        double c2 = c*c;
        double ab = a*b;
        double ac = a*c;
        double bc = b*c;
        InertiaMatrix inertiaMatrix;
        inertiaMatrix << b2+c2, -ab,   -ac,
                         -ab  , a2+c2, -bc,
                         -ac  , -bc,   a2+b2;
        inertiaMatrix *= mass;
        return inertiaMatrix;
    }

    std::ostream& FrInertiaTensor::cout(std::ostream &os) const {  // OK

        double Ixx, Iyy, Izz, Ixy, Ixz, Iyz;
        SplitMatrix33IntoCoeffs(m_inertiaAtCOG, Ixx, Ixy, Ixz, Ixy, Iyy, Iyz, Ixz, Iyz, Izz);

        os << std::endl;
        os << "Inertia (NWU): \n";
        os << "Mass (kg) : " << GetMass() << std::endl;
        os << "COG (m)   : " << m_cogPosition.GetX() << "\t" << m_cogPosition.GetY() << "\t" << m_cogPosition.GetZ() << std::endl;
        os << "Ixy : " << Ixy << ";\tIxz : " << Ixz << ";\tIyz : " << Iyz << std::endl;
        os << "Ixx : " << Ixx << ";\tIyy : " << Iyy << ";\tIzz : " << Izz << std::endl;
        os << std::endl;

        return os;

    }

    std::ostream& operator<<(std::ostream& os, const FrInertiaTensor& inertia) {
        return inertia.cout(os);
    }


}  // end namespace frydom
