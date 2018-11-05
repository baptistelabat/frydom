//
// Created by frongere on 01/10/18.
//

#include "FrInertia.h"

#include "FrVector.h"

#include "chrono/core/ChMatrix.h"

#include "chrono/core/ChMatrixDynamic.h"


namespace frydom {


    FrInertiaTensor_::FrInertiaTensor_() : m_mass(0.), m_cogPosition(0., 0., 0.) {}

    FrInertiaTensor_::FrInertiaTensor_(double mass,
                                       double Ixx, double Iyy, double Izz,
                                       double Ixy, double Ixz, double Iyz,
                                       const FrFrame_& coeffsFrame,
                                       const Position& cogPosition,
                                       FRAME_CONVENTION fc) {

        Position cogPosTmp = cogPosition;
        if (IsNED(fc)) {
            internal::SwapInertiaFrameConvention(Ixx, Iyy, Izz, Ixy, Ixz, Iyz); // Convert to NWU
            cogPosTmp = internal::SwapFrameConvention<Position>(cogPosTmp);
        }

        FrRotation_ rot_rp = coeffsFrame.GetRotation();

        m_inertiaAtCOG << Ixx, Ixy, Ixz, Ixy, Iyy, Iyz, Ixz, Ixy, Izz;

        m_inertiaAtCOG = rot_rp * m_inertiaAtCOG * rot_rp.Inverse();

        Position PG = cogPosTmp - coeffsFrame.GetPosition(NWU);

        m_inertiaAtCOG -= PG;
        m_cogPosition = cogPosTmp;
        m_mass = mass;



//
////        chrono::ChMatrix33<double> chInertia_pp = internal::BuildChInertiaMatrix(Ixx, Iyy, Izz, Ixy, Ixz, Iyz);
//
//        // TODO : utiliser l'API pour les rotations !!!!!!!!!!!
//
//        auto chRotation_rp = coeffsFrame.m_chronoFrame.Amatrix; // Allowed because FrInertiaTensor_ is a friend of FrFrame_ // FIXME : retirer cette amitie
//        // Cette rotation multiplie un vecteur exprime dans p en un vecteur exprime dans r
//
//        // Applying the generalized Huygens theorem to transport inertia at G, expressed in local frame coordinate system
//        m_inertiaAtCOG.Reset();
//        m_inertiaAtCOG.MatrMultiplyT(chRotation_rp * chInertia_pp, chRotation_rp);
//
//        Position PG;
//        PG = cogPosTmp - coeffsFrame.GetPosition(NWU);
//
//        m_inertiaAtCOG -= internal::GetPointMassInertia(mass, PG); // Transport inertia matrix at COG
//        m_cogPosition = cogPosTmp;
//        m_mass = mass;

    }

    FrInertiaTensor_::FrInertiaTensor_(double mass, double Ixx, double Iyy, double Izz, double Ixy, double Ixz,
                                       double Iyz, const FrFrame_ &cogFrame, FRAME_CONVENTION fc) {

        auto cogPosition = cogFrame.GetPosition(fc);

        FrFrame_ coeffsFrame;
        coeffsFrame.SetPosition(cogPosition, fc);
        coeffsFrame.SetRotation(cogFrame.GetRotation());

        *this = FrInertiaTensor_(mass, Ixx, Iyy, Izz, Ixy, Ixz, Iyz, coeffsFrame, cogPosition, fc);

    }

    double FrInertiaTensor_::GetMass() const {
        return m_mass;
    }

    const Position FrInertiaTensor_::GetCOGPosition(FRAME_CONVENTION fc) const {
        auto cogPos = m_cogPosition;
        if (IsNED(fc)) internal::SwapFrameConvention<Position>(cogPos);
        return cogPos;
    }

    void
    FrInertiaTensor_::GetInertiaCoeffs(double &Ixx, double &Iyy, double &Izz, double &Ixy, double &Ixz, double &Iyz,
                                       FRAME_CONVENTION fc) const {
        internal::ChInertia2Coeffs(m_inertiaAtCOG, Ixx, Iyy, Izz, Ixy, Ixz, Iyz);
        if (IsNED(fc)) internal::SwapInertiaFrameConvention(Ixx, Iyy, Izz, Ixy, Ixz, Iyz);
    }

    std::ostream& FrInertiaTensor_::cout(std::ostream &os) const {  // OK

        double Ixx, Iyy, Izz, Ixy, Ixz, Iyz;
        internal::ChInertia2Coeffs(m_inertiaAtCOG, Ixx, Iyy, Izz, Ixy, Ixz, Iyz);

        os << std::endl;
        os << "Inertia : \n";
        os << "Mass (kg) : " << GetMass() << std::endl;
        os << "COG (m)   : " << m_cogPosition.GetX() << "\t" << m_cogPosition.GetY() << "\t" << m_cogPosition.GetZ() << std::endl;
        os << "Ixy : " << Ixy << ";\tIxz : " << Ixz << ";\tIyz : " << Iyz << std::endl;
        os << "Ixx : " << Ixx << ";\tIyy : " << Iyy << ";\tIzz : " << Izz << std::endl;
        os << std::endl;

        return os;

    }

    std::ostream& operator<<(std::ostream& os, const FrInertiaTensor_& inertia) {
        return inertia.cout(os);
    }


}  // end namespace frydom