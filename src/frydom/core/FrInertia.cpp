//
// Created by frongere on 01/10/18.
//

#include "FrInertia.h"

#include "FrVector.h"

#include "FrMatrix.h"

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

        auto rot_rp = coeffsFrame.GetRotation().GetRotationMatrix();

        m_inertiaAtCOG << Ixx, Ixy, Ixz,
                          Ixy, Iyy, Iyz,
                          Ixz, Iyz, Izz;

        m_inertiaAtCOG = rot_rp * m_inertiaAtCOG * rot_rp.transpose();

        Position PG = cogPosTmp - coeffsFrame.GetPosition(NWU);


        m_inertiaAtCOG -= GetPointMassInertiaMatrix(mass, PG); // Avoir un GetPointMassInertia



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
        SplitMatrix33IntoCoeffs(m_inertiaAtCOG, Ixx, Ixy, Ixz, Ixy, Iyy, Iyz, Ixz, Iyz, Izz);

        if (IsNED(fc)) internal::SwapInertiaFrameConvention(Ixx, Iyy, Izz, Ixy, Ixz, Iyz);
    }

    FrInertiaTensor_::InertiaMatrix FrInertiaTensor_::GetPointMassInertiaMatrix(double mass,
                                                                                const frydom::Position &cogPos) {
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

    std::ostream& FrInertiaTensor_::cout(std::ostream &os) const {  // OK

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

    std::ostream& operator<<(std::ostream& os, const FrInertiaTensor_& inertia) {
        return inertia.cout(os);
    }


}  // end namespace frydom