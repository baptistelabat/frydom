//
// Created by frongere on 20/09/18.
//

#include "FrFrame.h"
#include "frydom/core/math/FrMatrix.h"

#include "chrono/core/ChMatrixDynamic.h"

#include "FrRotation.h"
#include "frydom/core/FrOffshoreSystem.h"


namespace frydom {


    FrFrame_::FrFrame_() : m_chronoFrame() {};  // OK

    FrFrame_::FrFrame_(const Position &pos, const FrRotation_ &rotation, FRAME_CONVENTION fc) {  // OK
        SetPosition(pos, fc);
        SetRotation(rotation);
    }

    FrFrame_::FrFrame_(const Position &pos, const frydom::FrUnitQuaternion_ &quaternion, FRAME_CONVENTION fc) {  // OK
        SetPosition(pos, fc);
        SetRotation(quaternion);
    }

    FrFrame_& FrFrame_::FrFrame(const FrFrame_ &otherFrame) {  // OK
        m_chronoFrame = otherFrame.m_chronoFrame;
    }

    void FrFrame_::SetPosition(double x, double y, double z, FRAME_CONVENTION fc) {  // OK

        if (IsNED(fc)) internal::SwapCoordinateConvention(x, y, z); // Express In NWU
        m_chronoFrame.SetPos(chrono::ChVector<double>(x, y, z));
    }

    void FrFrame_::SetPosition(const Position& position, FRAME_CONVENTION fc) {  // OK
        auto posTmp = position;
        if (IsNED(fc)) internal::SwapFrameConvention<Position>(posTmp);
        m_chronoFrame.SetPos(internal::Vector3dToChVector(posTmp));
    }

    void FrFrame_::GetPosition(double &x, double &y, double &z, FRAME_CONVENTION fc) const {  // OK
        auto pos = m_chronoFrame.GetPos();  // In NWU
        x = pos.x();
        y = pos.y();
        z = pos.z();

        if (IsNED(fc)) internal::SwapCoordinateConvention(x, y, z);
    }

    void FrFrame_::GetPosition(Position &position, FRAME_CONVENTION fc) const {  // OK
        position = GetPosition(fc);
    }

    Position FrFrame_::GetPosition(FRAME_CONVENTION fc) const {  // OK
        auto pos = internal::ChVectorToVector3d<Position>(m_chronoFrame.GetPos()); // In NWU
        if (IsNED(fc)) internal::SwapFrameConvention<Position>(pos);
        return pos;
    }

    void FrFrame_::SetX(double x, FRAME_CONVENTION fc) {  // OK
        m_chronoFrame.GetPos()[0] = x;
    }

    void FrFrame_::SetY(double y, FRAME_CONVENTION fc) {  // OK
        if (IsNED(fc)) y = -y;
        m_chronoFrame.GetPos()[1] = y;
    }

    void FrFrame_::SetZ(double z, FRAME_CONVENTION fc) {  // OK
        if (IsNED(fc)) z = -z;
        m_chronoFrame.GetPos()[2] = z;
    }

    double FrFrame_::GetX(FRAME_CONVENTION fc) const {  // OK
        return m_chronoFrame.GetPos()[0];
    }

    double FrFrame_::GetY(FRAME_CONVENTION fc) const {  // OK
        double y = m_chronoFrame.GetPos()[1];
        if (IsNED(fc)) y = -y;
        return y;
    }

    double FrFrame_::GetZ(FRAME_CONVENTION fc) const {  // OK
        double z = m_chronoFrame.GetPos()[2];
        if (IsNED(fc)) z = -z;
        return z;
    }

    void FrFrame_::Set(Position pos, Direction e1, Direction e2, Direction e3, FRAME_CONVENTION fc) {
        SetPosition(pos, fc);

        mathutils::Matrix33<double> matrix;
        matrix <<   e1.Getux(), e2.Getux(), e3.Getux(),
                e1.Getuy(), e2.Getuy(), e3.Getuy(),
                e1.Getuz(), e2.Getuz(), e3.Getuz();

        m_chronoFrame.SetRot(internal::Matrix33ToChMatrix33(matrix));
    }

    void FrFrame_::SetRotation(const FrRotation_ &rotation) {  // OK
        SetRotation(rotation.GetQuaternion());
    }

    void FrFrame_::SetRotation(const FrUnitQuaternion_ &quaternion) {  // OK
        m_chronoFrame.SetRot(internal::Fr2ChQuaternion(quaternion));
    }

    void FrFrame_::SetNoRotation() {  // OK
        auto curPos = m_chronoFrame.GetPos();
        m_chronoFrame.SetIdentity();
        m_chronoFrame.SetPos(curPos);
    }

    void FrFrame_::SetNoTranslation() {  // OK
        m_chronoFrame.GetPos().SetNull();
    }

    void FrFrame_::SetIdentity() {  // OK
        m_chronoFrame.SetIdentity();
    }

    FrRotation_ FrFrame_::GetRotation() const {  // OK
        return FrRotation_(GetQuaternion());
    }

    FrUnitQuaternion_ FrFrame_::GetQuaternion() const {  // OK
        return internal::Ch2FrQuaternion(m_chronoFrame.GetRot());  // In NWU
    }

    FrFrame_ FrFrame_::operator*(const FrFrame_ &otherFrame) const {
        auto newFrame = FrFrame_();
        newFrame.m_chronoFrame = m_chronoFrame * otherFrame.m_chronoFrame;
        return newFrame;
    }

    void FrFrame_::operator*=(const FrFrame_ &otherFrame) {
        m_chronoFrame >>= otherFrame.m_chronoFrame;  // TODO : verifier !!
    }

    FrFrame_ FrFrame_::GetOtherFrameRelativeTransform_WRT_ThisFrame(const frydom::FrFrame_ &otherFrame) const {  // OK
        return this->GetInverse() * otherFrame;
    }

    FrFrame_ FrFrame_::GetThisFrameRelativeTransform_WRT_OtherFrame(const frydom::FrFrame_ &otherFrame) const {  // OK
        return GetOtherFrameRelativeTransform_WRT_ThisFrame(otherFrame).GetInverse();
    }

    std::ostream& FrFrame_::cout(std::ostream &os) const {  // OK

        double x, y, z;
        GetPosition(x, y, z, NWU);

        os << std::endl;
        os << "Frame :\n";
        os << "-------\n";
        os << "Translation (m, In NWU): ";
        os << "X = " << x;
        os << "; Y = " << y;
        os << "; Z = " << z;
        os << std::endl;
        os << GetRotation();
        os << std::endl;

        return os;

    }

    std::ostream& operator<<(std::ostream& os, const FrFrame_& frame) {  // OK
        return frame.cout(os);
    }

//    void FrFrame_::RotateInFrame(const Direction &direction, double angleRad, FRAME_CONVENTION fc) {
//
//
//
//
//
////        auto tmpDirection = direction;
////        if (localAxis) {
////            tmpDirection = ProjectVectorFrameInParent<Direction>(tmpDirection, fc);
////            // FIXME : les donnees (position, orientation) de FrFrame_ sont relatives au repere parent.
////            // Pour la position, c'est la position de l'origine du repere courant par rapport au repere parent, exprime
////            // dans les axes du repere parent.
////            // Pour la rotation, c'est la matrice qui multipliee par un vecteur exprime dans le vecteur courant, donne
////            // les coordonnees de ce meme vecteur dans le repere parent.
////        }
////
////        FrUnitQuaternion_ rotQuat(tmpDirection, angleRad, fc);
//
//
////        GetQuaternion()
//
//
//        /*
//         * Reflexion sur le Rotate ou Translate sur un frame ou node :
//         *
//         * Pour la rotation, soit on multiplie a gauche, soit a droite.
//         *
//         * Si on pose iRj la rotation qui fait iu = iRj ju, alors:
//         *
//         * En multipliant a gauche par kRi permet d'avroi ku = kRi iRj ju = kRj ju   soit on incremente la rotation
//         *
//         *
//         * TRES BIEN : A RETENIR !!!!!!!!!!!!!!!!!!!!
//         *
//         * Lorsqu'on multiplie par une rotation exprimee dans le repere parent, on multiplie a gauche
//         * Lorsqu'on multiplie par une rotation exprimee dans le repere cible, on multiplie a droite...
//         *
//         */
//
//
//
//    }

    void FrFrame_::RotateInFrame(const FrUnitQuaternion_& quaternion) {
        m_chronoFrame.SetRot(m_chronoFrame.GetRot() * internal::Fr2ChQuaternion(quaternion));
    }

    void FrFrame_::RotateInFrame(const FrRotation_& rotation) {
        RotateInFrame(rotation.GetQuaternion());
    }

    void FrFrame_::RotateInFrame(const Direction &direction, double angleRad, FRAME_CONVENTION fc) {
        RotateInFrame(FrUnitQuaternion_(direction, angleRad, fc));
    }

    void FrFrame_::RotateInFrame(double phiRad, double thetaRad, double psiRad,  EULER_SEQUENCE seq, FRAME_CONVENTION fc) {
        FrRotation_ rotation;
        rotation.SetEulerAngles_DEGREES(phiRad, thetaRad, psiRad, seq, fc);
        RotateInFrame(rotation);
    }

    void FrFrame_::RotateInParent(const FrUnitQuaternion_& quaternion) {
        m_chronoFrame.SetRot(internal::Fr2ChQuaternion(quaternion) * m_chronoFrame.GetRot());
    }

    void FrFrame_::RotateInParent(const FrRotation_& rotation) {
        RotateInParent(rotation.GetQuaternion());
    }

    void FrFrame_::RotateInParent(const Direction &direction, double angleRad, FRAME_CONVENTION fc) {
        RotateInParent(FrUnitQuaternion_(direction, angleRad, fc));
    }

    void FrFrame_::RotateInParent(double phiRad, double thetaRad, double psiRad,  EULER_SEQUENCE seq, FRAME_CONVENTION fc) {
        FrRotation_ rotation;
        rotation.SetEulerAngles_DEGREES(phiRad, thetaRad, psiRad, seq, fc);
        RotateInParent(rotation);
    }

    void FrFrame_::TranslateInFrame(const Translation& translation, FRAME_CONVENTION fc) {
        TranslateInParent(ProjectVectorFrameInParent<Translation>(translation, fc), fc);
    }

    void FrFrame_::TranslateInFrame(const Direction& direction, double distance, FRAME_CONVENTION fc) {
        assert(direction.IsUnit());
        TranslateInFrame(direction * distance, fc);
    }

    void FrFrame_::TranslateInFrame(double x, double y, double z, FRAME_CONVENTION fc) {
        TranslateInFrame(Translation(x, y, z), fc);
    }

    void FrFrame_::TranslateInParent(const Translation& translation, FRAME_CONVENTION fc) {
        auto tmpTranslation = translation;
        if (IsNED(fc)) internal::SwapFrameConvention<Translation>(tmpTranslation);
        m_chronoFrame.Move(internal::Vector3dToChVector(tmpTranslation));
    }

    void FrFrame_::TranslateInParent(const Direction& direction, double distance, FRAME_CONVENTION fc) {
        assert(direction.IsUnit());
        TranslateInParent(direction * distance, fc);
    }

    void FrFrame_::TranslateInParent(double x, double y, double z, FRAME_CONVENTION fc) {
        TranslateInParent(Translation(x, y, z), fc);
    }



    // FIXME : c'est une multiplication a gauche !
    // FIXME : changer les implementations de tous les RotX etc... et reposer sur des methodes de FrRotation et FrQuaternion !!!!
    void FrFrame_::RotX_RADIANS(double angle, FRAME_CONVENTION fc, bool localAxis) {
        chrono::ChVector<double> axis;
        if (localAxis) {
            axis = m_chronoFrame.GetRot().GetXaxis();  // In NWU
        } else {
            axis = chrono::ChVector<double>(1., 0., 0.);
        }

        m_chronoFrame.SetRot(chrono::Q_from_AngAxis(angle, axis) * m_chronoFrame.GetRot());
    }

    void FrFrame_::RotX_DEGREES(double angle, FRAME_CONVENTION fc, bool localAxis) {
        RotX_RADIANS(angle*DEG2RAD, fc, localAxis);
    }

    void FrFrame_::RotY_RADIANS(double angle, FRAME_CONVENTION fc, bool localAxis) {
        chrono::ChVector<double> axis;
        if (localAxis) {
            axis = m_chronoFrame.GetRot().GetYaxis();  // In NWU
        } else {
            axis = chrono::ChVector<double>(0., 1., 0.);
        }

        if (IsNED(fc)) angle = -angle;

        m_chronoFrame.SetRot(chrono::Q_from_AngAxis(angle, axis) * m_chronoFrame.GetRot());
    }

    void FrFrame_::RotY_DEGREES(double angle, FRAME_CONVENTION fc, bool localAxis) {
        RotY_RADIANS(angle*DEG2RAD, fc, localAxis);
    }

    void FrFrame_::RotZ_RADIANS(double angle, FRAME_CONVENTION fc, bool localAxis) {
        chrono::ChVector<double> axis;
        if (localAxis) {
            axis = m_chronoFrame.GetRot().GetZaxis();  // In NWU
        } else {
            axis = chrono::ChVector<double>(0., 0., 1.);
        }

        if (IsNED(fc)) angle = -angle;

        m_chronoFrame.SetRot(chrono::Q_from_AngAxis(angle, axis) * m_chronoFrame.GetRot());
    }

    void FrFrame_::RotZ_DEGREES(double angle, FRAME_CONVENTION fc, bool localAxis) {
        RotZ_RADIANS(angle*DEG2RAD, fc, localAxis);
    }

    void FrFrame_::SetRotX_RADIANS(double angle, FRAME_CONVENTION fc) {  // OK
        SetIdentity();
        RotX_RADIANS(angle, fc, false);
    }

    void FrFrame_::SetRotX_DEGREES(double angle, FRAME_CONVENTION fc) {  // OK
        SetIdentity();
        RotX_DEGREES(angle, fc, false);
    }

    void FrFrame_::SetRotY_RADIANS(double angle, FRAME_CONVENTION fc) {  // OK
        SetIdentity();
        RotY_RADIANS(angle, fc, false);
    }

    void FrFrame_::SetRotY_DEGREES(double angle, FRAME_CONVENTION fc) {  // OK
        SetIdentity();
        RotY_DEGREES(angle, fc, false);
    }

    void FrFrame_::SetRotZ_RADIANS(double angle, FRAME_CONVENTION fc) {  // OK
        SetIdentity();
        RotZ_RADIANS(angle, fc, false);
    }

    void FrFrame_::SetRotZ_DEGREES(double angle, FRAME_CONVENTION fc) {  // OK
        SetIdentity();
        RotZ_DEGREES(angle, fc, false);
    }

    FrFrame_ &FrFrame_::Inverse() {  // OK
        m_chronoFrame.Invert();
        return *this;
    }

    FrFrame_ FrFrame_::GetInverse() const {  // OK
        return internal::Ch2FrFrame(m_chronoFrame.GetInverse());
    }

    FrFrame_ FrFrame_::ProjectToXYPlane(FRAME_CONVENTION fc) const {

        Direction xaxis = this->GetRotation().GetXAxis(fc);
        xaxis.z() = 0.;
        xaxis.normalize();

        Direction yaxis = this->GetRotation().GetYAxis(fc);
        yaxis.z() = 0.;
        yaxis.normalize();

        Direction zaxis = Direction(0., 0., 1.);
        Position origin = this->GetPosition(fc);

        return FrFrame_(origin, FrRotation_(xaxis, yaxis, zaxis, fc), fc);
    }


}  // end namespace frydom
