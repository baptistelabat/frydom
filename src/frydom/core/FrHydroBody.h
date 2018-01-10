//Class Name
// Created by frongere on 21/06/17.
//

#ifndef FRYDOM_FRHYDROBODY_H
#define FRYDOM_FRHYDROBODY_H

#include "chrono/physics/ChBodyAuxRef.h"
#include "frydom/mesh/FrTriangleMeshConnected.h"
#include "FrBody.h"
#include "FrConstants.h"
#include "FrEulerAngles.h"
#include "FrOffshoreSystem.h"


// Forward declaration
namespace chrono {
    class ChBodyAuxRef;
}

namespace frydom {

    // Forward declaration
    class FrBEMBody;

    class FrHydroBody : public FrBody {

    private:
        std::shared_ptr<FrTriangleMeshConnected> m_hydro_mesh;
        std::shared_ptr<FrTriangleMeshConnected> m_visu_mesh;

        chrono::ChVector<> m_current_relative_velocity = chrono::VNULL;
        double m_current_relative_angle = 0.;
        double m_heading = 0.;
        double m_course = 0.;

        // Geometric properties of the hydro body
        // TODO: creer classe de donnees geometriques
        double m_transverse_area = 0.;
        double m_lateral_area = 0.;
        double m_length_between_perpendicular = 0.;

        double m_wetted_surface = 0.;

        std::shared_ptr<FrBEMBody> m_BEMBody;
        bool m_UpdateHydroPosition = false;  // If true, the body position will be updated while moving in the horizontal plane
                                            // It will slow down the simulation as linear steady elevation components
                                            // of the wavefield will be updated and it may be really false in the case of
                                            // several interacting bodies. For no interactions, it could be an easy
                                            // way to take forward speed into account concerning the wave excitation...
                                            // TODO: use it...

        chrono::ChFrame<double> m_equilibriumFrame;


        boost::circular_buffer<double> m_vx_Recorder;  // circular buffer to store velocity state data for a fixed time persistence
        boost::circular_buffer<double> m_vy_Recorder;  // circular buffer to store velocity state data for a fixed time persistence
        boost::circular_buffer<double> m_vz_Recorder;  // circular buffer to store velocity state data for a fixed time persistence
        boost::circular_buffer<double> m_wx_Recorder;  // circular buffer to store velocity state data for a fixed time persistence
        boost::circular_buffer<double> m_wy_Recorder;  // circular buffer to store velocity state data for a fixed time persistence
        boost::circular_buffer<double> m_wz_Recorder;  // circular buffer to store velocity state data for a fixed time persistence


    public:

        void SetBEMBody(std::shared_ptr<FrBEMBody> BEMBody);

        std::shared_ptr<FrBEMBody> GetBEMBody() const { return m_BEMBody; }

        /// Set the hydrodynamic mesh from a mesh shared instance
        void SetHydroMesh(std::shared_ptr<FrTriangleMeshConnected> mesh, bool as_asset=true);

        /// Set the hydrodynamic mesh from a wavefront obj file
        void SetHydroMesh(std::string obj_filename, bool as_asset=true);

        /// Update hydrodynamics data relative to the hydro body then updates other body stuffs
        void Update(bool update_assets = true) override;

        /// Get the pointer to the parent ChSystem()
        FrOffshoreSystem* GetSystem() const { return dynamic_cast<FrOffshoreSystem*>(system); }  // FIXME: cache GetSystem() de ChPhysicsItem...

        /// Get the body position
        chrono::ChVector<> GetPosition(FrFrame frame= NWU) {  // FIXME: cache le GetPosition de FrBody
            switch (frame) {
                case NWU:
                    return GetPos();
                case NED:
                    return NWU2NED(GetPos());
            }
        }

        /// Get the body orientation
        chrono::ChVector<> GetOrientation(FrFrame frame= NWU) {  // FIXME: cache le GetOrientation de FrBody
            // TODO
        }

        chrono::ChVectorDynamic<double> GetSpatialVelocity(FrFrame frame=NWU) {
            // TODO: faire une classe speciale pour le vecteur spatial permettant de recuperer directement la partie lineaire et angulare (inline)
            chrono::ChVectorDynamic<double> velocity(6);

            auto linear = GetVelocity(frame);
            auto angular = GetAngularVelocity(frame);

            for (unsigned int i=0; i<3; i++) {
                velocity.ElementN(i) = linear[i];
                velocity.ElementN(i+3) = angular[i];
            }
            return velocity;
        }

        /// Get the body velocity
        chrono::ChVector<> GetVelocity(FrFrame frame= NWU) {  // FIXME: cache le GetVelocity de FrBody
            switch (frame) {
                case NWU:
                    return GetPos_dt();
                case NED:
                    return NWU2NED(GetPos_dt());
            }
        }

        /// Get the body angular velocity
        chrono::ChVector<> GetAngularVelocity(FrFrame frame= NWU) {  // FIXME: cache le GetAngularVelocity de FrBody
            switch (frame) {
                case NWU:
                    return GetWvel_par();
                case NED:
                    return NWU2NED(GetWvel_par());
            }
        }

        /// Get the heading angle defined as the angle between the x axis of the
        /// absolute frame and the x axis of the body
        double GetHeadingAngle(FrFrame frame, ANGLE_UNIT angleUnit= RAD) {

            double heading = m_heading;
            if (angleUnit == DEG) {
                heading = degrees(heading);
            }

            switch (frame) {
                case NWU:
                    if (angleUnit == DEG) {
                        return Normalize_0_360(heading);
                    } else {
                        return Normalize_0_2PI(heading);
                    }
                case NED:
                    if (angleUnit == DEG) {
                        return Normalize_0_360(-heading);
                    } else {
                        return Normalize_0_2PI(-heading);
                    }
//                    return -heading;  // FIXME: assurer des angles entre 0 et 360...
            }
        }

        /// Get the course angle defined as the angle between the x axis of the
        /// absolute frame and the velocity vector
        double GetCourseAngle(FrFrame frame, ANGLE_UNIT angleUnit= RAD) {
            double course = m_course;
            if (angleUnit == DEG) {
                course = degrees(course);
            }

            switch (frame) {
                case NWU:
                    return course;
                case NED:
                    return -course;  // FIXME: assurer des angles entre 0 et 360...
            }
        }

        /// Get the Sidesplip angle (or the drift angle) defined as the angle between
        /// the x axis of the body and the velocity vector
        double GetSideslipAngle(FrFrame frame, ANGLE_UNIT angleUnit= RAD) {

            auto heading = GetHeadingAngle(frame, angleUnit);
            auto course = GetCourseAngle(frame, angleUnit);
            return course - heading;
        }

        /// Set the heading of the body from angle in the NED frame
        void SetNEDHeading(double heading_angle, ANGLE_UNIT angleUnit= DEG);

        /// Set the heading of the body from a unit direction in the NED frame
        void SetNEDHeading(const chrono::ChVector<>& unit_vector);


        void SetEquilibriumFrame(const chrono::ChFrame<double>& eqFrame) {
            m_equilibriumFrame = eqFrame;
        }

        void SetEquilibriumFrame(const chrono::ChVector<double>& eqPos,
                                 const chrono::ChQuaternion<double>& eqQuat) {
            chrono::ChFrame<double> eqFrame;
            eqFrame.SetPos(eqPos);
            eqFrame.SetRot(eqQuat);

            SetEquilibriumFrame(eqFrame);
        }

        void SetCurrentRefFrameAsEquilibrium();

        chrono::ChFrame<double> GetEquilibriumFrame() const;


        // TODO: introduire une classe geometricProperties qui rassemble les differentes donnees...

        /// Get the transverse underwater area of the body
        double GetTransverseUnderWaterArea() const;

        /// Set the transverse underwater area of the body
        void SetTransverseUnderWaterArea(double transverse_area);

        /// Get the lateral underwater area of the body
        double GetLateralUnderWaterArea() const;

        /// Set the lateral underwater area of the body
        void SetLateralUnderWaterArea(double lateral_area);

        /// Get the length between perpendicular of the body
        double GetLpp() const;

        /// Set the length between perpendicular of the body
        void SetLpp(double lpp);

        /// Get the wetted surface of the body
        double GetWettedSurface() const;

        /// Set the wetted surface of the body
        void SetWettedSurface(double wetted_surface);

        // ==========================================================================
        // METHODS ABOUT CURRENT
        // ==========================================================================

        /// Get the relative velocity of the current field, taking into account the body's own velocity
        chrono::ChVector<> GetCurrentRelativeVelocity(FrFrame frame= NWU);

        /// Get the current relative angle
        double GetCurrentRelativeAngle(FrFrame frame= NWU, ANGLE_UNIT angleUnit= RAD) {
            double angle = m_current_relative_angle;
            if (angleUnit == DEG) {
                angle = degrees(angle);
            }

            switch (frame) {
                case NWU:
                    return angle;
                case NED:
                    return -angle;  // TODO: A verifier...
            }

        }

        // ==========================================================================
        // METHODS ABOUT ADDED MASS
        // ==========================================================================
        void IntLoadResidual_Mv(const unsigned int off,      // offset in R residual
                                chrono::ChVectorDynamic<>& R,        // result: the R residual, R += c*M*v
                                const chrono::ChVectorDynamic<>& w,  // the w vector
                                const double c               // a scaling factor
        ) override {


            auto vabs = GetPos_dt() - w.ClipVector(off, 0);  // Premiere partie de w
            auto wloc = GetWvel_loc() - w.ClipVector(off+3, 0); // Seconde partie de w

//            w(0:3) est donne par Get

            R(off + 0) += c * GetMass() * w(off + 0);
            R(off + 1) += c * GetMass() * w(off + 1);
            R(off + 2) += c * GetMass() * w(off + 2);
            chrono::ChVector<> Iw = this->GetInertia() * w.ClipVector(off + 3, 0);
            Iw *= c;
            R.PasteSumVector(Iw, off + 3, 0);

        }

        void InitializeVelocityState(unsigned int N) {  // TODO : Voir comment recuperer le N qui pourrait etre mis en cache quelque part ??

            m_vx_Recorder = boost::circular_buffer<double>(N, N, 0.);
            m_vy_Recorder = boost::circular_buffer<double>(N, N, 0.);
            m_vz_Recorder = boost::circular_buffer<double>(N, N, 0.);
            m_wx_Recorder = boost::circular_buffer<double>(N, N, 0.);
            m_wy_Recorder = boost::circular_buffer<double>(N, N, 0.);
            m_wz_Recorder = boost::circular_buffer<double>(N, N, 0.);

        }

        void RecordVelocityState(FrFrame frame=NWU) {

            std::cout << "Time : " << GetSystem()->GetChTime() << "\n";

            auto linear_velocity = GetVelocity(frame);
            auto angular_velocity = GetAngularVelocity(frame);

            // FIXME: le update de frhydrobody est appelé plusieurs fois pour le meme pas de temps !!!

            m_vx_Recorder.push_front(linear_velocity[0]);
            m_vy_Recorder.push_front(linear_velocity[1]);
            m_vz_Recorder.push_front(linear_velocity[2]);
            m_wx_Recorder.push_front(angular_velocity[0]);
            m_wy_Recorder.push_front(angular_velocity[1]);
            m_wz_Recorder.push_front(angular_velocity[2]);

        }

//        boost::circular_buffer<chrono::ChVectorDynamic<double>> GetVelocityStateBuffer() const {
//            return m_velocityStateRecorder;
//        }






    };

}  // end namespace frydom


#endif //FRYDOM_FRHYDROBODY_H
