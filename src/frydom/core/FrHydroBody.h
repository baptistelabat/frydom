//Class Name
// Created by frongere on 21/06/17.
//

#ifndef FRYDOM_FRHYDROBODY_H
#define FRYDOM_FRHYDROBODY_H

#include <chrono/physics/ChLinkMate.h>
#include "chrono/physics/ChBodyAuxRef.h"
#include "frydom/mesh/FrTriangleMeshConnected.h"
#include "FrBody.h"
#include "FrConstants.h"
#include "FrEulerAngles.h"
#include "FrOffshoreSystem.h"
#include "frydom/hydrodynamics/FrVariablesBEMBodyMass.h"


// Forward declaration
namespace chrono {
    class ChBodyAuxRef;
    class ChSystemDescriptor;
}

namespace frydom {

    class FrHydroBody : public FrBody {


    protected:
        std::shared_ptr<FrTriangleMeshConnected> m_hydro_mesh;
        std::shared_ptr<FrTriangleMeshConnected> m_visu_mesh;

        // Attributes to let the body into the horizontal plane
        bool is3DOF = false;
        std::shared_ptr<chrono::ChLinkMatePlane> constraint3DOF;


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

//        std::shared_ptr<FrBEMBody> m_BEMBody;
        bool m_UpdateHydroPosition = false;  // If true, the body position will be updated while moving in the horizontal plane
                                            // It will slow down the simulation as linear steady elevation components
                                            // of the wavefield will be updated and it may be really false in the case of
                                            // several interacting bodies. For no interactions, it could be an easy
                                            // way to take forward speed into account concerning the wave excitation...
                                            // TODO: use it...

        chrono::ChFrame<double> m_equilibriumFrame;

        chrono::ChVariables* variables_ptr;

    public:

        FrHydroBody();

        ~FrHydroBody() {
            if (variables_ptr != &variables) {
                delete variables_ptr;
            }
        }

//        void SetBEMBody(std::shared_ptr<FrBEMBody> BEMBody);

//        std::shared_ptr<FrBEMBody> GetBEMBody() const { return m_BEMBody; }

        // TODO: deplacer la plupart de ces methodes dans hydrobody !!
        bool Get3DOF() const { return is3DOF; };
        void Set3DOF(const bool flag);
        void Set3DOF_ON();
        void Set3DOF_OFF();
        void Set3DOF_ON(chrono::ChVector<> dir);


        /// Set the hydrodynamic mesh from a mesh shared instance
        void SetHydroMesh(std::shared_ptr<FrTriangleMeshConnected> mesh, bool as_asset=true);

        /// Set the hydrodynamic mesh from a wavefront obj file
        void SetHydroMesh(std::string obj_filename, bool as_asset=true);

        /// Update hydrodynamics data relative to the hydro body then updates other body stuffs
        void Update(bool update_assets = true) override;

        /// Get the pointer to the parent ChSystem()
        FrOffshoreSystem* GetSystem() const { return dynamic_cast<FrOffshoreSystem*>(system); }  // FIXME: cache GetSystem() de ChPhysicsItem...

        /// Get the body position
        chrono::ChVector<> GetPosition(FrFrame frame= NWU) const {  // FIXME: cache le GetPosition de FrBody
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
        /*chrono::ChVector<> GetVelocity(FrFrame frame= NWU) {  // FIXME: cache le GetVelocity de FrBody
            switch (frame) {
                case NWU:
                    return GetPos_dt();
                case NED:
                    return NWU2NED(GetPos_dt());
                    // TODO : ask F why he didn't call parents method instead
            }
        }*/

        /// Get the body angular velocity
        chrono::ChVector<> GetAngularVelocity(FrFrame frame= NWU) const { // FIXME: cache le GetAngularVelocity de FrBody
            switch (frame) {
                case NWU:
                    return GetWvel_par();
                case NED:
                    return NWU2NED(GetWvel_par());
            }
        }

        /// Get the heading angle defined as the angle between the x axis of the
        /// absolute frame and the x axis of the body
        double GetHeadingAngle(FrFrame frame, ANGLE_UNIT angleUnit= RAD) const {
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
        double GetCourseAngle(FrFrame frame, ANGLE_UNIT angleUnit= RAD) const{
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
        chrono::ChVector<> GetCurrentRelativeVelocity(FrFrame frame= NWU) const;

        /// Get the relative velocity of the current field respect to a point M
        chrono::ChVector<> GetCurrentRelativeVelocity(const chrono::ChVector<>& localpos, FrFrame frame = NWU)const;

        /// Get the current relative angle
        double GetCurrentRelativeAngle(FrFrame frame= NWU, ANGLE_UNIT angleUnit= RAD) const{
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

        void SetVariables(const FrVariablesBody vtype);

        /// Return the variablesHydro component
        chrono::ChVariablesBodyOwnMass& VariablesBody() override { return dynamic_cast<chrono::ChVariablesBodyOwnMass&>(*variables_ptr); }
        chrono::ChVariables& Variables() override { return *variables_ptr; }

        /// Add a variable of type variablesHydro in system descriptor
        virtual void InjectVariables(chrono::ChSystemDescriptor& mdescriptor) override;

        /// Definition of the infinite added mass (from BEMBody) FIXME : pass to variablesHydro
        void SetInfiniteAddedMass(const Eigen::MatrixXd& CMInf);

        void IntToDescriptor(const unsigned int off_v,  // offset in v, R
                             const chrono::ChStateDelta& v,
                             const chrono::ChVectorDynamic<>& R,
                             const unsigned int off_L,  // offset in L, Qc
                             const chrono::ChVectorDynamic<>& L,
                             const chrono::ChVectorDynamic<>& Qc) override;

        void IntFromDescriptor(const unsigned int off_v,  // offset in v
                          chrono::ChStateDelta& v,
                          const unsigned int off_L,  // offset in L
                          chrono::ChVectorDynamic<>& L) override;

        void VariablesFbReset() override;

        void VariablesFbIncrementMq() override;

        void VariablesFbLoadForces(double factor) override;

        void VariablesQbLoadSpeed() override;

        void VariablesQbSetSpeed(double step) override;

        void VariablesQbIncrementPosition(double dt_step) override;

        chrono::ChVariables* GetVariables1() override { return variables_ptr; }

        template <class T=chrono::ChVariables>
        T* GetVariables() { return dynamic_cast<T*>(variables_ptr); }

//        void InitializeVelocityState(unsigned int N) {  // TODO : Voir comment recuperer le N qui pourrait etre mis en cache quelque part ??
//
//            m_vx_Recorder = boost::circular_buffer<double>(N, N, 0.);
//            m_vy_Recorder = boost::circular_buffer<double>(N, N, 0.);
//            m_vz_Recorder = boost::circular_buffer<double>(N, N, 0.);
//            m_wx_Recorder = boost::circular_buffer<double>(N, N, 0.);
//            m_wy_Recorder = boost::circular_buffer<double>(N, N, 0.);
//            m_wz_Recorder = boost::circular_buffer<double>(N, N, 0.);
//
//        }
//
//        void RecordVelocityState(FrFrame frame=NWU) {
//
//            std::cout << "Time : " << GetSystem()->GetChTime() << "\n";
//
//            auto linear_velocity = GetVelocity(frame);
//            auto angular_velocity = GetAngularVelocity(frame);
//
//            // FIXME: le update de frhydrobody est appelé plusieurs fois pour le meme pas de temps !!!
//
//            m_vx_Recorder.push_front(linear_velocity[0]);
//            m_vy_Recorder.push_front(linear_velocity[1]);
//            m_vz_Recorder.push_front(linear_velocity[2]);
//            m_wx_Recorder.push_front(angular_velocity[0]);
//            m_wy_Recorder.push_front(angular_velocity[1]);
//            m_wz_Recorder.push_front(angular_velocity[2]);
//
//        }

//        boost::circular_buffer<chrono::ChVectorDynamic<double>> GetVelocityStateBuffer() const {
//            return m_velocityStateRecorder;
//        }

		void Initialize() override;

	};

}  // end namespace frydom


#endif //FRYDOM_FRHYDROBODY_H
