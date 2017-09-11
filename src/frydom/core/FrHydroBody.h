//Class Name
// Created by frongere on 21/06/17.
//

#ifndef FRYDOM_FRHYDROBODY_H
#define FRYDOM_FRHYDROBODY_H

#include "chrono/physics/ChBodyAuxRef.h"
#include "frydom/misc/FrTriangleMeshConnected.h"
#include "FrConstants.h"
#include "FrEulerAngles.h"
#include "FrOffshoreSystem.h"

// Forward declaration
namespace chrono {
    class ChBodyAuxRef;
}

namespace frydom {

    class FrHydroBody : public chrono::ChBodyAuxRef,
                        public std::enable_shared_from_this<FrHydroBody> {

    private:
        std::shared_ptr<FrTriangleMeshConnected> m_hydro_mesh;
        std::shared_ptr<FrTriangleMeshConnected> m_visu_mesh;

        chrono::ChVector<> m_current_relative_velocity = chrono::VNULL;
        double m_current_relative_angle = 0.;
        double m_heading = 0.;
        double m_course = 0.;

        // Geometric properties of the hydro body
        double m_transverse_area = 0.;
        double m_lateral_area = 0.;
        double m_length_between_perpendicular = 0.;

        double m_wetted_surface = 0.;

    public:

        std::shared_ptr<FrHydroBody> GetShared() { return shared_from_this(); };

        /// Set the hydrodynamic mesh from a mesh shared instance
        void SetHydroMesh(std::shared_ptr<FrTriangleMeshConnected> mesh, bool as_asset=true);

        /// Set the hydrodynamic mesh from a wavefront obj file
        void SetHydroMesh(std::string obj_filename, bool as_asset=true);

        /// Update hydrodynamics data relative to the hydro body then updates other body stuffs
        void Update(bool update_assets = true) override;

        /// Get the pointer to the parent ChSystem()
        FrOffshoreSystem* GetSystem() const { return dynamic_cast<FrOffshoreSystem*>(system); }

        /// Get the body position
        chrono::ChVector<> GetPosition(FrFrame frame= NWU) {
            switch (frame) {
                case NWU:
                    return GetPos();
                case NED:
                    return NWU2NED(GetPos());
            }
        }

        /// Get the body orientation
        chrono::ChVector<> GetOrientation(FrFrame frame= NWU) {
            // TODO
        }

        /// Get the body velocity
        chrono::ChVector<> GetVelocity(FrFrame frame= NWU) {
            switch (frame) {
                case NWU:
                    return GetPos_dt();
                case NED:
                    return NWU2NED(GetPos_dt());
            }
        }

        /// Get the body angular velocity
        chrono::ChVector<> GetAngularVelocity(FrFrame frame= NWU) {
            //TODO
        }

        /// Get the heading angle defined as the angle between the x axis of the
        /// absolute frame and the x axis of the body
        double GetHeadingAngle(FrFrame frame, FrAngleUnit angleUnit= RAD) {

            double heading = m_heading;
            if (angleUnit == DEG) {
                heading = degrees(heading);
            }

            switch (frame) {
                case NWU:
                    return heading;
                case NED:
                    return -heading;  // FIXME: assurer des angles entre 0 et 360...
            }
        }

        /// Get the course angle defined as the angle between the x axis of the
        /// absolute frame and the velocity vector
        double GetCourseAngle(FrFrame frame, FrAngleUnit angleUnit= RAD) {
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
        double GetSideslipAngle(FrFrame frame, FrAngleUnit angleUnit= RAD) {

            auto heading = GetHeadingAngle(frame, angleUnit);
            auto course = GetCourseAngle(frame, angleUnit);
            return course - heading;
        }

        /// Set the heading of the body from angle in the NED frame
        void SetNEDHeading(double heading_angle, FrAngleUnit angleUnit= DEG);

        /// Set the heading of the body from a unit direction in the NED frame
        void SetNEDHeading(const chrono::ChVector<>& unit_vector);

        /// Get the transverse underwater area of the body
        double GetTransverseUnderwaterArea() const;

        /// Set the transverse underwater area of the body
        void SetTransverseUnderwaterArea(double transverse_area);

        /// Get the lateral underwater area of the body
        double GetLateralUnderwaterArea() const;

        /// Set the lateral underwater area of the body
        void SetLateralUnderwaterArea(double lateral_area);

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
        double GetCurrentRelativeAngle(FrFrame frame= NWU, FrAngleUnit angleUnit= RAD) {
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

    };

}  // end namespace frydom


#endif //FRYDOM_FRHYDROBODY_H
