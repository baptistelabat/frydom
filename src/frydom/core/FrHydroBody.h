//Class Name
// Created by frongere on 21/06/17.
//

#ifndef FRYDOM_FRHYDROBODY_H
#define FRYDOM_FRHYDROBODY_H

#include "chrono/physics/ChBodyAuxRef.h"
#include "frydom/misc/FrTriangleMeshConnected.h"
#include "FrConstants.h"
#include "FrEulerAngles.h"

// Forward declaration
namespace chrono {
    class ChBodyAuxRef;
}

namespace frydom {

    class FrHydroBody : public chrono::ChBodyAuxRef,
                        public std::enable_shared_from_this<FrHydroBody> {

    private:
        std::shared_ptr<FrTriangleMeshConnected> hydro_mesh;
        std::shared_ptr<FrTriangleMeshConnected> visu_mesh;

        chrono::ChVector<> m_current_relative_velocity;
        double m_current_relative_angle;
        double m_heading;
        double m_course;

        // Geometric properties of the hydro body
        double transverse_area;
        double lateral_area;
        double length_between_perpendicular;

        double wetted_surface;

    public:

//        FrHydroBody() : ChBodyAuxRef() {};  // TODO: est-il utile d'appeler le constructeur par defaut de la classe mere ??

        std::shared_ptr<FrHydroBody> GetShared() { return shared_from_this(); };

        /// Set the hydrodynamic mesh from a mesh shared instance
        void SetHydroMesh(std::shared_ptr<FrTriangleMeshConnected> mesh, bool as_asset=true);

        /// Set the hydrodynamic mesh from a wavefront obj file
        void SetHydroMesh(std::string obj_filename, bool as_asset=true);

        void Update(bool update_assets = true) override;



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

        void SetNEDHeading(double heading_angle, FrAngleUnit angleUnit= DEG);
        void SetNEDHeading(const chrono::ChVector<>& unit_vector);

        // ==========================================================================
        // METHODS ABOUT CURRENT
        // ==========================================================================

        /// Get the current vector flow as seen by the moving body on water
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
