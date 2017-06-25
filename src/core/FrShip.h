//
// Created by frongere on 21/06/17.
//

#ifndef FRYDOM_FRSHIP_H
#define FRYDOM_FRSHIP_H

#include "chrono/physics/ChLinkMate.h"
#include "FrHydroBody.h"

namespace frydom {

    // Forward declaration
    class FrPropeller;

    class FrShip : public FrHydroBody {

    private:
        // Special attributes for ships
        std::vector<std::shared_ptr<FrPropeller>> propellerlist;  // FIXME pourquoi avoir des propeller shared ???

        bool is3DOF;
        std::shared_ptr<chrono::ChLinkMatePlane> constraint3DOF;

    public:
        FrShip() {}

        ~FrShip() {}

        void AddPropeller(std::shared_ptr<FrPropeller> propeller); // FIXME: shared ?

        void RemovePropeller(std::shared_ptr<FrPropeller> propeller); // FIXME: shared ?

        int GetNbPropellers() { return int(propellerlist.size()); }

        bool Get3DOF() const { return is3DOF; };
        void Set3DOF(bool flag);
        void Set3DOF_ON();
        void Set3DOF_OFF();


        chrono::ChVector<double> GetShipVelocity() const ;


        /// Get the heading angle between the X-Axis of the NED frame and the X-Axis of the ship reference frame
        double GetHeadingAngle() {return 0;}

        /// Get the course angle between the X-Axis of the NED frame and the velocity vector of the ship
        double GetCourseAngle() {return 0;}

        /// Get the drift angle of the ship between the X-Axis of the ship and the velocity vector
        double GetDriftAngle() {return 0;}



        inline double GetSurgeVelocity() {return 0;}
        inline double GetSwayVelocity() {return 0;}
        inline double GetHeaveVelocity() {return 0;}
        inline double GetRollVelocity() {return 0;}
        inline double GetPitchVelocity() {return 0;}
        inline double GetYawVelocity() {return 0;}

    };

}  // end namespace frydom

#endif //FRYDOM_FRSHIP_H
