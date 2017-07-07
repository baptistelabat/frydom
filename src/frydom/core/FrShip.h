//
// Created by frongere on 21/06/17.
//

#ifndef FRYDOM_FRSHIP_H
#define FRYDOM_FRSHIP_H

#include "chrono/physics/ChLinkMate.h"
#include "FrHydroBody.h"
#include "FrConstants.h"

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
        FrShip() : is3DOF(false),
                   FrHydroBody() {}

        ~FrShip() {}

        void AddPropeller(std::shared_ptr<FrPropeller> propeller); // FIXME: shared ?

        void RemovePropeller(std::shared_ptr<FrPropeller> propeller); // FIXME: shared ?

        int GetNbPropellers() { return int(propellerlist.size()); }


        // TODO: deplacer la plupart de ces methodes dans hydrobody !!
        bool Get3DOF() const { return is3DOF; };
        void Set3DOF(const bool flag);
        void Set3DOF_ON();
        void Set3DOF_OFF();

        // FIXME: le get position renvoie la position mais pas l'attitude !!!!
        // TODO: dupliquer tout ca pour l'attitude...
        chrono::ChVector<double> GetPosition(FrFrame = NED) const;

        chrono::ChQuaternion<double> GetAttitude(FrFrame = NED) const { return chrono::QUNIT; };


        void SetPosition(const double x, const double y, const double z, FrFrame = NED) {}
        void SetXPosition(const double x, FrFrame = NED) {}
        void SetYPosition(const double y, FrFrame = NED) {}
        void SetZPosition(const double z, FrFrame = NED) {}

        // TODO: recuperer les positions pertubees par rapport a un repere hydrodynamique en mouvement stationnaire
        // par rapport au repere absolu (NWU ou NED)



        // FIXME voir pour la vitesse de rotation !!
        chrono::ChVector<double> GetVelocity(FrFrame = NED) const { return chrono::VNULL; }
        void SetVelocity(const double vx, const double vy, const double vz, FrFrame = NED) {}

        double GetSurgeVelocity() {return 0;}
        double GetSwayVelocity() {return 0;}
        double GetHeaveVelocity() {return 0;}
        double GetRollVelocity() {return 0;}
        double GetPitchVelocity() {return 0;}
        double GetYawVelocity() {return 0;}





        // FIXME : voir pour l'acceleration angulaire !!
        chrono::ChVector<double> GetAcceleration(FrFrame = NED) const { return chrono::VNULL; }


        chrono::ChVector<double> GetShipVelocity() const ; //FIXME : degager cette fonction...


        /// Get the heading angle between the X-Axis of the NED frame and the X-Axis of the ship reference frame
        double GetHeadingAngle() {return 0;}

        /// Get the course angle between the X-Axis of the NED frame and the velocity vector of the ship
        double GetCourseAngle() {return 0;}

        /// Get the drift angle of the ship between the X-Axis of the ship and the velocity vector
        double GetDriftAngle() {return 0;}





    };

}  // end namespace frydom

#endif //FRYDOM_FRSHIP_H
