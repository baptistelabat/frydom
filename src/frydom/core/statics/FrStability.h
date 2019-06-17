//
// Created by lletourn on 14/06/19.
//

#ifndef FRYDOM_FRSTABILITY_H
#define FRYDOM_FRSTABILITY_H

#include <memory>
#include <vector>
#include <frydom/core/body/FrInertiaTensor.h>

namespace frydom {

    // Forward declaration
    class Position;
    class FrRotation;
    class FrInertiaTensor;
    class FrOffshoreSystem;
    class FrBody;
    class FrNonlinearHydrostaticForce;

    class FrStability {

    public:

        explicit FrStability(const std::shared_ptr<FrBody>& body, const std::shared_ptr<FrNonlinearHydrostaticForce>& force);

        void SetRotations(const std::vector<FrRotation>& rotations);

        void AddRotation(const FrRotation& rotation);

        void CleanRotation();;

        void ComputeGZ(const FrInertiaTensor& inertia);

        void WriteResults(const std::string& filename);


    private:

        std::shared_ptr<FrBody> m_body;
        std::vector<FrRotation> m_rotations;
        std::vector<double> m_GZ;

        FrInertiaTensor m_tempInertia;

        std::shared_ptr<FrNonlinearHydrostaticForce> m_tempForce;

        FrOffshoreSystem* system();

    };

} // end namespace frydom

#endif //FRYDOM_FRSTABILITY_H
