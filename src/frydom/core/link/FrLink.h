//
// Created by frongere on 20/09/18.
//

#ifndef FRYDOM_FRLINK_H
#define FRYDOM_FRLINK_H

#include <chrono/physics/ChLink.h>
#include "frydom/core/common/FrObject.h"
//
#include "memory"


// Forward declaration
namespace chrono {
    class ChLinkLock;
    class ChLinkMotor;
}



namespace frydom {



    // Forward declarations
    class FrOffshoreSystem_;
    class FrNode_;



    class FrLink_ : public FrObject {

    protected:

//        std::shared_ptr<chrono::ChLink> m_chronoLink;

        FrOffshoreSystem_* m_system;

        std::shared_ptr<FrNode_> m_node1;
        std::shared_ptr<FrNode_> m_node2;

    public:
        FrLink_(std::shared_ptr<FrNode_> node1, std::shared_ptr<FrNode_> node2, FrOffshoreSystem_* system);

        virtual void Broke(); // TODO : Voir a rendre virtuel pur...

        FrNode_* GetNode1();

        FrNode_* GetNode2();

        // TODO : ajotuer fonctions pour donner la distance entre les noeuds...




    protected:  // TODO : voir si on rend cela private
        virtual void SetMarkers(FrNode_* node1, FrNode_* node2) = 0;

//    private:
//        friend class FrNode_;

    };




    /// Base class to be used to replicate Chrono links using the Lock approach

    // Forward declaration
//    namespace chrono {
//        class ChLinkLock;
//    }

    class _FrLinkLockBase : public FrLink_ {

    public:
        _FrLinkLockBase(std::shared_ptr<FrNode_> node1, std::shared_ptr<FrNode_> node2, FrOffshoreSystem_ *system);

    protected:
        std::shared_ptr<chrono::ChLinkLock> m_chronoLink;


    };
















    /// Base class to used to replicate Chrono ChLinkMotor classes
//    namespace chrono {
//        class ChLinkMotor;
//    }

    class _FrActuatorBase : public FrLink_ {

    protected:
        std::shared_ptr<chrono::ChLinkMotor> m_chronoLink;



    };








    // Forward declarations

    // Classes representing common links
//    class FrPrismaticLink;
//    class FrRevoluteLink;
//    class FrSphericalLink;
//    class FrCylindricalLink;
//    class FrFreeLink;
//    class FrFixedLink;
//    class FrScrewLink;
//
//
//    std::shared_ptr<FrRevoluteLink> make_revolute_link(std::shared_ptr<FrNode_> node1, std::shared_ptr<FrNode_> node2, FrOffshoreSystem_* system);
//    std::shared_ptr<FrSphericalLink> make_spherical_link(std::shared_ptr<FrNode_> node1, std::shared_ptr<FrNode_> node2, FrOffshoreSystem_* system);
//    std::shared_ptr<FrCylindricalLink> make_cylindrical_link(std::shared_ptr<FrNode_> node1, std::shared_ptr<FrNode_> node2, FrOffshoreSystem_* system);
//    std::shared_ptr<FrFreeLink> make_free_link(std::shared_ptr<FrNode_> node1, std::shared_ptr<FrNode_> node2, FrOffshoreSystem_* system);
//    std::shared_ptr<FrFixedLink> make_fixed_link(std::shared_ptr<FrNode_> node1, std::shared_ptr<FrNode_> node2, FrOffshoreSystem_* system);
//    std::shared_ptr<FrScrewLink> make_screw_link(std::shared_ptr<FrNode_> node1, std::shared_ptr<FrNode_> node2, FrOffshoreSystem_* system);
//
//
//    // Classes representing constraints between markers
//    class FrParallelConstraint;
//    class FrPerpendicularConstraint;
//    class FrPointOnLineConstraint;
//    class FrPlaneOnPlaneConstraint;
//    class FrPointOnPlaneConstraint;
//    class FrPointOnSplineConstraint;
//    class FrDistanceConstraint;
//    class FrPointDistanceToAxisConstraint;
//
//
//    // Classes representing actuators
//    class FrLinearActuatorPosition;
//    class FrLinearActuatorVelocity;
//    class FrLinearActuatorForce;
//    class FrAngularActuatorAngle;
//    class FrAngularActuatorVelocity;
//    class FrAngularActuatorTorque;


//    / Helper functions to place links of different type between markers belonging to two bodies







//    // Forward declaration
//    class FrLink_;
//
//    struct _FrLinkBase : public chrono::ChLink {
//
//        FrLink_* m_frydomLink;
//
//        explicit _FrLinkBase(FrLink_* link);
//
//        void SetupInitial() override;
//
//        void Update(bool update_assets) override;
//
//    };
//
//    // Forward declaration
//    class FrOffshoreSystem_;
//    class FrNode_;
//
//    class FrLink_ : public FrObject {
//
//    protected:
//
//        std::shared_ptr<_FrLinkBase> m_chronoLink;
//
//        FrOffshoreSystem_* m_system;
//
//
//    public:
//
//        FrLink_();
//
//        FrOffshoreSystem_* GetSystem();
//
//        void SetName(const char name[]);
//
//        std::string GetName() const;
//
//        virtual void Update() = 0;
//
//    };




}  // end namespace frydom


#endif //FRYDOM_FRLINK_H
