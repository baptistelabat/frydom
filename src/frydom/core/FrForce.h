//
// Created by frongere on 08/06/17.
//

#ifndef FRYDOM_FRFORCE_H
#define FRYDOM_FRFORCE_H

#include "frydom/core/FrGeographic.h"
#include "chrono/physics/ChForce.h"
#include "hermes/hermes.h"

#include "FrVector.h"

#include "FrObject.h"

// Forward declaration
//namespace chrono {
//    class ChForce;
//
//    template <class Real=double>
//    class ChVector;
//}

namespace frydom {

    class FrForce :
            public chrono::ChForce,
            public FrObject
    {

    protected:
//        chrono::ChBody* Body;
//        chrono::ChVector<> force = chrono::VNULL;
        chrono::ChVector<> moment = chrono::VNULL;
        hermes::Message m_log;
        bool is_log = true;

        std::string m_logPrefix = "";           //TODO : à definir dans hermes

    public:

//        FrForce() : moment(chrono::VNULL) {};
//        FrForce() = default;


        /// Updates the time of the object and other stuff that are time-dependent
        /// into the object
        void UpdateTime(double mytime) {
            ChTime = mytime;

            // ... put time-domain stuff here
        }

        /// Update the force object.
        /// Must be implemented into the child classes.
        virtual void UpdateState() = 0;

        /// Get the force-torque applied to rigid, body as force vector.
        /// CAUTION !!!! : The force must be returned in the absolute coordinates while the torque must be
        /// expressed in body coordinates
        void GetBodyForceTorque(chrono::ChVector<>& body_force, chrono::ChVector<>& body_torque) const {
            body_force = force;
            body_torque = moment;
        }

        /// Return the moment of the force
        virtual chrono::ChVector<> GetTorque() const { return moment; }

        virtual void Initialize() override {
            if (is_log) {
                SetLog();
                InitializeLogs();
            }
        }

        virtual void StepFinalize() override {
            if (is_log) {
                UpdateLogs();
            }
        }

        /// Set the definition of the log message
        virtual void SetLog();

        virtual void SetLogNameAndDescription(std::string name = "Force_message",
                                         std::string description = "Message of the force") {
            m_log.SetNameAndDescription(name, description);
            is_log = true;
        }

        /// Deactivate the generation of log from the body
        virtual void DeactivateLog() { is_log = false; };

        /// Initialization of the log message
        virtual void InitializeLogs();

        /// Update of the log message
        virtual void UpdateLogs();

        hermes::Message* GetLog() { return &m_log; }

        /// Definition of the log messsage name
        virtual void SetLogPrefix(std::string prefix_name = "") { m_logPrefix = prefix_name; };

    };





























    /// REFACTORING ------------->>>>>>>>>>>>><

//    using ForceVector  = Vector3d_;
//    using MomentVector = Vector3d_;

    class FrForce_;
    struct _FrForceBase : public chrono::ChForce {

        FrForce_* m_frydomForce;

        _FrForceBase();

//        void Update(double time) override;

        friend class FrForce_;

    };


    // Forward declaration;
    class FrOffshoreSystem_;
    class FrBody_;
    class FrNode_;


    class FrForce_ : public FrObject {

    protected:

        FrBody_* m_body;

        std::shared_ptr<_FrForceBase> m_chronoForce;

        std::shared_ptr<FrNode_> m_node;

        // Limits on forces to stabilize simulation
        bool m_limitForce = false;
        double m_forceLimit;
        double m_torqueLimit;


    public:

        explicit FrForce_(std::shared_ptr<FrNode_> node);

//        explicit FrForce_(FrBody_* body);

        virtual void Update(double time) = 0;

        FrOffshoreSystem_* GetSystem();

        // Force Limits

        void SetMaxForceLimit(double fmax);

        void SetMaxTorqueLimit(double tmax);

        void SetLimit(bool val);


        // Force Getters

        void GetAbsForce(Force& force, FRAME_CONVENTION fc) const;

        Force GetAbsForce(FRAME_CONVENTION fc) const;

        void GetAbsForce(double& fx, double& fy, double& fz, FRAME_CONVENTION fc) const;

        void GetLocalForce(Force& force, FRAME_CONVENTION fc) const;

        Force GetLocalForce(FRAME_CONVENTION fc) const;

        void GetLocalForce(double& fx, double& fy, double& fz, FRAME_CONVENTION fc) const;

        void GetAbsMoment(Force& force, FRAME_CONVENTION fc) const;

        Force GetAbsMoment(FRAME_CONVENTION fc) const;

        void GetAbsMoment(double& fx, double& fy, double& fz, FRAME_CONVENTION fc) const;

        void GetLocalMoment(Force& force, FRAME_CONVENTION fc) const;

        Force GetLocalMoment(FRAME_CONVENTION fc) const;

        void GetLocalMoment(double& fx, double& fy, double& fz, FRAME_CONVENTION fc) const;

        double GetForceNorm() const;

        double GetTorqueNorm() const;



    protected:

        std::shared_ptr<chrono::ChForce> GetChronoForce();

        friend class FrBody_;

        void SetLocalMomentAtNode(const chrono::ChVector<double> &momentAtNode);

        void SetAbsMomentAtNode(const chrono::ChVector<double> &momentAtNode);

        void SetLocalForce(const chrono::ChVector<double>& force);

        void SetAbsForce(const chrono::ChVector<double>& force);


//        explicit FrForce_(FrBody_* body);
//
//
//        ForceVector GetForceVectorAbsFrame();
//
//        ForceVector GetForceVectorLocalFrame();
//
////        ForceVector GetForceVectorOtherFrame(std::shared_ptr<FrFrame_> frame);
//
//        MomentVector GetMomentVectorAbsFrame();
//
//        MomentVector GetMomentVectorLocalFrame();
//
////        MomentVector GetMomentVectorOtherFrame(std::shared_ptr<FrFrame_> frame);
//
//
//
//        virtual void Update(double time) = 0;
//
////        void Initialize() override;
////
////        void StepFinalize() override;
//
//
//
//
    };











}  // end namespace frydom

#endif //FRYDOM_FRFORCE_H
