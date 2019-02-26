// ==========================================================================
// FRyDoM - frydom-ce.org
//
// Copyright (c) Ecole Centrale de Nantes (LHEEA lab.) and D-ICE Engineering.
// All rights reserved.
//
// Use of this source code is governed by a GPLv3 license that can be found
// in the LICENSE file of FRyDoM.
//
// ==========================================================================


#ifndef FRYDOM_FRFORCE_H
#define FRYDOM_FRFORCE_H


#include "chrono/physics/ChForce.h"

#include "frydom/core/math/FrVector.h"
#include "frydom/core/common/FrObject.h"


namespace frydom {

    /**
     * \class FrForce_
     * \brief Base class for every external forces on bodies
     */
    class FrForce;

    namespace internal {

        struct _FrForceBase : public chrono::ChForce {

            FrForce *m_frydomForce;
            chrono::ChVector<double> m_torque; // Expressed in body coordinates at COG


            explicit _FrForceBase(FrForce *force);

            void UpdateState() override;

            void GetBodyForceTorque(chrono::ChVector<double> &body_force,
                                    chrono::ChVector<double> &body_torque) const override;

            void GetForceInWorldNWU(Force &body_force) const;

            void GetTorqueInBodyNWU(Torque &body_torque) const;

            void SetForceInWorldNWU(const Force &body_force);

            void SetTorqueInBodyNWU(const Torque &body_torque);

            friend class FrForce_;

        };

    }  // end namespace frydom::internal

    // Forward declaration;
    class FrOffshoreSystem;
    class FrBody;
    class FrNode;
    class FrForceAsset;

    /**
     * \class FrForce_
     * \brief  Class defining an effort with force and torque vector
     */
    class FrForce : public FrObject {

    protected:

        FrBody* m_body;                ///< Pointer to the body to which the force is applied

        std::shared_ptr<internal::_FrForceBase> m_chronoForce;     ///< Pointer to the force chrono object

        // Force Asset
        bool m_isForceAsset = false;            ///< A ForceAsset (vector) is displayed if true
        std::shared_ptr<FrForceAsset> m_forceAsset = nullptr;  ///< pointer to the ForceAsset object.

        // Limits on forces to stabilize simulation
        bool m_limitForce = false;              ///< Flag equals to true if the maximum force and torque limit are used, false otherwise
        double m_forceLimit  = 1e20;            ///< Taking very high values by default in case we just set limit to true without
        double m_torqueLimit = 1e20;            ///< setting the values individually.


    public:

        /// Default constructor that builds a new force with zero force and torque
        FrForce();

        /// This subroutine initializes the object FrForce.
        void Initialize() override;

        // TODO : boucle de StepFinalize à mettre en place dans FrBody
        void StepFinalize() override;

        /// Virtual function to allow updating the child object from the solver
        /// \param time Current time of the simulation from begining, in seconds
        virtual void Update(double time) = 0;

        /// Return the system to which the force is linked
        /// \return Offshore system object pointer
        FrOffshoreSystem* GetSystem();

        // Force Asset
        /// Inquire if a ForceAsset is displayed
        /// \return true if a ForceAsset is displayed
        bool IsForceAsset();

        /// Set if a ForceAsset is to be displayed
        /// \param isAsset true if a ForceAsset is to be displayed
        void SetIsForceAsset(bool isAsset);

        // Force Limits

        /// Define the maximum force amplitude. If the force amplitude is higher to this value
        /// the force is scaled according to the maximum force amplitude
        /// \param fmax Maximum force amplitude, in Newton
        void SetMaxForceLimit(double fmax);

        /// Return the maximum force amplitude
        /// \return Maximum force amplitude, in Newton
        double GetMaxForceLimit() const;

        /// Define the maximum torque amplitude. If the torque amplitude is higher to this value
        /// the torque is scaled according to the maximum torque amplitude
        /// \param tmax Maximum torque amplitude, in N.m
        void SetMaxTorqueLimit(double tmax);

        /// Return the maximum torque amplitude
        /// \return Maximum torque amplitude, in N.m
        double GetMaxTorqueLimit() const;

        /// Define if the maximum amplitude limit for torque and force are used. Setting is true if
        /// the limit are used, false otherwise.
        /// \param val Boolean true/false
        void SetLimit(bool val);

        /// Return true if the maximum limits for torque and force are used, false otherwise
        /// \return Bollean true/false
        bool GetLimit() const;


        // Force Getters

        /// Get the application point of the force, in the world reference frame.
        /// \param fc Frame convention (NED/NWU)
        /// \return position of the application point in the world reference frame.
        Position GetForceApplicationPointInWorld(FRAME_CONVENTION fc) const;

        /// Get the application point of the force, in the body reference frame.
        /// \param fc Frame convention (NED/NWU)
        /// \return position of the application point in the body reference frame.
        Position GetForceApplicationPointInBody(FRAME_CONVENTION fc) const;

        /// Return the force vector at COG in world coordinates with the given convention
        /// \param force Force vector at COG in world coordinates
        /// \param fc Frame convention
        void GetForceInWorld(Force& force, FRAME_CONVENTION fc) const;

        /// Return the force vector at COG in world coordinates with the given convention
        /// \param fc Frame convention
        /// \return Force vector at COG in world coordinates
        Force GetForceInWorld(FRAME_CONVENTION fc) const;

        /// Return the force vector components at COG in world coordinates with the given convention
        /// \param fx x-component of the force vector in world coordinates
        /// \param fy y-component of the force vector in world coordinates
        /// \param fz z-component of the force vector in world coordinates
        /// \param fc Frame convention
        void GetForceInWorld(double& fx, double& fy, double& fz, FRAME_CONVENTION fc) const;

        /// Return the force vector at COG in body coordinates with the given convention
        /// \param force Force vector at COG in body coordinates
        /// \param fc Frame convention
        void GetForceInBody(Force& force, FRAME_CONVENTION fc) const;

        /// Return the force vector at COG in body coordinates with the given convention
        /// \param fc Force vector at COG in body coordinates
        /// \return Frame convention
        Force GetForceInBody(FRAME_CONVENTION fc) const;

        /// Return the force vector components at COG in body coordinates with the given convention
        /// \param fx x-component of the force vector in body coordinates
        /// \param fy y-component of the force vector in body coordinates
        /// \param fz z-component of the force vector in body coordinates
        /// \param fc Frame convention
        void GetForceInBody(double& fx, double& fy, double& fz, FRAME_CONVENTION fc) const;

        /// Return the torque vector at COG in world coordinates with the given convention
        /// \param torque Torque vector at COG in world coordinates
        /// \param fc Frame convention
        void GetTorqueInWorldAtCOG(Torque &torque, FRAME_CONVENTION fc) const;

        /// Return the torque vector at COG in world coordinates with the given convention
        /// \param fc Torque vector at COG in world coordinates
        /// \return Frame convention
        Torque GetTorqueInWorldAtCOG(FRAME_CONVENTION fc) const;

        /// Return the torque vector components at COG in world coordinates with the given convention
        /// \param mx x-component of the torque vector in world coordinates
        /// \param my y-component of the torque vector in world coordinates
        /// \param mz z-component of the torque vector in world coordinates
        /// \param fc Frame convention
        void GetTorqueInWorldAtCOG(double &mx, double &my, double &mz, FRAME_CONVENTION fc) const;

        /// Return the torque vector at COG in body coordinates with the given convention
        /// \param torque Torque vector at COG in body coordinates
        /// \param fc Frame convention
        void GetTorqueInBodyAtCOG(Torque &torque, FRAME_CONVENTION fc) const;

        /// Return the torque vector at COG in body coordinates with the given convention
        /// \param fc Frame convention
        /// \return Torque vector at COG in body coordinates
        Torque GetTorqueInBodyAtCOG(FRAME_CONVENTION fc) const;

        /// Return the torque vector components at COG in body coordinates with given convention
        /// \param mx x-component of the torque vector in body coordinates
        /// \param my y-component of the torque vector in body coordinates
        /// \param mz z-component of the torque vector in body coordinates
        /// \param fc Frame convention
        void GetTorqueInBodyAtCOG(double &mx, double &my, double &mz, FRAME_CONVENTION fc) const;

        /// Return the amplitude of the force
        /// \return Amplitude of the force, in Newton
        double GetForceNorm() const;

        /// Return the amplitude of the torque
        /// \return Amplitude of the torque, in N.m
        double GetTorqueNormAtCOG() const;


    protected:

        // The following methods are to be used in the implementation of Update method to set the force and torque
        // of the force model used

        /// Set the force expressed in world coordinates at COG. It does not generate a torque
        /// \param worldForce Force expressed in world coordinates
        /// \param fc Frame convention
        void SetForceInWorldAtCOG(const Force& worldForce, FRAME_CONVENTION fc);

        /// Set the force expressed in world coordinates applying to a point expressed in body coordinates.
        /// It generates a torque.
        /// \param worldForce Force expressed in world coordinates
        /// \param bodyPos Point position expressed in body coordinates
        /// \param fc Frame convention
        void SetForceInWorldAtPointInBody(const Force& worldForce, const Position& bodyPos, FRAME_CONVENTION fc);

        /// Set the force expressed in world coordinates applying to a point expressed in world coordinates.
        /// It generates a torque.
        /// \param worldForce Force expressend in world coordinates
        /// \param worldPos Point position expressed in world coordinates
        /// \param fc Frame convention
        void SetForceInWorldAtPointInWorld(const Force& worldForce, const Position& worldPos, FRAME_CONVENTION fc);

        /// Set the force at COG expressed in body coordinates. It does not generate a torque.
        /// \param bodyForce Force expressed in body coordinates
        /// \param fc Frame convention
        void SetForceInBody(const Force& bodyForce, FRAME_CONVENTION fc);

        /// Set the force expressed in body coordinates applying to a point expressed in body coordinates.
        /// It generates a torque.
        /// \param bodyForce Force expressed in body coordinates
        /// \param bodyPos Point position expressed in body coordinates
        /// \param fc Frame convention
        void SetForceInBodyAtPointInBody(const Force& bodyForce, const Position& bodyPos, FRAME_CONVENTION fc);

        /// Set the force expressed in body coordinates applying to a point expressed in world coordinates.
        /// It generates a torque.
        /// \param bodyForce Force expressed in body coordinates
        /// \param worldPos Point position expressed in world coordinates
        /// \param fc Frame convention
        void SetForceInBodyAtPointInWorld(const Force& bodyForce, const Position& worldPos, FRAME_CONVENTION fc);

        /// Set the torque expressed in world coordinates and at COG.
        /// \param worldTorque Torque expressed in world coordinates
        /// \param fc Frame convention
        void SetTorqueInWorldAtCOG(const Torque& worldTorque, FRAME_CONVENTION fc);

        /// Set the torque expressed in body coordinates and at COG.
        /// \param bodyTorque Torque expressed in body coordinates
        /// \param fc Frame convention
        void SetTorqueInBodyAtCOG(const Torque& bodyTorque, FRAME_CONVENTION fc);

        /// Set force and torque expressed in world coordinates and at COG.
        /// \param worldForce Force expressed in world coordinates
        /// \param worldTorque Torque expressed in world coordinates
        /// \param fc Frame convention
        void SetForceTorqueInWorldAtCOG(const Force& worldForce, const Torque& worldTorque, FRAME_CONVENTION fc);

        /// Set force and torque expressed in body coordinates and at COG
        /// \param bodyForce Force expressed in body coordinates
        /// \param bodyTorque Torque expressed in body coordinates
        /// \param fc Frame convention
        void SetForceTorqueInBodyAtCOG(const Force& bodyForce, const Torque& bodyTorque, FRAME_CONVENTION fc);

        /// Set force and torque expressed in world coordinates and reduced to a point expressed in body coordinates
        /// \param worldForce Force expressed in world coordinates
        /// \param worldTorque Force expressed in body coordinates
        /// \param bodyPos Point position expressed in body coordinates
        /// \param fc Frame convention
        void SetForceTorqueInWorldAtPointInBody(const Force &worldForce, const Torque &worldTorque,
                                                const Position &bodyPos, FRAME_CONVENTION fc);

        /// Set force and torque expressed in world coordinates and reduced to a point expressed in world coordinates
        /// \param worldForce  Force expressed in world coordinates
        /// \param worldTorque Force expressed in body coordinates
        /// \param worldPoint Point position expressed in world coordinates
        /// \param fc Frame convention
        void SetForceTorqueInWorldAtPointInWorld(const Force &worldForce, const Torque &worldTorque,
                                                 const Position &worldPoint, FRAME_CONVENTION fc);

        /// Set force and torque expressed in body coordinates and reduced to a point expressed in body coordinates
        /// \param bodyForce Force expressed in body coordinates
        /// \param bodyTorque Torque expressed in body coordinates
        /// \param bodyPos Point position expressed in body coordinates
        /// \param fc Frame convention
        void SetForceTorqueInBodyAtPointInBody(const Force &bodyForce, const Torque &bodyTorque,
                                               const Position &bodyPos, FRAME_CONVENTION fc);

        /// Set force and torque expressed in body coordinates and reduced to a point expressed in world coordinates
        /// \param bodyForce Force expressed in body coordinates
        /// \param bodyTorque Torque expressed in body coordinates
        /// \param worldPos Point position expressed in world coordinates
        /// \param fc Frame convention
        void SetForceTorqueInBodyAtPointInWorld(const Force &bodyForce, const Torque &bodyTorque,
                                                const Position &worldPos, FRAME_CONVENTION fc);

        /// Return the force as a chrono object.
        /// \return Force vector as a chrono object
        std::shared_ptr<chrono::ChForce> GetChronoForce();


        friend class FrBody;

    };

}  // end namespace frydom

#endif //FRYDOM_FRFORCE_H
