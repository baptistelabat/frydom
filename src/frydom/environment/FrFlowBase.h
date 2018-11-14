//
// Created by camille on 13/11/18.
//

#ifndef FRYDOM_FRFLOWBASE_H
#define FRYDOM_FRFLOWBASE_H

#include "frydom/frydom.h"

namespace frydom {

    // Forward declarations
    class FrEnvironment_;
    class FrFrame_;
    class FrFieldBase;

    ///
    /// FrFlowBase : Base flow field
    ///

    class FrFlowBase : public FrObject {

    private:
        FrEnvironment_* m_environment;              ///< Link to the environment variable
        std::unique_ptr<FrFieldBase> m_field;       ///< Flow field model

    public:

        /// Default constructor
        explicit FrFlowBase(FrEnvironment_* environment): m_environment(environment) {}

        /// Destructor of the flow base object
        ~FrFlowBase_() = default;

        /// Return the flow velocity at a given point in world frame
        /// \param worldPos Position of the Point in world frame
        /// \param fc Frame convention (NED/NWU)
        /// \return Velocity in world frame
        Velocity GetFluxVelocityInWorld(const Position &worldPos, FRAME_CONVENTION fc) const;

        /// Return the flow velocity observed from the local frame
        /// \param frame Local frame in which the velocity is computed
        /// \param worldVel Translation velocity of the frame in world frame
        /// \param fc Frame convention (NED/NWU)
        /// \return Velocity in local frame
        Velocity GetRelativeVelocityInFrame(const FrFrame_ &frame, const Velocity &worldVel, FRAME_CONVENTION fc) const;

        ///
        /// \tparam Field
        template <class Field>
        void NewField();

        /// Return the field model of the flow
        /// \tparam Field Field model
        /// \return Field model pointer
        template <class Field>
        Field* GetField() const;

        /// Method of initialization of the flow base model
        void Initialize() override;

        /// Update the state of the flow base object
        /// \param time Current time of the simulation
        virtual void Update(double time);

        //// Method to be applied at the end of each time step
        void StepFinalize() override;

    };

} // end of namespace frydom

#endif //FRYDOM_FRFLOWBASE_H
