//
// Created by camille on 13/11/18.
//

#ifndef FRYDOM_FRFLOWBASE_H
#define FRYDOM_FRFLOWBASE_H

#include "frydom/core/FrObject.h"
#include "frydom/core/FrVector.h"
#include "frydom/environment/field/FrFieldBase.h"
#include "frydom/environment/field/FrUniformField.h"

namespace frydom {

    // Forward declarations
    class FrEnvironment_;
    class FrFrame_;

    enum FIELD {
        UNIFORM
    };

    ///
    /// FrFlowBase : Base flow field
    ///

    class FrFlowBase : public FrObject {

    private:
        FrEnvironment_* m_environment;              ///< Link to the environment variable
        std::unique_ptr<FrFieldBase> m_field;        ///< Flow field model

    public:

        /// Default constructor
        explicit FrFlowBase(FrEnvironment_* environment);

        /// Destructor of the flow base object
        ~FrFlowBase() = default;

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
        template <class T>
        void NewField();

        void MakeFieldUniform();

        /// Return the field model of the flow
        /// \tparam Field Field model
        /// \return Field model pointer
        template <class T=FrUniformField>
        T* GetField() const;

        FrUniformField* GetFieldUniform() const;

        /// Method of initialization of the flow base model
        void Initialize() override;

        /// Update the state of the flow base object
        /// \param time Current time of the simulation
        virtual void Update(double time);

        //// Method to be applied at the end of each time step
        void StepFinalize() override;

    };


    class FrCurrent_ : public FrFlowBase {

    public:
        FrCurrent_(FrEnvironment_* environment) : FrFlowBase(environment) {}

    };




    class FrWind_ : public FrFlowBase {

    public:
        FrWind_(FrEnvironment_* environment) : FrFlowBase(environment) {}

    };

} // end of namespace frydom

#endif //FRYDOM_FRFLOWBASE_H
