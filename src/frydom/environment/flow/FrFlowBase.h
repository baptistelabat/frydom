//
// Created by camille on 13/11/18.
//

#ifndef FRYDOM_FRFLOWBASE_H
#define FRYDOM_FRFLOWBASE_H

#include "frydom/core/FrObject.h"
#include "frydom/core/FrVector.h"
#include "FrFieldBase.h"
#include "FrUniformField.h"

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
        std::unique_ptr<FrFieldBase> m_field;        ///< Flow field model
    protected:
        double c_ramp=1.;   ///> cache value of the time ramp applied on the flow field
    public:

        /// Default constructor
        explicit FrFlowBase();

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

        /// Create a new templated field
        /// \tparam Field templated field
        template <class T>
        void NewField();

        /// Create a uniform field
        void MakeFieldUniform();

        /// Return the field model of the flow
        /// \tparam Field Field model
        /// \return Field model pointer
        template <class T=FrUniformField>
        T* GetField() const;

        /// Get the field as a uniform field
        /// \return uniform field
        FrUniformField* GetFieldUniform() const;

        /// Method of initialization of the flow base model
        void Initialize() override;

        /// Update the state of the flow base object
        /// \param time Current time of the simulation
        virtual void Update(double time);

        //// Method to be applied at the end of each time step
        void StepFinalize() override;

    };


    //Forward Declaration
    class FrOcean_;
    class FrAtmosphere_;

    class FrCurrent_ : public FrFlowBase {
    private:

        FrOcean_* m_ocean;  ///> Pointer to the ocean containing this current model

    public:
        /// Default constructor
        /// \param ocean ocean containing this current model
        explicit FrCurrent_(FrOcean_* ocean) : FrFlowBase() { m_ocean = ocean;}

        /// Get the ocean containing this current model
        /// \return ocean containing this current model
        FrOcean_* GetOcean() const {return m_ocean;}

        /// Method to be applied at the end of each time step
        void StepFinalize() override;
    };




    class FrWind_ : public FrFlowBase {
    private:

        FrAtmosphere_* m_atmosphere;  ///> Pointer to the atmosphere containing this wind model

    public:

        /// Default constructor
        /// \param atmosphere containing this wind model
        explicit FrWind_(FrAtmosphere_* atmosphere) : FrFlowBase() { m_atmosphere = atmosphere;}

        /// Get the atmosphere containing this wind model
        /// \return atmosphere containing this wind model
        FrAtmosphere_* GetAtmosphere() const {return m_atmosphere;}

        /// Method to be applied at the end of each time step
        void StepFinalize() override;
    };

} // end of namespace frydom

#endif //FRYDOM_FRFLOWBASE_H
