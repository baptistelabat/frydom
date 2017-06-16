//
// Created by frongere on 16/06/17.
//

#ifndef FRYDOM_FRPROPELLER_H
#define FRYDOM_FRPROPELLER_H


#include "../core/FrForce.h"


namespace frydom {
    class FrPropeller : public FrForce{

    enum FrRotationalUnit {
        RPM,   ///< Round Per Minute
        Hz,    ///< s**-1
        RAD_S  ///< rad/s
    };

    enum FrEnginePorwerUnit {
        HP,  ///< HorsePower (1HP = 735,499 W)
        W,   ///< Watt
        KW,  ///< Kilo-Watt
        MW   ///< Mega-Watt
    };


      private:
        FrRotationalUnit w_unit;      ///< the unit in which to expressed the angular velocities
        FrEnginePorwerUnit pow_unit;  ///< the unit in which to express the power
        double gear_ratio;            ///< the ratio between the engine angular velocity and the propeller angular velocity
        double engine_torque;         ///< the torque developped on the engine side (P_engine = torque*w_engine) (Nm)
        double propeller_torque;      ///< the torque developped on the shaft by the propeller (P_prop = torque*w_prop) (Nm)
        double gearbox_efficiency;    ///< the efficiency of the gearbox (torque_prop = gearbox_eff * gear_ratio * torque_eng)
        double engine_power;          ///< the power delivered by engine (in the unit fixed by pow_unit)
        double propeller_shaft_power; ///< the powwer delivered by the propeller (in the unit fixed by pow_unit)
        double propeller_diameter;    ///< the properller diameter
        double water_speed;           ///< the speed of water at the level of the propeller (might not be the ship speed...)
        double maximum_engine_torque; ///< the maximum allowed torque developped by the engine (Nm)
        double fuel_rate;             ///< the fuel consumption by the engine (kg/s)
        double max_fuel_rate;         ///< the maximum fuel rate allowed by the engine (kg/s)
        double thrust;                ///< the thrust force developped by the propeller (N)
        double engine_w;              ///< the angular speed of the engine (in unit specified by w_unit)
        double max_engine_w;          ///< the maximum angular speed allowed by the engine (in unit specified by w_unit)
        double propeller_w;           ///< the propeller angular speed (in unit specified by w_unit)

        double wake_fraction;         ///< The wake fraction due to the presence of the hull.
        double Kt;                    ///< The adimensional thrust coefficient
        double Kq;                    ///< The adimensional torque coefficient

    public:
        /// Get the propeller thrust force (in N)
        double getThrust() const {
            return thrust;
        }

        /// Set the propeller thrust force (in N)
        void setThrust(double thrust) {
            FrPropeller::thrust = thrust;
        }

        /// Get the engine angular rotational speed (in Hz)
        double getEngine_w() const {
            return engine_w;
        }

        void setEngine_w(double engine_w) {
            FrPropeller::engine_w = engine_w;
        }

        double getMax_engine_w() const {
            return max_engine_w;
        }

        void setMax_engine_w(double max_engine_w) {
            FrPropeller::max_engine_w = max_engine_w;
        }

        double getPropeller_w() const {
            return propeller_w;
        }

        void setPropeller_w(double propeller_w) {
            FrPropeller::propeller_w = propeller_w;
        }

        FrEnginePorwerUnit getPow_unit() const {
            return pow_unit;
        }

        void setW_unit(FrEnginePorwerUnit pow_unit) {
            FrPropeller::pow_unit = pow_unit;
        }

        FrRotationalUnit getW_unit() const {
            return w_unit;
        }

        void setW_unit(FrRotationalUnit w_unit) {
            FrPropeller::w_unit = w_unit;
        }

        double getGear_ratio() const {
            return gear_ratio;
        }

        void setGear_ratio(double gear_ratio) {
            FrPropeller::gear_ratio = gear_ratio;
        }

        double getEngine_torque() const {
            return engine_torque;
        }

        void setEngine_torque(double engine_torque) {
            FrPropeller::engine_torque = engine_torque;
        }

        double getPropeller_torque() const {
            return propeller_torque;
        }

        void setPropeller_torque(double propeller_torque) {
            FrPropeller::propeller_torque = propeller_torque;
        }

        double getGearbox_efficiency() const {
            return gearbox_efficiency;
        }

        void setGearbox_efficiency(double gearbox_efficiency) {
            FrPropeller::gearbox_efficiency = gearbox_efficiency;
        }

        double getEngine_power() const {
            return engine_power;
        }

        void setEngine_power(double engine_power) {
            FrPropeller::engine_power = engine_power;
        }

        double getPropeller_shaft_power() const {
            return propeller_shaft_power;
        }

        void setPropeller_shaft_power(double propeller_shaft_power) {
            FrPropeller::propeller_shaft_power = propeller_shaft_power;
        }

        double getPropeller_diameter() const {
            return propeller_diameter;
        }

        void setPropeller_diameter(double propeller_diameter) {
            FrPropeller::propeller_diameter = propeller_diameter;
        }

        double getWater_speed() const {
            return water_speed;  // TODO: avoir un getVesselSpeed...
        }

        void setWater_speed(double water_speed) {
            FrPropeller::water_speed = water_speed;
        }

        double getMaximum_engine_torque() const {
            return maximum_engine_torque;
        }

        void setMaximum_engine_torque(double maximum_engine_torque) {
            FrPropeller::maximum_engine_torque = maximum_engine_torque;
        }

        double getFuel_rate() const {
            return fuel_rate;
        }

        void setFuel_rate(double fuel_rate) {
            FrPropeller::fuel_rate = fuel_rate;
        }

        double getMax_fuel_rate() const {
            return max_fuel_rate;
    }

        void setMax_fuel_rate(double max_fuel_rate) {
            FrPropeller::max_fuel_rate = max_fuel_rate;
        }

        double getWake_fraction() const {
            return wake_fraction;
        }

        void setWake_fraction(double wake_fraction) {
            FrPropeller::wake_fraction = wake_fraction;
        }


      public:
        FrPropeller()
                : w_unit(RPM),
        gear_ratio(1.),
        engine_torque(0.),
        propeller_torque(0.),
        gearbox_efficiency(1.),
        engine_power(1e6),

        {
        };
        ~FrPropeller();

        double getAdvanceRatio() {
            return
        };

        void UpdateState();






    };
}  // End namespace frydom

#endif //FRYDOM_FRPROPELLER_H
