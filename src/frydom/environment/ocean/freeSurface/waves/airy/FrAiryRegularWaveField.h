//
// Created by Lucas Letournel on 21/11/18.
//

#ifndef FRYDOM_FRAIRYREGULARWAVEFIELD_H
#define FRYDOM_FRAIRYREGULARWAVEFIELD_H

#include "frydom/environment/ocean/freeSurface/waves/FrWaveField.h"


namespace frydom {


    class FrAiryRegularWaveField : public FrWaveField_ {
    private:
        /// Wave Height
        double m_height = 0.;
        /// Wave Period
        double m_period = 0.;
        /// Wave Frequency
        double m_omega = 0;
        /// Wave Number
        double m_k = 0.;
        /// Wave direction
        double m_dirAngle = 0.; // used internally with the conventions : NWU, GOTO, and unit : RAD; [0,2PI]
        Direction m_direction = NORTH(NWU);

    public:


        void SetWaveHeight(double height);

        void SetWavePeriod(double period, FREQUENCY_UNIT unit = S);

        void SetDirection(double dirAngle, ANGLE_UNIT unit, FRAME_CONVENTION fc, DIRECTION_CONVENTION dc);

        void SetDirection(Direction direction, FRAME_CONVENTION fc, DIRECTION_CONVENTION dc);

        double GetWaveHeight() const;;

        double GetWavePeriod(FREQUENCY_UNIT unit = S) const;;

        double GetDirectionAngle(ANGLE_UNIT unit, FRAME_CONVENTION fc, DIRECTION_CONVENTION dc) const;;

        Direction GetDirection(FRAME_CONVENTION fc, DIRECTION_CONVENTION dc) const;;


        void Update(double time) final;

        double GetElevation(double x, double y) const final;

        Complex GetComplexElevation(double x, double y) const;

        /// Return the eulerian fluid particule velocity in global reference frame (implemented in child)
        Velocity GetVelocity(double x, double y, double z) const final;

        /// Return the eulerian fluid particule acceleration in global reference frame (implemented in child)
        Acceleration GetAcceleration(double x, double y, double z) const final;

        std::vector<std::vector<double>> GetElevation(const std::vector<double>& xVect,
                                                              const std::vector<double>& yVect) const final;


        std::vector<std::vector<std::vector<chrono::ChVector<double>>>> GetVelocityGrid(const std::vector<double>& xvect,
                                                                                        const std::vector<double>& yvect,
                                                                                        const std::vector<double>& zvect) const final;

    };
}

#endif //FRYDOM_FRAIRYREGULARWAVEFIELD_H
