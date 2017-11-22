//
// Created by frongere on 09/10/17.
//

#ifndef FRYDOM_FRTIDALMODEL_H
#define FRYDOM_FRTIDALMODEL_H

#include <cassert>
#include <frydom/misc/FrLookupTable1D.h>

// TODO: La hauteur de marée (+ sonde a recuperer de seabed) doivent etre retranscrite sur le corps embarque dans la
// surface libre.

namespace chrono {
    template <class Real>
    class ChFrame;
}

namespace frydom {

    // FIXME: utiliser https://github.com/HowardHinnant/date comme sous module !!!

    // TODO: utiliser boost ou une autre lib speciale pour la gestion des geoides
    class FrUTCTime {
    private:
        unsigned int m_hours = 0;
        unsigned int m_minutes = 0;
        unsigned int m_seconds = 0;

        int m_local_correction = 0;

        void Check() {
            assert(m_hours > 0. && m_hours < 24);
            assert(m_minutes > 0. && m_minutes <= 60.);
            assert(m_seconds > 0. && m_seconds <= 60.);
            assert(m_local_correction > -24 && m_local_correction < 24);  // TODO verifier
        }

    public:
        FrUTCTime() : m_hours(0), m_minutes(0), m_seconds(0) {}

        FrUTCTime(const unsigned int hours, const unsigned int minutes, const unsigned int seconds) :
                m_hours(hours),
                m_minutes(minutes),
                m_seconds(seconds) {
            Check();
        }

        FrUTCTime(const unsigned int hours, const unsigned int minutes) :
                m_hours(hours),
                m_minutes(minutes),
                m_seconds(0) {
            Check();
        }

        unsigned int GetH() const { return m_hours; }
        unsigned int GetM() const { return m_minutes; }
        unsigned int GetS() const { return m_seconds; }

        double GetHours() const {
            return (double)m_hours + (double)m_minutes/60. + (double)m_seconds/3600.;
        }

        double GetMinutes() const {
            return (double)m_hours*60 + (double)m_minutes + (double)m_seconds/60.;
        }

        double GetSeconds() const {
            return (double)m_hours*3600 + (double)m_minutes*60 + (double)m_seconds;
        }

        // TODO: ajouter operateurs de comparaison !!

    };


    class FrTidal {

        enum TidalLevel {
            LOW,
            HIGH
        };

        enum TidalMode {
            NO_TIDAL,
            TWELFTH_RULE
        };

    private:

        std::unique_ptr<chrono::ChFrame<double>> m_tidalFrame;

        double m_time = 0.;

        TidalMode m_mode = NO_TIDAL;

        TidalLevel m_level1;
        FrUTCTime m_t1;
        double m_h1 = 0.;

        TidalLevel m_level2;
        FrUTCTime m_t2;
        double m_h2;

//        double c_waterHeight = 0.;

        FrLookupTable1D<double> tidalTable;

        void BuildTable();

        void BuildTwelfthRuleTable();

    public:

        FrTidal() {
            m_tidalFrame = std::make_unique<chrono::ChFrame<double>>();
        };

        FrTidal(const FrUTCTime t1, const double h1, TidalLevel level1, const FrUTCTime t2, const double h2, TidalLevel level2);

        void Update(const double time);

        const double GetWaterHeight() const;

        const chrono::ChFrame<double>* GetTidalFrame() const;

    };

}  // end namespace frydom


#endif //FRYDOM_FRTIDALMODEL_H
