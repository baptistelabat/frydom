//
// Created by frongere on 09/10/17.
//

#ifndef FRYDOM_FRTIDALMODEL_H
#define FRYDOM_FRTIDALMODEL_H

#include <cassert>
#include <frydom/misc/FrLookupTable1D.h>

// TODO: La hauteur de marée (+ sonde a recuperer de seabed) doivent etre retranscrite sur le corps embarque dans la
// surface libre.


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
            NONE,
            TWELFTH_RULE
        };

    private:
        double m_time = 0.;

        TidalMode m_mode = NONE;

        TidalLevel m_level1;
        FrUTCTime m_t1;
        double m_h1 = 0.;

        TidalLevel m_level2;
        FrUTCTime m_t2;
        double m_h2;

        double c_waterHeight = 0.;

        FrLookupTable1D<double> tidalTable;

        void BuildTable() {

            switch (m_mode) {
                case NONE:
                    return;
                case TWELFTH_RULE:
                    BuildTwelfthRuleTable();
            }

        }

        void BuildTwelfthRuleTable() {
            double h_twelfth = fabs(m_h2 - m_h1) / 12.; // One twelfth

            double t1 = m_t1.GetMinutes();
            double t2 = m_t2.GetMinutes();

            double dt = (t2 - t1) / 6.;

            std::vector<double> timeVect;
            timeVect.reserve(7);
            for (uint it=0; it<7; ++it) {
                timeVect.push_back(t1 + it*dt);  // TODO: verifier que la derniere valeur est egale a t2 !!
            }

            std::vector<double> hVect;
            hVect.reserve(13);

            int op;
            if (m_level1 == LOW) {
                op = 1;
            } else {
                op = -1;
            }

            hVect.push_back(m_h1);  // Last extremum tidal level
            hVect.push_back(m_h1 + op *  1 * h_twelfth); // 1 twelfth
            hVect.push_back(m_h1 + op *  3 * h_twelfth); // 2 twelfth
            hVect.push_back(m_h1 + op *  6 * h_twelfth); // 3 twelfth
            hVect.push_back(m_h1 + op *  9 * h_twelfth); // 3 twelfth
            hVect.push_back(m_h1 + op * 11 * h_twelfth); // 2 twelfth
            hVect.push_back(m_h1 + op * 12 * h_twelfth); // 1 twelfth // TODO: verifier qu'on a h2...

            // Populating the interpolation table
            tidalTable.SetX(timeVect);
            tidalTable.AddY("tidal_height", hVect);

        }

    public:

        FrTidal() {};

        FrTidal(const FrUTCTime t1, const double h1, TidalLevel level1, const FrUTCTime t2, const double h2, TidalLevel level2) :
                m_t1(t1),
                m_h1(h1),
                m_level1(level1),
                m_t2(t2),
                m_h2(h2),
                m_level2(level2),
                m_mode(TWELFTH_RULE) {
            assert(h1 >= 0. && h2 >= 0.);
            assert(level1 != level2);  // Levels have to be different
            assert(t2.GetSeconds() > t1.GetSeconds());  // Ajouter operateur de comparaison de temps

            BuildTable();
        }

        void UpdateState() {
            // TODO
        }

        void UpdateTime(const double time) {
            m_time = time;
        };

        void Update(const double time) {
            UpdateTime(time);
            UpdateState();
        }

        double GetWaterHeight() const {
            if (m_mode == NONE) {
                return m_h1;
            }

            if (m_mode == TWELFTH_RULE) {
                return tidalTable.Eval("tidal_height", m_time);
            }

        }

    };

}  // end namespace frydom


#endif //FRYDOM_FRTIDALMODEL_H
