//
// Created by frongere on 09/10/17.
//

#ifndef FRYDOM_FRTIDALMODEL_H
#define FRYDOM_FRTIDALMODEL_H

#include <cassert>

#include "MathUtils/MathUtils.h"

#include "frydom/core/FrObject.h"

using namespace mathutils;
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

        void Check();

    public:
        FrUTCTime() : m_hours(0), m_minutes(0), m_seconds(0) {}

        FrUTCTime(const unsigned int hours, const unsigned int minutes, const unsigned int seconds);

        FrUTCTime(const unsigned int hours, const unsigned int minutes);

        unsigned int GetH() const { return m_hours; }
        unsigned int GetM() const { return m_minutes; }
        unsigned int GetS() const { return m_seconds; }

        double GetHours() const;

        double GetMinutes() const;

        double GetSeconds() const;

        // TODO: ajouter operateurs de comparaison !!

    };


    class FrTidal : public FrObject {

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

        LookupTable1D<double, double> tidalTable;

        void BuildTable();

        void BuildTwelfthRuleTable();

    public:

        FrTidal();

        ~FrTidal();

        FrTidal(const FrUTCTime t1, const double h1, TidalLevel level1, const FrUTCTime t2, const double h2, TidalLevel level2);

        void Update(const double time);

        const double GetWaterHeight() const;

        const chrono::ChFrame<double>* GetTidalFrame() const;

        virtual void Initialize() override;

        virtual void StepFinalize() override;

    };







    /// REFACTORING --------------->>>>>>>>>>>>>>>

    class FrFreeSurface_;


    class FrTidal_ : public FrObject {

        enum TidalLevel {
            LOW,
            HIGH
        };

        enum TidalMode {
            NO_TIDAL,
            TWELFTH_RULE
        };

    private:

        FrFreeSurface_* m_freeSurface;

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

        LookupTable1D<double, double> tidalTable;

        void BuildTable();

        void BuildTwelfthRuleTable();

    public:

        FrTidal_(FrFreeSurface_* freeSurface);

        FrTidal_(FrFreeSurface_* freeSurface, const FrUTCTime t1, const double h1, TidalLevel level1, const FrUTCTime t2, const double h2, TidalLevel level2);

        ~FrTidal_();

        void Update(const double time);

        const double GetWaterHeight() const;

        const chrono::ChFrame<double>* GetTidalFrame() const;

        void Initialize() override;

        void StepFinalize() override;

    };

}  // end namespace frydom


#endif //FRYDOM_FRTIDALMODEL_H
