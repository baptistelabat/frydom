//
// Created by frongere on 06/10/17.
//

#ifndef FRYDOM_FRWAVEFIELD_H
#define FRYDOM_FRWAVEFIELD_H

#include <complex>
#include <random>
#include "FrWaveSpectrum.h"
#include "FrWaveDispersionRelation.h"
#include "FrWaveProbe.h"

#define J std::complex<double>(0, 1)


namespace frydom {

    // =================================================================================================================

    class FrWaveField {  // TODO: ajouter dans cette classe un modele de tidal...

//        enum TYPE {
//            LINEAR_REGULAR,
//            LINEAR_IRREGULAR,
//            LINEAR_IRREGULAR_DIRECTIONNAL
//        };

    protected:
        double m_time = 0.;

        std::vector<std::shared_ptr<FrWaveProbe>> m_waveProbes;

        virtual void Update_ejwt() = 0;

    public:

//        virtual std::complex<double> GetCmplxFreeSurfaceElevation(const double x, const double y) const = 0;

        virtual double GetFreeSurfaceElevation(const double x, const double y) const = 0;

        std::vector<std::vector<double>> GetFreeSurfaceElevationGrid(const std::vector<double>& xVect,
                                                                 const std::vector<double>& yVect) const {
            ulong nx = xVect.size();
            ulong ny = yVect.size();

            std::vector<std::vector<double>> fse;
            fse.reserve(nx);

            std::vector<double> line;
            line.reserve(ny);

            for (double x: xVect) {
                line.clear();
                for (double y: yVect) {
                    line.push_back(GetFreeSurfaceElevation(x, y));
                }
                fse.push_back(line);
            }
            return fse;
        }

//        virtual std::vector<std::vector<double>> GetFreeSurfaceElevation(const std::vector<double> x,
//                                                                         const std::vector<double> y) = 0;
//
//        virtual double GetPressure(const double x, const double y, const double z) = 0;
//
//        virtual chrono::ChVector<double> GetVelocity(const double x, const double y, const double z) = 0;

        double GetTime() const { return m_time; }

        void UpdateTime(const double time) {
            m_time = time;
            Update_ejwt();  // FIXME: ne doit pas apparaitre ici, c'est une methode propre a la houle d'Airy !!

        }

        void UpdateState() {

        }

        void Update(const double time) {
            UpdateTime(time);
            UpdateState();
        }

        virtual std::shared_ptr<FrWaveProbe> NewWaveProbe(double x, double y) = 0;


    };

    // =================================================================================================================

    class FrLinearWaveField : public FrWaveField {

    protected:
        double m_mean_wave_dir=0.;
        std::vector<double> m_wave_directions;
        std::vector<double> m_wave_pulsation;

    public:
        FrLinearWaveField() = default;

        explicit FrLinearWaveField(const double mean_wave_dir) : m_mean_wave_dir(mean_wave_dir*M_DEG) {};

        virtual std::vector<double> GetWaveDirections() const = 0;

        virtual std::vector<double> GetWavePulsations() const = 0;

        virtual std::vector<std::vector<double>> GetWavePhases() const = 0;

        virtual void SetWavePhases(const std::vector<double>& wavePhases) = 0;

//        virtual std::vector<double> GetWaveNumbers() const = 0;
//
//        virtual std::vector<double> GetWaveLengths() const = 0;
//
        virtual std::vector<std::vector<double>> GetWaveAmplitudes() const = 0;

        virtual std::vector<std::complex<double>> GetSteadyElevation(const double x, const double y) const = 0;

    };

    // =================================================================================================================

    class FrRegularLinearWaveField : public FrLinearWaveField {

    private:
        double m_wave_period;
        double m_wave_height;

        double c_wave_number;
        std::complex<double> c_ejwt;


        void Update_ejwt() override {
            c_ejwt = std::exp(-J * GetWavePulsations()[0] * m_time);
        }

        void UpdateWaveNumber() {
            // FIXME : aller chercher water_height et grav dans l'environnement !!!
            // Pour le moment, on hard code
            double water_height = 10.;
            double w = S2RADS(m_wave_period);
            double grav = 9.81;
            c_wave_number = SolveWaveDispersionRelation(water_height, w, grav);
        }


    public:
        // TODO: permettre de specifier des degres pour la direction
        // TODO: ajouter un getter pour mean_wave_dir
        // FIXME: ordre tp, hs non consistant !! (renverser)
        FrRegularLinearWaveField(const double wave_period,
                                 const double wave_height,
                                 const double mean_wave_dir) :  // Wave directions have to be given in degrees !
                m_wave_period(wave_period),
                m_wave_height(wave_height),
                FrLinearWaveField(mean_wave_dir) {

            Update_ejwt();
            UpdateWaveNumber();
        }

        std::vector<double> GetWaveDirections() const override {
            std::vector<double> wave_dir;
            wave_dir.push_back(m_mean_wave_dir);
            return wave_dir;
        }

        std::vector<double> GetWavePulsations() const override {
            std::vector<double> omega;
            omega.push_back(S2RADS(m_wave_period));
            return omega;
        }

        std::vector<std::vector<double>> GetWavePhases() const override {
            std::vector<std::vector<double>> phases;
            std::vector<double> phi;
            phi.push_back(0.);
            phases.push_back(phi);
            return phases;
        }

        void SetWavePhases(const std::vector<double>& wavePhases) override {}

        std::vector<std::vector<double>> GetWaveAmplitudes() const override {
            std::vector<std::vector<double>> amplitudes;
            std::vector<double> wave_ampl;
            wave_ampl.push_back(0.5 * m_wave_height);
            amplitudes.push_back(wave_ampl);
            return amplitudes;
        }

        std::vector<std::complex<double>> GetSteadyElevation(const double x, const double y) const override {

            double w_ = x * cos(m_mean_wave_dir) + y * sin(m_mean_wave_dir);
            std::complex<double> cmplx_elev = 0.5 * m_wave_height * std::exp(J*c_wave_number*w_);
            std::vector<std::complex<double>> steady_elev;
            steady_elev.push_back(cmplx_elev);
            return steady_elev;
        }

        std::complex<double> GetCmplxFreeSurfaceElevation(const double x, const double y) const {
            return GetCmplxFreeSurfaceElevation(GetSteadyElevation(x, y)[0]);
        }

        std::complex<double> GetCmplxFreeSurfaceElevation(std::complex<double> steady_elevation) const {
            return steady_elevation * c_ejwt;
        }

        double GetFreeSurfaceElevation(const double x, const double y) const override {
            auto cmplx_elev = GetCmplxFreeSurfaceElevation(x, y);
            return std::imag(cmplx_elev);
        }

        std::shared_ptr<FrWaveProbe> NewWaveProbe(double x, double y) override {

            auto waveProbe = std::make_shared<FrLinearRegularWaveProbe>(x, y);
            waveProbe->SetWaveField(this);
            waveProbe->Initialize();
            m_waveProbes.push_back(waveProbe);
            return waveProbe;
        }

    };

    // =================================================================================================================

    class FrIrregularLinearWaveField : public FrLinearWaveField {

    protected:
        std::unique_ptr<FrWaveSpectrum> m_wave_spectrum;
        unsigned int m_nb_freq;
        double m_wmin;
        double m_wmax;

        std::vector<double> m_phases;

        std::vector<double> c_omega; // TODO: tenir a jour et utiliser plutot que d'appeler toujours linspace !!

        std::vector<double> c_wave_numbers;
        std::vector<std::complex<double>> c_ejwt;

        void Update_ejwt() override {

            c_ejwt.clear();
            c_ejwt.reserve(m_nb_freq);
            auto omega = GetWavePulsations();
            for (auto w: omega) {
                c_ejwt.push_back(std::exp(-J * w * m_time));
            }

        }

        void UpdateWaveNumber() {
            // FIXME : aller chercher water_height et grav dans l'environnement !!!
            // Pour le moment, on hard code
            double water_height = 10.;
            auto w = GetWavePulsations();
            double grav = 9.81;
            c_wave_numbers = SolveWaveDispersionRelation(water_height, w, grav);
        }

    public:

        FrIrregularLinearWaveField(const unsigned int nw,
                                   const double wmin,
                                   const double wmax,
                                   const double mean_wave_dir,
                                   FrWaveSpectrum* waveSpectrum) :
                m_wave_spectrum(waveSpectrum),
                m_nb_freq(nw),
                m_wmin(wmin),
                m_wmax(wmax),
                FrLinearWaveField(mean_wave_dir) {

            // Random phases generation
            GenerateRandomPhases();

            Update_ejwt();
            // Updating cache for Wave Number
            UpdateWaveNumber();
        }

        virtual void GenerateRandomPhases() {
            std::random_device rd;
            std::mt19937 gen(rd());
            std::uniform_real_distribution<double> dis(0., M_2PI);

            m_phases.clear();
            m_phases.reserve(m_nb_freq);

            for (uint iw=0; iw<m_nb_freq; ++iw) {
                m_phases.push_back(dis(gen));
            }
        }

        std::vector<std::vector<double>> GetWavePhases() const override {
            std::vector<std::vector<double>> phases;
            phases.push_back(m_phases);
            return phases;
        }

        void SetWavePhases(const std::vector<double>& wavePhases) override {
            assert(wavePhases.size() == m_nb_freq);
            // TODO: tester que les valeurs sont bien etalees sur un range 0 2PI
            m_phases = wavePhases;
        }

        FrWaveSpectrum* GetWaveSpectrum() const { return m_wave_spectrum.get(); }

        void SetWaveSpectrum(FrWaveSpectrum* waveSpectrum) {
            m_wave_spectrum.reset(waveSpectrum);
        }

        std::vector<double> GetWaveDirections() const override {
            std::vector<double> wave_dir;
            wave_dir.push_back(m_mean_wave_dir);
            return wave_dir;
        }

        std::vector<double> GetWavePulsations() const override {
            return linspace(m_wmin, m_wmax, m_nb_freq);  // FIXME: mettre le vecteur pulsation en cache !
        }

        void SetReturnPeriod(const double period) {

            auto dw = M_2PI / (period * 3600.);
            auto w_range = m_wmax - m_wmin;
            m_nb_freq = (unsigned int)(w_range / dw) + 1; // FIXME: utiliser un SetNbFreq !! --> MAJ K

            auto dw_new = w_range / m_nb_freq;
            auto true_return_period = M_2PI / dw_new / 3600.;

            // TODO: passer a terme ce message en 0mq
            std::cout << "Frequency discretization of the wave spectrum adjusted to get a wave return period of "
                      << true_return_period
                      << " s"
                      << std::endl;
        }

        double GetReturnPeriod() const {  // TODO: donner la possibilite de fournir une unite de temps (s, m, h)
            auto dw = (m_wmax - m_wmin) / m_nb_freq;
            return M_2PI / dw;
        }

        std::vector<std::vector<double>> GetWaveAmplitudes() const override {
            // FIXME: les amplitudes de vague (sqrt(2*...)) devraient etre calculees plutot dans le wavefield !! rien a voir avec le spectre...
            std::vector<std::vector<double>> waveAmplitudes;
            waveAmplitudes.push_back(m_wave_spectrum->GetWaveAmplitudes(m_nb_freq, m_wmin, m_wmax));
            return waveAmplitudes;
        }

        std::vector<std::complex<double>> GetSteadyElevation(const double x, const double y) const override {
            std::vector<std::complex<double>> steady_elevation;
            steady_elevation.reserve(m_nb_freq);

            double w_ = x * cos(m_mean_wave_dir) + y * sin(m_mean_wave_dir);

            auto wave_ampl = GetWaveAmplitudes()[0];

            std::complex<double> val;
            for (uint iw=0; iw<m_nb_freq; iw++) {
                val = wave_ampl[iw] * std::exp( J * (c_wave_numbers[iw] * w_ + m_phases[iw]) );
                steady_elevation.push_back(val);
            }

            return steady_elevation;
        }

        std::vector<std::complex<double>> GetCmplxFreeSurfaceElevation(const double x, const double y) const {
            auto steady_elev = GetSteadyElevation(x, y);
            return GetCmplxFreeSurfaceElevation(steady_elev);
        }

        std::vector<std::complex<double>>
        GetCmplxFreeSurfaceElevation(std::vector<std::complex<double>>& steady_elevation) const {
            assert(steady_elevation.size() == m_nb_freq);
            std::vector<std::complex<double>> eta_cmplx;
            eta_cmplx.reserve(m_nb_freq);

            for (uint iw=0; iw<m_nb_freq; iw++) {
                eta_cmplx.push_back(steady_elevation[iw] * c_ejwt[iw]);
            }
            return eta_cmplx;
        }

        double GetFreeSurfaceElevation(const double x, const double y) const override {
            auto cmplx_fs_elev = GetCmplxFreeSurfaceElevation(x, y);
            double sum = 0.;
            for (auto component: cmplx_fs_elev) {
                sum += std::imag(component);
            }
            return sum;
        }

        std::shared_ptr<FrWaveProbe> NewWaveProbe(double x, double y) override {
            auto waveProbe = std::make_shared<FrLinearIrregularWaveProbe>(x, y);
            waveProbe->SetWaveField(this);
            waveProbe->Initialize();
            m_waveProbes.push_back(waveProbe);
            return waveProbe;
        }

    };

    // =================================================================================================================

    class FrDirectionalLinearWaveField : public FrIrregularLinearWaveField {

    protected:
        unsigned int m_nb_wave_dir;
        double m_dir_min;
        double m_dir_max;

        std::vector<std::vector<double>> m_phases;

    public:
        FrDirectionalLinearWaveField(const unsigned int nw,
                                     const double wmin,
                                     const double wmax,
                                     const double mean_wave_dir,  // DEG
                                     const unsigned int nb_dir,
                                     const double dir_min,  // DEG
                                     const double dir_max,  // DEG
                                     FrWaveSpectrum* waveSpectrum) :
                m_nb_wave_dir(nb_dir),
                m_dir_min(dir_min * M_DEG),
                m_dir_max(dir_max * M_DEG),
                FrIrregularLinearWaveField(nw, wmin, wmax, mean_wave_dir, waveSpectrum) {

            // Random phases generation
            GenerateRandomPhases();

//            Update_ejwt();  // TODO: voir si utile ... (fait par le constructeur de la classe de base ?
//            UpdateWaveNumber();


        }

        void GenerateRandomPhases() override { // TODO: verifier
            std::random_device rd;
            std::mt19937 gen(rd());
            std::uniform_real_distribution<double> dis(0., M_2PI);

            m_phases.clear();
            m_phases.reserve(m_nb_wave_dir);

            std::vector<double> phases;
            phases.reserve(m_nb_freq);

            for (uint idir=0; idir<m_nb_wave_dir; ++idir) {
                phases.clear();
                for (uint iw=0; iw<m_nb_freq; ++iw) {
                    phases.push_back(dis(gen));
                }
                m_phases.push_back(phases);
            }
        }

        void SetWavePhases(const std::vector<std::vector<double>>& wavePhases) {
            assert(wavePhases.size() == m_nb_wave_dir);
            assert(wavePhases[0].size() == m_nb_freq);
            m_phases = wavePhases;
        }

        std::vector<std::vector<double>> GetWavePhases() const override {
            return m_phases;
        }

        std::vector<double> GetWaveDirections() const override {
            return linspace(m_dir_min, m_dir_max, m_nb_wave_dir);
        }

        std::vector<std::vector<double>> GetWaveAmplitudes() const override {
            return m_wave_spectrum->GetWaveAmplitudes(m_nb_freq, m_wmin, m_wmax, m_nb_wave_dir,
                                                      m_dir_min, m_dir_max, m_mean_wave_dir);
        }

        std::vector<std::complex<double>> GetSteadyElevation(const double x, const double y) const override {
            std::vector<std::complex<double>> steadyElevation;
            steadyElevation.reserve(m_nb_freq);

            std::vector<double> w_;
            w_.reserve(m_nb_wave_dir);

            double val;
            auto wave_dirs = GetWaveDirections();
            for (auto direction: wave_dirs) {
                w_.push_back(x * cos(direction) + y * sin(direction));
            }

            auto waveAmplitudes = GetWaveAmplitudes(); // is ntheta x nw, ie wA[i] est de longueur

            double ki;
            std::complex<double> cval;
            for (unsigned int iw=0; iw<m_nb_freq; ++iw) {
                cval.imag(0.);
                cval.real(0.);

                ki = c_wave_numbers[iw];

                for (unsigned int itheta=0; itheta<m_nb_wave_dir; ++itheta) {

                    cval += waveAmplitudes[itheta][iw] * exp(J * (ki * w_[itheta] + m_phases[itheta][iw]));

                }

                steadyElevation.push_back(cval);

            }

            return steadyElevation;
            // TODO: verifier !!!!
        }



















//        std::shared_ptr<FrWaveProbe> NewWaveProbe(double x, double y) override {
//            // TODO
//        }

    };

    // =================================================================================================================




}  // end namespace frydom


#endif //FRYDOM_FRWAVEFIELD_H
