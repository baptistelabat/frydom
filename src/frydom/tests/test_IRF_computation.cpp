//
// Created by frongere on 05/01/18.
//

#include "frydom/frydom.h"
#include "matplotlibcpp.h"


using namespace frydom;

int main(int argc, char* argv[]) {

    // Loading a HDB
    FrHydroDB HDB = LoadHDB5("frydom_hdb.h5");

    // Retrieving frequency data
    auto wmin = HDB.GetMinFrequency();
    auto wmax = HDB.GetMaxFrequency();
    auto nw = HDB.GetNbFrequencies();
    auto dw = (wmax-wmin) / (nw-1);
    auto df = dw / MU_2PI;

    // This lead to the period time of the time domain signal (IRF)
    auto T = 1. / df;


    // Optimum sampling frequency
    auto fs = 2 * MU_2PI*wmax;
    auto dt = 1./fs;


    HDB.GenerateImpulseResponseFunctions(T, dt); // Travailler dessus !!!! (T est en fait lie a dw ...

    auto time = arange<double>(0., T, dt); // c'est ce qui est fait dans GeerateImpulseResonse(), meilleur moyen ???

    auto k22 = HDB.GetBody(0)->GetImpulseResponseFunction(0, 2, 2);

    // Ploting
    std::vector<double> tmp;
    for (unsigned int i; i<k22.rows(); i++) {
        tmp.push_back(k22[i]);
    }
//    matplotlibcpp::plot(time, tmp);
//    matplotlibcpp::show();

    // We show here that the impulse response function that we by this way is periodic and only half of the signal is usefull...


    // On essaie de faire la meme avec une ifft
    // On recupere la reponse frequentielle 22
    auto A22_w = HDB.GetBody(0)->GetAddedMass(0, 2, 2);
    auto B22_w = HDB.GetBody(0)->GetRadiationDamping(0, 2, 2);
    auto A22inf = HDB.GetBody(0)->GetInfiniteAddedMass(0)(2, 2);
    auto omega_vec = HDB.GetFrequencies();

    Eigen::VectorXd omega_eig = Eigen::Map<Eigen::VectorXd>(omega_vec.data(), nw);

    // Using a B-spline to reinterpolate
    mathutils::Spline<double, 1, 3> A22_w_spl(omega_eig, A22_w);  // Renommer en MUSpline -> ambiguite avec eigen spline
    mathutils::Spline<double, 1, 3> B22_w_spl(omega_eig, B22_w);

    auto nfft = (uint)pow(2,5);

    auto omega_interp = linspace<double>(0, wmax, (uint)nfft);
    auto dw_interp = omega_interp[1] - omega_interp[0];


    std::vector<double> A22_interp((uint)nfft), B22_interp((uint)nfft);

    for (uint i=0; i < nfft; i++) {
        A22_interp[i] = A22_w_spl(omega_interp[i]);
        B22_interp[i] = B22_w_spl(omega_interp[i]);
    }


//    matplotlibcpp::plot(omega_interp, A22_interp);
//    matplotlibcpp::plot(omega_interp, B22_interp);
//    matplotlibcpp::show();


    std::vector<std::complex<double>> K22_w(nfft+1);
    K22_w[0] = 0.; // We add the fundamental which must be 0 for the resulting signal
    for (uint i=1; i<nfft; i++) {
        K22_w[i] = B22_interp[i] + MU_JJ*omega_interp[i] * (A22_interp[i] - A22inf);
    }


    auto fft = mathutils::FFT<double>();
    fft.HalfSpectrumON();
    fft.ScalingON();

    std::vector<double> K_fft;
    fft.ifft(K_fft, K22_w);

    auto time_interp = linspace<double>(0., MU_2PI/dw_interp, 2*nfft);

    matplotlibcpp::plot(time_interp, K_fft);
    matplotlibcpp::plot(time, tmp);
    matplotlibcpp::show();


    return 0;
}