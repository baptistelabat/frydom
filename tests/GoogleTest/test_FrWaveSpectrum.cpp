//
// Created by Lucas Letournel on 14/12/18.
//

#include "frydom/frydom.h"

#include "gtest/gtest.h"

using namespace frydom;

TEST(FrWaveSpectrum,FrCos2sDirectionalModel){

    
    double thetaMean = M_PI;
    std::vector<double> thetaVect; thetaVect.clear();
    thetaVect = linspace(0.,2.*M_PI,21);
//    thetaVect.push_back(0.1);


    // test FrCos2sDirectionalModel_
    FrCos2sDirectionalModel_ Cos2sDirModel;

    auto Cos2SDirfunc = Cos2sDirModel.GetSpreadingFunction(thetaVect, thetaMean);

    double s = Cos2sDirModel.GetSpreadingFactor();
    double two_s = 2. * s;
    double c_s = ( pow(2., two_s - 1.) / M_PI ) * pow(std::tgamma(s + 1.), 2.) / std::tgamma(two_s + 1.);

    double spreading_fcn;
    for (int iv=0; iv<thetaVect.size(); ++iv) {
        spreading_fcn = c_s * pow(cos(0.5 * (thetaVect[iv] - thetaMean)), two_s);
        EXPECT_NEAR(spreading_fcn, Cos2SDirfunc[iv], 1.E-8);
    }

}

TEST(FrWaveSpectrum,BandWidth){



}



TEST(FrWaveSpectrum,Jonswap){
    double Hs = 3., Tp = S2RADS(9.);
    double Gamma = 3.3;

    std::vector<double> wVect; wVect.clear();
    wVect = linspace(0.5,2.,21);

    FrJonswapWaveSpectrum_ JonswapSpectrum(Hs,Tp,RADS,Gamma);

    auto waveAmp = JonswapSpectrum.GetWaveAmplitudes(wVect);

    double FromPython[21] = {0.1350065 , 0.26988954, 0.47368477, 0.57775458, 0.36633141,
            0.28137033, 0.24320673, 0.21093058, 0.18274343, 0.15865051,
            0.138254  , 0.12103303, 0.10647771, 0.09413711, 0.08363016,
            0.07464189, 0.06691458, 0.06023814, 0.05444138, 0.04938449,
            0.04495295};

    for (int i=0;i<22;++i) {
        EXPECT_NEAR(FromPython[i],waveAmp[i],1.E-8);
    }

    double wmin,wmax;
    JonswapSpectrum.GetFrequencyBandwidth(wmin,wmax);

    double wminFromPython = 0.431574732929136, wmaxFromPython = 2.38620060411016;

    EXPECT_NEAR(wminFromPython,wmin,1.E-8);
    EXPECT_NEAR(wmaxFromPython,wmax,1.E-8);
}

   
