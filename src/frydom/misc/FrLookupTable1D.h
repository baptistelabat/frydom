//
// Created by frongere on 11/07/17.
//

#ifndef FRYDOM_FRLOOKUPTABLE1D_H
#define FRYDOM_FRLOOKUPTABLE1D_H


#include <memory>
#include <vector>
#include <map>
#include <iostream>

#include "FrInterp1d.h"


namespace frydom {


    template <class Real=double>
    class FrLookupTable1D {

    private:
        Interp1dMethod interp_method;

        std::shared_ptr<std::vector<Real>> Xcoords;
        std::map<std::string, unsigned long> assoc;
        std::vector<std::shared_ptr<std::vector<Real>>> Ydata;  // TODO : tenir une map a part et donner la possibilite de manipulaer du non nomme
        std::vector<std::unique_ptr<FrInterp1d<Real>>> interpolators;

    public:
        FrLookupTable1D() : interp_method(LINEAR) {};
        ~FrLookupTable1D() {};

        /// Set interpolation method
        void SetInterpolationMethod(Interp1dMethod method);

        /// Set the X vector of the lookup table
        void SetX(const std::vector<Real> X);

        std::vector<Real> GetX() const { return *Xcoords.get(); }

        /// Get the number of series
        unsigned long GetNbSeries() const { return Ydata.size(); }

        /// Get the number of samples
        unsigned long GetNbSample() const { return Xcoords->size(); }

        /// Add a Serie to the LUT
        bool AddY(std::string name, std::vector<Real> Y);

        /// Evaluates the LUT giving the key of the serie and a value
        Real Eval(std::string, Real x);

        /// Evaluates the LUT giving the key of the serie and a vector of values
        std::vector<Real> Eval(std::string, std::vector<Real> xvect);

        /// Evaluates the LUT giving the index of the serie and a value
        Real Eval(unsigned long i, Real x);

        /// Evaluates the LUT giving the index of the serie and a vector of values
        std::vector<Real> Eval(unsigned long i, std::vector<Real> xvect);

        /// Evaluates the LUT giving a value
        std::map<std::string, Real> Eval(Real x);

        /// Evaluates the LUT giving a vector of values
        std::map<std::string, std::vector<Real>> Eval(std::vector<Real> xvect);

    };

    template <class Real>
    void FrLookupTable1D<Real>::SetInterpolationMethod(Interp1dMethod method) {
        // 2 cas: si on a deja des donnees dans Ydata (on reinitialise tous les interpolateurs)

        if (GetNbSeries() > 0) {
            // TODO: reinitialiser tous les interpolateurs
        }
        interp_method = method;
    }


    template <class Real>
    void FrLookupTable1D<Real>::SetX(const std::vector<Real> X) {
        Xcoords = std::make_shared<std::vector<Real>>(X);
    }

    template <class Real>
    bool FrLookupTable1D<Real>::AddY(std::string name, std::vector<Real> Y) {

        // Get the position of the new Serie
        auto i = GetNbSeries();

        auto res_pair = assoc.insert(std::pair<std::string, unsigned long>(name, i));

        if (!res_pair.second) {
            std::cout << "Data have not been added to the LUT" << std::endl;

        } else {
            // We can add data
            auto Y_shared = std::make_shared<std::vector<Real>>(Y);
            Ydata.push_back(Y_shared);

            // Building the interpolator based on the interpolation method of the LUT
            auto interp_ptr = FrInterp1d<Real>::MakeInterp1d(interp_method);
            // FIXME: ICI, on ne recupere pas comme voulu un pointeur vers un objet FrInterp1dLinear
            // Du coup, la methode Initialize appelee apres n'est que celle de

            // Initializing the interpolator
            interp_ptr->Initialize(Xcoords, Y_shared);

            auto interp_unique = std::unique_ptr<FrInterp1d<Real>>(interp_ptr);

            interpolators.push_back(std::move(interp_unique));
        }

        return res_pair.second;
    }




}


#endif //FRYDOM_FRLOOKUPTABLE1D_H
