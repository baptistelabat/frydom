//
// Created by frongere on 11/07/17.
//

#ifndef FRYDOM_FRLOOKUPTABLE1D_H
#define FRYDOM_FRLOOKUPTABLE1D_H


#include <memory>
#include <vector>
#include <unordered_map>
#include <iostream>

#include "FrInterp1d.h"


namespace frydom {

    template <class Real=double>
    class FrLookupTable1D {

    private:
        Interp1dMethod interp_method;

        std::shared_ptr<std::vector<Real>> Xcoords;
        std::unordered_map<std::string, unsigned long> assoc;
        std::vector<std::shared_ptr<std::vector<Real>>> Ydata;
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
        Real Eval(std::string name, Real x);

        /// Evaluates the LUT giving the key of the serie and a vector of values
        std::vector<Real> Eval(std::string name, std::vector<Real> xvect);

        /// Evaluates the LUT giving the index of the serie and a value
        Real Eval(unsigned long i, Real x);

        /// Evaluates the LUT giving the index of the serie and a vector of values
        std::vector<Real> Eval(unsigned long i, std::vector<Real> xvect);

        /// Evaluates the LUT giving a value
        template <class T>
        std::unordered_map<std::string, T> Eval(T x);

//        /// Evaluates the LUT giving a vector of values
//        std::unordered_map<std::string, std::vector<Real>> Eval(std::vector<Real> xvect);

    private:
        /// Get the index of the series from its name
        inline unsigned long GetIndex(std::string name);
    };

    template <class Real>
    void FrLookupTable1D<Real>::SetInterpolationMethod(Interp1dMethod method) {
        // 2 cas: si on a deja des donnees dans Ydata (on reinitialise tous les interpolateurs)

        if (GetNbSeries() > 0) {
            // TODO: Reinitialiser tous les interpolateurs existants pour les adapter a la nouvelle methode
            std::cout << "TODO: reinitialiser tous les interpolateurs" << std::endl;
        }
        interp_method = method;
    }


    template <class Real>
    void FrLookupTable1D<Real>::SetX(const std::vector<Real> X) {
        Xcoords = std::make_shared<std::vector<Real>>(X);
    }

    template <class Real>
    bool FrLookupTable1D<Real>::AddY(std::string name, std::vector<Real> Y) {
        // TODO: verifier qu'on a le meme nombre d'elt que dans X...
        // Get the future position of the new Serie
        auto i = GetNbSeries();

        // Trying to insert the new name/index into the association map
        auto res_pair = assoc.insert(std::pair<std::string, unsigned long>(name, i));

        if (!res_pair.second) { // Name already exists into the map
            std::cout << "Data have not been added to the LUT" << std::endl;

        } else {
            // We can add data
            auto Y_shared = std::make_shared<std::vector<Real>>(Y);
            Ydata.push_back(Y_shared);

            // Building the interpolator based on the global interpolation method of the LUT
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

    template <class Real>
    Real FrLookupTable1D<Real>::Eval(std::string name, Real x) {
        return interpolators.at(GetIndex(name))->Eval(x);
    }

    template <class Real>
    std::vector<Real> FrLookupTable1D<Real>::Eval(std::string name, std::vector<Real> xvect) {
        return interpolators.at(GetIndex(name))->Eval(xvect);
    }

    template <class Real>
    Real FrLookupTable1D<Real>::Eval(unsigned long i, Real x) {
        return interpolators.at(i)->Eval(x);
    }

    template <class Real>
    std::vector<Real> FrLookupTable1D<Real>::Eval(unsigned long i, std::vector<Real> xvect) {
        return interpolators.at(i)->Eval(xvect);
    }

    template <class Real>
    template <class T>
    std::unordered_map<std::string, T> FrLookupTable1D<Real>::Eval(T x) {

        std::unordered_map<std::string, T> out;
        out.reserve(GetNbSeries());

        std::string name;
        unsigned long idx;

        for (auto elt : assoc) {
            name = elt.first;
            idx = elt.second;

            out[name] = interpolators[idx]->Eval(x);

//            std::cout << "coucou" << std::endl;
        }
        return out;

    }

    template <class Real>
    unsigned long FrLookupTable1D<Real>::GetIndex(std::string name) {
        return assoc.at(name);
    }


}


#endif //FRYDOM_FRLOOKUPTABLE1D_H
