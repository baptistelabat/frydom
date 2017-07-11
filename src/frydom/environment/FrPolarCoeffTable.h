//
// Created by frongere on 03/07/17.
//

#ifndef FRYDOM_FRPOLARCOEFFTABLE_H
#define FRYDOM_FRPOLARCOEFFTABLE_H


#include <memory>
#include <vector>
#include <iostream>
#include "yaml-cpp/yaml.h"

#include "frydom/misc/FrInterp1d.h"
#include "frydom/core/FrConstants.h"

namespace frydom {
namespace environment {


    template <class Real=double>
    class FrPolarCoeffTable {

    private:
//        std::shared_ptr<Real> angles;

        Interp1dMethod m_interp_method;

//        std::unique_ptr<FrInterp1d> Cx_i;
//        std::unique_ptr<FrInterp1d<Real>> Cx_i;
//        FrInterp1d* Cy_i;
//        FrInterp1d* Cpsi_i;

    public:
        FrPolarCoeffTable() : m_interp_method(LINEAR) {};

        ~FrPolarCoeffTable() {};

        Interp1dMethod GetInterpolationMethod() { return m_interp_method;}

        void SetInterpolationMethod(Interp1dMethod interp_method) {
            m_interp_method = interp_method;
        }

        /// To Initialize the table (may be overloaded following the way we load coefficients)
        void Initialize(const std::vector<Real>& angles,
                        const std::vector<Real>& cx,
                        const std::vector<Real>& cy,
                        const std::vector<Real>& cz) {



            const std::vector<Real>* angles_ptr(&angles);
            const std::vector<Real>* cx_ptr(&cx);
            const std::vector<Real>* cy_ptr(&cy);
            const std::vector<Real>* cz_ptr(&cz);

            // Building shared angle
            auto angles_shared = std::shared_ptr<const std::vector<Real>>(angles_ptr);

            // Building interpolators
            auto cx_unique = std::unique_ptr<const std::vector<Real>>(cx_ptr);
            auto interp_cx = FrInterp1d<Real>::MakeInterp1d(m_interp_method);
            interp_cx->Initialize(angles_shared, std::move(cx_unique));

            auto cy_unique = std::unique_ptr<const std::vector<Real>>(cy_ptr);
            auto interp_cy = FrInterp1d<Real>::MakeInterp1d(m_interp_method);
            interp_cy->Initialize(angles_shared, std::move(cy_unique));

            auto cz_unique = std::unique_ptr<const std::vector<Real>>(cz_ptr);
            auto interp_cz = FrInterp1d<Real>::MakeInterp1d(m_interp_method);
            interp_cz->Initialize(angles_shared, std::move(cz_unique));

            auto a = 1;




        };


        /// Get the Cx coefficient given the field incidence
        Real Cx(const Real angle, FrAngleUnit unit=RAD) { return 0.; };

        /// Get the Cy coefficient given the field incidence
        Real Cy(const Real angle, FrAngleUnit unit=RAD) { return 0.; };

        /// Get the Cpsi coefficient given the field incidence
        Real Cpsi(const Real angle, FrAngleUnit unit=RAD) { return 0.; };

        /// Get Cx, Cy, Cpsi coefficients given de field incidence
        void GetCoeffs(const Real angle, Real &Cx, Real &Cy, Real &Cpsi, FrAngleUnit unit=RAD) {};


    };

    template <class Real=double>
    void LoadPolarCoeffsFromYaml(const std::string yaml_file,
                                 std::vector<Real>& angles,
                                 std::vector<Real>& cx,
                                 std::vector<Real>& cy,
                                 std::vector<Real>& cz) {

        YAML::Node data = YAML::LoadFile(yaml_file);  // TODO: throw exception if not found !

        if (data["PolarCurrentCoeffs"]) {
            auto node = data["PolarCurrentCoeffs"];
            // All 4 angles, cx, cy, and cz must be present in the yaml file into the PolarCurrentCoeffs node.

            // Getting angles Node
            try {
                angles = node["angles"].as<std::vector<Real>>();
            } catch (YAML::BadConversion err) {
                // TODO: throw exception
            }

            // Getting cx Node
            try {
                cx = node["cx"].as<std::vector<Real>>();
            } catch (YAML::BadConversion err) {
                // TODO: throw exception
            }

            // Getting cy Node
            try {
                cy = node["cy"].as<std::vector<Real>>();
            } catch (YAML::BadConversion err) {
                // TODO: throw exception
            }

            // Getting cz Node
            try {
                cz = node["cz"].as<std::vector<Real>>();
            } catch (YAML::BadConversion err) {
                // TODO: throw exception
            }

        } else {
            // TODO: trhow an exception if the node is not present

        }
    }

    template <class Real=double>
    FrPolarCoeffTable<Real> MakePolarCoeffTable(const std::vector<Real>& angles,
                                                const std::vector<Real>& cx,
                                                const std::vector<Real>& cy,
                                                const std::vector<Real>& cz) {

//        assert( angles.size() == cx.size() );
//        static_assert( angles.size() == cy.size() );
//        static_assert( angles.size() == cz.size() );

        // FIXME : on ne manipulera les shared et unique qu'ici ??

        FrPolarCoeffTable<Real> table;
        table.Initialize(angles, cx, cy, cz);

        return table;
    }

    template <class Real=double>
    FrPolarCoeffTable<Real> MakePolarCoeffTable(const std::string yaml_file) {

        // Loading the data from yaml file
        std::vector<Real> angles;
        std::vector<Real> cx;
        std::vector<Real> cy;
        std::vector<Real> cz;

        LoadPolarCoeffsFromYaml(yaml_file, angles, cx, cy, cz);

        return MakePolarCoeffTable(angles, cx, cy, cz);
    }






}  // end namespace environment
}  // end namespace frydom


#endif //FRYDOM_FRPOLARCOEFFTABLE_H
