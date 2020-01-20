// ==========================================================================
// FRyDoM - frydom-ce.org
//
// Copyright (c) Ecole Centrale de Nantes (LHEEA lab.) and D-ICE Engineering.
// All rights reserved.
//
// Use of this source code is governed by a GPLv3 license that can be found
// in the LICENSE file of FRyDoM.
//
// ==========================================================================

#ifndef FRYDOM_FRHYDROSTATICEQUILIBRIUM_H
#define FRYDOM_FRHYDROSTATICEQUILIBRIUM_H

#include <memory>
#include "MathUtils/Constants.h"
#include "frydom/core/common/FrFrame.h"
//#include <frydom/mesh/FrHydrostaticsProperties.h>

namespace frydom {

  // Forward declarations
  class FrBody;

  class FrHydroMesh;

  class FrHydroStaticEquilibrium {

   public:

    FrHydroStaticEquilibrium(std::shared_ptr<FrBody> body,
                             const std::string &meshFile,
                             FrFrame meshOffset);

    ~FrHydroStaticEquilibrium();

    bool Solve(double mass, const Position &COGPosInBody);

    FrHydroMesh *GetHydroMesh() const;

    std::string GetReport(const Position &COGPosInBody, const Position &refPosInBody, FRAME_CONVENTION fc) const;

    void SetMaxIterations(unsigned int max_iterations);

    unsigned int GetMaxIterations() const;

    void SetRelativeTolerance(double tolerance);

    double GetRelativeTOlerance() const;

    void SetLinearRelaxation(double relaxation);

    void SetAngularRelaxation(double relaxation);

    void SetRelaxation(Vector3d<double> &relaxation);

    Vector3d<double> GetRelaxation() const;

    FrFrame GetEquilibriumFrame() const;

   private:
    std::shared_ptr<FrBody> m_body;
    std::shared_ptr<FrHydroMesh> m_hydroMesh;
//    std::unique_ptr<FrHydrostaticsProperties> m_properties;

    Vector3d<double> m_residual;
    Vector3d<double> m_solution;
//    Vector3d<double> c_scale;


    unsigned int m_iteration = 0;
    unsigned int m_iterations_max = 100;
    double m_relative_tolerance = 1e-3;

    Vector3d<double> m_relax;

  };

  FrHydroStaticEquilibrium
  solve_hydrostatic_equilibrium(const std::shared_ptr<FrBody> &body,
                                const std::string &meshFile,
                                FrFrame meshOffset,
                                double mass, const Position &COGPosInBody);


} //end namespace frydom


#endif //FRYDOM_FRHYDROSTATICEQUILIBRIUM_H
