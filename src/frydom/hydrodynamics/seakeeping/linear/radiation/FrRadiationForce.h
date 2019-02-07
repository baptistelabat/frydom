// =============================================================================
// FRyDoM - frydom-ce.gitlab.host.io
//
// Copyright (c) D-ICE Engineering and Ecole Centrale de Nantes (LHEEA lab.)
// All rights reserved.
//
// Use of this source code is governed by a GPLv3 license that can be found
// in the LICENSE file of FRyDOM.
//
// =============================================================================


#ifndef FRYDOM_FRRADIATIONFORCE_H
#define FRYDOM_FRRADIATIONFORCE_H


#include "frydom/core/force/FrForce.h"
#include "FrRadiationModel.h"

namespace frydom {

    // Forward declaration
    class FrRadiationModel;

    // TODO: les forces de radiation doivent pouvoir etre extraites depuis le modele de radiation...
    /**
     * \class FrRadiationForce
     * \brief Class for computing the radiation loads.
     */
    class FrRadiationForce : public FrForce {

        // FIXME : enum a placer dans RadiationModel
//        enum class Type {
//            CONVOLUTION,
//            STATE_SPACE // Not used...
//        };  // TODO : utiliser !!!

    protected:
        std::shared_ptr<FrRadiationModel> m_radiationModel;  // TODO : il faut que le modele de radiation soit en mesure de generer les forces de radiation
        // Une possibilite serait qu'a l'initialisation du modele de radiation, les forces de radiation soient ajoutees automatiquemet aux corps...

    public:
        FrRadiationForce() = default;

        explicit FrRadiationForce(std::shared_ptr<FrRadiationModel> radiationModel);

        void SetRadiationModel(const std::shared_ptr<FrRadiationModel> radiationModel);

        std::shared_ptr<FrRadiationModel> GetRadiationModel() const;

        void SetLogPrefix(std::string prefix_name) override;

    };


    // Forward declaration
    class FrRadiationConvolutionModel;

    /**
     * \class FrRadiationConvolutionForce
     * \brief Class for computing the hydrodynamic damping loads.
     */
    class FrRadiationConvolutionForce : public FrRadiationForce {

    private:


    public:

        explicit FrRadiationConvolutionForce(std::shared_ptr<FrRadiationConvolutionModel> radiationConvolutionModel);

        void Initialize() override;

        void UpdateState() override;

    };












    /// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< REFACTORING

    /**
     * \class FrRadiationForce_
     * \brief Class for computing the radiation loads.
     */
    class FrRadiationForce_ : public FrForce_ {

    protected:

        FrRadiationModel_* m_radiationModel;

    public:

        FrRadiationForce_() = default;

        explicit FrRadiationForce_(FrRadiationModel_* radiationModel);

        void SetRadiationModel(FrRadiationModel_* radiationModel);

        FrRadiationModel_* GetRadiationModel() const;

        void StepFinalize() override { }

    };


    /**
     * \class FrRadiationConvolutionForce_
     * \brief Class for computing the hydrodynamic damping loads.
     */
    class FrRadiationConvolutionForce_: public FrRadiationForce_ {

    public:

        explicit FrRadiationConvolutionForce_(FrRadiationConvolutionModel_* radiationModel);

        void Initialize() override;

        void Update(double time) override;

    };

}  // end namespace frydom

#endif //FRYDOM_FRRADIATIONFORCE_H
