//
// Created by frongere on 14/06/17.
//

#ifndef FRYDOM_FRFORCEASSET_H
#define FRYDOM_FRFORCEASSET_H

#include "chrono/assets/ChGlyphs.h"
#include "chrono/physics/ChBody.h"
#include "FrForce.h"
#include "frydom/asset/FrAsset.h"


// Cet asset doit etre ajoute directement au corps qui contient la force
// On peut faire de telle maniere que quand on ajoute l'asset a la force, c'est au corps de la force qu'il est ajoute...

namespace frydom {

    class FrForceAsset : public chrono::ChGlyphs {

    private:
        std::shared_ptr<FrForce> m_force;  //< The force that this asset represents
        double OrderOfMagnitude;
        bool adaptive_OOM;

        double CharacteristicLength;
        bool inverse_direction;
        // TODO ajouter flag pour dire si on affiche la force qui s'applique ou la force delivree pour la visu (propulseurs...)

    public:
        FrForceAsset(std::shared_ptr<FrForce> myforce)  // TODO: ajouter couleur
                : m_force(myforce){
            // TODO: migrer vers l'implementation dans le cpp

            SetDrawMode(eCh_GlyphType::GLYPH_VECTOR);

            auto point = myforce->GetVrelpoint();
            auto forcevect = myforce->GetRelForce();
            SetGlyphVector(0, point, forcevect);
            SetGlyphsSize(20);  // Ne semble pas faire de difference avec Irrlicht


        }

        void Update(chrono::ChPhysicsItem* updater,
                    const chrono::ChCoordsys<>& coords) {

            // Here, the asset point is automatically following the motion but the force has to be updated
            SetGlyphVector(0, m_force->GetVpoint(), m_force->GetForce());

        }

    };












    //>>>>>>>>>>>>>>>>>>>>>>> REFACTORING




    class FrForceAsset_;

    namespace internal{

    struct FrForceAssetBase_ : public chrono::ChGlyphs {

        FrForceAsset_ * m_frydomForceAsset;

        explicit FrForceAssetBase_(FrForceAsset_ * forceAsset);

        void Update(chrono::ChPhysicsItem* updater, const chrono::ChCoordsys<>& coords) override;

    };

    }


    class FrForceAsset_ : public FrAsset {

    private:
        std::shared_ptr<internal::FrForceAssetBase_> m_chronoAsset;

        std::shared_ptr<FrForce_> m_force;  //< The force that this asset represents
        double OrderOfMagnitude;
        bool adaptive_OOM;

        double CharacteristicLength;
        bool inverse_direction;
        // TODO ajouter flag pour dire si on affiche la force qui s'applique ou la force delivree pour la visu (propulseurs...)

    protected:
        std::shared_ptr<chrono::ChAsset> GetChronoAsset() override;

    public:

        explicit FrForceAsset_(std::shared_ptr<FrForce_> force);

        void Update();

    };




}  // end of frydom namespace


#endif //FRYDOM_FRFORCEASSET_H
