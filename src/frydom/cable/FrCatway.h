//
// Created by frongere on 05/10/18.
//

#ifndef FRYDOM_FRCATWAY_H
#define FRYDOM_FRCATWAY_H


#include "FrCable.h"

#include "frydom/core/FrForce.h"

#include "catenary/CatenaryLine.h"


namespace catenary {
//    class CatenaryLine;
    class CatenaryNode;
}

namespace chrono {
    class ChLineShape;
}

namespace frydom {





    class FrCatForce;
    class CatenaryCableAsset;
    class FrCatway;

    namespace internal {

        struct _CatenaryBase : public catenary::CatenaryLine, public chrono::ChPhysicsItem {

            FrCatway* m_frydomCatLine;

            double m_breakingTension = 1e5;  // TODO : passer dans catway !!

            _CatenaryBase(FrCatway* frydomCatLine,
                          std::shared_ptr<catenary::CatenaryProperties> properties,
                          std::shared_ptr<catenary::CatenaryNode> startNode,
                          std::shared_ptr<catenary::CatenaryNode> endNode,
                          const double unstretchedLength);

            void Update(double time, bool update_assets) override;

            void SetupInitial() override;

            double GetBreakingTension();

        };

    }  // end namespace internal


    class FrCatway : public FrCable_ {

    private:

        std::shared_ptr<internal::_CatenaryBase> m_catLine;

        std::shared_ptr<catenary::CatenaryNode> m_startCatNode;
        std::shared_ptr<catenary::CatenaryNode> m_endCatNode;

        std::shared_ptr<FrCatForce> m_startForce;
        std::shared_ptr<FrCatForce> m_endForce;

        std::shared_ptr<CatenaryCableAsset> m_asset;

    public:

        FrCatway(double youngModulus, double diameter, double linearDensity, double length,
                         unsigned int nbElt, std::shared_ptr<FrNode_> startNode, std::shared_ptr<FrNode_> endNode);

        ~FrCatway();

        Force GetTension(double s) const override;

        Force GetStartNodeTension() const;

        Force GetEndNodeTension() const;

        Position GetAbsPosition(double s) const override;

        void Update();

        void Initialize() override;

        void StepFinalize() override;

        double GetBreakingTension();


        // For asset
        std::shared_ptr<CatenaryCableAsset> GetAsset();

    private:
        std::shared_ptr<chrono::ChPhysicsItem> GetChronoPhysicsItem() override;


    };


    class FrCatForce : public FrForce_ { // Doit etre mis en std::shared_ptr<FrForce_> dans FrBody_...

        enum SIDE {
            START,
            END
        };

        FrCatway* m_catenaryLine;

        SIDE m_side;

        friend class FrCatway;

    public:

        FrCatForce(FrCatway* catenaryLine, std::shared_ptr<FrNode_> node);

        void Update(double time) override;

        void Initialize() override;

        void StepFinalize() override;

    private:

        void SetAbsTension(const Force& tension);

    };


    namespace internal {

        class CatLineGeom : chrono::geometry::ChLine {

        private:
            FrCatway *m_catLine;

        public:
            explicit CatLineGeom(FrCatway *catLine);

            void Evaluate(chrono::ChVector<double> &pos, const double u) const;

        };

        class CatLineAsset : public chrono::ChLineShape {

        public:

            explicit CatLineAsset(std::shared_ptr<CatLineGeom> catLineGeom);



        };


    }  // end namespace internal


    class CatenaryCableAsset {

    private:

        internal::_CatenaryBase* m_cable;

        std::vector<std::shared_ptr<chrono::ChLineShape>> m_elements;

        unsigned int m_nbDrawnElements = 40;


    public:

        explicit CatenaryCableAsset(internal::_CatenaryBase* cable);

        void SetNbElements(unsigned int n);

        void Update();

        void Initialize();




    };





}  // end namespace frydom

#endif //FRYDOM_FRCATWAY_H
