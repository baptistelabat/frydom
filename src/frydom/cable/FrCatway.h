//
// Created by frongere on 05/10/18.
//

#ifndef FRYDOM_FRCATWAY_H
#define FRYDOM_FRCATWAY_H


#include "FrCable.h"

#include "frydom/core/FrForce.h"

#include "catenary/CatenaryLine.h"
#include "catenary/CatenaryNode.h"

#include "catenary/EnvironmentInterface.h"
#include "catenary/UniformDistributedLoad.h"  // TODO : placer un Catenary au debut ?


namespace catenary {
//    class CatenaryLine;
//    class CatenaryNode;
}

namespace chrono {
    class ChLineShape;
}

namespace frydom {


    class FrCatForce;
    class CatenaryCableAsset;
    class FrCatway;
    class FrCatForce;


    namespace internal {

        struct _CatenaryBase : public catenary::CatenaryLine, public chrono::ChPhysicsItem {

            FrCatway* m_frydomCatLine;

            double m_breakingTension = 12000;  // TODO : passer dans catway !!

            _CatenaryBase(FrCatway* frydomCatLine,
                          std::shared_ptr<catenary::CatenaryProperties> properties,
                          std::shared_ptr<catenary::CatenaryNode> startNode,
                          std::shared_ptr<catenary::CatenaryNode> endNode,
                          double unstretchedLength);

            void Update(double time, bool update_assets) override;

            void SetupInitial() override;

            double GetBreakingTension();

        };

        struct _CatenaryNode : public catenary::CatenaryNode {

            FrCatForce* m_force;

            explicit _CatenaryNode(FrCatForce* force);

            void Update(bool reverse);

        };


    }  // end namespace internal


    class FrCatway : public FrCable_ {

    private:

        std::shared_ptr<internal::_CatenaryBase> m_catLine;

        std::shared_ptr<internal::_CatenaryNode> m_startCatNode;
        std::shared_ptr<internal::_CatenaryNode> m_endCatNode;

        std::shared_ptr<FrCatForce> m_startForce;
        std::shared_ptr<FrCatForce> m_endForce;

        std::shared_ptr<CatenaryCableAsset> m_asset;

        unsigned int c_nbElt = 1;  // TODO : changer la maniere dont est traite la discretisation


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

        void AddMorrisonForce(double Ct, double Cn);


        // For asset
        std::shared_ptr<CatenaryCableAsset> GetAsset();

    private:
        std::shared_ptr<chrono::ChPhysicsItem> GetChronoPhysicsItem() const override;


    };


    class FrCatForce : public FrForce_ { // Doit etre mis en std::shared_ptr<FrForce_> dans FrBody_...

        enum SIDE {
            START,
            END
        };

        friend class FrCatway;

    private:
        std::shared_ptr<FrNode_> m_node;

    public:

        explicit FrCatForce(std::shared_ptr<FrNode_> node);

        void Update(double time) override;

        void Initialize() override;

        void StepFinalize() override;

    private:

        void SetAbsTension(const Force& tension);

        friend void internal::_CatenaryNode::Update(bool);

    };




//    namespace internal {
//
//        class CatLineGeom : chrono::geometry::ChLine {
//
//        private:
//            FrCatway *m_catLine;
//
//        public:
//            explicit CatLineGeom(FrCatway *catLine);
//
//            void Evaluate(chrono::ChVector<double> &pos, const double u) const override;
//
//        };
//
//        class CatLineAsset : public chrono::ChLineShape {
//
//        public:
//
//            explicit CatLineAsset(std::shared_ptr<CatLineGeom> catLineGeom);
//
//
//
//        };
//
//
//    }  // end namespace internal


    class CatenaryCableAsset {

    private:

        internal::_CatenaryBase* m_cable;

        using Triplet = std::tuple<double, double, std::shared_ptr<chrono::ChLineShape>>;
        std::vector<Triplet> m_elements;


        unsigned int m_nbDrawnElements = 40;


        static Triplet make_triplet(double s0, double s1, std::shared_ptr<chrono::ChLineShape> lineShape) {
            return std::make_tuple(s0, s1, lineShape);
        }


    public:

        explicit CatenaryCableAsset(internal::_CatenaryBase* cable);

        void SetNbElements(unsigned int n);

        void Update();

        void Initialize();


    };




    class FrCatwayEnvironmentInterface : public catenary::CatenaryEnvironmentInterface {

        // TODO : terminer !! Nécessite de changer de paradigme pour l'interface environnement dans Catway (abandonner le pattern singleton)

//    public:


    private:
        FrEnvironment_* m_frydomEnvironment;

    public:

        explicit FrCatwayEnvironmentInterface(FrEnvironment_* frydomEnvironment);

//        void Initialize();

        const double GetGravity() const override;
        const double GetWaterDensity() const override;
        const double GetAirDensity() const override;

        const double GetFreeSurfaceHeight(const Vector& position) const override;
        const double GetSeabedHeight(const Vector& position) const override;

        const Velocity GetEnvironmentFlux(const Position& position) const;


    };



    class FrCatenaryMorrison : public catenary::CatenaryUniformLoad { // TODO : voir a ajouter morrison directement dans Catway !!

    private:

        double m_Ct;
        double m_Cn;

    public:
        FrCatenaryMorrison(double Ct, double Cn);

        const catenary::CatenaryElement::Vector GetLoad(catenary::CatenaryElement* element) const override;




    };











//    class FrCatMorrison : public






}  // end namespace frydom

#endif //FRYDOM_FRCATWAY_H
