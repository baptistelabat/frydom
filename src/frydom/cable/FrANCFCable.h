//
// Created by lletourn on 05/03/19.
//

#ifndef FRYDOM_FRANCFCABLE_H
#define FRYDOM_FRANCFCABLE_H

#include "FrCable.h"
#include "frydom/core/math/FrVector.h"
#include "frydom/core/common/FrFEAMesh.h"
#include "frydom/core/FrOffshoreSystem.h"

#include <chrono/fea/ChMesh.h>
//#include <chrono/fea/ChNodeFEAxyzD.h>
//#include <chrono/fea/ChBeamSection.h>

namespace chrono{
    namespace fea {
        class ChNodeFEAxyzD;
        class ChBeamSectionCable;
        class ChLinkPointFrame;
    }
}


namespace frydom {

    // Forward declaration
    class FrANCFCable;

    namespace internal{

        struct FrANCFCableBase : public chrono::fea::ChMesh {

            bool m_drawCableElements = true;
            bool m_drawCableNodes = true;

            std::shared_ptr<chrono::fea::ChNodeFEAxyzD> m_starting_node_fea;
            std::shared_ptr<chrono::fea::ChNodeFEAxyzD> m_ending_node_fea;

            // TODO Passer en FrLink? générer les links entre FEAMesh et bodies en dehors de FEAMesh
            std::shared_ptr<chrono::fea::ChLinkPointFrame> m_startingHinge;
            std::shared_ptr<chrono::fea::ChLinkPointFrame> m_endingHinge;

            std::shared_ptr<chrono::fea::ChBeamSectionCable> m_section;
            FrANCFCable* m_frydomCable;

            FrANCFCableBase(FrANCFCable* cable);

            /// Initialize the cable
            void Initialize();

            /// Initialize the cable section
            void InitializeSection();

            /// Initialize the links between the cable and the bodies
            // TODO : générer les links entre FEAMesh et bodies à l'extérieur.
            void InitializeLinks();

            /// Generate assets for the cable
            void GenerateAssets();

            void Update(double time, bool update_assets) override;

            void SetStartingNode(Position position, Direction direction);
            void SetEndingNode(Position position, Direction direction);

        };


    }



    class FrANCFCable: public FrCable, public FrFEAMesh {

    private:

        std::shared_ptr<internal::FrANCFCableBase> m_chronoCable;


        double m_rayleighDamping;   ///< Rayleigh damping
        unsigned int m_nbElements;  ///< Number of elements in the finite element cable model

        double m_drawCableElementRadius = 0.05;
        double m_drawCableNodeSize = 0.1;

    protected:

        internal::FrANCFCableBase* GetChronoItem_ptr() const override { return m_chronoCable.get(); }

        std::shared_ptr<chrono::fea::ChMesh> GetChronoMesh() override { return m_chronoCable; }

    public:

        FrANCFCable(const std::shared_ptr<FrNode> startingNode,
                    const std::shared_ptr<FrNode> endingNode,
                    double cableLength,
                    double youngModulus,
                    double sectionArea,
                    double linearDensity,
                    double rayleighDamping,
                    unsigned int nbElements);

        /// Get the type name of this object
        /// \return type name of this object
        std::string GetTypeName() const override { return "ANCFCable"; }

        void SetRayleighDamping(const double damping);

        double GetRayleighDamping() const;

        void SetNumberOfElements(const unsigned int nbElements);

        unsigned int GetNumberOfElements() const;

        void SetTargetElementLength();

        void SetDrawRadius(const double radius);

        void SetDrawNodeSize(const double size);

        double GetDrawNodeSize() const;


        //

        /// Get the inside line tension at the lagrangian coordinate s, from the starting node to the ending node
        /// \param s lagrangian coordinate
        /// \param fc frame convention (NED/NWU)
        /// \return inside line tension
        Force GetTension(double s, FRAME_CONVENTION fc) const override {};

        /// Get the line position at lagrangian coordinate s
        /// \param s lagrangian coordinate
        /// \param fc frame convention (NED/NWU)
        /// \return line position
        Position GetAbsPosition(double s, FRAME_CONVENTION fc) const override {};

        /// Get the stretched length of the cable
        /// \return stretched length
        double GetStretchedLength() const override {};


        //

        /// Initialize the cable with given data
        void Initialize() override;

        void Update(double time) override {};

        void InitializeLog() override {};

//        void StepFinalize() override;

        friend void FrOffshoreSystem::AddANCFCable(std::shared_ptr<FrANCFCable>);

    };

} // end namespace frydom
#endif //FRYDOM_FRANCFCABLE_H
