//
// Created by lletourn on 05/03/19.
//

#ifndef FRYDOM_FRANCFCABLE_H
#define FRYDOM_FRANCFCABLE_H

#include "FrCable.h"
#include "frydom/core/math/FrVector.h"

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

            std::shared_ptr<chrono::fea::ChLinkPointFrame> m_startingHinge;
            std::shared_ptr<chrono::fea::ChLinkPointFrame> m_endingHinge;

            std::shared_ptr<chrono::fea::ChBeamSectionCable> m_section;
            FrANCFCable* m_frydomCable;

            /// Initialize the cable
            void Initialize();

            /// Initialize the cable section
            void InitializeSection();

            /// Initialize the links between the cable and the bodies
            void InitializeLinks();

            /// Generate assets for the cable
            void GenerateAssets();

            void Update(double time, bool update_assets) override;

            void AddToChronoSystem();

            void SetStartingNode(Position position, Direction direction);
            void SetEndingNode(Position position, Direction direction);

        };


    }



    class FrANCFCable: public FrCable {

    private:

        std::shared_ptr<internal::FrANCFCableBase> m_chronoCable;


        double m_rayleighDamping;   ///< Rayleigh damping
        unsigned int m_nbElements;  ///< Number of elements in the finite element cable model

        double m_drawCableElementRadius = 0.05;
        double m_drawCableNodeSize = 0.1;

    public:

        FrANCFCable(const std::shared_ptr<FrNode> startingNode,
                    const std::shared_ptr<FrNode> endingNode,
                    double cableLength,
                    double youngModulus,
                    double sectionArea,
                    double linearDensity);

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
        virtual Force GetTension(double s, FRAME_CONVENTION fc) const = 0;

        /// Get the line position at lagrangian coordinate s
        /// \param s lagrangian coordinate
        /// \param fc frame convention (NED/NWU)
        /// \return line position
        virtual Position GetAbsPosition(double s, FRAME_CONVENTION fc) const = 0;

        /// Get the stretched length of the cable
        /// \return stretched length
        virtual double GetStretchedLength() const = 0;


        //

        /// Initialize the cable with given data
        void Initialize();

        virtual void StepFinalize() override;


        /// Update internal time and time step for dynamic behaviour of the cable
        virtual void UpdateTime(const double time);;

        void Update();

    };

} // end namespace frydom
#endif //FRYDOM_FRANCFCABLE_H
