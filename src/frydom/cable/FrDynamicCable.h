//
// Created by lletourn on 05/03/19.
//

#ifndef FRYDOM_FRDYNAMICCABLE_H
#define FRYDOM_FRDYNAMICCABLE_H

#include "FrCable.h"
#include "frydom/core/common/FrFEAMesh.h"
#include "frydom/core/FrOffshoreSystem.h"

#include <chrono/fea/ChMesh.h>

namespace chrono{
    namespace fea {
        class ChNodeFEAxyzrot;
        class ChBeamSectionAdvanced;
    }
    class ChLinkMateGeneric;
}


namespace frydom {

    // Forward declaration
    class FrDynamicCable;

    namespace internal{

        /**
         * /class FrDynamicCableBase
         * /Brief Base class for the Dynamic Cable
         * This class contains the Finite Element Analysis (FEA) mesh, starting and ending nodes and hinges, along with
         * the section properties (linear density, section, inertia, Young modulus, Rayleigh damping, etc.)
         *
         * /see FrDynamicCable
         */
        struct FrDynamicCableBase : public chrono::fea::ChMesh {

            FrDynamicCable* m_frydomCable;      ///< pointer to the Dynamic cable containing this base class

            bool m_drawCableElements = true;    ///< Boolean to check if the FEA elements are to be drawnn
            bool m_drawCableNodes = true;       ///< Boolean to check if the FEA nodes are to be drawnn

            std::shared_ptr<chrono::fea::ChNodeFEAxyzrot> m_starting_node_fea;  ///< Starting node
            std::shared_ptr<chrono::fea::ChNodeFEAxyzrot> m_ending_node_fea;    ///< Ending node

            std::shared_ptr<chrono::ChLinkMateGeneric> m_startingHinge;         ///< Starting hinge, to connect to a body
            std::shared_ptr<chrono::ChLinkMateGeneric> m_endingHinge;           ///< Ending hinge, to connect to a body

            std::shared_ptr<chrono::fea::ChBeamSectionAdvanced> m_section;      ///< Section properties (linear density, section, inertia, Young modulus, Rayleigh damping, etc.)

            /// Constructor of the FrDynamicCableBase
            /// \param cable pointer to the FrDynamicCable containing this base class
            explicit FrDynamicCableBase(FrDynamicCable* cable);

            /// Initialize the cable
            void Initialize();

            /// Update time dependent data, for all elements.
            /// Updates all [A] coord.systems for all (corotational) elements.
            void Update(double time, bool update_assets) override;

            /// Get the position of the dynamic cable at the local position eta in [-1,1], of the indexth element
            /// \param index index of the cable element
            /// \param eta Local position on the element, in [-1,1]
            /// \return Position of the cable, in the world reference frame
            Position GetNodePositionInWorld(int index, double eta);

            /// Get the axial tension (compression only, no bending) of the dynamic cable at the local position
            /// eta in [-1,1], of the indexth element
            /// \param index index of the cable element
            /// \param eta Local position on the element, in [-1,1]
            /// \return Axial tension of the dynamic cable, in world reference frame
            Force GetTension(int index, double eta);

            /// Initialize the links (hinges) between the cable and the bodies
            //FYI : Can't be changed to private, since it's friend with FrBody (need chronoBody)
            void InitializeLinks();

        private:

            /// Initialize the cable section
            void InitializeSection();

            /// Generate assets for the cable
            void GenerateAssets();

            /// Define the constraints in the hinges
            void HingesConstraints();

        };


    }



    /**
     * \class FrDynamicCable FrDynamicCable.h
     * \brief Class for dynamic cable, subclass of FrCable and FrFEAMesh
     * The dynamic cable is based on a Finite Element Analysis (FEA) cable, with an Euler-Bernoulli formulation on a
     * simple beam element with two nodes. The section and material properties are assumed constant along the beam.
     *
     *For more information, refer to : http://www.projectchrono.org/assets/white_papers/FEA/euler_beams.pdf
     */
     //TODO : Additional linear loads (Morison, hydrostatic, etc.)
     //TODO : Breaking of cable
     //TODO : Unrolling
     //TODO : Contact with seabed or other cable/bodies
     //TODO : Check for deactivation
    class FrDynamicCable: public FrCable, public FrFEAMesh {
    public:

        enum HingeType { CONSTRAINED, SPHERICAL, NONE};

    private:

        std::shared_ptr<internal::FrDynamicCableBase> m_chronoCable;    ///< pointer to the Chrono cable

        double m_rayleighDamping;               ///< Rayleigh damping
        unsigned int m_nbElements;              ///< Number of elements in the finite element cable model

        // Hinges types
        HingeType m_startingHingeType = SPHERICAL;
        HingeType m_endingHingeType = SPHERICAL;

        // Asset parameters
        double m_drawCableElementRadius = 0.05; ///< Radius of the cable element assets
        double m_drawCableNodeSize = 0.1;       ///< Size of the cable node assets

    protected:

        internal::FrDynamicCableBase* GetChronoItem_ptr() const override { return m_chronoCable.get(); }

        std::shared_ptr<chrono::fea::ChMesh> GetChronoMesh() override { return m_chronoCable; }

    public:

        /// Constructor of the Dynamic Cable
        /// \param startingNode Starting node
        /// \param endingNode Ending node
        /// \param cableLength unstretched length of the cable, in m
        /// \param youngModulus Young modulus, in Pa
        /// \param sectionArea Section area, in m²
        /// \param linearDensity Linear density, in Kg/m
        /// \param rayleighDamping Rayleigh damping
        /// \param nbElements Number of elements/discretization
        FrDynamicCable( std::shared_ptr<FrNode> startingNode,
                        std::shared_ptr<FrNode> endingNode,
                        double cableLength,
                        double youngModulus,
                        double sectionArea,
                        double linearDensity,
                        double rayleighDamping,
                        unsigned int nbElements );

        /// Get the type name of this object
        /// \return type name of this object
        std::string GetTypeName() const override { return "DynamicCable"; }

        /// Set the Rayleigh damping coefficient (only stiffness related coefficient is included in chrono model, as in: R = r * K )
        /// \param damping Raleigh damping
        void SetRayleighDamping(double damping);

        /// Get the Rayleigh damping coefficient
        /// \return Raleigh damping
        double GetRayleighDamping() const;

        /// Set the number of elements/discretization of the cable
        /// \param nbElements Number of elements
        void SetNumberOfElements(unsigned int nbElements);

        /// Get the number of elements of the cable
        /// \return Number of elements
        unsigned int GetNumberOfElements() const;

        /// Set the number of elements based on a target element length
        /// \param ElementLength Target length of the elements
        void SetTargetElementLength(double ElementLength);

        /// Set the radius of the cable elements assets
        /// \param radius Radius of the cable elements assets
        void SetDrawElementRadius(double radius);

        /// Get the radius of the cable elements assets
        /// \return Radius of the cable elements assets
        double GetDrawElementRadius();

        /// Set the size of the cable nodes assets
        /// \param size Size of the cable nodes assets
        void SetDrawNodeSize(double size);

        /// Get the size of the cable nodes assets
        /// \return Size of the cable nodes assets
        double GetDrawNodeSize() const;

        /// Set the starting hinge type (CONSTRAINED, SPHERICAL, NONE)
        /// \param type starting hinge type (CONSTRAINED, SPHERICAL, NONE)
        void SetStartingHingeType(HingeType type);

        /// Get the starting hinge type (CONSTRAINED, SPHERICAL, NONE)
        /// \return starting hinge type (CONSTRAINED, SPHERICAL, NONE)
        HingeType GetStartingHingeType() const;

        /// Set the ending hinge type (CONSTRAINED, SPHERICAL, NONE)
        /// \param type ending hinge type (CONSTRAINED, SPHERICAL, NONE)
        void SetEndingHingeType(HingeType type);

        /// Get the ending hinge type (CONSTRAINED, SPHERICAL, NONE)
        /// \return ending hinge type (CONSTRAINED, SPHERICAL, NONE)
        HingeType GetEndingHingeType() const;


        // Virtual methods, from FrCable

        /// Set the number of asset elements depicted
        /// \param n number of asset elements
        virtual void SetAssetElements(unsigned int n) {assert(false && "Don't use this !");}

        /// Get the number of asset elements depicted
        /// \return number of asset elements
        virtual unsigned int GetAssetElements() { return m_nbElements;}

        /// Get the inside line tension at the lagrangian coordinate s, from the starting node to the ending node
        /// \param s lagrangian coordinate
        /// \param fc frame convention (NED/NWU)
        /// \return inside line tension
        Force GetTension(double s, FRAME_CONVENTION fc) const override;;

        /// Get the line position at lagrangian coordinate s
        /// \param s lagrangian coordinate
        /// \param fc frame convention (NED/NWU)
        /// \return line position
        Position GetNodePositionInWorld(double s, FRAME_CONVENTION fc) const override;;

        /// Get the stretched length of the cable
        /// \return stretched length
        double GetStretchedLength() const override;;


        // Virtual methods, from FEAMesh

        /// Initialize the cable with given data
        void Initialize() override;

        /// Update the internal parameters of the cable
        /// \param time time of the simulation
        void Update(double time) override {};

        /// Initialize the log for the dynamic cable
        void InitializeLog() override;;

        /// Method called at the send of a time step. Logging may be used here
        void StepFinalize() override;

        double GetStaticResidual() override;

        void Relax() override;


        // Friend definitions

        friend void FrOffshoreSystem::Add(std::shared_ptr<FrDynamicCable>);
        friend void FrOffshoreSystem::Remove(std::shared_ptr<FrDynamicCable>);

    };

    std::shared_ptr<FrDynamicCable>
    make_dynamic_cable(const std::shared_ptr<FrNode> &startingNode, const std::shared_ptr<FrNode> &endingNode,
                       FrOffshoreSystem *system, double unstretchedLength, double youngModulus, double sectionArea,
                       double linearDensity, double rayleighDamping, unsigned int nbElements);

} // end namespace frydom
#endif //FRYDOM_FRDYNAMICCABLE_H
