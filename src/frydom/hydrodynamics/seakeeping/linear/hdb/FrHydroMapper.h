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


#ifndef FRYDOM_FRHYDROMAPPER_H
#define FRYDOM_FRHYDROMAPPER_H

#include <unordered_map>
#include <memory>


namespace frydom {

    // Forward declaration
    template<typename OffshoreSystemType>
    class FrEquilibriumFrame;

    template<typename OffshoreSystemType>
    class FrBEMBody;

    template<typename OffshoreSystemType>
    class FrBody;

    /**
     * \class FrHydroMapper
     * \brief Class for mapping the bodies with the HDB.
     */

    template<typename OffshoreSystemType>
    class FrHydroMapper {

     private:
      std::unordered_map<FrBEMBody<OffshoreSystemType> *, FrBody<OffshoreSystemType> *> m_mapBEMToBody; // Mapping of FrBEMBodies with FrBodies.
      std::unordered_map<FrBody<OffshoreSystemType> *, FrBEMBody<OffshoreSystemType> *> m_mapBodyToBEM; // Mapping of FrBodies with FrBEMBodies.
      std::unordered_map<FrBEMBody<OffshoreSystemType> *, std::shared_ptr<FrEquilibriumFrame<OffshoreSystemType>>> m_mapEqFrame; // Mapping of BEMBodies with FrEquilibriumFrame.

     public:

      /// Default constructor
      FrHydroMapper() = default;

      /// Mapping a BEM body database with a body
      /// \param BEMBody BEM Body database
      /// \param body body (frydom object)
      /// \param eqFrame Equilibrium frame corresponding to the body
      void Map(FrBEMBody<OffshoreSystemType> *BEMBody, FrBody<OffshoreSystemType> *body,
               std::shared_ptr<FrEquilibriumFrame<OffshoreSystemType>> eqFrame);

      /// Get the number of bodies map
      /// \return Number of bodies map
      unsigned long GetNbMappings() const;

      /// Return the body corresponding to a given BEM body database
      /// \param BEMBody BEM body database
      /// \return body(frydom object)
      FrBody<OffshoreSystemType> *GetBody(FrBEMBody<OffshoreSystemType> *BEMBody) const;

      /// Return the BEM body database corresponding to the given body
      /// \param body body (frydom object)
      /// \return BEM body database
      FrBEMBody<OffshoreSystemType> *GetBEMBody(FrBody<OffshoreSystemType> *body) const;

      /// Return the corresponding index in the map of a given body
      /// \param body body (frydom object)
      /// \return Index of the body in the map
      unsigned int GetBEMBodyIndex(FrBody<OffshoreSystemType> *body) const;

      /// Return the equilibrium frame
      /// \param BEMBody BEM body database
      /// \return Equilibrium frame
      FrEquilibriumFrame<OffshoreSystemType> *GetEquilibriumFrame(FrBEMBody<OffshoreSystemType> *BEMBody) const;

      /// Return the equilibrium frame
      /// \param body body (frydom object)
      /// \return Equilibrium frame
      FrEquilibriumFrame<OffshoreSystemType> *GetEquilibriumFrame(FrBody<OffshoreSystemType> *body) const;

      typename std::unordered_map<FrBEMBody<OffshoreSystemType> *, FrBody<OffshoreSystemType> *>::iterator
      begin() { return m_mapBEMToBody.begin(); }

      typename std::unordered_map<FrBEMBody<OffshoreSystemType> *, FrBody<OffshoreSystemType> *>::iterator
      end() { return m_mapBEMToBody.end(); };

    };

}  // end namespace frydom


#endif //FRYDOM_FRHYDROMAPPER_H
