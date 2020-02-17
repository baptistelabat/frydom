//
// Created by frongere on 15/02/2020.
//

#include "FrCableShapeInitializer.h"
#include "frydom/logging/FrEventLogger.h"

#include "frydom/environment/FrEnvironmentInc.h"

#include "frydom/cable/FrCable.h"
#include "frydom/cable/FrCatenaryLine.h"


namespace frydom {


  std::unique_ptr<FrCableShapeInitializer>
  FrCableShapeInitializer::Create(FrCable *cable, FrEnvironment *environment) {

    auto startNode = cable->GetStartingNode();
    auto endNode = cable->GetEndingNode();
    auto unstretchedLength = cable->GetUnstrainedLength();

    auto ocean = environment->GetOcean();

    auto startPosition = startNode->GetPositionInWorld(NWU);
    auto endPosition = endNode->GetPositionInWorld(NWU);

    double node_distance = (startPosition - endPosition).norm();

    if (node_distance > unstretchedLength) {
      // TAUT LINE
      return std::make_unique<internal::FrCableShapeInitializerTaut>(cable);

    } else {

      auto seabed = ocean->GetSeabed();

      auto fluid_type = FLUID_TYPE::AIR;
      if (ocean->GetFreeSurface()->IsInWater(startPosition, NWU) ||
          ocean->GetFreeSurface()->IsInWater(endPosition, NWU)) {
        fluid_type = FLUID_TYPE::WATER;
      }

      // Yes, we know that we are computing twice a catenary line when the slack only case is activated...
      // But it is quick and initialization only so don't worry about that !!
      auto catenary_line = std::make_unique<FrCatenaryLine>("initialize", cable, true, fluid_type);
      catenary_line->Initialize();

      if (catenary_line->HasSeabedInteraction()) {
        // Slack with seabed interactions
        return std::make_unique<internal::FrCableShapeInitializerSlackSeabed>(cable, environment);

      } else {
        // Only slack
        return std::make_unique<internal::FrCableShapeInitializerSlack>(cable, std::move(catenary_line));

      }
    }
  }

  FrCableShapeInitializer::FrCableShapeInitializer(FrCable *cable) :
      m_cable(cable) {}


  namespace internal {


    FrCableShapeInitializerTaut::FrCableShapeInitializerTaut(FrCable *cable) :
        FrCableShapeInitializer(cable),
        m_unit_vector((cable->GetEndingNode()->GetPositionInWorld(NWU) -
                       cable->GetStartingNode()->GetPositionInWorld(NWU)).normalized()) {
    }

    Position FrCableShapeInitializerTaut::GetPosition(const double &s, FRAME_CONVENTION fc) const {
      assert(0. <= s <= m_cable->GetUnstrainedLength());
      auto position = m_cable->GetStartingNode()->GetPositionInWorld(NWU) + s * m_unit_vector;
      if (IsNED(fc)) {
        internal::SwapFrameConvention<Position>(position);
      }
      return position;
    }

    FrCableShapeInitializerSlack::FrCableShapeInitializerSlack(FrCable *cable,
                                                               std::unique_ptr<FrCatenaryLine> catenary_cable)
        :
        FrCableShapeInitializer(cable),
        m_catenary_line(std::move(catenary_cable)) {}

    Position FrCableShapeInitializerSlack::GetPosition(const double &s, FRAME_CONVENTION fc) const {
      assert(0. <= s <= m_cable->GetUnstrainedLength());
      return m_catenary_line->GetNodePositionInWorld(s, fc);
    }

    FrCableShapeInitializerSlackSeabed::FrCableShapeInitializerSlackSeabed(FrCable *cable,
                                                                           FrEnvironment *environment) :
        m_environment(environment),
        FrCableShapeInitializer(cable) {

      // FIXME: pour le moment, on est reducteur et on considere que l'interaction de cable se fait entierement sur
      // une portion limite de cable (gauche ou droite). On ne prend pas en compte le cas ou une partie milieu repose
      // sur le seabed

      // FIXME : pour le moment, on considere que origin_node est l'ancre et se situe sur le seabed...
      // C'est reducteur et il faudra ameliorer cela... (notamment verifier)
      // Prevoir egalement la bathymetrie variable dans GetPosition()...
      std::shared_ptr<FrNode> origin_node;
      std::shared_ptr<FrNode> final_node;
      if (cable->GetStartingNode()->GetBody()->IsFixedInWorld()) {
        origin_node = cable->GetStartingNode();
        final_node = cable->GetEndingNode();
        m_reversed = false;
      } else if (cable->GetEndingNode()->GetBody()->IsFixedInWorld()) {
        origin_node = cable->GetEndingNode();
        final_node = cable->GetStartingNode();
        m_reversed = true;
      } else {
        assert(false); // Pas pris en charge !!!
      }

      m_origin_position = origin_node->GetPositionInWorld(NWU);
      Position final_position = final_node->GetPositionInWorld(NWU);

      m_horizontal_direction = final_position - m_origin_position;
      double vertical_spreading = m_horizontal_direction.z(); // d

      m_horizontal_direction.z() = 0.;
      double horizontal_spreading = m_horizontal_direction.norm(); // h

      m_horizontal_direction.normalize();

      double L = m_cable->GetUnstrainedLength();
      m_lying_distance = 0.5 * (L + horizontal_spreading -
          (vertical_spreading * vertical_spreading) / (L - horizontal_spreading));

      m_touch_down_point_position = m_origin_position + m_lying_distance * m_horizontal_direction;

      m_raising_direction = (final_position - m_touch_down_point_position).normalized();

    }

    Position FrCableShapeInitializerSlackSeabed::GetPosition(const double &s, FRAME_CONVENTION fc) const {
      assert(0. <= s <= m_cable->GetUnstrainedLength());

      double stmp;
      if (m_reversed)
        stmp = m_cable->GetUnstrainedLength() - s;
      else {
        stmp = s;
      }

      Position position;
      if (stmp <= m_lying_distance) {
        position = m_origin_position + stmp * m_horizontal_direction;
      } else {
        position = m_touch_down_point_position + (stmp - m_lying_distance) * m_raising_direction;
      }

      return position;
    }

  }  // end namespace frydom::internal

}  // end namespace frydom
