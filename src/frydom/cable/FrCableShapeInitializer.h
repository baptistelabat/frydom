//
// Created by frongere on 15/02/2020.
//

#ifndef FRYDOM_FRCABLESHAPEINITIALIZER_H
#define FRYDOM_FRCABLESHAPEINITIALIZER_H

#include "frydom/core/math/FrVector.h"


namespace frydom {

  // Forward declarations
  class FrEnvironment;

  class FrCable;

  class FrCableShapeInitializer {

   public:
    // Factory static method to get the correct Shape initializer for the given cable
    static std::unique_ptr<FrCableShapeInitializer> Create(FrCable *cable, FrEnvironment *environment);

    virtual Position GetPosition(const double &s, FRAME_CONVENTION fc) const {
      assert(false); // We should never pass here !
    };

   protected:
    // info: constructor is protected as we must use the static Create method which is a factory method
    // and choose the rigth concrete class to implement
    explicit FrCableShapeInitializer(FrCable *cable);

    FrCable *m_cable;
  };

  class FrCatenaryLine;

  namespace internal {

    class FrCableShapeInitializerTaut : public FrCableShapeInitializer {
     public:
      explicit FrCableShapeInitializerTaut(FrCable *cable);

      Position GetPosition(const double &s, FRAME_CONVENTION fc) const override;

     private:
      Direction m_unit_vector;

    };

    class FrCableShapeInitializerSlack : public FrCableShapeInitializer {
     public:
      explicit FrCableShapeInitializerSlack(FrCable *cable, std::unique_ptr<FrCatenaryLine> catenary_cable);

      Position GetPosition(const double &s, FRAME_CONVENTION fc) const override;

     private:
      std::unique_ptr<FrCatenaryLine> m_catenary_line;

    };

    class FrCableShapeInitializerSlackSeabed : public FrCableShapeInitializer {
     public:
      FrCableShapeInitializerSlackSeabed(FrCable *cable,
                                         FrEnvironment *environment);

      Position GetPosition(const double &s, FRAME_CONVENTION fc) const override;

     private:
      FrEnvironment *m_environment;

      Position m_origin_position;
      Position m_touch_down_point_position;
      Direction m_horizontal_direction;
      Direction m_raising_direction;
      double m_lying_distance;

      bool m_reversed;

    };

  }  // end namespace frydom::internal
} // end namespace frydom

#endif //FRYDOM_FRCABLESHAPEINITIALIZER_H
