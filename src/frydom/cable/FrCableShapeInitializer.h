//
// Created by frongere on 15/02/2020.
//

#ifndef FRYDOM_FRCABLESHAPEINITIALIZER_H
#define FRYDOM_FRCABLESHAPEINITIALIZER_H

#include <frydom/core/math/FrVector.h>


namespace frydom {

  class FrOcean;

  class FrCableShapeInitializerBase {

//   public:

//    enum CONFIG {
//      TAUT,         // boudary nodes distance is higher than unstretched length, line is straight
//      SLACK,        // boundary nodes distance is less than unstretched length, line is slack
//      SLACK_SEABED  // line is slack and lies on the seabed
//    };

   public:


    FrCableShapeInitializerBase(const Position &start_position,
                                const Position &end_position,
                                const double &unstretchedLenght,
                                FrOcean *ocean);


   private:
    FrOcean *m_ocean;
    Position m_start_position;
    Position m_end_position;
    double m_unstretchedLength;

  };


  class FrCableShapeInitializerTaut : public FrCableShapeInitializerBase {

  };

  class FrCableShapeInitializerSlack : public FrCableShapeInitializerBase {

  };

  class FrCableShapeInitializerSlackSeabed : public FrCableShapeInitializerBase {

  };
  



  class FrCableShapeInitializerFactory {
   public:
    FrCableShapeInitializerBase Create(const Position &start_position,
                                       const Position &end_position,
                                       const double &unstretchedLenght,
                                       FrOcean *ocean);
  };


} // end namespace frydom

#endif //FRYDOM_FRCABLESHAPEINITIALIZER_H
