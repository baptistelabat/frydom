//
// Created by frongere on 03/03/2020.
//

#ifndef FRYDOM_FRCATENARYLINEBASE_H
#define FRYDOM_FRCATENARYLINEBASE_H

#include "FrCable.h"
#include "frydom/core/common/FrPhysicsItem.h"
#include "frydom/core/FrOffshoreSystem.h"
#include "frydom/core/math/FrVector.h"
#include "frydom/core/force/FrForce.h"
#include "frydom/asset/FrCatenaryLineAsset.h"


namespace frydom {

//  // Forward declarations:
//  class FrCatenaryForce;





//  class LocalizedForce { // Mettre dans catenaryline !!!
//
//   public:
//
//
//
//  };



  class FrCatenaryLineBase : public FrLoggable<FrOffshoreSystem>,
                             public FrCable,
                             public FrPrePhysicsItem,
                             public FrCatenaryAssetOwner {

   public:

    enum LINE_SIDE {
      LINE_START,
      LINE_END
    };

    using Tension = Force;

    FrCatenaryLineBase(const std::string &name,
                       const std::string &type,
                       const std::shared_ptr<FrNode> &startingNode,
                       const std::shared_ptr<FrNode> &endingNode,
                       const std::shared_ptr<FrCableProperties> &properties,
//                       bool elastic,
                       double unstretchedLength) :
        FrLoggable(name, type, startingNode->GetSystem()),
        FrPrePhysicsItem(),
        FrCable(startingNode, endingNode, properties, unstretchedLength),
        m_use_for_shape_initialization(false),
        m_tolerance(1e-6),
        m_maxiter(100) {}


    /// Set the Newton-Raphson solver tolerance
    /// \param tol solver tolerance
    void SetSolverTolerance(double tol) { m_tolerance = tol; }

    /// Set the Newton-Raphson solver maximum number of iterations
    /// \param maxiter maximum number of iterations
    void SetSolverMaxIter(unsigned int maxiter) { m_maxiter = maxiter; }

    /// Tells the line it is only for shape initialization and not for a real cable usage so that
    /// no force is added to boundary bodies
    void UseForShapeInitialization(bool use) {
      m_use_for_shape_initialization = use;
    };




    void Initialize() = 0;

    Force GetTension(const double &s, FRAME_CONVENTION fc) const = 0;

    Position GetPositionInWorld(const double &s, FRAME_CONVENTION fc) const = 0;

    double GetUnstretchedLength() const = 0;

    virtual bool HasSeabedInteraction() const = 0;

    virtual void solve() = 0;


   protected:

    // Attention: toutes les methodes qui suivent sont adimentionnelles !!!

    // Tension relative methods

//    inline const Direction &pi() const { return m_pi; };
//
//    inline const double &q() const { return m_q; };
//
//    virtual inline Tension t0() const = 0;
//
////    virtual inline auto t(const double &s) const = 0;
//
//    virtual inline Tension tL() const = 0;
//
//    virtual inline Position p0() const = 0;
//
//    virtual inline Position p(const double &s) const = 0;
//
//    virtual inline Position pL() const = 0;
//
//    virtual inline double L() const = 0;


   protected:
    bool m_use_for_shape_initialization;

    double m_tolerance = 1e-6;
    unsigned int m_maxiter = 100;
    const double Lmin = 1e-10;

    const Direction m_pi;
    double m_q;

//    double m_unstretchedLength;


//    // Forces to apply to bodies
//    std::shared_ptr<FrCatenaryForce> m_startingForce;   ///< Force applied by the catenary line to the body at the
//    ///< starting node
//    std::shared_ptr<FrCatenaryForce> m_endingForce;     ///< Force applied by the catenary line to the body at the
//    ///< ending node




  };


  /**
* \class FrCatenaryForce FrCatenaryForce.h
* \brief Class for getting the tension from the catenary line, subclass of FrForce.
* This class get the tension computed by the catenary line class, to the body on which the force is applied.
* A differenciation is done on which side of the cable (starting or ending), the force is applied.
* \see FrCatenaryLine_, FrForce
*/
  class FrCatenaryForce : public FrForce {

   private:

    FrCatenaryLineBase *m_line; ///< The parent line
    FrCatenaryLineBase::LINE_SIDE m_line_side;   ///< The side of the line where the tension is applied

   public:

    /// FrCatenaryForce constructor, from a catenary line, and the description of the side of this line
    /// \param line catenary line applying a tension
    /// \param side side of the line (starting or ending)
    FrCatenaryForce(const std::string &name,
                    FrBody *body,
                    FrCatenaryLineBase *line,
                    FrCatenaryLineBase::LINE_SIDE side);;

    /// Return true if the force is included in the static analysis
    bool IncludedInStaticAnalysis() const override;

   private:

    /// Update the catenary force : get the tension applied by the line on the corresponding node
    /// \param time time of the simulation
    void Compute(double time) override;

  };


}  // end namespace frydom



#endif //FRYDOM_FRCATENARYLINEBASE_H
