//
// Created by lletourn on 05/02/20.
//

#ifndef FRYDOM_FRHYDROSTATICASSEMBLY_H
#define FRYDOM_FRHYDROSTATICASSEMBLY_H

#include "frydom/core/body/FrBody.h"
#include "frydom/core/body/FrInertiaTensor.h"
#include "frydom/core/body/FrAssembly.h"

namespace frydom {

  class FrHydrostaticBodyBase {

   public:

    virtual FrInertiaTensor GetInertiaTensor() const = 0;

    virtual std::shared_ptr<FrBody> GetBody() const = 0;

  };

  class FrHydrostaticBody : public FrHydrostaticBodyBase {

   public:

    explicit FrHydrostaticBody(const std::shared_ptr<FrBody> &body) : m_body(body) {}

    FrInertiaTensor GetInertiaTensor() const override {return m_body->GetInertiaTensor();};

    std::shared_ptr<FrBody> GetBody() const override { return m_body;}

   private:
    std::shared_ptr<FrBody> m_body;

  };

  class FrHydrostaticAssembly : public FrHydrostaticBodyBase {

   public:

    explicit FrHydrostaticAssembly(const std::shared_ptr<FrAssembly> &assembly) : m_assembly(assembly) {}

    FrInertiaTensor GetInertiaTensor() const override {return m_assembly->GetInertiaTensor();};

    std::shared_ptr<FrBody> GetBody() const override { return m_assembly->GetMasterBody();}

   private:
    std::shared_ptr<FrAssembly> m_assembly;

  };


} // end namespace frydom

#endif //FRYDOM_FRHYDROSTATICASSEMBLY_H
