//
// Created by lletourn on 06/06/19.
//

#ifndef FRYDOM_FRASSEMBLY_H
#define FRYDOM_FRASSEMBLY_H

#include "frydom/core/common/FrObject.h"

namespace frydom {

    // Forward declarations
    class FrBody;
    class FrInertiaTensor;

    class FrAssembly : public FrObject {

    private:

      std::shared_ptr<FrBody> m_masterBody;

      std::vector<std::shared_ptr<FrBody>> m_bodyList;

    public:

      explicit FrAssembly(const std::shared_ptr<FrBody>& masterBody);

      /// Get the type name of the object
      /// \return type name of the object
      std::string GetTypeName() const override { return "Assembly"; };

      void Initialize() override {};

      void Clear() {
          m_bodyList.clear();
      }

      void AddToAssembly(const std::shared_ptr<FrBody>& body);

      void RemoveFromAssembly(const std::shared_ptr<FrBody>& body);

      std::shared_ptr<FrBody> GetMasterBody();

      std::vector<std::shared_ptr<FrBody>> GetBodyList();

      std::shared_ptr<FrBody> GetBody(int iBody);

      FrInertiaTensor GetInertiaTensor() const;

      void DoAssembly();

    };

} // end namespace frydom

#endif //FRYDOM_FRASSEMBLY_H
