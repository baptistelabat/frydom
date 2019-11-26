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

    /**
     *  \class : FrAssembly
     *  \brief : Assembly of FrBody instances, for inertia tensor manipulation
     */
    class FrAssembly : public FrObject {

    private:

      std::shared_ptr<FrBody> m_masterBody;             ///< Main body of the assembly, in which reference frame the
                                                        ///< assembly inertia tensor is given

      std::vector<std::shared_ptr<FrBody>> m_bodyList;  ///< list of bodies composing the assembly, except the master body

    public:

      /// Constructor of the assembly, requires a master body
      explicit FrAssembly(const std::shared_ptr<FrBody>& masterBody);

      /// Initialize the assembly
      void Initialize() override {};

      /// Clear the body list
      void Clear() {
          m_bodyList.clear();
      }

      /// Add a body to the assembly
      void AddToAssembly(const std::shared_ptr<FrBody>& body);

      /// Add a list of bodies to the assembly
      void AddToAssembly(const std::vector<std::shared_ptr<FrBody>>& bodyList);

      /// Remove a body to the assembly
      void RemoveFromAssembly(const std::shared_ptr<FrBody>& body);

      /// Get the master body of the assembly
      std::shared_ptr<FrBody> GetMasterBody();

      /// Get the list of bodies composing the assembly, except the master body
      std::vector<std::shared_ptr<FrBody>> GetBodyList();

      /// Get the i-th body of the list of bodies
      std::shared_ptr<FrBody> GetBody(int iBody);

      /// Get the inertia tensor of the assembly, given in the master body reference frame
      FrInertiaTensor GetInertiaTensor() const;

      /// Solve the constraints between the different bodies, the master body is considered fixed, the others bodies
      /// position according to the master body position and the constraints. Be careful if you have constraints not
      /// resolved between the master body and the WorldBody : use directly system->DoAssembly();
      void DoAssembly();

    };

    std::shared_ptr<FrAssembly> make_assembly(const std::shared_ptr<FrBody>& masterBody);

    std::shared_ptr<FrAssembly> make_assembly(const std::shared_ptr<FrBody>& masterBody, const std::vector<std::shared_ptr<FrBody>>& bodyList);

} // end namespace frydom

#endif //FRYDOM_FRASSEMBLY_H
