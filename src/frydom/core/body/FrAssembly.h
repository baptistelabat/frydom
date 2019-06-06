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

        std::vector<std::shared_ptr<FrBody>> m_bodyList;

    public:

        void Clear() {
            m_bodyList.clear();
        }

        void AddToAssembly(const std::shared_ptr<FrBody>& body);

        void RemoveFromAssembly(const std::shared_ptr<FrBody>& body);

        FrInertiaTensor GetInertiaTensor(FRAME_CONVENTION fc) const;

    };

} // end namespace frydom

#endif //FRYDOM_FRASSEMBLY_H
