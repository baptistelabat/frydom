//
// Created by frongere on 19/07/17.
//

#ifndef FRYDOM_FRCONVENTIONS_H_H
#define FRYDOM_FRCONVENTIONS_H_H

namespace frydom {
namespace environment {

    enum FrDirectionConvention {
        GOTO,           ///> The field "propagates towards", "goes to", "brings to". This is a flux vector.
        COMEFROM        ///> The field "comes from". This is the opposite of the flux vector
    };


}  // end namespace environment
}  // end namespace frydom

#endif //FRYDOM_FRCONVENTIONS_H_H