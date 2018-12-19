//
// Created by frongere on 11/01/18.
//

#ifndef FRYDOM_FRHYDRODBLOADER_H
#define FRYDOM_FRHYDRODBLOADER_H

#include <string>

namespace frydom {

    // Forward declaration
    class FrHydroDB;

    FrHydroDB LoadHDB5(std::string h5file);  // TODO: mettre dans fichier a part...

}  // end namespace frydom

#endif //FRYDOM_FRHYDRODBLOADER_H
