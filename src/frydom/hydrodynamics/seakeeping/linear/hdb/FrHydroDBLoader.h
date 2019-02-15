// ==========================================================================
// FRyDoM - frydom-ce.org
// 
// Copyright (c) Ecole Centrale de Nantes (LHEEA lab.) and D-ICE Engineering.
// All rights reserved.
// 
// Use of this source code is governed by a GPLv3 license that can be found
// in the LICENSE file of FRyDoM.
// 
// ==========================================================================


#ifndef FRYDOM_FRHYDRODBLOADER_H
#define FRYDOM_FRHYDRODBLOADER_H

#include <string>

namespace frydom {

    // Forward declaration
    class FrHydroDB;

    FrHydroDB LoadHDB5(std::string h5file);  // TODO: mettre dans fichier a part...








    /// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> REFACTORING

    // la methode fait  partie de FrHydroDB




}  // end namespace frydom

#endif //FRYDOM_FRHYDRODBLOADER_H
