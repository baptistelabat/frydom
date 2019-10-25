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

#ifndef FRYDOM_GITSHA1_H
#define FRYDOM_GITSHA1_H

#include <string>

namespace frydom {

  // TODO: mettre un warning si des modifs ne sont pas commitees...

  std::string GetGitSHA1();

} // end namespace frydom

#endif //FRYDOM_GITSHA1_H
