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


#ifndef FRYDOM_FREXCEPTION_H
#define FRYDOM_FREXCEPTION_H

#include <exception>
#include <string>

namespace frydom {

  /**
   * \class FrException
   * \brief Class for defining exceptions into FRyDoM
   */
  class FrException : public std::exception {

   protected:
    std::string msg_;

   public:

    explicit FrException(const char *message);

    explicit FrException(const std::string &message);

    ~FrException() throw() override;

    const char *what() const throw() override;

  };

} // end namespace frydom


#endif //FRYDOM_FREXCEPTION_H
