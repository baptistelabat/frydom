// =============================================================================
// FRyDoM - frydom-ce.gitlab.host.io
//
// Copyright (c) D-ICE Engineering and Ecole Centrale de Nantes (LHEEA lab.)
// All rights reserved.
//
// Use of this source code is governed by a GPLv3 license that can be found
// in the LICENSE file of FRyDOM.
//
// =============================================================================


#ifndef FRYDOM_FREXCEPTION_H
#define FRYDOM_FREXCEPTION_H

#include <exception>
#include <string>

namespace frydom {

    /**
     * \class Acceleration
     * \brief Class for defining exceptions.
     */
    class FrException : public std::exception {

    public:

        explicit FrException(const char* message) : msg_(message) {}

        explicit FrException(const std::string& message) : msg_(message) {}

        virtual ~FrException() throw () {}

        const char* what() const throw() override {
            return msg_.c_str();
        }

    protected:
        std::string msg_;

    };

} // end namespace frydom


#endif //FRYDOM_FREXCEPTION_H
