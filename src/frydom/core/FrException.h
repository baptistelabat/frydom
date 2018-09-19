//
// Created by frongere on 17/09/18.
//

#ifndef FRYDOM_FREXCEPTION_H
#define FRYDOM_FREXCEPTION_H

#include <exception>
#include <string>

namespace frydom {

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
