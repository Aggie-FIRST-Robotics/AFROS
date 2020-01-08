#ifndef AFROS_CODEABLE_HPP
#define AFROS_CODEABLE_HPP

#include "afros_core/raw_data.h"

namespace afros_core{
    class codeable{
    public:
        virtual void decode(const raw_data& data) = 0;
        virtual raw_data encode() = 0;
    };
}

#endif //AFROS_CODEABLE_HPP
