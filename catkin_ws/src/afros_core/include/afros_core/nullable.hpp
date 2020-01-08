#ifndef AFROS_NULLABLE_HPP
#define AFROS_NULLABLE_HPP

#include "afros_core/error_val.hpp"

namespace afros_core{
    enum nullable_error{
        NULL_VALUE,
    };

    template<typename T>
    class nullable{
        T* value;

    public:
        nullable();
        nullable(const T& value); // NOLINT(google-explicit-constructor,hicpp-explicit-conversions)

        nullable(const nullable<T>& other);
        nullable(nullable<T>&& other) noexcept;
        ~nullable();
        nullable<T>& operator=(const nullable<T>& other);
        nullable<T>& operator=(nullable<T>&& other) noexcept;

        error_val<T&, nullable_error> operator*();
        error_val<T&, nullable_error> operator->();

        error_val<T&, nullable_error> get_value();
        bool is_null();
    };

    template<typename T>
    nullable<T>::nullable() : value(nullptr){}

    template<typename T>
    nullable<T>::nullable(const T& value) : value(new T{value}){}

    template<typename T>
    nullable<T>::nullable(const nullable<T>& other) : value(new T{*other.value}){}

    template<typename T>
    nullable<T>::nullable(nullable<T>&& other) noexcept : value(other.value){
        other.value = nullptr;
    }

    template<typename T>
    nullable<T>::~nullable(){
        delete(value);
    }

    template<typename T>
    nullable<T>& nullable<T>::operator=(const nullable<T>& other){
        if(this == &other){
            return *this;
        }
        delete(value);
        value = new T{*other.value};
        return *this;
    }

    template<typename T>
    nullable<T>& nullable<T>::operator=(nullable<T>&& other) noexcept{
        if(this == &other){
            return *this;
        }
        delete(value);
        value = other.value;
        other.value = nullptr;
        return *this;
    }

    template<typename T>
    error_val<T&, nullable_error> nullable<T>::operator*(){
        if(value == nullptr){
            return error_val<T&, nullable_error>{NULL_VALUE};
        }
        return error_val<T&, nullable_error>{*value};
    }

    template<typename T>
    error_val<T&, nullable_error> nullable<T>::operator->(){
        if(value == nullptr){
            return error_val<T&, nullable_error>{NULL_VALUE};
        }
        return error_val<T&, nullable_error>{*value};
    }

    template<typename T>
    error_val<T&, nullable_error> nullable<T>::get_value(){
        return operator*();
    }

    template<typename T>
    bool nullable<T>::is_null(){
        return value == nullptr;
    }
}

#endif //AFROS_NULLABLE_HPP
