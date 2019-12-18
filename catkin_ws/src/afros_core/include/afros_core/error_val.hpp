#ifndef AFROS_CORE_ERROR_VAL_H
#define AFROS_CORE_ERROR_VAL_H

#include <functional>

/**
 * Usgae:
 *  AFROS_CORE_ERROR_CHECK(variable_to_be_made, error_val_call, error_variable){
 *      ENTRY IF ERROR
 *  }
 */
#define AFROS_CORE_ERROR_CHECK(x, y, z)\
    auto x = y.get(z);\
    if(x == nullptr)

namespace afros_core{
    struct void_t{
    };

    template<typename V, typename E>
    class error_val{
        bool _is_error;
        union{
            V _value;
            E _error;
        };

    public:
        explicit error_val(V value);
        error_val(E error, bool dummy);

        error_val(const error_val<V, E>&);
        error_val(error_val<V, E>&&) noexcept;
        ~error_val();
        error_val<V, E>& operator=(const error_val<V, E>&);
        error_val<V, E>& operator=(error_val<V, E>&&) noexcept;

        static error_val<V, E> value(V value);
        static error_val<V, E> value();
        static error_val<V, E> error(E error);
        static error_val<V, E> error();

        V* get(E& error);

        bool is_error();
        bool is_error(E& error);
        bool equals_error(E error);
        bool equals_value(V value);
    };

    template<typename V, typename E>
    error_val<V, E> error_val<V, E>::value(V value){
        return error_val<V, E>{value};
    }

    template<typename V, typename E>
    V* error_val<V, E>::get(E& error){
        if(_is_error){
            error = _error;
            return nullptr;
        }
        return &_value;
    }

    template<typename V, typename E>
    error_val<V, E>::error_val(V value) : _is_error(false), _value(value){}

    template<typename V, typename E>
    error_val<V, E> error_val<V, E>::error(E error){
        return error_val<V, E>(error, false);
    }

    template<typename V, typename E>
    error_val<V, E>::error_val(E error, bool dummy) : _is_error(true), _error(error){}

    template<typename V, typename E>
    bool error_val<V, E>::equals_error(E error){
        if(!_is_error){
            return false;
        }
        return _error == error;
    }

    template<typename V, typename E>
    bool error_val<V, E>::is_error(){
        return _is_error;
    }

    template<typename V, typename E>
    bool error_val<V, E>::equals_value(V value){
        if(is_error()){
            return false;
        }
        return _value == value;
    }

    template<typename V, typename E>
    error_val<V, E> error_val<V, E>::value(){
        return error_val<V, E>::value(V());
    }

    template<typename V, typename E>
    error_val<V, E> error_val<V, E>::error(){
        return error_val<V, E>::error(E());
    }

    template<typename V, typename E>
    error_val<V, E>::~error_val(){
        if(is_error()){
            _error.~E();
        }
        else{
            _value.~V();
        }
    }

    template<typename V, typename E>
    bool error_val<V, E>::is_error(E& error){
        if(_is_error){
            error = _error;
            return true;
        }
        return false;
    }

    template<typename V, typename E>
    error_val<V, E>::error_val(const error_val<V, E>& other) : _is_error(other._is_error){
        if(_is_error){
            _error = other._error;
        }
        else{
            _value = other._value;
        }
    }

    template<typename V, typename E>
    error_val<V, E>::error_val(error_val<V, E>&& other) noexcept : _is_error(other._is_error){
        if(_is_error){
            _error = std::move(other._error);
        }
        else{
            _value = std::move(other._value);
        }
    }

    template<typename V, typename E>
    error_val<V, E>& error_val<V, E>::operator=(const error_val<V, E>& other){
        if(this == &other){
            return *this;
        }
        ~error_val();
        _is_error = other._is_error;
        if(_is_error){
            _error = other._error;
        }
        else{
            _value = other._value;
        }
    }

    template<typename V, typename E>
    error_val<V, E>& error_val<V, E>::operator=(error_val<V, E>&& other) noexcept{
        if(this == &other){
            return *this;
        }
        ~error_val();
        _is_error = other._is_error;
        if(_is_error){
            _error = std::move(other._error);
        }
        else{
            _value = std::move(other._value);
        }
    }

}

#endif //AFROS_CORE_ERROR_VAL_H
