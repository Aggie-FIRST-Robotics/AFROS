#ifndef AFROS_CORE_REGISTRY_H
#define AFROS_CORE_REGISTRY_H

#include "afros_core/error_val.h"

#include <vector>
#include <boost/thread/shared_mutex.hpp>

namespace afros_core{
    template<typename T>
    struct registry_entry{
        uint32_t next_free;
        T* data;

        registry_entry<T> copy();
    };

    template<typename T>
    registry_entry<T> registry_entry<T>::copy(){
        registry_entry out{};
        out.next_free = next_free;
        out.data = malloc(sizeof(T));
        *out.data = *data;
    }

    enum registry_error{
        OUT_OF_BOUNDS,
        NOT_FOUND,
        ALREADY_EXISTS,
        OUT_OF_SPACE,
        ALREADY_REMOVED,
        EMPTY_ENTRY,
    };

    template<typename T>
    class registry{
        std::vector<registry_entry<T>> vec;
        uint32_t first_free;
        uint32_t size;

        boost::shared_mutex mutex;

    public:
        registry();
        explicit registry(uint32_t initial_capacity);

        registry(const registry&);
        registry(registry&&) = delete;
        ~registry();
        registry& operator=(const registry&) = delete;
        registry& operator=(registry&&) = delete;

        uint32_t get_size();
        error_val <uint32_t, registry_error> find(const T& value);
        error_val <T, registry_error> get(uint32_t value);

        error_val <uint32_t, registry_error> add(T value);
        error_val <void_t, registry_error> remove(const T& value);
        error_val <void_t, registry_error> remove(uint32_t value);
    };

    template<typename T>
    registry<T>::registry() = default;

    template<typename T>
    registry<T>::registry(uint32_t initial_capacity) : vec{initial_capacity}, first_free(0), size(0){}

    template<typename T>
    registry<T>::registry(const registry& other) : first_free(0), size(0){
        boost::shared_lock<boost::shared_mutex> lock{other.mutex};
        vec.reserve(other.vec.size());
        for(auto& entry : other.vec){
            vec.push_back(entry.copy());
        }
        first_free = other.first_free;
        size = other.size;
    }

    template<typename T>
    registry<T>::~registry(){
        boost::unique_lock<boost::shared_mutex> lock{mutex};
        for(auto& entry : vec){
            delete (entry.data);
        }
    }

    template<typename T>
    error_val <uint32_t, registry_error> registry<T>::find(const T& value){
        boost::shared_lock<boost::shared_mutex> lock{mutex};
        for(uint32_t x = 0; x < vec.size(); ++x){
            auto& entry = vec.at(x);
            if(entry.data != nullptr && *entry.data == value){
                return error_val<uint32_t, registry_error>::value(x);
            }
        }
        return error_val<uint32_t, registry_error>::error(NOT_FOUND);
    }

    template<typename T>
    error_val <T, registry_error> registry<T>::get(uint32_t value){
        boost::shared_lock<boost::shared_mutex> lock{mutex};
        if(vec.size() <= value){
            return error_val<T, registry_error>::error(OUT_OF_BOUNDS);
        }
        if(vec.at(value).data == nullptr){
            return error_val<T, registry_error>::error(EMPTY_ENTRY);
        }
        return error_val<T, registry_error>::value(*vec.at(value).data);
    }

    template<typename T>
    error_val <uint32_t, registry_error> registry<T>::add(T value){
        if(!find(value).equals_error(NOT_FOUND)){
            return error_val<uint32_t, registry_error>::error(ALREADY_EXISTS);
        }
        boost::unique_lock<boost::shared_mutex> lock{mutex};
        if(size == UINT32_MAX){
            return error_val<uint32_t, registry_error>::error(OUT_OF_SPACE);
        }
        if(first_free >= vec.size()){
            first_free = vec.size();
            vec.emplace_back();
            vec.at(first_free).next_free = first_free + 1;
        }
        auto out = first_free;
        auto& entry = vec.at(first_free);
        first_free = entry.next_free;
        entry.data = reinterpret_cast<T*>(malloc(sizeof(T)));
        *entry.data = std::move(value);
        ++size;
        return error_val<uint32_t, registry_error>::value(out);
    }

    template<typename T>
    uint32_t registry<T>::get_size(){
        boost::shared_lock<boost::shared_mutex> lock{mutex};
        return size;
    }

    template<typename T>
    error_val <void_t, registry_error> registry<T>::remove(const T& value){
        int64_t found = find(value);
        if(found < 0){
            return error_val<void_t, registry_error>::error(NOT_FOUND);
        }
        return remove(uint32_t(found));
    }

    template<typename T>
    error_val <void_t, registry_error> registry<T>::remove(uint32_t value){
        boost::unique_lock<boost::shared_mutex> lock{mutex};
        if(value >= vec.size()){
            return error_val<void_t, registry_error>::error(OUT_OF_BOUNDS);
        }
        if(vec.at(value).data == nullptr){
            return error_val<void_t, registry_error>::error(ALREADY_REMOVED);
        }
        delete (vec.at(value).data);
        vec.at(value).data = nullptr;
        if(value < first_free){
            vec.at(value).next_free = first_free;
            first_free = value;
            --size;
            return error_val<void_t, registry_error>::value();
        }
        auto* entry = &vec.at(first_free);
        while(entry->next_free > value){
            entry = &vec.at(entry->next_free);
        }
        vec.at(value).next_free = entry->next_free;
        entry->next_free = value;
        --size;
        return error_val<void_t, registry_error>::value();
    }
}

#endif //AFROS_CORE_REGISTRY_H
