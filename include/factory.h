#ifndef FACTORY_H
#define FACTORY_H

#include "type_util.h"
#include <memory>
#include <unordered_map>
#include <cassert>

template<class Base, class ...Args>
static auto &register_table() {
    static std::unordered_map<std::string, Base *(*)(const std::decay_t<Args> &...)> data;
    return data;
}

template<class Base, class Der>
struct Register {
    template<class ...Args>
    static bool OverLoad() {
        static_assert(std::is_base_of_v<Base, Der>, "There is no derivative relationship between this two types.");
        auto hash = std::string(Type<Der>::name);
        auto &table = register_table<Base, std::decay_t<Args>...>();
        table[hash] = [](const Args &...args) -> Base * { return new Der(args...); };
        return table.find(hash) != table.end();
    }
};


template<class Base, class ...Args>
static Base *Build(const std::string &cur_name, const Args &... args) {
    auto &data = register_table<Base, std::decay_t<Args>...>();
    auto cur_iter = data.find(cur_name);
    if (cur_iter == data.end()) assert(false);
    else return cur_iter->second(args...);
}

template<template<class> class BaseT, class ...T, class ...Args>
static BaseT<T...> *BuildT(const std::string &cur_name, const Args &... args) {
    auto &data = register_table<BaseT<T...>, std::decay_t<Args>...>();
    auto cur_iter = data.find(rename<T...>(cur_name));
    if (cur_iter == data.end()) assert(false);
    else return cur_iter->second(args...);
}


#define REGISTER_TO_FACTORY_IMPL(name, base, der, ...)\
namespace{static bool REGISTER_TO_FACTORY_UNIQUE_ID(trigger_register) = Register<base,der>::OverLoad<__VA_ARGS__>();}

#define REGISTER_TO_FACTORY(base, der, ...) REGISTER_TO_FACTORY_IMPL(trigger_reigster, base, der, __VA_ARGS__)
/*#pragma once

#include "type_util.h"
#include <memory>
#include <unordered_map>
#include <cassert>

template<class Base>
struct Factory {
    template<class ...Args>
    struct OverLoad {
        using Func = Base *(*)(Args...);

        static std::unordered_map<std::string, Func> &register_table() {
            static std::unordered_map<std::string, Func> data;
            return data;
        }

        template<class Der>
        static bool add() {
            printf("register: constructor %s\n", __PRETTY_FUNCTION__);
            static_assert(std::is_base_of_v<Base, Der>, "T should be derivate of Base");
            auto hash = std::string(Type<Der>::name);
            auto &table = register_table();
            table[hash] = [](Args...args) -> Base * { return new Der(std::forward<Args>(args)...); };
            return table.find(hash) != table.end();
        }

        template<class Der>
        struct Trigger {
            Trigger() { (void) registered; }

            static bool registered;
        };

        static auto *Build(const std::string &cur_name, Args... args) {
            auto &data = OverLoad<Args...>::register_table();
            auto cur_iter = data.find(cur_name);
            printf("Build %s\n", Type<decltype(cur_iter->second)>::name.data());
            if (cur_iter == data.end()) assert(false);
            else return cur_iter->second(std::forward<Args>(args)...);
        }
    };

    template<class ...Args>
    static auto *Build(const std::string &cur_name, Args... args) {
        return OverLoad<Args...>::Build(cur_name, std::forward<Args>(args)...);
    }
};

template<class Base>
template<class ...Args>
template<class Der>
bool Factory<Base>::OverLoad<Args...>::Trigger<Der>::registered = Factory<Base>::OverLoad<Args...>::add<Der>();


#define REGISTER_TO_FACTORY_UNIQUE_ID_MERGE_IMPL(a, b) a ## b //合并用的主体
#define REGISTER_TO_FACTORY_UNIQUE_ID_MERGE(a, b) REGISTER_TO_FACTORY_UNIQUE_ID_MERGE_IMPL(a, b) //中间层
#define REGISTER_TO_FACTORY_UNIQUE_ID(name) REGISTER_TO_FACTORY_UNIQUE_ID_MERGE(name, __COUNTER__)

#define REGISTER_TO_FACTORY_IMPL(name, base, der, ...)\
namespace{static Factory<base>::template OverLoad<__VA_ARGS__>::template Trigger<der> \
REGISTER_TO_FACTORY_UNIQUE_ID(trigger_register);}

#define REGISTER_TO_FACTORY(base, der, ...) REGISTER_TO_FACTORY_IMPL(trigger_reigster,base,der,__VA_ARGS__)*/

#endif