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
    printf("%s\n", cur_name.c_str());
    auto &data = register_table<Base, std::decay_t<Args>...>();
    auto cur_iter = data.find(cur_name);
    if (cur_iter == data.end()) assert(false);
    else return cur_iter->second(args...);
}

template<template<class> class BaseT, class ...T, class ...A>
static auto *BuildT(const std::string &name, const A &... a) { return Build<BaseT<T...>>(decorate<T...>(name), a...); }

template<class BaseT, class ...Args>
static auto *BuildT(const std::string &name, const Args &... args) {
    if constexpr (TTypeTrait<BaseT>::size == 0) { return Build<BaseT>(name, args...); }
    else return Build<BaseT>(TTypeTrait<BaseT>::with_template_arg(name), args...);
}


#define REGISTER_TO_FACTORY_IMPL(name, base, der, ...)\
namespace{static bool REGISTER_TO_FACTORY_UNIQUE_ID(trigger_register) = Register<base,der>::OverLoad<__VA_ARGS__>();}

#define REGISTER_TO_FACTORY(base, der, ...) \
REGISTER_TO_FACTORY_IMPL(trigger_reigster,  \
REGISTER_REMOVE_PARENTHESES(base),     \
REGISTER_REMOVE_PARENTHESES(der),      \
__VA_ARGS__)



#endif