#ifndef FACTORY_H
#define FACTORY_H

#include "type_util.h"
#include <memory>
#include <unordered_map>

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template<class Base, class ...Args>
struct Factory {
private:
    template<class ...T>
    static auto *BuildImpl(const std::string &name, T &&...t) {
        if (GetTable().find(name) == GetTable().end()) {
            fprintf(stderr, "build fail: %s no exist in %s.\n", name.c_str(),
                    Type<Factory<Base, Args...>>::c_str());
            return (Base *) nullptr;
        } else {
            return GetTable()[name](std::forward<T>(t)...);
        }
    }

    static inline auto &GetTable() {
        static std::unordered_map<std::string, Base *(*)(Args...)> table;
        return table;
    }

    template<class>
    struct raw_ptr;
public:

    template<class Der>
    static bool Register() {
        constexpr bool validation = std::is_default_constructible_v<Der> || std::is_constructible_v<Der, Args...>;
        static_assert(validation, "no Der(Args...) constructor function exist\n");
        if constexpr (validation) {
            static_assert(std::is_base_of_v<Base, Der>, "no derivative relationship between this two types.\n");
            auto hash_code = std::string(Type<Der>::name);
            if (GetTable().find(hash_code) == GetTable().end()) {
                printf("register [ %s ] to  [ %s ] as [ %s ].\n",
                       Type<Der>::c_str(),
                       Type<Factory<Base, Args...>>::c_str(),
                       Type<typename std::decay_t<decltype(GetTable())>::mapped_type>::c_str());
                GetTable()[hash_code] = [](Args ...args) -> Base * { return new Der(args...); };
            } else {
                fprintf(stderr, "[ %s ] has been registered to [ %s ] as [ %s ].\n",
                        Type<Der>::c_str(),
                        Type<Factory<Base, Args...>>::c_str(),
                        Type<typename std::decay_t<decltype(GetTable())>::mapped_type>::c_str());
            }
            return true;
        } else
            return false;
    }

    template<template<class> class PtrType=raw_ptr, class ...T>
    static auto Build(const std::string &name, T &&...t) {
        if constexpr (std::is_same_v<PtrType<Base>, raw_ptr<Base>>) return BuildImpl(name, std::forward<T>(t)...);
        else return PtrType<Base>(BuildImpl(name, std::forward<T>(t)...));
    }

    template<template<class> class PtrType=raw_ptr, class ...T>
    static auto BuildT(const std::string &name, T &&...t) {
        return Build<PtrType>(TTypeTrait<Base>::with_template_arg(name), std::forward<T>(t)...);
    }
};

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template<class Base, class Der>
struct AutoRegister {
    template<class ...Args>
    struct OverLoad {
        OverLoad() { (void) registered; /*必不可少，显式调用才会生成代码*/}

        static bool registered;
    };
};
template<class Base, class Der>
template<class ...Args>
bool AutoRegister<Base, Der>::OverLoad<Args...>::registered = Factory<Base, Args...>::template Register<Der>();

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#define REGISTER_TO_FACTORY_IMPL(name, base, der, ...)\
namespace{static bool REGISTER_TO_FACTORY_UNIQUE_ID(name) = Factory<base,__VA_ARGS__>::Register<der>();}

#define REGISTER_TO_FACTORY(base, der, ...) \
REGISTER_TO_FACTORY_IMPL(reigster,REGISTER_REMOVE_PARENTHESES(base),REGISTER_REMOVE_PARENTHESES(der),__VA_ARGS__)




#endif