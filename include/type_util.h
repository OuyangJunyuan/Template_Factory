#ifndef FACTORY_TYPE_UTILS_H
#define FACTORY_TYPE_UTILS_H

#include <tuple>
#include <string>
#include <string_view>


template<typename T>
struct Type {
private:
    static constexpr auto type_name_impl() noexcept {
        std::string_view name, prefix, suffix;
#ifdef __clang__
        name = __PRETTY_FUNCTION__;
    prefix = "static auto Type<T>::type_name_impl() [T = ";
    suffix = "]";
#elif defined(__GNUC__)
        name = __PRETTY_FUNCTION__;
        prefix = "static constexpr auto Type<T>::type_name_impl() [with T = ";
        suffix = "]";
#elif defined(_MSC_VER)
        name = __FUNCSIG__;
    prefix = "static auto __cdecl Type<T>::type_name_impl<";
    suffix = ">(void) noexcept";
#endif
        name.remove_prefix(prefix.size());
        name.remove_suffix(suffix.size());
        return name;
    }

public:
    static constexpr auto name = type_name_impl();
};

template<typename ...T>
struct Types {
private:
    static constexpr auto type_names_impl() noexcept {
        std::string_view name = Type<Types<T...>>::name, prefix = "Types";
        name.remove_prefix(prefix.size());
        return name;
    }

    static constexpr auto raw_type_names_impl() noexcept {
        std::string_view name = type_names_impl(), prefix = "<", suffix = ">";
        name.remove_prefix(prefix.size());
        name.remove_suffix(suffix.size());
        return name;
    }

public:

    static constexpr size_t size = sizeof...(T);
    template<size_t N>
    static constexpr auto name = Type<typename std::tuple_element<N, std::tuple<T...> >::type>::name;

    static constexpr auto names = type_names_impl();
    static constexpr auto raw_names = raw_type_names_impl();
};


#define DECORATE(name, ...) decorate<__VA_ARGS__>(name)

template<class  ...T>
inline std::string decorate(const std::string &name) { return name + std::string(Types<T...>::names); }

template<>
inline std::string decorate(const std::string &name) { return name; }


template<class T>
struct TTypeTrait {
    constexpr static size_t size = 0;
};

template<template<class> class TType, class ...T>
struct TTypeTrait<TType<T...>> {
private:
    template<size_t ...N>
    static std::string
    with_template_arg_helper(const std::string &name, std::index_sequence<N...>) {
        return decorate<arg_type<N>...>(name);
    }

public:
    constexpr static size_t size = sizeof ...(T);
    template<size_t N>
    using arg_type = typename std::tuple_element<N, std::tuple<T...>>::type;

    static std::string with_template_arg(const std::string &name) {
        return with_template_arg_helper(name, std::make_index_sequence<size>());
    }
};


#define REGISTER_TO_FACTORY_UNIQUE_ID_MERGE_IMPL(a, b) a ## b //合并用的主体
#define REGISTER_TO_FACTORY_UNIQUE_ID_MERGE(a, b) REGISTER_TO_FACTORY_UNIQUE_ID_MERGE_IMPL(a, b) //中间层
#define REGISTER_TO_FACTORY_UNIQUE_ID(name) REGISTER_TO_FACTORY_UNIQUE_ID_MERGE(name, __COUNTER__)

#define REGISTER_REMOVE_PARENTHESES_IMPL(X) X
#define REGISTER_REMOVE_PARENTHESES(X)  REGISTER_REMOVE_PARENTHESES_IMPL X
#endif