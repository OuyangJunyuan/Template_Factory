#pragma once

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>
#include <cassert>
#include <cstdlib>
#include "type_util.h"

namespace spiritsaway::entity_component_event {
template<typename T>
using raw_ptr_t = T *;
template<typename T>
using shr_ptr_t = std::shared_ptr<T>;
template<typename T>
using unq_ptr_t = std::unique_ptr<T>;

template<template<typename...> class ptr_t, typename T, class... Args>
struct base_creator_func {
    using raw_return_type = ptr_t<T>;
    template<typename D>
    using return_type = ptr_t<D>;

private:
    using func_type = raw_return_type(*)(Args...);

public:
    base_creator_func(func_type func_ptr = nullptr) : m_func_ptr(func_ptr) {
    }

    template<class D>
    static base_creator_func instance() {
        func_type temp = [](Args... args) -> raw_return_type {
            return return_type<D>(new D(args...));
        };
        return base_creator_func(temp);
    }

    template<typename D>
    return_type<D> create(Args &&... ts) {
        return return_type<D>(m_func_ptr(std::forward<Args>(ts)...));
    }

private:
    func_type m_func_ptr;
};

/// 偏特化
template<class T, class... Args>
struct base_creator_func<unq_ptr_t, T, Args...> {
    using raw_return_type = unq_ptr_t<T>;
    template<typename D>
    using return_type = unq_ptr_t<D>;

private:
    using func_type = raw_return_type(*)(Args...);

public:
    base_creator_func(func_type func_ptr = nullptr) : m_func_ptr(func_ptr) {
    }

    template<class D>
    static base_creator_func instance() {
        func_type temp = [](Args... args) -> raw_return_type {
            return std::make_unique<T>(std::forward<Args>(args)...);
        };
        return base_creator_func(temp);
    }

    template<typename D>
    return_type<D> create(Args &&... ts) {
        auto temp = m_func_ptr(std::forward<Args>(ts)...);
        return return_type<D>{dynamic_cast<D *>(temp.release())};
    }

private:
    func_type m_func_ptr;
};

template<class T, class... Args>
struct base_creator_func<shr_ptr_t, T, Args...> {
    using raw_return_type = shr_ptr_t<T>;
    template<typename D>
    using return_type = shr_ptr_t<D>;

private:
    using func_type = raw_return_type(*)(Args...);

public:
    base_creator_func(func_type func_ptr = nullptr) : m_func_ptr(func_ptr) {
    }

    template<class D>
    static base_creator_func instance() {
        func_type temp = [](Args... args) -> raw_return_type {
            return std::make_shared<D>(std::forward<Args>(args)...);
        };
        return base_creator_func(temp);
    }

    template<typename D>
    return_type<D> create(Args &&... ts) {
        auto temp = m_func_ptr(std::forward<Args>(ts)...);
        return_type<D> derivate = std::dynamic_pointer_cast<D>(temp);
        return derivate;
    }

private:
    func_type m_func_ptr;
};

template<class T, class... Args>
struct base_creator_func<raw_ptr_t, T, Args...> {
    using raw_return_type = raw_ptr_t<T>;
    template<typename D>
    using return_type = raw_ptr_t<D>;


private:
    using func_type = raw_return_type(*)(Args...);

public:
    base_creator_func(func_type func_ptr = nullptr) : m_func_ptr(func_ptr) {
    }

    template<class D>
    static base_creator_func instance() {
        func_type temp = [](Args... args) -> raw_return_type {
            return new T(std::forward<Args>(args)...);
        };
        return base_creator_func(temp);
    }

    template<typename D, class... Ts>
    return_type<D> create(Ts &&... ts) {
        return dynamic_cast<return_type<D>>(m_func_ptr(std::forward<Ts>(ts)...));
    }

private:
    func_type m_func_ptr;
};


template<typename Base, typename CreatorFunc>
struct creator_by_typeid {
private:
    static std::unordered_map<std::size_t, CreatorFunc> data;
public:
    using FuncT = CreatorFunc;

    template<typename T>
    static bool add(CreatorFunc f) {
        static_assert(std::is_base_of_v<Base, T>, "T should be derivate of Base");
        auto cur_hash = base_type_hash<Base>::template hash<T>();
        auto &data = GetData();

        bool result = data.find(cur_hash) != data.end();
        data[cur_hash] = f;
        return result;
    }

    template<typename T, class... Ts>
    static auto make(Ts &&... args) {
        static_assert(std::is_base_of_v<Base, T>, "T should be derivate of Base");
        auto cur_hash = base_type_hash<Base>::template hash<T>();
        auto &data = GetData();

        auto cur_iter = data.find(cur_hash);
        if (cur_iter == data.end()) {
            assert(false);
            return typename FuncT::template return_type<T>(nullptr);
        } else {
            auto ret = cur_iter->second.template create<T>(std::forward<Ts>(args)...);
            return ret;
        }
    }

    static std::unordered_map<std::size_t, CreatorFunc> &GetData() {
        static std::unordered_map<std::size_t, CreatorFunc> data;
        return data;
    }
};

template<typename Base, typename CreatorFunc>
struct creator_by_typename {
private:
public:
    using FuncT = CreatorFunc;

    template<typename T>
    static bool add(CreatorFunc f) {
        static_assert(std::is_base_of_v<Base, T>, "T should be derivate of Base");
        auto cur_hash = T::class_name();
        auto &data = GetData();
        bool result = data.find(cur_hash) != data.end();
        data[cur_hash] = f;
        return result;
    }

    template<class... Ts>
    static auto make_by_name(std::string_view cur_name, Ts &&... args) {
        auto &data = GetData();
        auto cur_iter = data.find(cur_name);
        if (cur_iter == data.end()) {
            assert(false);
            return typename FuncT::template return_type<Base>();
        } else {
            return cur_iter->second.template create<Base>(std::forward<Ts>(args)...);
        }
    }

    static std::unordered_map<std::string_view, CreatorFunc> &GetData() {
        static std::unordered_map<std::string_view, CreatorFunc> data;
        return data;
    }
};


template<typename CreateMapT>
struct type_registration {
    template<class derived>
    static bool register_derived() {
        auto factory = CreateMapT::FuncT::template instance<derived>();
        CreateMapT::template add<derived>(factory);
        return true;
    }
};

///通过可变参数模板，定义了工厂函数的返回类型和参数列表
template<template<typename ...> class ptr_t,
        class Base, class... Args>
struct basic_poly_factory {
    using create_func_T = base_creator_func<ptr_t, Base, Args...>;
public:
    ///
    template<class D, class... T>
    static typename create_func_T::template return_type<D> make(T &&... args) {
        return creator_by_typeid<Base, create_func_T>::template make<D>(std::forward<T>(args)...);
    }

    template<class... T>
    static typename create_func_T::template return_type<Base> make_by_name(const std::string_view name, T &&... args) {
        return creator_by_typename<Base, create_func_T>::template make_by_name(name, std::forward<T>(args)...);
    }

    template<class ...C, class... T>
    static typename create_func_T::template return_type<Base>
    make_by_name_template(const std::string_view name, T &&... args) {
        return creator_by_typename<Base, create_func_T>::template make_by_name(name, std::forward<T>(args)...);
    }

    template<class T, class B = Base>
    struct sub_class : public B {
        friend T;

        /***
         * 自动注册函数中：
         * 通过注册函数这个模板函数，其模板为构造方法，包括create_bt_type_id和create_by_typename
         * 不同的构造方法需要使用不同构造函数，因此不同构造方法内有不同的 map
         */
        static bool trigger() {
            static_assert(std::is_final_v<T>, "sub class should have final specified");
            type_registration<creator_by_typeid<Base, create_func_T>>::template register_derived<T>();
            type_registration<creator_by_typename<Base, create_func_T>>::template register_derived<T>();
            return true;
        }

        /// 自动注册的关键，通过对静态变量的构造，赋值时候调用注册函数 trigger。
        static bool registered;

    private:
        sub_class(Args... args) : B(std::forward<Args>(args)...) {
            (void) registered;
        }
    };

    friend Base;

private:
    using FuncType = ptr_t<Base> (*)(Args...);

    basic_poly_factory() = default;
};

template<template<typename ...> class ptr_t, class Base, class... Args>
template<class T, class B>
bool basic_poly_factory<ptr_t, Base, Args...>::sub_class<T, B>::registered
        = basic_poly_factory<ptr_t, Base, Args...>::sub_class<T, B>::trigger();


} // namespace spiritsaway::entity_mesh