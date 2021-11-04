#include "factory.h"
#include <iostream>
#include <vector>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <boost/core/demangle.hpp>
#include <functional>

using PCXYZ = pcl::PointCloud<pcl::PointXYZ>;

template<class Base>
struct Factory {
    template<class ...Args>
    struct OverLoad {
        using Func = Base *(*)(Args...);

        static std::unordered_map<std::string_view, Func> &register_table() {
            static std::unordered_map<std::string_view, Func> data;
            return data;
        }

        template<class Der>
        static bool add() {
            static_assert(std::is_base_of_v<Base, Der>, "T should be derivate of Base");
            auto hash = full_type_name<Der>();
            printf("%s %s\n", std::string(full_type_name<Der>().data()).c_str(), std::string(hash.data()).c_str());
            auto &table = register_table();
            table[hash] = [](Args...args) -> Base * { return new Der(std::forward<Args...>(args...)); };
            return table.find(hash) != table.end();
        }
    };

    template<class ...Args>
    static auto *Build(std::string cur_name, Args... args) {
        auto &data = OverLoad<Args...>::register_table();
        auto cur_iter = data.find(cur_name);
        printf("%s\n", cur_name.c_str());
        if (cur_iter == data.end()) assert(false);
        else return cur_iter->second(std::forward<Args>(args)...);
    }

    template<class D, class ...Args>
    static auto Build(std::string cur_name, Args... args) { return dynamic_cast<D *>(Build(cur_name, args...)); }
};


template<class T>
class Remover {
public:
    virtual void
    filter(T t) = 0;

};

template<class T>
class PatchWorkRemover final : public Remover<T> {
public:
    PatchWorkRemover(const std::vector<double> config) {};

    void filter(T t) override {
        std::cout << "PatchWork Remover" << std::endl;
    }
};

template<class T>
class RansacRemover final : public Remover<T> {
public:
    RansacRemover(const std::vector<double> config) {};

    void filter(T t) override {
        std::cout << "Ransac Remover" << std::endl;
    };

};

class Cluster {
public:
    virtual void
    filter(int t) = 0;

};

class IntRemover final : public Cluster {
public:
    IntRemover(const std::vector<double> config) {};

    void filter(int t) override {
        std::cout << "IntRemover Remover" << std::endl;
    };
};


bool is_registered = Factory<Remover<pcl::PointXYZ>>::OverLoad<std::vector<double>>::add<PatchWorkRemover<pcl::PointXYZ>>();
bool is_registered_1 = Factory<Remover<pcl::PointXYZ>>::OverLoad<std::vector<double>>::add<RansacRemover<pcl::PointXYZ>>();
bool is_registered_2 = Factory<Cluster>::OverLoad<std::vector<double>>::add<IntRemover>();

template<typename ClassName>
struct template_traits {
    constexpr static size_t size = 0;
};

template<template<typename ...Args> class ClassName, typename ...Args>
struct template_traits<ClassName<Args...>> {
    template<size_t i>
    using type = typename std::tuple_element<i, std::tuple<Args...> >::type;
    constexpr static size_t size = sizeof...(Args);
    using class_type = ClassName<Args...>;
};


template<class Base, class ...Args, unsigned long... N>
static auto BuildWrapper(const std::string &name, Args ... args, std::index_sequence<N...> is) {
    return Factory<Base>::Build(append_template_name<typename template_traits<Base>::template type<N>...>(name),
                                args...);
}


template<class Base, class ...Args>
auto Build(const std::string &name, Args ...args) {
    if constexpr(template_traits<Base>::size == 0) {
        return Factory<Base>::Build(name, args...);
    } else {
        return BuildWrapper<Base, Args...>(name, args..., std::make_index_sequence<template_traits<Base>::size>());
    }
}




int main() {
    std::vector<double> data;
    auto a = Build<Remover<pcl::PointXYZ>>("PatchWorkRemover",data); //Factory<Remover<pcl::PointXYZ>>::Build(REGISTER_NAME("PatchWorkRemover", pcl::PointXYZ), data);
    auto b =  Build<Remover<pcl::PointXYZ>>("RansacRemover",data);//Factory<Remover<pcl::PointXYZ>>::Build(REGISTER_NAME("RansacRemover", pcl::PointXYZ), data);
    auto c =  Build<Cluster>("IntRemover",data); //Factory<Cluster>::Build(REGISTER_NAME("IntRemover"), data);

    a->filter(pcl::PointXYZ{1,2,3});
    b->filter(pcl::PointXYZ{1,2,3});
    c->filter(1);



//    std::cout << full_type_name<template_traits<Remover<pcl::PointXYZ>>::type<0>>() << std::endl;
//    std::cout << full_type_name<template_traits<Remover<pcl::PointXYZ>>::class_type>() << std::endl;
//    std::cout << template_traits<Cluster>::size << std::endl;
    return 0;
}