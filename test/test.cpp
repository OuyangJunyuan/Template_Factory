//
// Created by ou on 2021/11/4.
//
#include "factory.h"
#include <gtest/gtest.h>
#include <dbg.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

using PCXYZ = pcl::PointCloud<pcl::PointXYZ>;

namespace pc_utils {

template<class PointT, class ...T>
struct Cluster {
    virtual void Segment(const pcl::PointCloud<PointT> &input, std::vector<int> &cluster, std::vector<int> &id) = 0;
};


template<class PointT>
struct CurvedVoxel :
        Cluster<PointT>,
        AutoRegister<Cluster<PointT>, CurvedVoxel<PointT>>::template OverLoad<>,
        AutoRegister<Cluster<PointT>, CurvedVoxel<PointT>>::template OverLoad<const std::vector<double> &> {
    CurvedVoxel() = default;

    explicit CurvedVoxel(const std::vector<double> &config) {}

    void Segment(const pcl::PointCloud<PointT> &input, std::vector<int> &cluster, std::vector<int> &id) {
        printf("CurvedVoxel -> Segment()\n");
    };
};

template
struct CurvedVoxel<pcl::PointXYZ>;


template<class PointT>
struct Euclidean : Cluster<PointT>, AutoRegister<Cluster<PointT>, Euclidean<PointT>>::template OverLoad<> {
    Euclidean() = default;

    explicit Euclidean(const std::vector<double> &config) {}

    void Segment(const pcl::PointCloud<PointT> &input, std::vector<int> &cluster, std::vector<int> &id) override {
        printf("Euclidean -> Segment()\n");
    };
};
REGISTER_TO_FACTORY((Cluster<pcl::PointXYZ>), (Euclidean<pcl::PointXYZ>), const std::vector<double>&)

template
struct Euclidean<pcl::PointXYZ>;
}


using namespace pc_utils;
using namespace pcl;
using FactoryVB = Factory<Cluster<PointXYZ>, const std::vector<double> &>;
using FactoryDe = Factory<Cluster<PointXYZ>>;

std::vector<double> config;
std::vector<int> result, id;
const std::string name1 = "pc_utils::Euclidean", name2 = "pc_utils::CurvedVoxel";

TEST(test, test1) {
    if (auto cluster = FactoryDe::Build(decorate<PointXYZ>(name1))) {
        cluster->Segment(PCXYZ{}, result, id);
        printf("%s\n", Type<decltype(cluster)>::c_str());
    }
}

TEST(test, test2) {
    if (auto cluster = FactoryVB::BuildT<std::shared_ptr>(name1, config)) {
        cluster->Segment(PCXYZ{}, result, id);
        printf("%s\n", Type<decltype(cluster)>::c_str());
    }
}

TEST(test, test3) {
    if (auto cluster = FactoryVB::BuildT<std::unique_ptr>(name2, config)) {
        cluster->Segment(PCXYZ{}, result, id);
        printf("%s\n", Type<decltype(cluster)>::c_str());
    }
}

/*
namespace variadic_toolbox {
template<unsigned count,
        template<unsigned...> class meta_functor, unsigned...  indices>
struct apply_range {
    typedef typename apply_range<count - 1, meta_functor, count - 1, indices...>::result result;
};

template<template<unsigned...> class meta_functor, unsigned...  indices>
struct apply_range<0, meta_functor, indices...> {
    typedef typename meta_functor<indices...>::result result;
};
}

namespace compile_time {
template<char...  str>
struct string {
    static constexpr const char chars[sizeof...(str) + 1] = {str..., '\0'};
};

template<char...  str>
constexpr const char  string<str...>::chars[sizeof...(str) + 1];
}

namespace compile_time {
template<typename lambda_str_type>
struct string_builder {
    template<unsigned... indices>
    struct produce {
        typedef string<lambda_str_type{}.chars[indices]...> result;
    };
};
}

#define  CSTRING(string_literal)                                                        \
    []{                                                                                 \
        struct  constexpr_string_type { const char * chars = string_literal; };         \
        return  variadic_toolbox::apply_range<sizeof(string_literal)-1,                 \
            compile_time::string_builder<constexpr_string_type>::produce>::result{};    \
    }()
namespace compile_time {
template<char...  str0, char...  str1>
string<str0..., str1...> operator*(string<str0...>, string<str1...>) {
    return {};
}
}

TEST(string_test, string_test1) {
    constexpr auto str0 = CSTRING("hello");
    constexpr auto str1 = CSTRING(" world");

    std::cout << "runtime concat: " << str0.chars << str1.chars << "\n <=> \n";
    std::cout << "compile concat: " << (str0 * str1).chars << std::endl;
}

namespace blog {
typedef long long hash64;
namespace const_expr {
constexpr hash64 prime = 0x100000001B3ull;
constexpr hash64 basis = 0xCBF29CE484222325ull;
}

constexpr hash64 make_hash_static(char const *str) {
    return (*str) ? (*(str + 1)) ? (((*str) * const_expr::prime + const_expr::basis) ^ make_hash_static(str + 1)) : (
            (*str) * const_expr::prime + const_expr::basis) : 0;
}

constexpr hash64 operator "" _hash(char const *p, size_t) {
    return make_hash_static(p);
}

template<char> using charDummy = char;

template<int N>
constexpr char at(const char *a) { return a[N]; }

template<int... dummy>
struct F {
    const char Name[sizeof...(dummy) + 1];
    const hash64 Hash;
    const int Length;
    const int Size;

    constexpr F(const char *a) : Name{at<dummy>(a)..., 0}, Length(sizeof...(dummy)), Size(sizeof...(dummy) + 1),
                                 Hash(a[0] * const_expr::prime + const_expr::basis) {}

    constexpr F(hash64 h, charDummy<dummy>... a) : Name{a..., 0}, Length(sizeof...(dummy)), Size(sizeof...(dummy) + 1),
                                                   Hash(h) {}

    constexpr F(const F &a) : Name{a.Name[dummy]..., 0}, Length(a.Length), Size(a.Size), Hash(a.Hash) {}

    template<int... dummyB>
    constexpr F<dummy..., sizeof...(dummy) + dummyB...> operator+(F<dummyB...> b) const {
        return {this->Hash ^ b.Hash, this->Name[dummy]..., b.Name[dummyB]...};
    }

    operator const char *() const { return Name; }
};

template<int I>
struct get_string {
    constexpr static auto g(const char *a) -> decltype(get_string<I - 1>::g(a) + F<0>(a + I)) {
        return get_string<I - 1>::g(a) + F<0>(a + I);
    }
};

template<>
struct get_string<0> {
    constexpr static F<0> g(const char *a) {
        return {a};
    }
};

template<int I>
constexpr auto str(const char(&a)[I]) -> decltype(get_string<I - 2>::g(a)) {
    return get_string<I - 2>::g(a);
}
}
TEST(test, test_str) {
    constexpr auto s = blog::str("abc") +  blog::str("edf");
    std::cout << s.Name << std::endl;
    std::cout << s.Hash << std::endl;
    std::cout << s.Length << std::endl;
    std::cout << s.Size << std::endl;
    using namespace blog;
    switch (s.Hash) {
        case "abcedf"_hash:
            std::cout << "s is " << s.Hash << std::endl;
            break;
        case s.Hash + 1:
            break;
        case "abc12345678edf"_hash:
            std::cout << "s is " << s.Name << std::endl;
            break;
    }
}*/
