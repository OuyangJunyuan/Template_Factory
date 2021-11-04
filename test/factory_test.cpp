#include "factory.h"
#include <iostream>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <boost/core/demangle.hpp>

using PCXYZ = pcl::PointCloud<pcl::PointXYZ>;

using namespace spiritsaway::entity_component_event;
using namespace std;

struct Animal {
public:
    Animal(int x) : m_x(x) {

    }

    virtual void makeNoise() {};

protected:
    int m_x;
};

using AnimalFactory = basic_poly_factory<shr_ptr_t, Animal, int>;


struct Dog final : public AnimalFactory::sub_class<Dog> {
public:
    Dog(int x)
            : AnimalFactory::sub_class<Dog>(x) {

    }

public:
    void makeNoise() { std::cerr << "Dog: " << m_x << "\n"; }

    static std::string_view class_name() {
        return "Dog";
    }
};

struct Cat final : public AnimalFactory::sub_class<Cat> {
public:
    Cat(int x) : AnimalFactory::sub_class<Cat>(x) {

    }

    void makeNoise() { std::cerr << "Cat: " << m_x << "\n"; }

    static std::string_view class_name() {
        return "Cat";
    }
};


void test_hash() {
    auto x = AnimalFactory::make<Dog>(3);
    auto y = AnimalFactory::make<Cat>(2);
    x->makeNoise();
    y->makeNoise();
}

void test_name() {
    auto x = AnimalFactory::make_by_name("Dog", 3);
    auto y = AnimalFactory::make_by_name("Cat", 2);
    x->makeNoise();
    y->makeNoise();
}

template<class ...C>
auto template_name() {
    std::string tt((boost::core::demangle(typeid(C).name()) + "," +...));
    tt.erase(tt.size() - 1);
    tt = '<' + tt + '>';
    return tt;
}

int main() {
    test_hash();
    test_name();
//    std::cout << full_type_name<PCXYZ>() << std::endl;
//    std::cout << boost::core::demangle(typeid(PCXYZ).name()) << std::endl;
    std::cout << template_name<int, PCXYZ>() << std::endl;
    return 0;
}