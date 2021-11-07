#include "factory.h"
#include <iostream>
#include <vector>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <boost/core/demangle.hpp>
#include <functional>

using PCXYZ = pcl::PointCloud<pcl::PointXYZ>;





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