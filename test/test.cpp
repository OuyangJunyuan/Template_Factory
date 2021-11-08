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