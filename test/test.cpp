//
// Created by ou on 2021/11/4.
//
#include "factory.h"
#include <gtest/gtest.h>
#include <dbg.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

namespace YAML {
class Node;
}
using PCXYZ = pcl::PointCloud<pcl::PointXYZ>;

namespace pc_utils {
template<class PointT, class ...T>
struct Cluster {
    virtual void Segment(const pcl::PointCloud<PointT> &input, std::vector<int> &cluster, std::vector<int> &id) = 0;
};

template<class PointT, class ...T>
struct CurvedVoxelCluster : Cluster<PointT, T...> {
    explicit CurvedVoxelCluster(const std::vector<double> &config) {
        printf("CurvedVoxelCluster(const std::vector<double> &config)\n");
    }

    explicit CurvedVoxelCluster(const YAML::Node &config) {
        printf("CurvedVoxelCluster(const YAML::Node& config)\n");
    }

    void Segment(const pcl::PointCloud<PointT> &input, std::vector<int> &cluster, std::vector<int> &id) {
        printf("CurvedVoxel -> Segment()\n");
    };

};
REGISTER_TO_FACTORY((Cluster<pcl::PointXYZ>), (CurvedVoxelCluster<pcl::PointXYZ>), std::vector<double>)
REGISTER_TO_FACTORY((Cluster<pcl::PointXYZ>), (CurvedVoxelCluster<pcl::PointXYZ>), YAML::Node)
//template class CurvedVoxelCluster<pcl::PointXYZ>;


template<class PointT, class ...T>
struct EuclideanCluster : Cluster<PointT, T...> {
    EuclideanCluster() {};


    explicit EuclideanCluster(const std::vector<double> &config) {
        printf("EuclideanCluster(const std::vector<double> &config)\n");
    }

    explicit EuclideanCluster(const YAML::Node &config) {
        printf("EuclideanCluster(const YAML::Node& config)\n");
    }

    void Segment(const pcl::PointCloud<PointT> &input, std::vector<int> &cluster, std::vector<int> &id) {
        printf("EuclideanCluster -> Segment()\n");
    };
};
REGISTER_TO_FACTORY((Cluster<pcl::PointXYZ>), (EuclideanCluster<pcl::PointXYZ>), std::vector<double>)
REGISTER_TO_FACTORY((Cluster<pcl::PointXYZ>), (EuclideanCluster<pcl::PointXYZ>), YAML::Node)
REGISTER_TO_FACTORY((Cluster<pcl::PointXYZ>), (EuclideanCluster<pcl::PointXYZ>))
}


TEST(TYPE_NAME, type_name_test) {
    dbg("===========================");
    dbg((Type<int>::name));
    dbg((Types<int, float>::names));
    dbg((Types<int, float>::raw_names));
    dbg((Types<int, float>::size));
    dbg((Types<int, float>::name<1>));
}

TEST(TYPE_NAME, concate_type_name_with_str) {
    dbg("===========================");
    dbg((Type<pc_utils::Cluster<int, PCXYZ>>::name));
    dbg((decorate<int, PCXYZ>("ClusterBase")));
    dbg((decorate("ClusterBase")));

    dbg(TTypeTrait<PCXYZ>::size);
    dbg(Type<TTypeTrait<PCXYZ>::arg_type<0>>::name);
    dbg(TTypeTrait<PCXYZ>::with_template_arg("Cluster"));
}

TEST(Factory, test_all) {
    std::vector<double> config;
    std::vector<int> result, id;
    std::string euclidean = "pc_utils::EuclideanCluster", cvc = "pc_utils::CurvedVoxelCluster";

    auto c1 = Build<pc_utils::Cluster<pcl::PointXYZ>>(decorate<pcl::PointXYZ>(euclidean), config);
    c1->Segment(pcl::PointCloud<pcl::PointXYZ>{}, result, id);

    auto c2 = Build<pc_utils::Cluster<pcl::PointXYZ>>(DECORATE(cvc, pcl::PointXYZ), config);
    c2->Segment(pcl::PointCloud<pcl::PointXYZ>{}, result, id);

    auto c3 = BuildT<pc_utils::Cluster, pcl::PointXYZ>(cvc, config);
    c3->Segment(pcl::PointCloud<pcl::PointXYZ>{}, result, id);


    auto c4 = BuildT<pc_utils::Cluster<pcl::PointXYZ>>(cvc, config);
    c4->Segment(pcl::PointCloud<pcl::PointXYZ>{}, result, id);
}

template<typename... Args>
inline void
test(Args &&... __args) {
    (dbg(Type<decltype(std::forward<Args>(__args))>::name, Type<Args>::name), ...);
}


TEST(TYPE_NAME, test_forward) {
    dbg("===========================");
    std::vector<double> config;
    const std::vector<int> result, id;
    const std::string euclidean = "pc_utils::EuclideanCluster", &cvc = euclidean;


    test(result, euclidean, cvc, result, config, 1);

}