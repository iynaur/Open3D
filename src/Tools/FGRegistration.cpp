
#include <Open3D/Open3D.h>
#include <Open3D/IO/ClassIO/PointCloudIO.h>
#include <Open3D/Registration/FastGlobalRegistration.h>
#include <Open3D/Registration/Feature.h>
#include <Open3D/Geometry/PointCloud.h>

#include <limits>

int main(int argc, char **argv) {
    using namespace open3d;
    using namespace open3d::io;
    using namespace open3d::registration;
    using namespace open3d::utility;
    using namespace open3d::geometry;

    open3d::utility::SetVerbosityLevel(open3d::utility::VerbosityLevel::VerboseAlways);

    if (argc < 3 || utility::ProgramOptionExists(argc, argv, "--help") ||
        utility::ProgramOptionExists(argc, argv, "-h")) {
        return 0;
    }

    geometry::PointCloud pointcloud1, pointcloud2;
    ReadPointCloud(std::string(argv[1]), pointcloud1);
    ReadPointCloud(std::string(argv[2]), pointcloud2);

    if (bool mirror = false)
    {
        for (auto &p : pointcloud2.points_)
        {
            p(1) *= -1;
        }

        WritePointCloud("./pre/save1.pcd", pointcloud1, true);
        WritePointCloud("./pre/save3.pcd", pointcloud2, true);
    }

    EstimateNormals(pointcloud1);
    OrientNormalsTowardsCameraLocation(pointcloud1);
    EstimateNormals(pointcloud2);
    OrientNormalsTowardsCameraLocation(pointcloud2);

    std::shared_ptr<Feature> f1(new Feature);
    std::shared_ptr<Feature> f2(new Feature);
    f1 = ComputeFPFHFeature(pointcloud1);
    f2 = ComputeFPFHFeature(pointcloud2);

    auto res = FastGlobalRegistration(
            pointcloud1,
            pointcloud2,
            *f1,
            *f2);

    std::cerr<<res.transformation_<<std::endl;

    pointcloud2.Transform(res.transformation_.inverse());

    WritePointCloud("save1.pcd", pointcloud1, true);
    WritePointCloud("save3.pcd", pointcloud2, true);

    return 0;
}
