/*
Developer: Chunran Zheng <zhengcr@connect.hku.hk>

This file is subject to the terms and conditions outlined in the 'LICENSE' file,
which is included as part of this source code package.
*/

#include "qr_detect.hpp"
#include "lidar_detect.hpp"
#include "data_preprocess.hpp"
#include "vehicle_config_reader.hpp"

#include <sys/stat.h>

namespace
{

void printUsage(const char *argv0)
{
    std::cerr << "Usage: " << argv0
              << " --image <path> --pointcloud <path|folder> --settings <path>"
              << " -c <vehicle_config_dir> --camera <camera_name> -o <output_dir>"
              << " [--lidar-type auto|solid|mech]" << std::endl;
}

bool parseArgs(int argc, char **argv, std::string &image_path, std::string &pointcloud_path,
               std::string &settings_path, std::string &vehicle_config_dir,
               std::string &camera_name, std::string &output_dir, std::string &lidar_type_override)
{
    lidar_type_override = "auto";
    for (int i = 1; i < argc; ++i)
    {
        std::string arg = argv[i];
        auto next = [&]() -> std::string {
            if (i + 1 >= argc)
            {
                std::cerr << "[Args] Missing value for " << arg << std::endl;
                std::exit(1);
            }
            return std::string(argv[++i]);
        };

        if (arg == "--image") image_path = next();
        else if (arg == "--pointcloud") pointcloud_path = next();
        else if (arg == "--settings") settings_path = next();
        else if (arg == "-c" || arg == "--vehicle-config") vehicle_config_dir = next();
        else if (arg == "--camera") camera_name = next();
        else if (arg == "-o" || arg == "--output") output_dir = next();
        else if (arg == "--lidar-type") lidar_type_override = next();
        else
        {
            std::cerr << "[Args] Unknown argument: " << arg << std::endl;
            return false;
        }
    }

    if (image_path.empty() || pointcloud_path.empty() || settings_path.empty() ||
        vehicle_config_dir.empty() || camera_name.empty() || output_dir.empty())
    {
        std::cerr << "[Args] Missing required argument(s)." << std::endl;
        return false;
    }
    if (lidar_type_override != "auto" && lidar_type_override != "solid" && lidar_type_override != "mech")
    {
        std::cerr << "[Args] --lidar-type must be auto|solid|mech" << std::endl;
        return false;
    }
    return true;
}

void saveDebugPcd(const std::string &output_dir, const std::string &name,
                  const pcl::PointCloud<Common::Point>::Ptr &cloud)
{
    if (!cloud || cloud->empty()) return;
    pcl::io::savePCDFileBinaryCompressed(output_dir + "/" + name, *cloud);
}

void saveDebugPcd(const std::string &output_dir, const std::string &name,
                  const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud)
{
    if (!cloud || cloud->empty()) return;
    pcl::io::savePCDFileBinaryCompressed(output_dir + "/" + name, *cloud);
}

}  // namespace

int main(int argc, char **argv)
{
    std::string image_path, pointcloud_path, settings_path;
    std::string vehicle_config_dir, camera_name, output_dir, lidar_type_override;

    if (!parseArgs(argc, argv, image_path, pointcloud_path, settings_path,
                   vehicle_config_dir, camera_name, output_dir, lidar_type_override))
    {
        printUsage(argv[0]);
        return 1;
    }

    Params params = loadSettingsYaml(settings_path);
    if (!loadCameraFromVehicleConfig(vehicle_config_dir, camera_name, params))
    {
        return 1;
    }

    while (!output_dir.empty() && output_dir.back() == '/') output_dir.pop_back();
    mkdir(output_dir.c_str(), 0755);
    params.output_path = output_dir;

    DataPreprocess data_preprocess(image_path, pointcloud_path, lidar_type_override);
    if (data_preprocess.img_input_.empty() || data_preprocess.cloud_input_->empty())
    {
        ROS_ERROR("[Main] Failed to load image or point cloud, abort.");
        return 1;
    }

    cv::Mat img_input = data_preprocess.img_input_;
    applyFisheyeUndistortIfNeeded(params, img_input);

    QRDetect qr_detect(params);
    LidarDetect lidar_detect(params);

    pcl::PointCloud<pcl::PointXYZ>::Ptr qr_center_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    qr_center_cloud->reserve(4);
    qr_detect.detect_qr(img_input, qr_center_cloud);

    pcl::PointCloud<pcl::PointXYZ>::Ptr lidar_center_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    lidar_center_cloud->reserve(4);
    switch (data_preprocess.lidarType())
    {
        case LiDARType::Solid:
            lidar_detect.detect_solid_lidar(data_preprocess.cloud_input_, lidar_center_cloud);
            break;
        case LiDARType::Mech:
            lidar_detect.detect_mech_lidar(data_preprocess.cloud_input_, lidar_center_cloud);
            break;
        default:
            ROS_ERROR("[Main] Unknown LiDAR type.");
            return 1;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr qr_centers(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr lidar_centers(new pcl::PointCloud<pcl::PointXYZ>);
    sortPatternCenters(qr_center_cloud, qr_centers, "camera");
    sortPatternCenters(lidar_center_cloud, lidar_centers, "lidar");

    validateTargetGeometry(qr_centers, params.delta_width_circles, params.delta_height_circles, "QR");
    validateTargetGeometry(lidar_centers, params.delta_width_circles, params.delta_height_circles, "LiDAR");

    saveTargetHoleCenters(lidar_centers, qr_centers, params);

    Eigen::Matrix4f transformation;
    pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ> svd;
    svd.estimateRigidTransformation(*lidar_centers, *qr_centers, transformation);

    pcl::PointCloud<pcl::PointXYZ>::Ptr aligned_lidar_centers(new pcl::PointCloud<pcl::PointXYZ>);
    aligned_lidar_centers->reserve(lidar_centers->size());
    alignPointCloud(lidar_centers, aligned_lidar_centers, transformation);

    double rmse = computeRMSE(qr_centers, aligned_lidar_centers);
    if (rmse > 0)
    {
        std::cout << BOLDYELLOW << "[Result] RMSE: " << BOLDRED << std::fixed << std::setprecision(4)
                  << rmse << " m" << RESET << std::endl;
    }

    std::cout << BOLDYELLOW << "[Result] Single-scene calibration: extrinsic parameters T_cam_lidar = "
              << RESET << std::endl;
    std::cout << BOLDCYAN << std::fixed << std::setprecision(6) << transformation << RESET << std::endl;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    projectPointCloudToImage(data_preprocess.cloud_input_, transformation,
                             qr_detect.cameraMatrix_, qr_detect.distCoeffs_, img_input, colored_cloud);

    saveCalibrationResults(params, transformation, colored_cloud, qr_detect.imageCopy_);

    if (DEBUG)
    {
        saveDebugPcd(output_dir, "debug_filtered_cloud.pcd", lidar_detect.getFilteredCloud());
        saveDebugPcd(output_dir, "debug_plane_cloud.pcd", lidar_detect.getPlaneCloud());
        saveDebugPcd(output_dir, "debug_annulus_cloud.pcd", lidar_detect.getAnnulusOriginalCloud());
        saveDebugPcd(output_dir, "debug_boundary_cloud.pcd", lidar_detect.getBoundaryOriginalCloud());
        saveDebugPcd(output_dir, "debug_aligned_cloud.pcd", lidar_detect.getAlignedCloud());
        saveDebugPcd(output_dir, "debug_edge_cloud.pcd", lidar_detect.getEdgeCloud());
        saveDebugPcd(output_dir, "debug_center_z0_cloud.pcd", lidar_detect.getCenterZ0Cloud());
        saveDebugPcd(output_dir, "debug_aligned_lidar_centers.pcd", aligned_lidar_centers);
    }

    return 0;
}
