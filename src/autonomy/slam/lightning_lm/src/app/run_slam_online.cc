//
// Created by xiang on 25-3-18.
//

#include <gflags/gflags.h>
#include <glog/logging.h>

#include "core/system/slam.h"
#include "utils/timer.h"
#include "wrapper/bag_io.h"
#include "wrapper/ros_utils.h"

DEFINE_string(config, "./config/default.yaml", "配置文件");

/// 运行一个LIO前端，带可视化
int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);
    FLAGS_colorlogtostderr = true;
    FLAGS_stderrthreshold = google::INFO;

    // ros2 launch always appends "--ros-args ..." to argv. gflags rejects
    // unknown flags, so split argv: gflags sees everything before --ros-args,
    // rclcpp::init sees the original.
    int gflags_argc = argc;
    for (int i = 1; i < argc; ++i) {
        if (std::string(argv[i]) == "--ros-args") {
            gflags_argc = i;
            break;
        }
    }
    char** gflags_argv = argv;
    google::ParseCommandLineFlags(&gflags_argc, &gflags_argv, false);

    using namespace lightning;

    /// 需要rclcpp::init
    rclcpp::init(argc, argv);

    SlamSystem::Options options;
    options.online_mode_ = true;

    SlamSystem slam(options);
    if (!slam.Init(FLAGS_config)) {
        LOG(ERROR) << "failed to init slam";
        return -1;
    }

    slam.StartSLAM("new_map");
    slam.Spin();

    Timer::PrintAll();

    rclcpp::shutdown();

    LOG(INFO) << "done";

    return 0;
}