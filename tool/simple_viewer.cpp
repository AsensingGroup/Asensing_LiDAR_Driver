#include "ag_driver/api/lidar_driver.hpp"
#include "ag_driver/msg/pcl_point_cloud_msg.hpp"

#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

using namespace asensing::lidar;
using namespace pcl::visualization;

typedef PointCloudT<PointXYZIRT> PointCloudMsg;
typedef PointXYZIRT PCLMsg;
//typedef pcl::PointXYZI PCLMsg

std::shared_ptr<PCLVisualizer> pcl_viewer;
std::mutex mtx_viewer;

SyncQueue<std::shared_ptr<PointCloudMsg>> free_cloud_queue;
SyncQueue<std::shared_ptr<PointCloudMsg>> stuffed_cloud_queue;

bool ViewerStopFlag =false;

bool checkKeywordExist(int argc, const char* const* argv, const char* str)
{
  for (int i = 1; i < argc; i++)
  {
    if (strcmp(argv[i], str) == 0)
    {
      return true;
    }
  }
  return false;
}

bool parseArgument(int argc, const char* const* argv, const char* str, std::string& val)
{
  int index = -1;

  for (int i = 1; i < argc; i++)
  {
    if (strcmp(argv[i], str) == 0)
    {
      index = i + 1;
    }
  }

  if (index > 0 && index < argc)
  {
    val = argv[index];
    return true;
  }

  return false;
}

void parseParam(int argc, char* argv[], AGDriverParam& param)
{
  std::string result_str;

  //
  // input param
  //
  parseArgument(argc, argv, "-pcap", param.input_param.pcap_path);
  if (param.input_param.pcap_path.empty())
  {
    param.input_type = InputType::ONLINE_LIDAR;
  }
  else
  {
    param.input_type = InputType::PCAP_FILE;
    // param.input_param.pcap_repeat = false;
  }

  if (parseArgument(argc, argv, "-msop", result_str))
  {
    param.input_param.msop_port = std::stoi(result_str);
  }

  if (parseArgument(argc, argv, "-difop", result_str))
  {
    param.input_param.difop_port = std::stoi(result_str);
  }

  parseArgument(argc, argv, "-group", param.input_param.group_address);
  parseArgument(argc, argv, "-host", param.input_param.host_address);
 
  //
  // decoder param
  //
  if (parseArgument(argc, argv, "-type", result_str))
  {
    param.lidar_type = strToLidarType(result_str);
  }

  if (parseArgument(argc, argv, "-x", result_str))
  {
    param.decoder_param.transform_param.x = std::stof(result_str);
  }

  if (parseArgument(argc, argv, "-y", result_str))
  {
    param.decoder_param.transform_param.y = std::stof(result_str);
  }

  if (parseArgument(argc, argv, "-z", result_str))
  {
    param.decoder_param.transform_param.z = std::stof(result_str);
  }

  if (parseArgument(argc, argv, "-roll", result_str))
  {
    param.decoder_param.transform_param.roll = std::stof(result_str);
  }

  if (parseArgument(argc, argv, "-elevation", result_str))
  {
    param.decoder_param.transform_param.elevation = std::stof(result_str);
  }

  if (parseArgument(argc, argv, "-azimuth", result_str))
  {
    param.decoder_param.transform_param.azimuth = std::stof(result_str);
  }
}

void printHelpMenu()
{
  AG_MSG << "Arguments: " << AG_REND;
  AG_MSG << "  -type   = LiDAR type (A0, A2)" << AG_REND;
  AG_MSG << "  -pcap   = The path of the pcap file, off-line mode if it is true, else online mode." << AG_REND;
  AG_MSG << "  -msop   = LiDAR msop port number,the default value is 51180" << AG_REND;
  AG_MSG << "  -difop  = LiDAR difop port number,the default value is 9988" << AG_REND;
  AG_MSG << "  -group  = LiDAR destination group address if multi-cast mode." << AG_REND;
  AG_MSG << "  -host   = Host address." << AG_REND;
  AG_MSG << "  -x      = Transformation parameter, unit: m " << AG_REND;
  AG_MSG << "  -y      = Transformation parameter, unit: m " << AG_REND;
  AG_MSG << "  -z      = Transformation parameter, unit: m " << AG_REND;
  AG_MSG << "  -roll   = Transformation parameter, unit: radian " << AG_REND;
  AG_MSG << "  -elevation  = Transformation parameter, unit: radian " << AG_REND;
  AG_MSG << "  -azimuth    = Transformation parameter, unit: radian " << AG_REND;
  AG_MSG << "" << AG_REND;
  AG_MSG << "Example: simple_viewer -type A0 -pcap /opt/data/A0-test.pcap" << AG_REND;
}

void exceptionCallback(const Error& code)
{
  AG_WARNING << code.toString() << AG_REND;
}

std::shared_ptr<PointCloudMsg> pointCloudGetCallback(void)
{
  std::shared_ptr<PointCloudMsg> msg = free_cloud_queue.pop();
  if (msg.get() != NULL)
  {
    return msg;
  }

  return std::make_shared<PointCloudMsg>();
}

void pointCloudPutCallback(std::shared_ptr<PointCloudMsg> msg)
{
  stuffed_cloud_queue.push(msg);
}

bool to_exit_process = false;
void processCloud(void)
{
  while (!to_exit_process)
  {
    std::shared_ptr<PointCloudMsg> msg = stuffed_cloud_queue.popWait();
    if (msg.get() == NULL)
    {
      continue;
    }

    AG_DEBUG << "[" << msg->frame_id << "] Width = " << msg->width << ", Height = " << msg->height << ", Size = " << msg->points.size() << AG_REND;

    //
    // show the point cloud
    //
    pcl::PointCloud<PCLMsg>::Ptr pcl_pointcloud(new pcl::PointCloud<PCLMsg>);
    pcl_pointcloud->points.swap(msg->points);
    pcl_pointcloud->height = msg->height;
    pcl_pointcloud->width = msg->width;
    pcl_pointcloud->is_dense = msg->is_dense;

    PointCloudColorHandlerGenericField<PCLMsg> point_color_handle(pcl_pointcloud, "range");

    {
      const std::lock_guard<std::mutex> lock(mtx_viewer);
      pcl_viewer->updatePointCloud<PCLMsg>(pcl_pointcloud, point_color_handle, "aglidar");
    }

    free_cloud_queue.push(msg);
  }
}

int main(int argc, char* argv[])
{
  AG_TITLE << "------------------------------------------------------" << AG_REND;
  AG_TITLE << "            AG_Driver Viewer Version: v" << getDriverVersion() << AG_REND;
  AG_TITLE << "------------------------------------------------------" << AG_REND;

  if (argc < 2)
  {
    printHelpMenu();
    return 0;
  }

  if (checkKeywordExist(argc, argv, "-h") || checkKeywordExist(argc, argv, "--help"))
  {
    printHelpMenu();
    return 0;
  }

  std::thread cloud_handle_thread = std::thread(processCloud);
#ifndef WIN32
  pthread_setname_np(cloud_handle_thread.native_handle(), "process_cloud");
#endif

  AGDriverParam param;
  param.decoder_param.dense_points = true;
  parseParam(argc, argv, param);
  param.print();

  pcl_viewer = std::make_shared<PCLVisualizer>("Simple PointCloud Viewer");
  pcl_viewer->setBackgroundColor(0.0, 0.0, 0.0);
  pcl_viewer->setCameraPosition(0, 0, 20, 0, 0, -1, 1, 0, 0);
  pcl_viewer->addCoordinateSystem(1.0);

  pcl::PointCloud<PCLMsg>::Ptr pcl_pointcloud(new pcl::PointCloud<PCLMsg>);
  pcl_viewer->addPointCloud<PCLMsg>(pcl_pointcloud, "aglidar");
  pcl_viewer->setPointCloudRenderingProperties(PCL_VISUALIZER_POINT_SIZE, 2, "aglidar");

  LidarDriver<PointCloudMsg> driver;
  driver.regExceptionCallback(exceptionCallback);
  driver.regPointCloudCallback(pointCloudGetCallback, pointCloudPutCallback);
  if (!driver.init(param))
  {
    AG_ERROR << "Driver Initialize Error..." << AG_REND;
    return -1;
  }

  AG_INFO << "Asensing Lidar-Driver Viewer start......" << AG_REND;

  // driver.setDefaultSelectionParam();
  SelectionParam selection;
  driver.getSelectionParam(selection);
  selection.min_module = 0;
  selection.max_module = 4;
  driver.setSelectionParam(selection);

  driver.start();

  while (!pcl_viewer->wasStopped())
  {
    {
      const std::lock_guard<std::mutex> lock(mtx_viewer);
      pcl_viewer->spinOnce();
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  driver.stop();

  to_exit_process = true;
  cloud_handle_thread.join();

  return 0;
}
