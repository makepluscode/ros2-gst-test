#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <thread>

#include <queue>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/image.hpp"

#include <gst/gst.h>
#include <gst/app/gstappsink.h>


using namespace std::chrono_literals;

class GstNode : public rclcpp::Node
{
public:
  GstNode()
  : Node("GstNode"), data_(0)
  {
    init_pub();
    init_gst();
  }

private:
  uint8_t data_;

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr cinfo_pub_;

  rmw_qos_reliability_policy_t reliability_policy_;
  rmw_qos_history_policy_t history_policy_;
  size_t depth_;

  bool init_pub()
  {
    auto qos = rclcpp::QoS(
      rclcpp::QoSInitialization(
        history_policy_,
        depth_
    ));
    qos.reliability(reliability_policy_);

    publisher_ = create_publisher<sensor_msgs::msg::Image>("image", qos);
  }

  // Gstreamer references
  GstElement*  pipeline_ = NULL;
  GstBus*      bus_      = NULL;
  _GstAppSink* appsink_ = NULL;

  bool init_gst()
  {
    GError* err = NULL;

    gst_init(NULL, NULL);

    const std::string gst_launch = "videotestsrc pattern=ball ! video/x-raw,width=1920,height=1080,format=RGB ! \
      videorate ! video/x-raw, framerate=30/1  ! appsink name=mysink";

    pipeline_ = gst_parse_launch(gst_launch.c_str(), &err);

    if (err != NULL)
    {
      RCLCPP_ERROR(this->get_logger(), "failed to create pipeline : (%s)", err->message);
      g_error_free(err);
      return false;
    }

    GstPipeline* pipeline = GST_PIPELINE(pipeline_);

    bus_ = gst_pipeline_get_bus(pipeline);

    if (!bus_)
    {
      RCLCPP_ERROR(this->get_logger(), "failed to retrieve GstBus from pipeline");
      return false;
    }

    // Get the appsrc
    GstElement* appsinkElement = gst_bin_get_by_name(GST_BIN(pipeline), "mysink");
    GstAppSink* appsink = GST_APP_SINK(appsinkElement);

    if (!appsink)
    {
      RCLCPP_ERROR(this->get_logger(), "failed to retrieve AppSink element from pipeline");
      return false;
    }

    appsink_ = appsink;

    GstAppSinkCallbacks cb;
    memset(&cb, 0, sizeof(GstAppSinkCallbacks));

    cb.eos         = on_eos;
    cb.new_preroll = on_preroll;;
    cb.new_sample  = on_buffer;

    gst_app_sink_set_callbacks(appsink_, &cb, (void*)this, NULL);

    const GstStateChangeReturn result = gst_element_set_state(pipeline_, GST_STATE_PLAYING);

    if (result == GST_STATE_CHANGE_ASYNC)
    {
      // here will be a debug point in the future
      RCLCPP_INFO(get_logger(), "success to set pipeline state to PLAYING");
    }
    else if (result != GST_STATE_CHANGE_SUCCESS)
    {
      RCLCPP_ERROR(get_logger(), "failed to set pipeline state to PLAYING (error %u)", result);
      return false;
    }
  }

  static void on_eos(_GstAppSink* sink, void* user_data)
  {
    GstNode* node = (GstNode*)user_data;

    RCLCPP_INFO(node->get_logger(), "on_eos called");
  }

  static GstFlowReturn on_preroll(_GstAppSink* sink, void* user_data)
  {
    GstNode* node = (GstNode*)user_data;

    RCLCPP_INFO(node->get_logger(), "on_preroll called");
    return GST_FLOW_OK;
  }

  static GstFlowReturn on_buffer(_GstAppSink* sink, void* user_data)
  {
    GstNode* node = (GstNode*)user_data;
    GstSample *sample;

    //RCLCPP_INFO(node->get_logger(), "on_buffer called");

    // Retrieve the buffer
    g_signal_emit_by_name (sink, "pull-sample", &sample);

    if (sample)
    {
      GstMapInfo info;
      GstBuffer *buffer = gst_sample_get_buffer(sample);

      gst_buffer_map(buffer, &info, GST_MAP_READ);

      if (info.data != NULL)
      {
        int width, height;
        GstStructure *str;

        GstCaps *caps = gst_sample_get_caps(sample);
        // Get a string containg the pixel format, width and height of the image
        str = gst_caps_get_structure (caps, 0);

        gst_structure_get_int (str, "width", &width);
        gst_structure_get_int (str, "height", &height);

        // ### publish
        auto img = std::make_unique<sensor_msgs::msg::Image>();

        img->width            = width;
        img->height           = height;

        img->encoding         = "rgb8";
        img->is_bigendian     = false;
        img->data.resize(width * height * 3);

        img->step             = width * 3;

        // Copy the image so we can free the buffer allocated by gstreamer
        std::copy(
          info.data,
          info.data + info.size,
          img->data.begin()
        );

#undef DEBUG_ADDRESS
//#ifdef DEBUG_ADDRESS
#if 1
        static int count = 0;
        RCLCPP_INFO(node->get_logger(),
          "[#%4d], size %d, address %p",
          count++, info.size, reinterpret_cast<std::uintptr_t>(img.get()));
#endif
        node->publisher_->publish(std::move(img));
        gst_buffer_unmap (buffer, &info);
      }
      gst_sample_unref(sample);

      return GST_FLOW_OK;
    }

    return GST_FLOW_ERROR;
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor executor;

  auto gst_node = std::make_shared<GstNode>();

  executor.add_node(gst_node);

  executor.spin();

  rclcpp::shutdown();
  return 0;
}