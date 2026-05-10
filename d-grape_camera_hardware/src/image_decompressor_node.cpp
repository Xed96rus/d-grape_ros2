/**
 * image_decompressor_node.cpp
 *
 * Нода для ноутбука: подписывается на CompressedImage топики от stereo_cameras_node,
 * декодирует JPEG и переиздаёт как обычный sensor_msgs/Image.
 *
 * По умолчанию слушает 6 топиков:
 *   /front_cams/f_left_camera/image/compressed
 *   /front_cams/f_right_camera/image/compressed
 *   /left_cams/l_left_camera/image/compressed
 *   /left_cams/l_right_camera/image/compressed
 *   /right_cams/r_left_camera/image/compressed
 *   /right_cams/r_right_camera/image/compressed
 *
 * Публикует Image на тех же путях без суффикса /compressed:
 *   /front_cams/f_left_camera/image  …и т.д.
 *
 * Параметры:
 *   topics          — список топиков CompressedImage (yaml sequence)
 *   qos_reliability — "best_effort" | "reliable"   (default: best_effort)
 *   qos_depth       — глубина очереди              (default: 1)
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgcodecs.hpp>

#include <string>
#include <vector>

// ---------------------------------------------------------------------------

static const std::vector<std::string> kDefaultTopics = {
    "/front_cams/f_left_camera/image/compressed",
    "/front_cams/f_right_camera/image/compressed",
    "/left_cams/l_left_camera/image/compressed",
    "/left_cams/l_right_camera/image/compressed",
    "/right_cams/r_left_camera/image/compressed",
    "/right_cams/r_right_camera/image/compressed",
};

// Убирает суффикс "/compressed" чтобы получить имя выходного топика.
static std::string stripCompressed(const std::string & topic)
{
    const std::string suffix = "/compressed";
    if (topic.size() > suffix.size() &&
        topic.compare(topic.size() - suffix.size(), suffix.size(), suffix) == 0)
    {
        return topic.substr(0, topic.size() - suffix.size());
    }
    return topic + "/raw";   // fallback
}

// ---------------------------------------------------------------------------

class ImageDecompressor : public rclcpp::Node
{
public:
    ImageDecompressor()
    : Node("image_decompressor")
    {
        // ── параметры ──────────────────────────────────────────────────────
        declare_parameter<std::vector<std::string>>("topics",          kDefaultTopics);
        declare_parameter<std::string>("qos_reliability",              "best_effort");
        declare_parameter<int>        ("qos_depth",                    1);

        auto topics    = get_parameter("topics").as_string_array();
        auto rel       = get_parameter("qos_reliability").as_string();
        int  depth     = get_parameter("qos_depth").as_int();

        // ── QoS ───────────────────────────────────────────────────────────
        rclcpp::QoS sub_qos(depth);
        rclcpp::QoS pub_qos(depth);
        if (rel == "reliable") {
            sub_qos.reliable();
            pub_qos.reliable();
        } else {
            sub_qos.best_effort();
            pub_qos.best_effort();
        }

        // ── создаём пары sub/pub для каждого топика ───────────────────────
        for (const auto & cmp_topic : topics) {
            const std::string img_topic = stripCompressed(cmp_topic);

            auto pub = create_publisher<sensor_msgs::msg::Image>(img_topic, pub_qos);

            auto sub = create_subscription<sensor_msgs::msg::CompressedImage>(
                cmp_topic, sub_qos,
                [this, pub, cmp_topic](
                    const sensor_msgs::msg::CompressedImage::SharedPtr msg)
                {
                    onCompressed(msg, pub, cmp_topic);
                });

            subs_.push_back(sub);
            pubs_.push_back(pub);

            RCLCPP_INFO(get_logger(), "  %s  →  %s",
                cmp_topic.c_str(), img_topic.c_str());
        }

        RCLCPP_INFO(get_logger(),
            "image_decompressor ready (%zu channel(s), QoS=%s depth=%d)",
            topics.size(), rel.c_str(), depth);
    }

private:
    void onCompressed(
        const sensor_msgs::msg::CompressedImage::SharedPtr & msg,
        const rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr & pub,
        const std::string & topic_name)
    {
        // Декодируем JPEG → cv::Mat
        cv::Mat img = cv::imdecode(
            cv::InputArray(msg->data),
            cv::IMREAD_COLOR);

        if (img.empty()) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                "Failed to decode compressed image from %s", topic_name.c_str());
            return;
        }

        // Конвертируем в ROS Image (BGR8 — то, что пришло от cv::imdecode)
        auto out = cv_bridge::CvImage(msg->header, "bgr8", img).toImageMsg();
        pub->publish(*out);
    }

    std::vector<rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr> subs_;
    std::vector<rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr>               pubs_;
};

// ---------------------------------------------------------------------------

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImageDecompressor>());
    rclcpp::shutdown();
    return 0;
}
