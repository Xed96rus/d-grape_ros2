#include <memory>
#include <string>
#include <array>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "cv_bridge/cv_bridge.h"
#include <opencv2/core.hpp>

#include "d-grape_camera_hardware/frame_grabber.hpp"

// ---------------------------------------------------------------------------

static sensor_msgs::msg::CameraInfo makeDefaultCameraInfo(
    int width, int height, const std::string & frame_id)
{
    sensor_msgs::msg::CameraInfo ci;
    ci.width  = static_cast<unsigned>(width);
    ci.height = static_cast<unsigned>(height);
    ci.header.frame_id   = frame_id;
    ci.distortion_model  = "plumb_bob";

    double fov_horizontal_deg = 90.0;
    double fov_rad = fov_horizontal_deg * M_PI / 180.0;
    double fx = (width / 2.0) / std::tan(fov_rad / 2.0);
    double fy = fx;  // предполагаем квадратные пиксели
    
    double cx = width  / 2.0;  // оптический центр X
    double cy = height / 2.0;  // оптический центр Y

    ci.d = {0.0, 0.0, 0.0, 0.0, 0.0};
    ci.k = {fx, 0.0, cx,
            0.0, fy, cy,
            0.0, 0.0, 1.0};
    ci.r = {1.0, 0.0, 0.0,
            0.0, 1.0, 0.0,
            0.0, 0.0, 1.0};
    ci.p = {fx, 0.0, cx, 0.0,
            0.0, fy, cy, 0.0,
            0.0, 0.0, 1.0, 0.0};
    return ci;
}

static double cvMatGetDouble(const cv::Mat & m, int row, int col)
{
    switch (m.type()) {
        case CV_32F: return static_cast<double>(m.at<float>(row, col));
        case CV_64F: return m.at<double>(row, col);
        case CV_32S: return static_cast<double>(m.at<int>(row, col));
        default: return 0.0;
    }
}

static bool loadCameraInfoFromYaml(
    const std::string & yaml_path,
    sensor_msgs::msg::CameraInfo & info)
{
    cv::FileStorage fs(yaml_path, cv::FileStorage::READ);
    if (!fs.isOpened()) {
        return false;
    }

    int width = 0;
    int height = 0;
    std::string distortion_model;
    cv::Mat camera_matrix;
    cv::Mat distortion_coefficients;
    cv::Mat rectification_matrix;
    cv::Mat projection_matrix;

    fs["image_width"] >> width;
    fs["image_height"] >> height;
    fs["distortion_model"] >> distortion_model;
    fs["camera_matrix"] >> camera_matrix;
    fs["distortion_coefficients"] >> distortion_coefficients;
    fs["rectification_matrix"] >> rectification_matrix;
    fs["projection_matrix"] >> projection_matrix;

    if (width <= 0 || height <= 0 || camera_matrix.empty() ||
        distortion_coefficients.empty() || rectification_matrix.empty() ||
        projection_matrix.empty())
    {
        return false;
    }

    info.width = static_cast<unsigned>(width);
    info.height = static_cast<unsigned>(height);
    info.distortion_model = distortion_model.empty() ? "plumb_bob" : distortion_model;

    if (camera_matrix.rows * camera_matrix.cols != 9) {
        return false;
    }
    std::array<double, 9> k_vals;
    for (int r = 0; r < camera_matrix.rows; ++r) {
        for (int c = 0; c < camera_matrix.cols; ++c) {
            k_vals[r * camera_matrix.cols + c] = cvMatGetDouble(camera_matrix, r, c);
        }
    }
    info.k = k_vals;

    info.d.clear();
    for (int r = 0; r < distortion_coefficients.rows; ++r) {
        for (int c = 0; c < distortion_coefficients.cols; ++c) {
            info.d.push_back(cvMatGetDouble(distortion_coefficients, r, c));
        }
    }

    if (rectification_matrix.rows * rectification_matrix.cols != 9) {
        return false;
    }
    std::array<double, 9> r_vals;
    for (int r = 0; r < rectification_matrix.rows; ++r) {
        for (int c = 0; c < rectification_matrix.cols; ++c) {
            r_vals[r * rectification_matrix.cols + c] = cvMatGetDouble(rectification_matrix, r, c);
        }
    }
    info.r = r_vals;

    if (projection_matrix.rows * projection_matrix.cols != 12) {
        return false;
    }
    std::array<double, 12> p_vals;
    for (int r = 0; r < projection_matrix.rows; ++r) {
        for (int c = 0; c < projection_matrix.cols; ++c) {
            p_vals[r * projection_matrix.cols + c] = cvMatGetDouble(projection_matrix, r, c);
        }
    }
    info.p = p_vals;

    return true;
}

// ---------------------------------------------------------------------------

class StereoCameraNode : public rclcpp::Node
{
public:
    StereoCameraNode()
    : Node("stereo_camera_node")
    {
        // Parameters — identical to Python package
        device_path_    = declare_parameter<std::string>("device_path",    "/dev/stereo_cam0");
        fps_            = declare_parameter<double>     ("camera_fps",     30.0);
        width_          = declare_parameter<int>        ("frame_width",    1280);
        height_         = declare_parameter<int>        ("frame_height",   480);
        frame_id_left_  = declare_parameter<std::string>("frame_id_left",  "left_camera_optical");
        frame_id_right_ = declare_parameter<std::string>("frame_id_right", "right_camera_optical");
        left_camera_info_url_  = declare_parameter<std::string>("left_camera_info_url",  "");
        right_camera_info_url_ = declare_parameter<std::string>("right_camera_info_url", "");
        reconnect_sec_  = declare_parameter<double>     ("reconnect_sec",  2.0);
        baseline_m_     = declare_parameter<double>     ("baseline_m",     0.178);

        half_w_ = width_ / 2;

        // Publishers  (RELIABLE, depth 5 — same as Python)
        using Image = sensor_msgs::msg::Image;
        using CamInfo = sensor_msgs::msg::CameraInfo;
        auto qos = rclcpp::QoS(5).reliable();

        pub_left_img_   = create_publisher<Image>   ("left/image_raw",    qos);
        pub_left_info_  = create_publisher<CamInfo> ("left/camera_info",  qos);
        pub_right_img_  = create_publisher<Image>   ("right/image_raw",   qos);
        pub_right_info_ = create_publisher<CamInfo> ("right/camera_info", qos);

        // Pre-built CameraInfo (zero calibration, stamp updated per frame)
        ci_left_  = makeDefaultCameraInfo(half_w_, height_, frame_id_left_);
        ci_right_ = makeDefaultCameraInfo(half_w_, height_, frame_id_right_);
        // P[3] = -fx * baseline_m  (fx = K[0]; becomes non-zero after calibration)
        ci_right_.p[3] = -ci_right_.k[0] * baseline_m_;

        if (!left_camera_info_url_.empty()) {
            if (loadCameraInfoFromYaml(left_camera_info_url_, ci_left_)) {
                ci_left_.header.frame_id = frame_id_left_;
                RCLCPP_INFO(get_logger(), "Loaded left CameraInfo from %s", left_camera_info_url_.c_str());
            } else {
                RCLCPP_WARN(get_logger(), "Failed to load left CameraInfo from %s", left_camera_info_url_.c_str());
            }
        }

        if (!right_camera_info_url_.empty()) {
            if (loadCameraInfoFromYaml(right_camera_info_url_, ci_right_)) {
                ci_right_.header.frame_id = frame_id_right_;
                RCLCPP_INFO(get_logger(), "Loaded right CameraInfo from %s", right_camera_info_url_.c_str());
            } else {
                RCLCPP_WARN(get_logger(), "Failed to load right CameraInfo from %s", right_camera_info_url_.c_str());
            }
        }

        RCLCPP_INFO(get_logger(), "Stereo baseline: %.1f mm", baseline_m_ * 1000.0);

        // Background capture thread with auto-reconnect
        grabber_ = std::make_unique<FrameGrabber>(
            device_path_, width_, height_, fps_, reconnect_sec_,
            [this](bool ok) {
                if (ok)
                    RCLCPP_INFO(get_logger(),  "Camera connected:    %s", device_path_.c_str());
                else
                    RCLCPP_WARN(get_logger(),  "Camera disconnected: %s", device_path_.c_str());
            });

        // Publish timer
        auto period = std::chrono::duration<double>(1.0 / fps_);
        timer_ = create_wall_timer(period, [this] { onTimer(); });

        RCLCPP_INFO(get_logger(),
                    "StereoCameraNode started: %s  %dx%d  %.0f fps",
                    device_path_.c_str(), width_, height_, fps_);
    }

    ~StereoCameraNode() override
    {
        timer_.reset();
        grabber_.reset();
    }

private:
    void onTimer()
    {  
        cv::Mat wide;
        if (!grabber_->read(wide)) return;

        if (wide.cols != width_ || wide.rows != height_) return;

        auto now = get_clock()->now();

        cv::Mat left  = wide(cv::Rect(0,      0, half_w_, height_)).clone();
        cv::Mat right = wide(cv::Rect(half_w_, 0, half_w_, height_)).clone();

        cv::rotate(left,  left,  cv::ROTATE_180);
        cv::rotate(right, right, cv::ROTATE_180);

        std_msgs::msg::Header hdr;
        hdr.stamp = now;

        hdr.frame_id = frame_id_left_;
        auto img_left = cv_bridge::CvImage(hdr, "bgr8", left).toImageMsg();

        hdr.frame_id = frame_id_right_;
        auto img_right = cv_bridge::CvImage(hdr, "bgr8", right).toImageMsg();

        pub_left_img_->publish(*img_left);
        pub_right_img_->publish(*img_right);

        ci_left_.header.stamp  = now;
        ci_right_.header.stamp = now;
        pub_left_info_->publish(ci_left_);
        pub_right_info_->publish(ci_right_);
    }

    // Parameters
    std::string device_path_, frame_id_left_, frame_id_right_;
    std::string left_camera_info_url_, right_camera_info_url_;
    double      fps_, reconnect_sec_, baseline_m_;
    int         width_, height_, half_w_;

    // Publishers
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr      pub_left_img_,  pub_right_img_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr pub_left_info_, pub_right_info_;

    // CameraInfo (reused every frame — only stamp changes)
    sensor_msgs::msg::CameraInfo ci_left_, ci_right_;

    rclcpp::TimerBase::SharedPtr  timer_;
    std::unique_ptr<FrameGrabber> grabber_;
};

// ---------------------------------------------------------------------------

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<StereoCameraNode>());
    rclcpp::shutdown();
    return 0;
}
