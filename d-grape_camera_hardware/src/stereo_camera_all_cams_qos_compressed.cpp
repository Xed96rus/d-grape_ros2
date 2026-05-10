/**
 * stereo_cameras_node.cpp
 *
 * Один ROS 2 узел — все три стерео-пары, единый таймер → единый timestamp.
 *
 * Параметры запуска:
 *   camera_fps            — FPS (30.0)
 *   frame_width/height    — размер SBS кадра (1280 × 480)
 *   baseline_m            — база стерео в метрах (0.178)
 *   config_root           — путь к директории с YAML калибровками
 *   publish_raw           — публиковать sensor_msgs/Image          (true)
 *   publish_compressed    — публиковать sensor_msgs/CompressedImage (false)
 *   jpeg_quality          — качество JPEG для CompressedImage 1-100 (80)
 *   img_reliability       — QoS изображений: "best_effort" / "reliable" ("best_effort")
 *   img_depth             — глубина очереди изображений              (1)
 *   info_reliability      — QoS camera_info: "best_effort" / "reliable" ("reliable")
 *   info_depth            — глубина очереди camera_info              (10)
 *
 * Топики (compressed добавляет суффикс /compressed к пути image):
 *   /front_cams/f_left_camera/image[/compressed]   + /camera_info
 *   /front_cams/f_right_camera/image[/compressed]  + /camera_info
 *   /left_cams/l_left_camera/image[/compressed]    + /camera_info
 *   /left_cams/l_right_camera/image[/compressed]   + /camera_info
 *   /right_cams/r_left_camera/image[/compressed]   + /camera_info
 *   /right_cams/r_right_camera/image[/compressed]  + /camera_info
 */

#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include <opencv2/imgcodecs.hpp>
#include <yaml-cpp/yaml.h>

#include "d-grape_camera_hardware/frame_grabber.hpp"

// ─── утилита: построить QoS по строковым параметрам ─────────────────────────

static rclcpp::QoS makeQoS(const std::string & reliability, int depth)
{
    rclcpp::QoS qos(depth);
    if (reliability == "reliable")
        qos.reliable();
    else
        qos.best_effort();
    return qos;
}

// ─── утилиты калибровки ──────────────────────────────────────────────────────

static sensor_msgs::msg::CameraInfo makeDefaultCameraInfo(
    int width, int height, const std::string & frame_id)
{
    sensor_msgs::msg::CameraInfo ci;
    ci.width            = static_cast<unsigned>(width);
    ci.height           = static_cast<unsigned>(height);
    ci.header.frame_id  = frame_id;
    ci.distortion_model = "plumb_bob";

    const double fov_rad = 90.0 * M_PI / 180.0;
    const double fx = (width / 2.0) / std::tan(fov_rad / 2.0);
    const double cx = width  / 2.0;
    const double cy = height / 2.0;

    ci.d = {0.0, 0.0, 0.0, 0.0, 0.0};
    ci.k = {fx,  0.0, cx,  0.0, fx,  cy,  0.0, 0.0, 1.0};
    ci.r = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
    ci.p = {fx,  0.0, cx,  0.0, 0.0, fx,  cy,  0.0, 0.0, 0.0, 1.0, 0.0};
    return ci;
}

static bool loadCameraInfoFromYaml(
    const std::string & path,
    sensor_msgs::msg::CameraInfo & info)
{
    YAML::Node doc;
    try { doc = YAML::LoadFile(path); }
    catch (const YAML::Exception &) { return false; }

    if (!doc["image_width"] || !doc["image_height"]) return false;

    info.width  = doc["image_width"].as<int>();
    info.height = doc["image_height"].as<int>();

    if (doc["distortion_model"])
        info.distortion_model = doc["distortion_model"].as<std::string>();

    auto readVec = [&](const std::string & key, size_t n, auto & arr) -> bool {
        if (!doc[key] || !doc[key]["data"]) return false;
        auto d = doc[key]["data"];
        if (d.size() != n) return false;
        for (size_t i = 0; i < n; ++i) arr[i] = d[i].as<double>();
        return true;
    };

    std::array<double, 9>  k{}, r{};
    std::array<double, 12> p{};
    if (!readVec("camera_matrix",        9,  k)) return false;
    if (!readVec("rectification_matrix", 9,  r)) return false;
    if (!readVec("projection_matrix",    12, p)) return false;
    info.k = k; info.r = r; info.p = p;

    if (doc["distortion_coefficients"] && doc["distortion_coefficients"]["data"]) {
        info.d.clear();
        for (auto v : doc["distortion_coefficients"]["data"])
            info.d.push_back(v.as<double>());
    }
    return true;
}

// ─── структура одной стерео-пары ─────────────────────────────────────────────

using ImgPub  = rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr;
using CmpPub  = rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr;
using InfoPub = rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr;

struct StereoPair
{
    std::string ns;
    std::string device;
    std::string frame_id_left;
    std::string frame_id_right;

    std::unique_ptr<FrameGrabber> grabber;

    // raw Image (опционально)
    ImgPub  pub_left_img;
    ImgPub  pub_right_img;

    // CompressedImage (опционально)
    CmpPub  pub_left_cmp;
    CmpPub  pub_right_cmp;

    // CameraInfo (всегда)
    InfoPub pub_left_info;
    InfoPub pub_right_info;

    sensor_msgs::msg::CameraInfo ci_left;
    sensor_msgs::msg::CameraInfo ci_right;
};

// ─── нода ────────────────────────────────────────────────────────────────────

class StereoCamerasNode : public rclcpp::Node
{
public:
    StereoCamerasNode()
    : Node("stereo_cameras_node")
    {
        // ── параметры ────────────────────────────────────────────────────
        fps_    = declare_parameter<double>("camera_fps",    30.0);
        width_  = declare_parameter<int>   ("frame_width",  1280);
        height_ = declare_parameter<int>   ("frame_height", 480);
        half_w_ = width_ / 2;

        const double reconnect_sec  = declare_parameter<double>("reconnect_sec",  2.0);
        const double baseline_m     = declare_parameter<double>("baseline_m",     0.178);
        const std::string cfg_root  = declare_parameter<std::string>("config_root", "");

        pub_raw_        = declare_parameter<bool>  ("publish_raw",        true);
        pub_compressed_ = declare_parameter<bool>  ("publish_compressed", false);
        jpeg_quality_   = declare_parameter<int>   ("jpeg_quality",       80);

        // QoS параметры
        const std::string img_rel  = declare_parameter<std::string>("img_reliability",  "best_effort");
        const int         img_dep  = declare_parameter<int>        ("img_depth",        1);
        const std::string info_rel = declare_parameter<std::string>("info_reliability", "reliable");
        const int         info_dep = declare_parameter<int>        ("info_depth",       10);

        if (!pub_raw_ && !pub_compressed_)
            RCLCPP_WARN(get_logger(),
                "publish_raw и publish_compressed оба false — топики публиковаться не будут!");

        // QoS объекты
        const auto img_qos  = makeQoS(img_rel,  img_dep);
        const auto cmp_qos  = makeQoS(img_rel,  std::max(img_dep, 5));  // буфер чуть больше для сети
        const auto info_qos = makeQoS(info_rel, info_dep);

        RCLCPP_INFO(get_logger(),
            "QoS image: %s/%d  info: %s/%d  jpeg_quality: %d",
            img_rel.c_str(), img_dep, info_rel.c_str(), info_dep, jpeg_quality_);
        RCLCPP_INFO(get_logger(),
            "Publish: raw=%s  compressed=%s",
            pub_raw_ ? "YES" : "no", pub_compressed_ ? "YES" : "no");

        // ── конфигурация трёх пар ────────────────────────────────────────
        struct CamCfg { std::string ns, dev, pfx, ly, ry; };
        const std::vector<CamCfg> cfgs = {
            {"front_cams", "/dev/front_cams", "f",
             "front_cal/left.yaml",  "front_cal/right.yaml"},
            {"left_cams",  "/dev/left_cams",  "l",
             "left_cal/left.yaml",   "left_cal/right.yaml"},
            {"right_cams", "/dev/right_cams", "r",
             "right_cal/left.yaml",  "right_cal/right.yaml"},
        };

        for (const auto & c : cfgs) {
            StereoPair p;
            p.ns             = c.ns;
            p.device         = c.dev;
            p.frame_id_left  = c.pfx + "_left_camera";
            p.frame_id_right = c.pfx + "_right_camera";

            const auto t = [&](const std::string & s) {
                return "/" + c.ns + "/" + s;
            };

            // raw Image паблишеры
            if (pub_raw_) {
                p.pub_left_img  = create_publisher<sensor_msgs::msg::Image>(
                    t(c.pfx + "_left_camera/image"),  img_qos);
                p.pub_right_img = create_publisher<sensor_msgs::msg::Image>(
                    t(c.pfx + "_right_camera/image"), img_qos);
            }

            // CompressedImage паблишеры (топик = image + /compressed)
            if (pub_compressed_) {
                p.pub_left_cmp  = create_publisher<sensor_msgs::msg::CompressedImage>(
                    t(c.pfx + "_left_camera/image/compressed"),  cmp_qos);
                p.pub_right_cmp = create_publisher<sensor_msgs::msg::CompressedImage>(
                    t(c.pfx + "_right_camera/image/compressed"), cmp_qos);
            }

            // CameraInfo — всегда
            p.pub_left_info  = create_publisher<sensor_msgs::msg::CameraInfo>(
                t(c.pfx + "_left_camera/camera_info"),  info_qos);
            p.pub_right_info = create_publisher<sensor_msgs::msg::CameraInfo>(
                t(c.pfx + "_right_camera/camera_info"), info_qos);

            // ── калибровка ───────────────────────────────────────────────
            p.ci_left  = makeDefaultCameraInfo(half_w_, height_, p.frame_id_left);
            p.ci_right = makeDefaultCameraInfo(half_w_, height_, p.frame_id_right);
            p.ci_right.p[3] = -p.ci_right.k[0] * baseline_m;

            if (!cfg_root.empty()) {
                auto tryLoad = [&](const std::string & yaml,
                                   sensor_msgs::msg::CameraInfo & ci,
                                   const std::string & fid) {
                    const std::string full = cfg_root + "/" + yaml;
                    if (loadCameraInfoFromYaml(full, ci)) {
                        ci.header.frame_id = fid;
                        RCLCPP_INFO(get_logger(), "[%s] Loaded CameraInfo: %s",
                                    c.ns.c_str(), full.c_str());
                    } else {
                        RCLCPP_WARN(get_logger(), "[%s] Cannot load CameraInfo: %s",
                                    c.ns.c_str(), full.c_str());
                    }
                };
                tryLoad(c.ly, p.ci_left,  p.frame_id_left);
                tryLoad(c.ry, p.ci_right, p.frame_id_right);
            }

            // ── FrameGrabber ─────────────────────────────────────────────
            p.grabber = std::make_unique<FrameGrabber>(
                c.dev, width_, height_, fps_, reconnect_sec,
                [this, ns = c.ns](bool ok) {
                    if (ok) RCLCPP_INFO(get_logger(), "[%s] Camera connected",    ns.c_str());
                    else    RCLCPP_WARN(get_logger(), "[%s] Camera disconnected", ns.c_str());
                });

            RCLCPP_INFO(get_logger(),
                "StereoPair [%s]  device=%s  baseline=%.1f mm",
                c.ns.c_str(), c.dev.c_str(), baseline_m * 1000.0);

            pairs_.push_back(std::move(p));
        }

        // ── ЕДИНЫЙ ТАЙМЕР ────────────────────────────────────────────────
        const auto period = std::chrono::duration<double>(1.0 / fps_);
        timer_ = create_wall_timer(period, [this] { onTimer(); });

        RCLCPP_INFO(get_logger(),
            "StereoCamerasNode ready  %dx%d  %.0f fps", width_, height_, fps_);
    }

    ~StereoCamerasNode() override
    {
        timer_.reset();
        pairs_.clear();
    }

private:
    // ─────────────────────────────────────────────────────────────────────
    void onTimer()
    {
        // Единый timestamp для всех трёх пар в этом тике
        const rclcpp::Time now = get_clock()->now();

        for (auto & p : pairs_) {
            cv::Mat wide;
            if (!p.grabber->read(wide)) continue;   // нет нового кадра

            if (wide.cols != width_ || wide.rows != height_) {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                    "[%s] Unexpected frame size %dx%d", p.ns.c_str(),
                    wide.cols, wide.rows);
                continue;
            }

            // ── разрезать SBS пополам и повернуть ────────────────────────
            cv::Mat left  = wide(cv::Rect(0,       0, half_w_, height_)).clone();
            cv::Mat right = wide(cv::Rect(half_w_, 0, half_w_, height_)).clone();
            cv::rotate(left,  left,  cv::ROTATE_180);
            cv::rotate(right, right, cv::ROTATE_180);

            // ── заголовки (один stamp для left и right) ───────────────────
            std_msgs::msg::Header hl, hr;
            hl.stamp = hr.stamp = now;
            hl.frame_id = p.frame_id_left;
            hr.frame_id = p.frame_id_right;

            // ── raw Image ─────────────────────────────────────────────────
            if (pub_raw_) {
                p.pub_left_img->publish(
                    *cv_bridge::CvImage(hl, "bgr8", left).toImageMsg());
                p.pub_right_img->publish(
                    *cv_bridge::CvImage(hr, "bgr8", right).toImageMsg());
            }

            // ── CompressedImage ───────────────────────────────────────────
            if (pub_compressed_) {
                p.pub_left_cmp->publish(
                    encodeJpeg(hl, left));
                p.pub_right_cmp->publish(
                    encodeJpeg(hr, right));
            }

            // ── CameraInfo (всегда, тот же stamp) ────────────────────────
            p.ci_left.header.stamp  = now;
            p.ci_right.header.stamp = now;
            p.pub_left_info->publish(p.ci_left);
            p.pub_right_info->publish(p.ci_right);
        }
    }

    // ── вспомогательный метод: кодировать cv::Mat в CompressedImage ──────
    sensor_msgs::msg::CompressedImage encodeJpeg(
        const std_msgs::msg::Header & header,
        const cv::Mat & img) const
    {
        sensor_msgs::msg::CompressedImage msg;
        msg.header = header;
        msg.format = "jpeg";

        const std::vector<int> params = {
            cv::IMWRITE_JPEG_QUALITY, jpeg_quality_
        };
        cv::imencode(".jpg", img, msg.data, params);
        return msg;
    }

    // ── поля ──────────────────────────────────────────────────────────────
    std::vector<StereoPair>      pairs_;
    rclcpp::TimerBase::SharedPtr timer_;

    double fps_;
    int    width_, height_, half_w_;
    bool   pub_raw_;
    bool   pub_compressed_;
    int    jpeg_quality_;
};

// ─── main ────────────────────────────────────────────────────────────────────

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<StereoCamerasNode>());
    rclcpp::shutdown();
    return 0;
}
