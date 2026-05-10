#include "d-grape_camera_hardware/frame_grabber.hpp"
#include "d-grape_camera_hardware/mjpg_capture.hpp"

#include <chrono>
#include <iostream>

// ---------------------------------------------------------------------------

FrameGrabber::FrameGrabber(const std::string & device,
                            int width, int height,
                            double fps, double reconnect_sec,
                            std::function<void(bool)> on_connect)
: device_(device), width_(width), height_(height),
  fps_(fps), reconnect_sec_(reconnect_sec),
  on_connect_(std::move(on_connect))
{
    thread_ = std::thread(&FrameGrabber::loop, this);
}

FrameGrabber::~FrameGrabber() { stop(); }

// ---------------------------------------------------------------------------

void FrameGrabber::stop()
{
    stop_ = true;
    if (thread_.joinable()) thread_.join();
    cap_.reset();
}

// ---------------------------------------------------------------------------

bool FrameGrabber::read(cv::Mat & frame)
{
    std::lock_guard<std::mutex> lk(mutex_);
    if (!has_frame_ || !new_frame_) return false;
    last_frame_.copyTo(frame);
    new_frame_ = false;   // потреблён — следующий read вернёт false до нового кадра
    return true;
}

// ---------------------------------------------------------------------------

void FrameGrabber::loop()
{
    constexpr int kMaxTimeouts = 5;
    int timeouts = 0;

    while (!stop_) {
        // ── (пере)подключение ────────────────────────────────────────────
        if (!cap_ || !cap_->isOpened()) {
            try {
                cap_ = std::make_unique<MJPGCapture>(device_, width_, height_, fps_);
                connected_ = true;
                timeouts   = 0;
                if (on_connect_) on_connect_(true);
                std::cout << "  FrameGrabber: connected to " << device_ << "\n";
            } catch (const std::exception & e) {
                std::cerr << "  FrameGrabber: cannot open " << device_
                          << ": " << e.what() << "\n";
                std::this_thread::sleep_for(
                    std::chrono::duration<double>(reconnect_sec_));
                continue;
            }
        }

        // ── захват кадра ─────────────────────────────────────────────────
        cv::Mat frame;
        if (cap_->read(frame)) {
            std::lock_guard<std::mutex> lk(mutex_);
            last_frame_ = std::move(frame);
            has_frame_  = true;
            new_frame_  = true;   // сигнал: новый кадр готов
            timeouts    = 0;
        } else {
            if (++timeouts >= kMaxTimeouts) {
                std::cerr << "  FrameGrabber: " << device_
                          << " timeout, reconnecting…\n";
                cap_.reset();
                connected_ = false;
                {
                    std::lock_guard<std::mutex> lk(mutex_);
                    has_frame_ = false;
                    new_frame_ = false;
                }
                if (on_connect_) on_connect_(false);
                std::this_thread::sleep_for(
                    std::chrono::duration<double>(reconnect_sec_));
                timeouts = 0;
            }
        }
    }
}
