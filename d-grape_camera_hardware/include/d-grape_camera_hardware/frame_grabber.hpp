#pragma once

#include <atomic>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <opencv2/core.hpp>

class MJPGCapture;

// Captures frames in a background thread with automatic reconnection.
class FrameGrabber
{
public:
    FrameGrabber(const std::string & device, int width, int height,
                 double fps, double reconnect_sec,
                 std::function<void(bool)> on_connect = nullptr);
    ~FrameGrabber();

    // Thread-safe: copies the latest frame. Returns false if no frame yet.
    bool read(cv::Mat & frame);

    bool isConnected() const { return connected_.load(); }
    void stop();

private:
    void loop();

    std::string               device_;
    int                       width_, height_;
    double                    fps_, reconnect_sec_;
    std::function<void(bool)> on_connect_;

    std::unique_ptr<MJPGCapture> cap_;
    cv::Mat                      last_frame_;
    bool                         has_frame_ = false;
    std::mutex                   mutex_;
    std::atomic<bool>            stop_{false};
    std::atomic<bool>            connected_{false};
    std::thread                  thread_;
};
