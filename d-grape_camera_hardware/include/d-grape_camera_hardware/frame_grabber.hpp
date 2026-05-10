#pragma once

#include <string>
#include <thread>
#include <mutex>
#include <atomic>
#include <functional>
#include <memory>

#include <opencv2/core.hpp>

#include "d-grape_camera_hardware/mjpg_capture.hpp"

// ---------------------------------------------------------------------------
// FrameGrabber — захват кадров в фоновом потоке с авто-переподключением.
//
// read() возвращает true ТОЛЬКО при наличии НОВОГО кадра с момента
// предыдущего вызова. После успешного read() флаг сбрасывается, и
// следующий вызов вернёт false пока камера не пришлёт новый кадр.
// ---------------------------------------------------------------------------

class FrameGrabber
{
public:
    FrameGrabber(const std::string & device,
                 int width, int height,
                 double fps,
                 double reconnect_sec,
                 std::function<void(bool)> on_connect = {});

    ~FrameGrabber();

    /// Копирует последний НОВЫЙ кадр в frame и сбрасывает флаг новизны.
    /// Возвращает false если новых кадров не было с прошлого вызова.
    bool read(cv::Mat & frame);

    bool isConnected() const { return connected_; }

    void stop();

private:
    void loop();

    // -- параметры --
    std::string  device_;
    int          width_, height_;
    double       fps_, reconnect_sec_;
    std::function<void(bool)> on_connect_;

    // -- поток и синхронизация --
    std::thread              thread_;
    std::mutex               mutex_;
    cv::Mat                  last_frame_;
    bool                     has_frame_ {false};
    bool                     new_frame_ {false};   ///< сбрасывается в read()

    std::atomic<bool>        stop_      {false};
    std::atomic<bool>        connected_ {false};

    std::unique_ptr<MJPGCapture> cap_;
};
