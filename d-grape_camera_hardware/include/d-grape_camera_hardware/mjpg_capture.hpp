#pragma once

#include <string>
#include <vector>
#include <opencv2/core.hpp>

// Direct V4L2 MJPEG capture with mmap buffers.
// Bypasses OpenCV VideoCapture which resets MJPG 1280x480 back to YUYV 640x480.
class MJPGCapture
{
public:
    MJPGCapture(const std::string & device, int width, int height,
                double fps = 30.0, int n_bufs = 2);
    ~MJPGCapture();

    // Decodes next MJPEG frame into BGR Mat. Returns false on timeout / error.
    bool read(cv::Mat & frame);

    bool isOpened() const { return fd_ >= 0; }
    void release();

    int width()  const { return width_; }
    int height() const { return height_; }

private:
    void setFormat(int w, int h);
    void setFps(double fps);
    void requestBuffers(int n);
    void queueBuffer(unsigned int idx);
    void streamOn();

    struct MmapBuf { void * start = nullptr; std::size_t length = 0; };

    int                    fd_ = -1;
    int                    width_{}, height_{};
    std::vector<MmapBuf>   bufs_;
};
