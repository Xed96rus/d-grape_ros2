#include "d-grape_camera_hardware/mjpg_capture.hpp"

#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/select.h>
#include <linux/videodev2.h>
#include <cerrno>
#include <cstring>
#include <iostream>
#include <stdexcept>

#include <opencv2/imgcodecs.hpp>

static void xioctl(int fd, unsigned long req, void * arg)
{
    int r;
    do { r = ::ioctl(fd, req, arg); } while (r == -1 && errno == EINTR);
    if (r == -1)
        throw std::runtime_error(std::string("ioctl: ") + strerror(errno));
}

MJPGCapture::MJPGCapture(const std::string & device, int width, int height,
                          double fps, int n_bufs)
: width_(width), height_(height)
{
    fd_ = ::open(device.c_str(), O_RDWR | O_NONBLOCK);
    if (fd_ < 0)
        throw std::runtime_error("Cannot open " + device + ": " + strerror(errno));

    setFormat(width, height);
    setFps(fps);
    requestBuffers(n_bufs);
    streamOn();
}

MJPGCapture::~MJPGCapture() { release(); }

void MJPGCapture::setFormat(int w, int h)
{
    v4l2_format fmt{};
    fmt.type                = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    fmt.fmt.pix.width       = static_cast<unsigned>(w);
    fmt.fmt.pix.height      = static_cast<unsigned>(h);
    fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_MJPEG;
    fmt.fmt.pix.field       = V4L2_FIELD_ANY;
    xioctl(fd_, VIDIOC_S_FMT, &fmt);

    bool ok = fmt.fmt.pix.pixelformat == V4L2_PIX_FMT_MJPEG
              && static_cast<int>(fmt.fmt.pix.width)  == w
              && static_cast<int>(fmt.fmt.pix.height) == h;
    std::cout << "  MJPGCapture: " << fmt.fmt.pix.width << "x" << fmt.fmt.pix.height
              << "  fmt=" << (ok ? "MJPG" : "OTHER")
              << (ok ? "  OK" : "  WARN: wrong resolution/format") << "\n";
}

void MJPGCapture::setFps(double fps)
{
    v4l2_streamparm parm{};
    parm.type                                  = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    parm.parm.capture.timeperframe.numerator   = 1;
    parm.parm.capture.timeperframe.denominator = static_cast<unsigned>(fps);
    try {
        xioctl(fd_, VIDIOC_S_PARM, &parm);
        std::cout << "  FPS: " << parm.parm.capture.timeperframe.denominator << "\n";
    } catch (...) {}  // some drivers don't support VIDIOC_S_PARM
}

void MJPGCapture::requestBuffers(int n)
{
    v4l2_requestbuffers req{};
    req.count  = static_cast<unsigned>(n);
    req.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_MMAP;
    xioctl(fd_, VIDIOC_REQBUFS, &req);

    bufs_.resize(req.count);
    for (unsigned i = 0; i < req.count; ++i) {
        v4l2_buffer buf{};
        buf.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index  = i;
        xioctl(fd_, VIDIOC_QUERYBUF, &buf);

        bufs_[i].length = buf.length;
        bufs_[i].start  = ::mmap(nullptr, buf.length,
                                 PROT_READ | PROT_WRITE, MAP_SHARED,
                                 fd_, buf.m.offset);
        if (bufs_[i].start == MAP_FAILED)
            throw std::runtime_error("mmap failed");
        queueBuffer(i);
    }
}

void MJPGCapture::queueBuffer(unsigned int idx)
{
    v4l2_buffer buf{};
    buf.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    buf.index  = idx;
    xioctl(fd_, VIDIOC_QBUF, &buf);
}

void MJPGCapture::streamOn()
{
    v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    xioctl(fd_, VIDIOC_STREAMON, &type);
}

bool MJPGCapture::read(cv::Mat & frame)
{
    fd_set fds;
    FD_ZERO(&fds);
    FD_SET(fd_, &fds);
    timeval tv{1, 0};  // 1 s timeout — same as Python select()

    if (::select(fd_ + 1, &fds, nullptr, nullptr, &tv) <= 0)
        return false;

    v4l2_buffer buf{};
    buf.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    if (::ioctl(fd_, VIDIOC_DQBUF, &buf) < 0)
        return false;

    auto * data = static_cast<unsigned char *>(bufs_[buf.index].start);
    cv::Mat jpeg(1, static_cast<int>(buf.bytesused), CV_8UC1, data);
    frame = cv::imdecode(jpeg, cv::IMREAD_COLOR);

    queueBuffer(buf.index);
    return !frame.empty();
}

void MJPGCapture::release()
{
    for (auto & b : bufs_)
        if (b.start && b.start != MAP_FAILED)
            ::munmap(b.start, b.length);
    bufs_.clear();

    if (fd_ >= 0) {
        ::close(fd_);
        fd_ = -1;
    }
}
