#ifndef USB_WEBCAM_USB_WEBCAM_H
#define USB_WEBCAM_USB_WEBCAM_H

#include <asm/types.h>          /* for videodev2.h */

extern "C"
{
#include <linux/videodev2.h>
#include <libavcodec/avcodec.h>
#include <libswscale/swscale.h>
#include <libavutil/mem.h>
}

// legacy reasons
#include <libavcodec/version.h>
#if LIBAVCODEC_VERSION_MAJOR < 55
#define AV_CODEC_ID_MJPEG CODEC_ID_MJPEG
#endif

#include <string>
#include <sstream>

#include <sensor_msgs/Image.h>

namespace usb_webcam {

class UsbCam {
 public:
  typedef enum
  {
    IO_METHOD_READ, IO_METHOD_MMAP, IO_METHOD_USERPTR, IO_METHOD_UNKNOWN,
  } io_method;

  typedef enum
  {
    PIXEL_FORMAT_YUYV, PIXEL_FORMAT_UYVY, PIXEL_FORMAT_MJPEG, PIXEL_FORMAT_YUVMONO10, PIXEL_FORMAT_RGB24, PIXEL_FORMAT_GREY, PIXEL_FORMAT_UNKNOWN
  } pixel_format;

  UsbCam();
  ~UsbCam();

  // start camera
  void start(const std::string& dev, io_method io, pixel_format pf,
		    int image_width, int image_height, int framerate);
  // shutdown camera
  void shutdown(void);

  // grabs a new image from the camera
  void grab_image(sensor_msgs::Image* image);

  // enables/disable auto focus
  void set_auto_focus(int value);

  // Set video device parameters
  void set_v4l_parameter(const std::string& param, int value);
  void set_v4l_parameter(const std::string& param, const std::string& value);

  static io_method io_method_from_string(const std::string& str);
  static pixel_format pixel_format_from_string(const std::string& str);

  void stop_capturing(void);
  void start_capturing(void);
  bool is_capturing();

 private:
  typedef struct
  {
    int width;
    int height;
    int bytes_per_pixel;
    int image_size;
    char *image;
    int is_new;
  } camera_image_t;

  struct buffer
  {
    void * start;
    size_t length;
  };


  int init_mjpeg_decoder(int image_width, int image_height);
  void mjpeg2rgb(char *MJPEG, int len, char *RGB, int NumPixels);
  void process_image(const void * src, int len, camera_image_t *dest);
  int read_frame();
  void uninit_device(void);
  void init_read(unsigned int buffer_size);
  void init_mmap(void);
  void init_userp(unsigned int buffer_size);
  void init_device(int image_width, int image_height, int framerate);
  void close_device(void);
  void open_device(void);
  void grab_image();
  bool is_capturing_;


  std::string camera_dev_;
  unsigned int pixelformat_;
  bool monochrome_;
  io_method io_;
  int fd_;
  buffer * buffers_;
  unsigned int n_buffers_;
  AVFrame *avframe_camera_;
  AVFrame *avframe_rgb_;
  AVCodec *avcodec_;
  AVDictionary *avoptions_;
  AVCodecContext *avcodec_context_;
  int avframe_camera_size_;
  int avframe_rgb_size_;
  struct SwsContext *video_sws_;
  camera_image_t *image_;

};

}

#endif

