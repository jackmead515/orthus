#include <iostream>
#include <chrono>

#include <opencv2/opencv.hpp>

#include "calib.h"

int main() {

    // auto build_info { cv::getBuildInformation() };

    // std::cout << build_info << std::endl;

    // std::exit(0);
    
    /*
    Input #0, video4linux2,v4l2, from '/dev/video2':
  Duration: N/A, start: 18479.441055, bitrate: N/A
  Stream #0:0: Video: mjpeg (Baseline), yuvj422p(pc, bt470bg/unknown/unknown), 2560x720, 60 fps, 60 tbr, 1000k tbn, 1000k tbc
Stream mapping:
  Stream #0:0 -> #0:0 (mjpeg (native) -> mpeg1video (native))
Press [q] to stop, [?] for help
[swscaler @ 0x60163de543c0] deprecated pixel format used, make sure you did set range correctly
Output #0, mpegts, to 'udp://0.0.0.0:3131':
  Metadata:
    encoder         : Lavf58.76.100
  Stream #0:0: Video: mpeg1video, yuv420p(tv, bt470bg/unknown/unknown, progressive), 2560x720, q=2-31, 200 kb/s, 30 fps, 90k tbn
    Metadata:
      encoder         : Lavc58.134.100 mpeg1video
    Side data:
      cpb: bitrate max/min/avg: 0/0/200000 buffer size: 0 vbv_delay: N/A
frame=  334 fps= 30 q=10.0 Lsize=   12271kB time=00:00:11.53 bitrate=8716.1kbits/s dup=0 drop=291 speed=1.04x 
     */


    /*
    """

    gst-launch-1.0 \
        -v v4l2src \
        device=/dev/video2 \
        io-mode=2 \
        do-timestamp=true \
    ! image/jpeg,framerate=30/1,width=2560,height=720,format=BGR \
    ! jpegdec \
    ! videoflip method=rotate-180 \
    ! videoconvert \
    ! decodebin \
    ! videorate \
    ! video/x-raw,framerate=10/1,format=I420 \
    ! videoconvert \
    ! avenc_mpeg1video \
    ! mpegtsmux \
    ! udpsink host=0.0.0.0 port=3131

    ! avenc_mpeg1video
    ! mpegpsmux
    ! mpeg2enc
    ! mplex \
    ! mpegtsmux \
    ! avmux_mpegts

        gst-launch-1.0 \
        -v v4l2src \
        device=/dev/video2 \
        io-mode=2 \
        do-timestamp=true \
    ! image/jpeg,framerate=30/1,width=2560,height=720,format=BGR \
    ! jpegdec \ #avdec_mjpeg
    ! videoflip method=rotate-180 \
    ! videorate \
    ! mpeg2enc disable-encode-retries=true \
    ! mplex \
    ! udpsink host=localhost port=3131

    

    """
    */

    
    //auto input_pipe = "v4l2src device=0 do-timestamp=true num-buffers=1 ! video/x-h264 ! h264parse ! queue ! rtph264pay config-interval=10 pt=96 ! fakesink"
    //auto input_pipe { "4l2src device=2 ! video/x-raw, framerate=30/1, width=2560, height=720, format=RGB ! videoconvert ! appsink" };
    //auto input_pipe { "v4l2src device=/dev/video2 io-mode=2 ! image/jpeg,framerate=30/1,width=(int)2560,height=(int)720 ! jpegdec ! video/x-raw ! videoconvert ! video/x-raw,format=BGR ! appsink" };

    //auto input_pipe { "4l2src device=/dev/video2 ! avdec_mjpeg ! videoconvert ! video/x-raw,framerate=30/1,width=(int)2560,height=(int)720,format=BGR ! appsink" };
    
    auto input_pipe { "v4l2src device=/dev/video2 io-mode=2 do-timestamp=true ! image/jpeg,framerate=30/1,width=(int)2560,height=(int)720 ! jpegdec ! videoflip method=rotate-180 ! videoconvert ! appsink" };

    //auto input_pipe { "/home/jack/Mounts/DiskOne/stereo/zed_calibration/compiled.mp4" };
    //auto input_pipe { "/dev/video2" };
    //auto input_pipe { "videotestsrc pattern=10 ! videoconvert ! appsink" };

    // stream-type=1 is-live=true
    auto output_pipe { "appsrc ! videoconvert ! avenc_mpeg1video bitrate=100000 maxrate=200000 ! mpegtsmux ! udpsink host=0.0.0.0 port=3131" };
    //auto output_pipe { "appsrc stream-type=1 is-live=true ! videoconvert ! video/x-raw,framerate=30/1,format=I420 ! videoconvert ! videorate ! video/x-raw,framerate=15/1 ! avenc_mpeg1video noise-reduction=512 bitrate=100000 ! mpegtsmux ! udpsink host=0.0.0.0 port=3131" };

    try {
        calib::calibrate(
            1000,
            input_pipe,
            output_pipe,
            cv::Size(2560, 720),
            cv::Size(10, 7),
            2.5
        );
    } catch (const std::exception& e) {
        std::cerr << e.what() << std::endl;
        std::exit(1);
    }
    
    return 0;
}