cmake_minimum_required(VERSION 2.8.3)
project(adjustAngle)
add_compile_options(-std=c++11)
# 寻找OpenCV库
set(OpenCV_DIR /home/bubble/opencv-3.4.5)
set(OpenCV_INCLUDE_DIRS /home/bubble/opencv-3.4.5/include/opencv2)
find_package(OpenCV REQUIRED )
# 添加头文件
include_directories(/home/bubble/opencv-3.4.5/include/opencv2)
link_directories(/home/bubble/opencv-3.4.5/build/lib)
message(${OpenCV_INCLUDE_DIRS})

# 生成.so动态库文件（输出文件 SHARED 输入文件）
add_library(adjustAngle SHARED
   AdjustAngle.cpp
 )

# 主程序生成可执行文件（输出文件 输入文件）
add_executable(test  test.cpp)

# 将可执行文件与.so动态库建立连接（可执行文件 .so动态库文件）
target_link_libraries(test
  adjustAngle
  /home/bubble/opencv-3.4.5/build/lib/libopencv_calib3d.so
  /home/bubble/opencv-3.4.5/build/lib/libopencv_core.so
  /home/bubble/opencv-3.4.5/build/lib/libopencv_features2d.so
  /home/bubble/opencv-3.4.5/build/lib/libopencv_xfeatures2d.so
  /home/bubble/opencv-3.4.5/build/lib/libopencv_imgproc.so
  /home/bubble/opencv-3.4.5/build/lib/libopencv_video.so  
  /home/bubble/opencv-3.4.5/build/lib/libopencv_videoio.so
  /home/bubble/opencv-3.4.5/build/lib/libopencv_imgcodecs.so 
  /home/bubble/opencv-3.4.5/build/lib/libopencv_highgui.so 
  /home/bubble/opencv-3.4.5/build/lib/libopencv_flann.so
)

