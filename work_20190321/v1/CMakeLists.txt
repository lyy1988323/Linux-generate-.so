cmake_minimum_required(VERSION 2.8) //cmake最低版本要求
project( test1 ) //定义工程名
find_package( OpenCV REQUIRED ) //声明需要opencv第三方软件包
# 声明如何生产可执行文件（输出文件 输入文件）
add_executable( test1 machine_vision_process_v6_Liunx.cpp ) 
# 可执行文件与动态库建立链接
target_link_libraries( test1 ${OpenCV_LIBS} )

