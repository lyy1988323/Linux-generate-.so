首先测试在windows下的源程序是否能在Linux下运行
1.打开.cpp主程序：把涉及到路径的分隔符由“\\”修改为“/”；main函数中两个固定路径位置更换
2.打开.h头文件：把“#include <xfeatures2d.hpp>”修改为“#include <opencv2/xfeatures2d.hpp>”
3.编写了一个最简单的“CMakeLists.txt”文件
4.在该文件夹下打开命令行，执行“cmake ."
  最后一个小点别忘了，这个小点的意思是在当前文件夹下进行编译
5.生成可执行文件，命令行执行“make”
6.执行可执行文件，命令行输入“./test1”
