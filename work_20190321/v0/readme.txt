参考资料：https://blog.csdn.net/flyztek/article/details/73612469


1.生产动态库文件libtest.so
gcc test_a.cpp test_b.cpp test_c.cpp -fPIC -shared -o libtest.so
2.将test.cpp与动态库libtest.so链接生成执行文件test
gcc test.cpp -L. -ltest -o test
3.执行test
LD_LIBRARY_PATH=. ./test



