# Usage  
## 1. ThirdParty Installation
**opencv-3.4.0**  

下载：  
https://github.com/opencv/opencv/tree/3.4.0  

安装：  
```
cd opencv-3.4.0
mkdir build
cd build
cmake -D CMAKE_BUILD_TYPE=RELEASE -D CMAKE_INSTALL_PREFIX=/usr/local ..
make -j8
sudo make install
```
安装过程中若报此错误: *cv2.cpp:885:34: error: invalid conversion from ‘const char*’ to ‘char*’ [-fpermissive]*  

则需要将```opencv-3.4.0/modules/python/src2/cv2.cpp```文件中如下行:

```
char* str = PyString_AsString(obj);
```
改写为：
```
const char* str = PyString_AsString(obj);
```

## 2. Compile & Run  
在/tmp 路径下新建 "OpenSpacePlannerDebug" 文件夹,以存放每次搜索过程的图片  
```
mkdir /tmp/OpenSpacePlannerDebug
```
编译&运行
```
bash ./scripts/build.sh
./test/bin/test_open_space_planner
```



