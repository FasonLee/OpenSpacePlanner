# OpenSpacePlanner  
## 1. ThirdParty Installation
**opencv-3.4.0**  

安装过程请参考此链接: https://docs.opencv.org/3.4.0/d7/d9f/tutorial_linux_install.html  

安装过程中若报如下错误:  

cv2.cpp:885:34: error: invalid conversion from ‘const char*’ to ‘char*’ [-fpermissive] 

则请参考此链接: https://github.com/opencv/opencv/issues/14856  

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



