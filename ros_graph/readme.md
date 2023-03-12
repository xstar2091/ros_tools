# ros_graph

ros_graph主要包含以下功能

* 分析依赖

## 分析依赖

使用方式

```shell
ros_graph depend [options]
```

支持的参数详见下表

|选项|默认值|说明|默认行为|
|---|---|---|---|
|package|""|分析package依赖了哪些模块，多个package以分号分隔|分析所有package的依赖包|
|workspace_dir||catkin workspace目录||

### workspace_dir

指定catkin workspace目录，指定方式有以下三种（优先级由高到底）

* workspace_dir所指定的目录
* .ros_tool所在目录
* 当前工作目录

对于ros1，workspace_dir目录结构如下

\+workspace_dir
    \+build
    \+devel
    \+src
        \-CMakeLists.txt
        \+package1
            \-CMakeLists.txt
            \-package.xml
            \+include
            \+launch
            \+src
        \+package2
        \+package3
