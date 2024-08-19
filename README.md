- [readme](#readme)

# readme

## ros_graph

ros_graph is a command line tool to analyze the dependencies of the ros1 programs. ros_graph analyzes the dependencies of the ros1 program and shows the topological order of the dependencies.

install dependencies on Ubuntu

```shell
sudo apt install -y libboost-all-dev libgtest-dev
```

build on Ubuntu

```shell
cd ros_tools
mkdir build
cd build
# build without unittest
cmkae -DUNITTEST=OFF ..
# build with unittest
# cmake -DUNITTEST=ON ..
make
```

Show usage

```shell
./ros_graph/ros_graph
usage:
./ros_graph/ros_graph <command> [options]

version: 0.1.0

command list:
depend:       显示指定包的依赖
dependby:     显示指定包被哪些包依赖
dependtree:   depend命令的树形显示
dependtreeby: dependby命令的树形显示

options:
    --indent        树形显示的缩进长度
    --level         分析深度
    --package       指定的包
    --separator     输出分隔符
    --workspace_dir catkin workspace 目录
```

Show topological order of navigation program dependencies.

```shell
~$ ros_graph depend --workspace_dir=navi --package=navigation
mapsrv;common;mapparser;topomap;costmap2d;navigationcore;semanticmap;navigation
```

Show as tree

In this case, ros1 node navigation depends on semanticmap, ros1 node semanticmap depends on mapparser, common and costmap2d, and ros1 node mapparser depends on mapsrv, and so on.

```shell
~$ ros_graph dependtree --workspace_dir=navi --package=navigation
navigation
    common
    semanticmap
        mapparser
            mapsrv
        common
        costmap2d
            mapparser
                mapsrv
            topomap
                mapparser
                    mapsrv
                mapsrv
                common
            mapsrv
            common
    mapparser
        mapsrv
    navigationcore
        costmap2d
            mapparser
                mapsrv
            topomap
                mapparser
                    mapsrv
                mapsrv
                common
            mapsrv
            common
    costmap2d
        mapparser
            mapsrv
        topomap
            mapparser
                mapsrv
            mapsrv
            common
        mapsrv
        common
```

See ros1 node common is depended by which nodes

```shell
~$ ./ros_tools/build/ros_graph/ros_graph dependby --workspace_dir=navi --package=common
common;topomap;costmap2d;semanticmap;navigationcore;navigation
```

Show as tree

```shell
~$ ros_graph dependtreeby --workspace_dir=navi --package=common
common
    semanticmap
        navigation
    costmap2d
        semanticmap
            navigation
        navigation
        navigationcore
            navigation
    navigation
    topomap
        costmap2d
            semanticmap
                navigation
            navigation
            navigationcore
                navigation
```

In this case, common is depended by semanticmap, and semanticmap is depended by navigation, and so on.

## mapmgr

mapmgr is a qt program for loading maps and cross-map navigation. mapmgr application tool has these features.

* load maps: load maps from user coustom path
* quick load maps: load maps from default path
* set start map: starting point for navigation on multiple maps
* set goal map: goal point for navigation on multiple maps
* multimap navigation: navigation on multiple maps(requires backend service support)

build

Compiling the mapmgr program requires ubuntu 18.04, ros melodic.
