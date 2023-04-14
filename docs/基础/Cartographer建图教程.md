# Cartographer 建图教程

这篇文章将介绍如何使用 Cartographer 来进行建图。

## 本地环境搭建

Cartographer 是一个独立的C++库，想要快速入门可以使用 [ROS](https://www.ros.org/) 集成。

因此，你需要：
- ROS
- Cartographer

### 安装 ROS

你可以在[这里](http://wiki.ros.org/cn/noetic/Installation/Ubuntu)查询 ROS 安装方法。

ROS 安装版本为**完整桌面版安装（Desktop-Full)**。

### 安装 Cartographer

你可以在[这里](https://google-Cartographer-ros.readthedocs.io/en/latest/compilation.html#building-installation)查询 Cartographer 安装方法。

Cartographer 的[**官方文档**](https://google-Cartographer-ros.readthedocs.io/en/latest/)地址。

## Cartographer 具体参数配置

### 创建 .lua 配置

每个机器人的配置必须从 Lua 脚本定义的数据结构读取的， 所以你需要配置不同的 .lua 文件来对应不同的机器人。  

官方示例配置都在：   
`src/cartographer_ros/cartographer_ros/configuration_files` 和  
`install_isolated/share/cartographer_ros/configuration_files/` 中。

如果想使用3D建图：

```bash
# 复制一份示例代码进行修改
cp install_isolated/share/cartographer_ros/configuration_files/backpack_3d.lua install_isolated/share/cartographer_ros/configuration_files/my_robot.lua
```

如果想使用2D建图：

```bash
# 复制一份示例代码进行修改
cp install_isolated/share/cartographer_ros/configuration_files/backpack_2d.lua install_isolated/share/cartographer_ros/configuration_files/my_robot.lua
```

具体配置可参考[ Lua 配置参考文档](https://google-cartographer-ros.readthedocs.io/en/latest/configuration.html)以及[Cartographer配置值](https://google-cartographer.readthedocs.io/en/latest/configuration.html)。

### 创建 .launch 配置

Cartographe r的推荐用法是为每个机器人都提供不同的 .launch 文件。官方示例 .launch 文件在`src/cartographer_ros/cartographer_ros/configuration_files` 和 `install_isolated/share/cartographer_ros/configuration_files/` 中。

可以先复制官方提供的示例之一：

```bash
cp install_isolated/share/cartographer_ros/launch/backpack_3d.launch install_isolated/share/cartographer_ros/launch/my_robot.launch
cp install_isolated/share/cartographer_ros/launch/demo_backpack_3d.launch install_isolated/share/cartographer_ros/launch/demo_my_robot.launch
cp install_isolated/share/cartographer_ros/launch/offline_backpack_3d.launch install_isolated/share/cartographer_ros/launch/offline_my_robot.launch
cp install_isolated/share/cartographer_ros/launch/demo_backpack_3d_localization.launch install_isolated/share/cartographer_ros/launch/demo_my_robot_localization.launch
cp install_isolated/share/cartographer_ros/launch/assets_writer_backpack_3d.launch install_isolated/share/cartographer_ros/launch/assets_writer_my_robot.launch
```

每个 .launch 文件的具体使用方法可查看[这里](https://google-cartographer-ros.readthedocs.io/en/latest/your_bag.html#create-launch-files-for-your-slam-scenarios)。

## 运行自己的 Cartographer ROS 

***特别注意：不要在 src 下写自己的代码，不然重新编译时可能会失败。官方教程也是在install_isolated/share 文件夹下修改代码。***

***温馨提示：请大家在接下来的操作中做好备份，以防文件丢失。*** 

***默认大家的 Cartographer 的工作空间为 catkin_ws*** 

***默认大家所使用的雷达为 lsn10，雷达的功能包为 lsn10*** 

### 2D 建图 + 激光雷达

**1. 在 Cartographer ROS 中使用雷达**

在 Cartographer 的工作空间下编译 lsn10 功能包。

将**雷达的功能包**放至 `catkin_ws/src/cartographer_ros` 下, 。

运行以下命令，编译新的工作空间：

```bash
# 重新编译工作空间
# 如果碰到报错，请删除所有已编译出来的文件，再次编译
catkin_make_isolated --install --use-ninja
```

看到 **Finish** 字眼说明编译成功。
![catkin lsn10 and Cartographer ROS](../../static/img/cartographer建图教程/catkin_lsn10.png)


**2. 创建 .lua 配置文件**

运行下列命令，复制 **revo_lds.lua** 文件作为自己的新的 .lua 配置文件。

```bash
# 进入 Cartographer 工作空间
cd catkin_ws
# 复制2D建图 revo_lds.lua 文件  
cp install_isolated/share/cartographer_ros/configuration_files/revo_lds.lua install_isolated/share/cartographer_ros/configuration_files/my_robot_laser_only.lua
```

将参数 `tracking_frame` 修改成雷达的**frame_id**，即 `laser`。

将参数 `published_frame` 修改成底盘的**frame_id**，即 `base_link`。

将参数 `provide_odom_frame` 设置为 **false**。  
没有用到里程计，所以将其设置成 false。

**<span id="lable"></span>**

`tracking_frame:` SLAM 算法跟踪的帧的 ROS 帧 ID。  
​tracking_frame 一般设置为发布频率最高的传感器的 frame_id，Cartographer 将会把其他数据都转移到该坐标系下进行计算。

`published_frame:` 要用作发布坐标的子帧的 ROS 帧 ID。  
如果 `odom` 框架由系统的不同部分提供，则设置为 `odom`。  
在这种情况下将发布 map_frame 中 `odom` 的坐标。否则，将其设置为 `base_link` 可能是合适的。   


具体代码如下：

```bash
tracking_frame = "laser"
published_frame = "base_link"

provide_odom_frame = false
```

将下列参数 `TRAJECTORY_BUILDER_2D.min_range` , `TRAJECTORY_BUILDER_2D.max_range` 修改成你所用的雷达的具体扫描参数。

具体代码如下：

```bash
TRAJECTORY_BUILDER_2D.min_range = 0.3
TRAJECTORY_BUILDER_2D.max_range = 11.
```

**3. 创建 .launch 配置文件**

运行下列命令，复制 **demo_revo_lds.launch** 文件作为自己的新的 .launch 配置文件。

```bash
# 进入 Cartographer 工作空间
cd catkin_ws
# 复制2D建图 demo_revo_lds.launch 文件
cp install_isolated/share/cartographer_ros/launch/demo_revo_lds.launch install_isolated/share/cartographer_ros/launch/my_robot_laser_only.launch
```

修改 `cartographer_node` 节点，映射雷达的 **topic**，即 `/scan`。

将 `rviz` 节点切换至你所用的 .rviz 文件。

关闭运行 `bag` 的节点。

增加 `map` 与 `laser` 的 **tf** 关系节点。

具体代码如下：

```bash
<launch>
  <param name="/use_sim_time" value="true" />

  <node name="cartographer_node" pkg="cartographer_ros"
      type="cartographer_node" args="
          -configuration_directory $(find cartographer_ros)/configuration_files
          -configuration_basename my_robot_only_laser.lua"
      output="screen">
    <remap from="scan" to="/scan" />
  </node>

  <node name="cartographer_occupancy_grid_node" pkg="cartographer_ros"
      type="cartographer_occupancy_grid_node" args="-resolution 0.05" />

  <node pkg="tf2_ros" type="static_transform_publisher" name="laser_broadcaster" 
      args="0 0 0 0 0 0 map laser" />

  <!-- 请修改成自己的 rviz 路径 -->      
  <node name="rviz" pkg="rviz" type="rviz" required="true"
      args="/home/zr/.ros/laser.rviz" />
</launch>
```

**<span id="lable">4. 合并雷达的功能包与 Cartographer ROS</span>**

将 lsn10 节点写进上述 .launch 文件。

节点代码如下：

```bash 
  <include file="$(find lsn10)/launch/lsn10.launch" />
```

整体 `my_robot_laser_only.launch` 代码如下：

```bash
<launch>
  <include file="$(find lsn10)/launch/lsn10.launch" />

  <param name="/use_sim_time" value="true" />

  <node name="cartographer_node" pkg="cartographer_ros"
      type="cartographer_node" args="
          -configuration_directory $(find cartographer_ros)/configuration_files
          -configuration_basename my_robot_only_laser.lua"
      output="screen">
    <remap from="scan" to="/scan" />
  </node>

  <node name="cartographer_occupancy_grid_node" pkg="cartographer_ros"
      type="cartographer_occupancy_grid_node" args="-resolution 0.05" />

  <node pkg="tf2_ros" type="static_transform_publisher" name="laser_broadcaster" 
      args="0 0 0 0 0 0 map laser" />
      
  <!-- 请修改成自己的 rviz 路径 -->      
  <node name="rviz" pkg="rviz" type="rviz" required="true"
      args="/home/zr/.ros/laser.rviz" />
</launch>
```

**5. 运行建图程序**

运行以下命令，启动建图程序。

```bash
# 进入 Cartographer 工作空间
cd catkin_ws
# 千万不要编译！！！否则你刚写的代码都没啦！！！
# 具体原因为你的 src 里没有改动，所以编译的话会将你的 install_isolated 替换，就恢复成原始状态了。
# source 工作空间环境，使用 zsh 的同学别忘了切换成 zsh
source install_isolated/setup.bash 
# 运行程序
roslaunch cartographer_ros my_robot_laser_only.launch
```

**6. 在 rviz 中添加 topic**

添加 `Submaps` 话题即可看到雷达建图。
![Submaps](../../static/img/cartographer%E5%BB%BA%E5%9B%BE%E6%95%99%E7%A8%8B/Submaps.png
)

其他 topic 可自行添加。

主要使用的 topic 为 `Map` 、`TF`、`LaserScan` 、`MarkArray` 。 

最终运行效果如下：
![retult of laser](../../static/img/cartographer%E5%BB%BA%E5%9B%BE%E6%95%99%E7%A8%8B/result_of_laser.png)

### 2D 建图 + 激光雷达 + 陀螺仪

**1. 创建 .lua 配置文件**

运行下列命令，复制 **revo_lds.lua** 文件作为自己的新的 .lua 配置文件。

```bash
# 进入 Cartographer 工作空间
cd catkin_ws
# 复制2D建图 revo_lds.lua 文件  
cp install_isolated/share/cartographer_ros/configuration_files/revo_lds.lua install_isolated/share/cartographer_ros/configuration_files/my_robot_laser_imu.lua
```

将参数 `tracking_frame` 修改成陀螺仪的 **frame_id**。我这里使用的陀螺仪的 frame_id 为 `imu_link`。

将参数 `publisher_frame` 修改成底盘的 **frame_id**。我这里使用的底盘的 frame_id 为 `base_link`。

这两个参数具体解释可以看<a href="#lable">**这里**</a>。

将参数 `provide_odom_frame` 设置为 **false**。  
没有用到里程计，所以将其设置成 false。

具体代码如下：

```bash
tracking_frame = "imu_link"
published_frame = "base_link"

provide_odom_frame = "false"
```

将下列参数 `TRAJECTORY_BUILDER_2D.min_range` , `TRAJECTORY_BUILDER_2D.max_range` 修改成你所用的雷达的具体扫描参数。

具体代码如下：

```bash
TRAJECTORY_BUILDER_2D.min_range = 0.3
TRAJECTORY_BUILDER_2D.max_range = 11.
```

**2. 创建 .urdf 配置文件**

URDF 是一种基于 XML 规范、用于描述机器人结构的格式。

所以你需要配置一个 .urdf 文件来适配不同的机器人。

你可以在[这里](http://wiki.ros.org/urdf/XML)查询 .urdf 文件的写法。

这里提供我所使用的 my_robot_laser_imu.urdf 配置文件：

```bash
<robot name="my_robot">
  <material name="orange">
    <color rgba="1.0 0.5 0.2 1" />
  </material>
  <material name="gray">
    <color rgba="0.2 0.2 0.2 1" />
  </material>

<link name="base_link" />

<link name="imu_link">
    <visual>
      <origin xyz="0.0 0.0 0.0" />
      <geometry>
        <box size="0.06 0.04 0.02" />
      </geometry>
      <material name="orange" />
    </visual>
</link>

<link name="laser">
    <visual>
      <origin xyz="0.0 0.0 0.0" />
      <geometry>
        <cylinder length="0.07" radius="0.3" />
      </geometry>
      <material name="gray" />
    </visual>
</link>

<joint name="imu_link_joint" type="fixed">
    <parent link="base_link" />
    <child link="imu_link" />
    <origin xyz="0.0 0.0 0.0" />
</joint>

<joint name="laser_link_joint" type="fixed">
    <parent link="base_link" />
    <child link="laser" />
    <origin xyz="0.0 0.0 0.0" />
</joint>

</robot>
```

**3. 创建 .launch 配置文件**

运行下列命令，复制 **demo_revo_lds.launch** 文件作为自己的新的 .launch 配置文件。

```bash
# 进入 Cartographer 工作空间
cd catkin_ws
# 复制2D建图 demo_revo_lds.launch 文件
cp install_isolated/share/cartographer_ros/launch/demo_revo_lds.launch install_isolated/share/cartographer_ros/launch/my_robot_laser_imu.launch
```

添加 `robot_state_publisher` 节点，即添加机器人模型。

修改 `cartographer_node` 节点，映射雷达的 **topic**，即 `/scan`；映射陀螺仪的 **topic**，即 `/IMU_data`。

将 `rviz` 节点切换至你所用的 .rviz 文件。

关闭运行 `bag` 的节点。

具体代码如下：

```bash
<launch>
  <param name="/use_sim_time" value="true" />

  <param name="robot_description"
    textfile="$(find cartographer_ros)/urdf/my_robot_laser_imu.urdf" />

  <node name="robot_state_publisher" pkg="robot_state_publisher"
    type="robot_state_publisher" />

  <node name="cartographer_node" pkg="cartographer_ros"
      type="cartographer_node" args="
          -configuration_directory $(find cartographer_ros)/configuration_files
          -configuration_basename my_robot_laser_imu.lua"
      output="screen">
    <remap from="scan" to="/scan" />
    <remap from="imu" to="/IMU_data">
  </node>

  <node name="cartographer_occupancy_grid_node" pkg="cartographer_ros"
      type="cartographer_occupancy_grid_node" args="-resolution 0.05" />
      
  <!-- 请修改成自己的 rviz 路径 -->      
  <node name="rviz" pkg="rviz" type="rviz" required="true"
      args="/home/zr/.ros/laser_imu.rviz" />
</launch>
```

**4. 创建陀螺仪的节点文件**

你需要有一份**陀螺仪节点文件**，并且添加 `map` 与 `base_link` 的 tf 关系，`base_link` 与 `imu_link` 的 tf 关系，`base_link` 与 `laser`的 tf 关系。

然后像上述<a href="#lable">**合并雷达的功能包与 Cartographer ROS**</a> 合并陀螺仪功能与 Cartographer ROS 即可。

**5. 运行建图程序**

运行以下命令，启动建图程序。

```bash
# 进入 Cartographer 工作空间
cd catkin_ws
# 千万不要编译！！！否则你刚写的代码都没啦！！！
# 具体原因为你的 src 里没有改动，所以编译的话会将你的 install_isolated 替换，就恢复成原始状态了。
# source 工作空间环境，使用 zsh 的同学别忘了切换成 zsh
source install_isolated/setup.bash 
# 运行程序
roslaunch cartographer_ros my_robot_laser_imu.launch
```

**6. 在 rviz 中添加 topic**

添加 `Submaps` 话题即可看到雷达建图。
![Submaps](../../static/img/cartographer%E5%BB%BA%E5%9B%BE%E6%95%99%E7%A8%8B/Submaps.png
)

其它 topic 可自行添加。

主要使用的 topic 为 `Map` 、`TF`、`LaserScan` 、`MarkArray`。 

最终运行效果如下：
![result of laser and imu](../../static/img/cartographer%E5%BB%BA%E5%9B%BE%E6%95%99%E7%A8%8B/imu_laser.png)

### 保存已建好的地图 

使用 Cartographer 建图后是不会保存地图的，如果要将构建完成的地图用于之后的定位是需要将地图保存下来的。

要在 Cartographer 工作空间下使用以下命令。

**1. 停止建图**

当建图完成后，输入以下命令停止地图构建：

```bash
# 停止地图构建
rosservice call /finish_trajectory 0
```

**2. 保存地图**

输入以下命令保存已构建完成的地图：

```bash
# 保存地图
rosservice call /write_state "{filename: '<绝对路径>/‘<地图名字>’.pbstream'}"
```

```bash
# 如：
rosservice call /write_state "{filename: ${HOME}/catkin_ws/map/map.pbstream}"
```

**3. 将 Cartographer 保存的地图转换为 ROS 格式下的栅格地图**

```bash
rosrun cartographer_ros cartographer_pbstream_to_ros_map -pbstream_filename=<绝对路径>/<地图名字>.pbstream -map_filestem=<绝对路径>/<地图名字>
```

```bash 
# 如：
rosrun cartographer_ros cartographer_pbstream_to_ros_map -pbstream_filename=${HOME}/catkin_ws/map/map.pbstream -map_filestem=${HOME}/catkin_ws/map/map>
```

#### 为了更加方便地保存地图，我们还可将这些命令做成一个脚本文件 `CartographerSaveMap.sh`。

脚本文件 `CartographerSaveMap.sh` 的代码如下：

```bash
#!/bin/bash
# enter the workspace
source install_isolated/setup.bash

# map  folder 
map_dir="${HOME}/catkin_ws/map"      
 # map name defined as current datatime   
map_name=$(date +%Y%m%d_%H-%M-%S)  

# check the file folder exist
if [ ! -d "$map_dir" ];then
  echo "文件夹不存在, 即将创建文件夹"
  mkdir -p $map_dir
fi

# finish slam
rosservice call /finish_trajectory 0

# make pbstream
rosservice call /write_state "{filename: '$map_dir/$map_name.pbstream'}"

# pbstream to map
rosrun cartographer_ros cartographer_pbstream_to_ros_map \
-pbstream_filename=$map_dir/$map_name.pbstream \
-map_filestem=$map_dir/$map_name
```

给脚本赋予权限:
```bash
  sudo chmod 777 CartographerSaveMap.sh
```

运行脚本文件即可保存地图。

## ***To Be Continued!!!***

有问题找作者—— ***@hzr555***

后续会更新 **如何使用地图** 部分（那也就是哨兵实车导航啦！）。
