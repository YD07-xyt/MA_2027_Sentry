##

参考https://docs.nav2.org/rolling/configuration_and_development/navigation_plugins/

现在设计 

small_gicp+point_lio--->
                        |--->pcl_process---->nav2----->serial
z_localization--------->

现在场景中有特殊隧道，

要求机器人在通过隧道前降低云台，通过后升云台，
我希望通过设计一个特殊的nav2 地图插件 表示隧道(比如手动标记)
在设计一个可以过滤隧道限高杆的地图插件
并设计一个nav2的bt插件，在隧道前后发布降升云台的指令(0/1)，并考虑完成后再通过，保证安全

点云----->patchwork-plusplus---->nav2地图插件(过滤掉隧道限高杆，坡面)--->nav2
