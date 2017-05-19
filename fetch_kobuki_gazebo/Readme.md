# fetch_kobuki_gazebo使用方法

## インストール及び使用方法
0. ros-indigo-fetch, ros-indigo-fetch-gazebo, ros-indigo-fetch-description, ros-indigo-kobuki-gazebo, ros-indigo-kobuki-description, ros-indigo-kobuki-random-walkerをapt-getでインストールしておく。 
1. fetch_kobuki_gazebo.zipファイルをcatkinのWork space配下の/srcに展開する。
2. catkin buildでbuildする。
3. : roslaunch fetch_kobuki_gazebo simulate.lauch で実行。

## 注意事項
- kobukiのtopicは/kobukiが追加 ex.)mobile_base/commands/velocity → kobuki/mobile_base/commands/velocity 
- roslaunch実行時、自動でgazebo, rvizが立ち上がり、kobukiのrandom_walkerも起動
- kobukiの見た目は、laser scanに掛かるように、高くしていて、単なる円柱になっている。そのため。前後がわかりづらくなっている。


