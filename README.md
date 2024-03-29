# yamasemi_sample

ロボットを直線で往復させるサンプルパッケージです。

# 必要なパッケージ

- beego_gazebo（本研究室で開発）
- yamasemi_sim（本研究室で開発）


# インストール方法

```
$ cd ~/catkin_ws/src
$ git clone https://github.com/dlab-ut/yamasemi_sample.git
$ cd ~/catkin_ws
$ catkin_make
```

# 実行方法

2020年度山彦セミナー最終課題のシミュレーションを行う場合，以下のコマンドを実行します。
```
（端末1）$ roslaunch yamasemi_sim final_2020.launch
```

以下のコマンドで，自分のロボットを出現させることができます。
```
（端末2）$ roslaunch beego_gazebo beego.launch
```

```
（端末3）$ roslaunch yamasemi_sample sample.launch
```
ロボットが1m進み、180度旋回して戻ってくるはずです。


# 新しく作成したノードをコンパイルする方法

src内にsample2.cppを作成した場合、CMakeLists.txtを変更しないとコンパイルできません。

コンパイルするため、CMakeLists.txt内で以下の2文を追加します。

- add_executable(sample2 src/sample2.cpp)

src内のsample2.cppをコンパイルしてsample2というノードを作成します。

- target_link_libraries(sample2 ${catkin_LIBRARIES})

sample2というノードにリンクするライブラリを指定します。

# その他

中間課題からURGを使いますが、URGを使うためにはsample.cpp内の以下のコメントアウトを外します。

ros::Subscriber scan_sub = nh.subscribe("scan", 10, scanCallback);