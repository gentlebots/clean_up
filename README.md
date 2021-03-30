# Clean up

[Rulebook 2013](https://athome.robocup.org/wp-content/uploads/2018/10/2013_rulebook.pdf)

## Flowchart

![clean_up_the_room](https://user-images.githubusercontent.com/22964725/111527471-4211b900-8760-11eb-96af-8e17921e00f5.png)

## Picking a lemon

### Container

```
roslaunch gb_tiago_manipulation_demo clean_up_demo.launch
```
```
source .bashrc_bridges
ros2 launch ros1_bridge dedicated_bridges_launch.py
```

### Host

Navigates near to lemon. Launch the nav2 system and use the Navigation2 Goal tool to approach the object.
```
ros2 launch gb_navigation nav2_tiago_launch.py
```

Pick the lemon
```
ros2 launch clean_up clean_up_launch.py
```
