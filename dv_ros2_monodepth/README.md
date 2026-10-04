# dv_ros2_monodepth

Monocular depth from events (F3 + DepthAnythingV2, one AOTI `.pt2`), and metric depth from it by
fitting the floor.

Every setting except `model_path` and `events_topic` is in [config/monodepth.yaml](config/monodepth.yaml).
With `--symlink-install`, an edit there takes effect at the next launch.

## Mapping a bag live

Inside the driver container (`bash /home/richeek/GitHub/neurofly/docker/run.sh driver`), one shell
each, in this order.

```
ros2 launch neurofly_interface planner_launch.launch.py depth_topic:=/depth camera_info_topic:=/depth/camera_info params_file:=/home/richeek/GitHub/neurofly/neurofly_interface/config/event_mapping.yaml map_origin_x:=-5.0 map_origin_y:=-5.0 map_size_x:=15.0 map_size_y:=20.0 map_size_z:=3.0
```
```
rviz2 -d /root/dv_ws/install/dv_ros2_monodepth/share/dv_ros2_monodepth/rviz/mapping.rviz
```
```
ros2 launch dv_ros2_monodepth monodepth_bag.launch.py model_path:=/home/richeek/GitHub/neurosim/outputs/monoculardepth/f3depth_50ms_lens_ft60/depth_ep59_sm89_torch291.pt2 bag:=/data/nf1_40deg_treehouse_flight1 rviz:=false
```

Restart the mapper before each replay: it keeps adding to the previous map.

To plot the floor fit (`depth = scale / (disparity - shift)`), the height and the attitude for a replay, start
this in another shell before step 3; it records until the bag ends, then writes the figure:

```
python3 /home/richeek/GitHub/dv-ros2-wrapper/dv_ros2_monodepth/scripts/fit_plot.py --bag /data/nf1_40deg_treehouse_flight1 --out /home/richeek/GitHub/neurosim/outputs/monoculardepth/mapping_eval/fit_plots/flight1_fit.png
```
