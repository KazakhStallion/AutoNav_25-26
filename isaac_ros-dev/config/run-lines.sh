ros2 run line_detection line_detector --ros-args \
    -p camera_topic:="/zed/zed_node/rgb/color/rect/image" \
    -p depth_camera_topic:="/zed/zed_node/depth/depth_registered" \
    -p camera_info_topic:="/zed/zed_node/rgb/color/rect/camera_info" \
    -p line_points_topic:="/line_points" \
    -p target_frame:="map" \
    -p line_hold_timeout_ms:=0 \
    -p enable_timer:=true
