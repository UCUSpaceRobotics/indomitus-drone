python3 -c "
import rclpy, time
rclpy.init()
from src.ros_bridge.vision_subscriber import VisionBridge
bridge = VisionBridge(
    topic='/erc/vision_targets',
    grid_config={
        'origin_x_m': -2.5, 'origin_y_m': -0.5, 'cell_size_m': 1.0,
        'columns': ['A','B','C','D','E','F'], 'rows': [1,2,3,4,5,6]
    }
)
print('Listening for 1000 seconds... Hold marker 102 in front of camera.')
start = time.time()
msg_count_start = bridge.get_message_count()
while time.time() - start < 1000:
    bridge.spin_once()
    t = bridge.get_latest_target()
    if t:
        print(f'  Marker {t[\"marker_id\"]} | x={t[\"x_offset_m\"]:+.3f}m | y={t[\"y_offset_m\"]:+.3f}m | age={t[\"age_s\"]*1000:.0f}ms')
    time.sleep(0.1)
total = bridge.get_message_count() - msg_count_start
print(f'\nTotal messages received: {total} (~{total/10:.1f} Hz)')
bridge.shutdown()
rclpy.shutdown()
"