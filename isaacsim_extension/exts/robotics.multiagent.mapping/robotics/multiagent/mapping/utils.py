import omni.kit.commands
import subprocess
from omni.timeline import get_timeline_interface

def reset_simulation_and_ros2():
    try:
        # Get the timeline interface
        timeline = get_timeline_interface()

        # Pause and stop the simulation using context management
        if timeline.is_playing():
            timeline.stop()
            print("[Extension] Simulation stopped.")

        # Run the bash script to reset ROS2 nodes
        subprocess.call(["bash", "stop_all_ros2_nodes.sh"])
        print("[Extension] ROS2 nodes reset.")

    except Exception as e:
        print(f"[Extension] Error during reset: {e}")
