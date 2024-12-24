import omni.graph.core as og
import omni.kit.app

class GlobalClockGraph:
    @staticmethod
    def ensure_extensions_loaded():
        extensions_to_load = [
            "omni.graph.core",
            "omni.graph.nodes",
            "omni.isaac.core_nodes",
            "omni.isaac.ros2_bridge"
        ]

        ext_manager = omni.kit.app.get_app().get_extension_manager()
        for ext in extensions_to_load:
            if not ext_manager.is_extension_enabled(ext):
                ext_manager.set_extension_enabled(ext, True)
                print(f"[Extension] Loaded extension: {ext}")
            else:
                print(f"[Extension] Extension already loaded: {ext}")

    @staticmethod
    def create_global_clock_omnigraph():
        try:
            # Ensure required extensions are loaded
            GlobalClockGraph.ensure_extensions_loaded()

            # Define keys
            keys = og.Controller.Keys

            # Create the graph
            graph_path = "/World/GlobalClockGraph"
            (graph_handle, list_of_nodes, _, _) = og.Controller.edit(
                {"graph_path": graph_path, "evaluator_name": "execution"},
                {
                    keys.CREATE_NODES: [
                        ("tick", "omni.graph.action.OnPlaybackTick"),
                        ("sim_time", "omni.isaac.core_nodes.IsaacReadSimulationTime"),
                        ("ros2_context", "omni.isaac.ros2_bridge.ROS2Context"),
                        ("ros2_clock", "omni.isaac.ros2_bridge.ROS2PublishClock")
                    ],
                    keys.SET_VALUES: [
                        ("sim_time.inputs:resetOnStop", True) # Enables reset on stop
                    ],
                    keys.CONNECT: [
                        ("tick.outputs:tick", "ros2_clock.inputs:execIn"),
                        ("sim_time.outputs:simulationTime", "ros2_clock.inputs:timeStamp"),
                        ("ros2_context.outputs:context", "ros2_clock.inputs:context"),
                    ]
                },
            )

            print("[GlobalClockGraph] Global clock Omnigraph created successfully.")

        except Exception as e:
            print(f"[GlobalClockGraph] Failed to create Omnigraph: {e}")
