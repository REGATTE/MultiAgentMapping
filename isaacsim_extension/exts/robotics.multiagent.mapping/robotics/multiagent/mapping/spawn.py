from pxr import Gf, UsdGeom, Usd
import omni.usd
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.prims import XFormPrim
import os, random, json
class RobotSpawner:
    """
    Handles the logic for spawning robots in the simulation.
    """

    def __init__(self, stage):
        """
        Initializes the robot spawner.

        Args:
            message_label (ui.Label): Label for displaying messages in the UI.
        """
        self.stage = stage
        self.initial_positions = {} # Dictionary to store initial robot positions

    def spawn_robots(self, num_robots, robot_usd_path, world_prim_path):
        """
        Spawns robots randomly within the bounds of the world map, ensuring equal spacing.

        Args:
            num_robots (int): Number of robots to spawn.
            robot_usd_path (list): Path to the robot USD file.
            world_prim_path (str): Path to the world prim.
        """
        try:

            # Validate that robot_usd_paths is a list
            if not isinstance(robot_usd_path, list):
                print(f"[RobotSpawner] Error: robot_usd_paths is not a list. Received: {type(robot_usd_path)}")
                return

            print(f"[RobotSpawner] Received robot_usd_paths: {robot_usd_path}")

            stage = omni.usd.get_context().get_stage()
            world_prim = stage.GetPrimAtPath(world_prim_path)
            if not world_prim.IsValid():
                print(f"[RobotSpawner] Invalid world prim path: {world_prim_path}")
                return

            # Correct construction of BBoxCache
            bbox_cache = UsdGeom.BBoxCache(
                Usd.TimeCode.Default(),
                [UsdGeom.Tokens.default_],
                useExtentsHint=True
            )
            bounding_box = bbox_cache.ComputeWorldBound(world_prim)
            bounds_min = bounding_box.GetRange().GetMin()
            bounds_max = bounding_box.GetRange().GetMax()

            positions = []
            spacing = 2.0  # Minimum spacing between robots

            for _ in range(num_robots):
                while True:
                    position = Gf.Vec3d(
                        random.uniform(bounds_min[0], bounds_max[0]),
                        random.uniform(bounds_min[1], bounds_max[1]),
                        bounds_min[2]
                    )

                    if all((pos - position).GetLength() > spacing for pos in positions):
                        positions.append(position)
                        break

            for i, position in enumerate(positions):
                # Access individual file path
                robot_file_path = robot_usd_path[i]
                print(f"[RobotSpawner] File path for robot_{i+1}: {robot_file_path}")

                # Validate the file path
                if not os.path.exists(robot_file_path):
                    print(f"[RobotSpawner] Invalid file path for robot_{i+1}: {robot_file_path}")
                    continue

                namespace = os.path.splitext(os.path.basename(robot_file_path))[0]
                if not namespace:
                    print(f"[RobotSpawner] Error: Namespace for robot_{i+1} could not be extracted.")
                    continue
                print(f"[RobotSpawner] Extracted namespace for robot_{i+1}: {namespace}")

                robot_name = f"robot_{i+1}"
                robot_path = f"/World/{robot_name}"
                # Add reference and handle errors
                try:
                    add_reference_to_stage(robot_file_path, robot_path)
                except Exception as e:
                    print(f"[RobotSpawner] Error adding reference for {robot_name}: {e}")
                    continue

                xform_prim = self.stage.GetPrimAtPath(robot_path)
                if xform_prim.IsValid():
                    # Set position
                    xform_prim.GetAttribute("xformOp:translate").Set(position)

                    # Get position after setting (ensure it's saved correctly)
                    position_attr = xform_prim.GetAttribute("xformOp:translate").Get()
                    position = position_attr if position_attr else [0, 0, 0]

                    # Set rotation (Euler XYZ or quaternion, depending on your setup)
                    rotation_attr = xform_prim.GetAttribute("xformOp:rotateXYZ").Get()
                    if rotation_attr:
                        # Convert Euler to quaternion
                        euler_angles = Gf.Vec3d(rotation_attr[0], rotation_attr[1], rotation_attr[2]) * (3.14159265 / 180.0)
                        quaternion = Gf.Quatf(
                            Gf.Rotation(Gf.Vec3d(1, 0, 0), euler_angles[0]) *
                            Gf.Rotation(Gf.Vec3d(0, 1, 0), euler_angles[1]) *
                            Gf.Rotation(Gf.Vec3d(0, 0, 1), euler_angles[2])
                        )
                        rotation = [quaternion.GetImaginary()[0], quaternion.GetImaginary()[1],
                                    quaternion.GetImaginary()[2], quaternion.GetReal()]
                    else:
                        # Default rotation
                        rotation = [0, 0, 0, 1]

                    # Save the position and rotation in the JSON
                    self.initial_positions[robot_name] = {
                        "namespace": namespace,
                        "position": {
                            "x": position[0],
                            "y": position[1],
                            "z": position[2],
                        },
                        "rotation": {
                            "x": rotation[0],
                            "y": rotation[1],
                            "z": rotation[2],
                            "w": rotation[3],
                        },
                    }
                    print(f"[RobotSpawner] Positioned {robot_name} at {position}.")
                else:
                    print(f"[RobotSpawner] Failed to retrieve prim for {robot_name}.")

            self.save_initial_positions_to_json()
        except Exception as e:
            print(f"[RobotSpawner] Error: {e}")
    
    def reset_to_initial_positions(self):
        """
        Resets all robots to their initial positions.
        """
        stage = omni.usd.get_context().get_stage()

        for robot_name, data in self.initial_positions.items():
            robot_path = f"/World/{robot_name}"
            xform_prim = stage.GetPrimAtPath(robot_path)

            if xform_prim.IsValid():
                # Reset position
                position = data["position"]  # Access nested position
                xform_prim.GetAttribute("xformOp:translate").Set(
                    Gf.Vec3d(position["x"], position["y"], position["z"])
                )

                # Reset rotation
                rotation = data["rotation"]  # Access nested rotation
                quaternion = Gf.Quatf(rotation["w"], rotation["x"], rotation["y"], rotation["z"])
                xform_prim.GetAttribute("xformOp:orient").Set(quaternion)

                # Log reset details
                print(f"[RobotSpawner] Reset {robot_name} to position {position} with rotation {rotation}.")
            else:
                print(f"[RobotSpawner] Prim {robot_name} not found for reset.")

    
    def save_initial_positions_to_json(self):
        """
        Saves the initial positions of robots to a JSON file.
        """
        try:
            json_path = os.path.abspath(os.path.join(os.path.dirname(__file__), "../../../../../../initial_positions.json"))
            print(f"[RobotSpawner] Saving initial positions to {json_path}")
            with open(json_path, "w") as json_file:
                json.dump(self.initial_positions, json_file, indent=4)
        except Exception as e:
            print(f"[RobotSpawner] Error saving JSON: {e}")
