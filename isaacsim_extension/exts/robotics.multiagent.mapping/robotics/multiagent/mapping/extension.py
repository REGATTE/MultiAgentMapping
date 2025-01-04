import os

import omni.ext
import omni.ui as ui
import omni.usd
from omni.isaac.core.utils.stage import add_reference_to_stage

from .spawn import RobotSpawner
from .file_manager import FileManager
from .global_clock import GlobalClockGraph
from .utils import reset_simulation_and_ros2

import threading

class RoboticsMultiagentMappingExtension(omni.ext.IExt):
    def on_startup(self, ext_id):
        print("[robotics.multiagent.mapping] Extension startup")
        self.file_manager = FileManager()
        self.robot_spawner = None
        self.robot_file_paths = []
        self.robot_rows = []  # List to track robot rows
        self.max_robots = 10

        # Add a message label for feedback
        self.message_label = None

        # Initialize the '+' button
        self.plus_button = None

        # Create UI window
        self._window = ui.Window(
            "Multi-Agent Robotic Mapping", width=550, height=270, flags=ui.WINDOW_FLAGS_NO_RESIZE
        )  # Non-resizable window
        with self._window.frame:
            with ui.VStack(spacing=2):  # Minimal spacing between sections
                self.setup_world_picker()
                self.setup_robot_selection()
                self.setup_controls()

    def setup_world_picker(self):
        """Sets up the file picker for the world file."""
        with ui.Frame(
            style={"background_color": ui.color(0.2, 0.2, 0.2, 0.9)},  # Slightly dark background
            padding=5,
            height=40,  # Compact height
        ):
            with ui.HStack(spacing=5):  # Compact horizontal layout
                ui.Label("World USD File:", width=200, height=20)
                self._world_file_path_label = ui.Label("No file selected", width=150, height=20)
                self.file_manager.add_file_picker(
                    parent_ui=ui.HStack(),
                    dialog_title="Select World USD File",
                    on_file_selected_callback=self.on_world_file_selected,
                )

                # Add the message label at the bottom
                self.message_label = ui.Label("Messages will appear here.", height=20, alignment=ui.Alignment.CENTER)

    def setup_robot_selection(self):
        """Sets up the scrollable UI for robot selection."""
        with ui.Frame(
            style={"background_color": ui.color(0.1, 0.1, 0.1, 0.9)},
            padding=2,  # Reduce internal padding
            margin=2,  # Minimize external margin
        ):
            with ui.ScrollingFrame(height=150):  # Fixed height for scrollable frame
                self.robot_ui_container = ui.VStack(spacing=2)  # Minimal spacing between robot rows
                self.add_robot_row()

    def add_robot_row(self):
        """Adds a new robot row with correct numbering."""
        robot_index = len(self.robot_rows) + 1  # The new index for the robot

        if len(self.robot_file_paths) >= self.max_robots:
            print("[Extension] Maximum robot limit reached.")
            return

        self.robot_file_paths.append(None)

        with self.robot_ui_container:
            row = ui.HStack(spacing=5, height=30)  # Compact spacing and height
            with row:
                label = ui.Label(f"Robot {robot_index} USD File:", width=200, height=20)  # Store reference
                robot_label = ui.Label("No file selected", width=150, height=20)
                self.file_manager.add_file_picker(
                    parent_ui=ui.HStack(),
                    dialog_title=f"Select Robot {robot_index} USD File",
                    on_file_selected_callback=lambda path, index=robot_index: self.on_robot_file_selected(
                        path, index, robot_label
                    ),
                )
                ui.Button(
                    "-",
                    width=30,
                    height=30,
                    clicked_fn=lambda index=robot_index - 1: self.remove_robot_row(index),
                    tooltip="Remove this robot",
                )
            self.robot_rows.append((row, label))  # Store both row and label as a tuple

        self.update_plus_button_position()

    def update_plus_button_position(self):
        """Repositions the '+' button."""
        if self.plus_button:
            self.plus_button.visible = False
            self.plus_button.destroy()
            self.plus_button = None
        """Updates the visibility of the '+' button based on the number of robots."""
        # If the '+' button doesn't exist yet, create it
        if not self.plus_button:
            with self.robot_ui_container:
                self.plus_button = ui.Button(
                    "+",
                    width=30,
                    height=30,
                    clicked_fn=self.add_robot_row,
                    tooltip="Add another robot",
                )

        # Toggle the visibility of the '+' button
        self.plus_button.visible = len(self.robot_rows) < self.max_robots

    def remove_robot_row(self, index):
        """Removes a robot row, clears its space, and reorders the remaining rows."""
        if len(self.robot_file_paths) <= 1:
            print("[Extension] At least one robot must remain.")
            return

        # Store the row to be removed
        row_to_remove, _ = self.robot_rows[index]

        # Schedule the destruction of the row after a slight delay
        def destroy_row():
            row_to_remove.destroy()

        threading.Timer(0.01, destroy_row).start()

        # Remove the row and file path from the internal lists
        del self.robot_rows[index]
        del self.robot_file_paths[index]

        # Reorder the remaining robot labels
        self.reorder_robot_labels()

        # Update the '+' button position
        self.update_plus_button_position()


    def reorder_robot_labels(self):
        """Reorders the labels of all robot rows."""
        for i, (_, label) in enumerate(self.robot_rows):  # Access stored label reference
            label.text = f"Robot {i + 1} USD File:"


    def on_robot_file_selected(self, file_path, robot_index, robot_label):
        if os.path.isfile(file_path) and file_path.endswith(".usd"):
            if file_path in self.robot_file_paths:
                print(f"[Extension] Duplicate file selected for Robot {robot_index}: {file_path}")
                self.message_label.text = f"File already selected: {os.path.basename(file_path)}"
                return  # Reject the duplicate file selection
            
            file_name = os.path.basename(file_path)
            self.robot_file_paths[robot_index - 1] = file_path
            robot_label.text = file_name
            print(f"[Extension] Selected Robot {robot_index}: {file_path}")
            if self.message_label:
                self.message_label.text = f"Selected Robot {robot_index}: {file_name}."
        else:
            print(f"[Extension] Invalid file selected for Robot {robot_index}.")
            if self.message_label:
                self.message_label.text = "Error: Invalid file selected."

    def on_world_file_selected(self, file_path):
        if os.path.isfile(file_path) and file_path.endswith(".usd"):
            file_name = os.path.basename(file_path)
            self.selected_world_path = file_path
            self._world_file_path_label.text = f"Selected: {file_name}"
            print(f"[Extension] World file selected: {file_path}")
        else:
            print("[Extension] Invalid world file selected.")

    def setup_controls(self):
        with ui.HStack(spacing=10, height=40):
            ui.Button(
                "Spawn Robots and World",
                clicked_fn=self.spawn_robots_and_world,
            )
            ui.Button(
                "Reset Robots",
                clicked_fn=self.reset_robots_to_initial_positions,
            )
            ui.Button(
                "Reset",
                clicked_fn=self.reset,
            )

    def spawn_robots_and_world(self):
        if not hasattr(self, "selected_world_path"):
            print("[Extension] No world file selected.")
            return

        valid_robot_paths = [path for path in self.robot_file_paths if path and os.path.exists(path)]
        if not valid_robot_paths:
            print("[Extension] No robot files selected.")
            return
    
        print(f"[Extension] Valid robot file paths: {valid_robot_paths}")

        try:
            GlobalClockGraph.create_global_clock_omnigraph()
            stage = omni.usd.get_context().get_stage()
            world_prim_path = "/World/Environment"
            add_reference_to_stage(self.selected_world_path, world_prim_path)

            self.robot_spawner = RobotSpawner(stage)
            self.robot_spawner.spawn_robots(len(valid_robot_paths), valid_robot_paths, world_prim_path)


            print(f"[Extension] Spawned {len(valid_robot_paths)} robots successfully.")
        except Exception as e:
            print(f"[Extension] Error during spawning: {e}")

    def reset_robots_to_initial_positions(self):
        # Stop Simulation and reset ROS2
        reset_simulation_and_ros2()

        if self.robot_spawner:
            self.robot_spawner.reset_to_initial_positions()
            print("[Extension] Robots reset to initial positions.")
        else:
            print("[Extension] No robots to reset.")

    def reset(self):
        try:
            # Stop Simulation and reset ROS2
            reset_simulation_and_ros2()

            # Reset the USD world by removing the /World prim
            stage = omni.usd.get_context().get_stage()
            if stage.GetPrimAtPath("/World"):
                stage.RemovePrim("/World")
                print("[Extension] World reset.")
            else:
                print("[Extension] No world prim found. Skipping world reset.")

            # Reset the world file selection
            self.selected_world_path = None
            self._world_file_path_label.text = "No file selected"

            # Destroy all robot rows in the UI
            if self.robot_rows:
                print(f"[Extension] Clearing {len(self.robot_rows)} robot rows from the UI.")
                for row, label in self.robot_rows:
                    if row:
                        row.destroy()
                        print(f"[Extension] Removed {label.text} from the list.")
                self.robot_rows.clear()  # Clear the list of rows

            # Reset the robot file paths
            self.robot_file_paths.clear()
            print("[Extension] Robot file paths cleared.")

            # Add a single default robot row
            self.robot_ui_container.clear()  # Clear all UI children
            self.robot_file_paths = [None]  # Reset to a single default file path
            self.add_robot_row()  # Add the initial robot row to the list

            # Update the '+' button visibility
            self.update_plus_button_position()

            print("[Extension] Reset complete. World reset and robot list cleared.")
        except Exception as e:
            print(f"[Extension] Error during reset: {e}")

    
    def clear_invisible_elements(self):
        """Clears any hidden or empty elements occupying space."""
        for child in self.robot_ui_container.get_children():
            if not child.visible:
                child.destroy()

    def on_shutdown(self):
        print("[robotics.multiagent.mapping] Extension shutdown")
