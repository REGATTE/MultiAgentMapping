from omni.ui import Button, Frame, Alignment
from omni.kit.window.filepicker import FilePickerDialog
import os


class FileManager:
    """
    Handles file selection using a folder/file picker icon.
    """

    def __init__(self):
        """
        Initializes the file manager.
        """
        pass

    def add_file_picker(self, parent_ui, dialog_title, on_file_selected_callback):
        """
        Adds a file picker button to the UI.

        Args:
            parent_ui: The parent UI container where the picker will be added.
            dialog_title (str): Title for the dialog box.
            on_file_selected_callback (function): Callback triggered when a file is selected.
        """
        def open_file_picker():
            """
            Opens the file picker dialog.
            """
            def on_selected(filename, path):
                """
                Handles the selection of a file and constructs the full file path.

                Args:
                    filename (str): Name of the selected file.
                    path (str): Path to the selected directory.
                """
                # Combine directory path and filename to get the full path
                full_path = os.path.join(path, filename)
                print(f"[FileManager] Full selected path: {full_path}")  # Debugging

                # Pass the full path to the callback function
                if filename and filename.endswith(".usd"):
                    on_file_selected_callback(full_path)
                else:
                    print("[FileManager] No valid USD file selected.")
                file_picker.hide()

            def on_canceled(a, b):
                """
                Handles the cancellation of the file picker.
                """
                print("[FileManager] File selection canceled.")
                file_picker.hide()

            file_picker = FilePickerDialog(
                dialog_title,
                allow_multi_selection=False,
                apply_button_label="Select",
                click_apply_handler=lambda a, b: on_selected(a, b),
                click_cancel_handler=lambda a, b: on_canceled(a, b),
                enable_versioning_pane=True,
            )

        # Create the file picker button
        with Frame(parent=parent_ui, tooltip=dialog_title):
            Button(
                name="IconButton",
                width=100,
                height=25,
                clicked_fn=open_file_picker,
                text="Browse",
                alignment=Alignment.LEFT_CENTER,
            )
