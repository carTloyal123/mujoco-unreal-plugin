import os
import shutil
import argparse

def clean_unreal_project(root_dir, delete_binaries, delete_intermediate, delete_saved):
    """
    Recursively searches for and deletes specified Unreal Engine folders within the given directory and its subdirectories.

    Args:
        root_dir (str): The root directory to start the cleaning process from.
        delete_binaries (bool): Whether to delete "Binaries" folders.
        delete_intermediate (bool): Whether to delete "Intermediate" folders.
        delete_saved (bool): Whether to delete "Saved" folders.
    """
    folders_to_delete = []
    if delete_binaries:
        folders_to_delete.append("Binaries")
    if delete_intermediate:
        folders_to_delete.append("Intermediate")
    if delete_saved:
        folders_to_delete.append("Saved")

    for root, dirs, files in os.walk(root_dir, topdown=False):
        for dir_name in dirs:
            if dir_name in folders_to_delete:
                folder_path = os.path.join(root, dir_name)
                try:
                    shutil.rmtree(folder_path)
                    print(f"Deleted folder: {folder_path}")
                except Exception as e:
                    print(f"Error deleting folder {folder_path}: {e}")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Clean Unreal Engine project folders.")
    parser.add_argument("-i", "--intermediate", action="store_true", help="Delete Intermediate folders")
    parser.add_argument("-b", "--binaries", action="store_true", help="Delete Binaries folders")
    parser.add_argument("-s", "--saved", action="store_true", help="Delete Saved folders")
    args = parser.parse_args()

    current_directory = os.getcwd()  # Get the current working directory

    # Ensure at least one folder type is selected
    if not (args.intermediate or args.binaries or args.saved):
        print("No folder type selected. Cleaning all!")
        clean_unreal_project(current_directory, True, True, True)
    else:
        print(f"Cleaning Unreal Engine project in: {current_directory}")
        clean_unreal_project(current_directory, args.binaries, args.intermediate, args.saved)
        print("Cleaning complete.")
