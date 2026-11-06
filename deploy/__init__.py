import argparse
import sys
import time
from subprocess import CalledProcessError

from rich.console import Console
from rich.progress import Progress

from deploy.Constants import *
from deploy.Utils import *

os.chdir(PROJECT_ROOT)

parser = argparse.ArgumentParser(description="VEX SD Card Deployment Tool")
parser.add_argument(
    "--no-unmount",
    action="store_true",
    help="Do not unmount the SD card after operations",
)
parser.add_argument(
    "--no-push-src", action="store_true", help="Skip pushing source files"
)
parser.add_argument(
    "--no-push-lib", action="store_true", help="Skip pushing library files"
)
parser.add_argument(
    "--no-push-assets", action="store_true", help="Skip pushing additional assets"
)
parser.add_argument(
    "--clear-logs", action="store_true", help="Clear the logs from the robot and the local logs before pushing"
)
parser.add_argument(
    "--clear-robot-logs", action="store_true", help="Clear the logs from the robot /logs directory before pushing"
)
parser.add_argument(
    "--clear-local-logs", action="store_true", help="Clear the logs from the local logs directory before pulling new logs"
)
parser.add_argument("--no-pull-logs", action="store_true", help="Skip pulling logs")
parser.add_argument(
    "--verbose", action="store_true", help="Enable verbose/debug output"
)
args = parser.parse_args()

console = Console()


def verbose_print(msg):
    if args.verbose:
        console.print(f"[blue][VERBOSE] {msg}")


# Helper function to scan a directory and get all file paths
def scan_directory(directory: str, exclude_fn=lambda path: False):
    return [
        str(os.path.join(root, file))
        for root, _, files in os.walk(directory)
        for file in files
        if os.path.isfile(os.path.join(root, file))
        and (not exclude_fn(os.path.join(root, file)))
    ]


# Function to handle the file copy process and update counts
def copy_files_and_update_count(
    source_files, destination_folder, base_folder, update_fn
):
    verbose_print(f"Files to copy: {source_files}")
    with Progress() as progress:
        task = progress.add_task(
            f"[cyan]Copying files to {destination_folder}...", total=len(source_files)
        )

        for file in source_files:
            progress.console.print(f"[cyan]Copying: {file}[/cyan]")
            update_fn(
                *copy_if_changed(
                    [file], destination_folder, base_folder_to_copy_from=base_folder
                )
            )
            progress.update(task, advance=1)


def clear_robot_logs(directory):
    print(f"removing files recursively from directory: {directory}")


def clear_local_logs(directory):
    print(f"removing files recursively from directory: {directory}")
    response = input("THIS WILL REMOVE ALL FILES IN THE DIRECTORY AND ITS SUBDIRECTORIES, ENTER Y TO CONTINUE: ")
    if response.lower().strip() == "y":
        os.unlink(directory)
    else:
        print("Aborting log removal, the upload will resume in 3 seconds...")
        time.sleep(3)


def main():
    with Progress() as progress:
        task = progress.add_task(
            f'[cyan]Searching for storage medium with "{DRIVE_IDENTIFIER_STRING}" in name...',
            total=FIND_VEX_DISK_MAX_ATTEMPTS,
        )

        vex_disk = None
        for attempt in range(FIND_VEX_DISK_MAX_ATTEMPTS):
            vex_disk = find_vex_disk(
                DRIVE_IDENTIFIER_STRING, 1, FIND_VEX_DISK_TIME_BETWEEN_ATTEMPTS
            )
            progress.update(task, advance=1)
            if vex_disk:
                break

        if not vex_disk:
            console.print(
                f'[bold red]Could not find any storage medium with "{DRIVE_IDENTIFIER_STRING}" in name[/bold red]'
            )
            sys.exit(19)  # ENODEV No such device

        vex_disk_path = vex_disk.path
        console.print(f"[bold green]Found VEX disk at {vex_disk_path}[/bold green]")
    total_files_deployed = 0
    total_bytes_uploaded = 0

    total_files_pulled = 0
    total_bytes_downloaded = 0

    def update_deployed_count_and_size(deployed_count, deployed_size_bytes):
        nonlocal total_files_deployed, total_bytes_uploaded
        total_files_deployed += deployed_count
        total_bytes_uploaded += deployed_size_bytes

    def update_pulled_count_and_size(deployed_count, deployed_size_bytes):
        nonlocal total_files_pulled, total_bytes_downloaded
        total_files_pulled += deployed_count
        total_bytes_downloaded += deployed_size_bytes

    start_time = time.perf_counter()

    if args.clear_logs or args.clear_robot_logs:
        clear_robot_logs(LOCAL_LOGS_DIRECTORY)
    if args.clear_logs or args.clear_local_logs:
        clear_local_logs(LOCAL_LOGS_DIRECTORY)

    if not args.no_pull_logs:
        log_objects = scan_directory(
            str(os.path.join(POSIX_MOUNT_POINT_DIR, vex_disk_path, "logs"))
        )
        verbose_print(f"Discovered logs: {log_objects}")
        copy_files_and_update_count(
            log_objects,
            os.path.join(PROJECT_ROOT, "logs/"),
            str(os.path.join(POSIX_MOUNT_POINT_DIR, vex_disk_path, "logs")),
            update_pulled_count_and_size,
        )

    if not args.no_push_src:
        src_objects = scan_directory(SRC_DIRECTORY, exclude_from_deploy)
        copy_files_and_update_count(
            src_objects,
            str(os.path.join(POSIX_MOUNT_POINT_DIR, vex_disk_path)),
            SRC_DIRECTORY,
            update_deployed_count_and_size,
        )

    if not args.no_push_lib:
        library_objects = scan_directory(VEXLIB_DIRECTORY, exclude_from_deploy)
        copy_files_and_update_count(
            library_objects,
            str(os.path.join(POSIX_MOUNT_POINT_DIR, vex_disk_path, "VEXlib")),
            VEXLIB_DIRECTORY,
            update_deployed_count_and_size,
        )

    if not args.no_push_assets:
        deploy_objects = scan_directory(ASSETS_DIRECTORY, exclude_from_deploy)
        copy_files_and_update_count(
            deploy_objects,
            str(os.path.join(POSIX_MOUNT_POINT_DIR, vex_disk_path, "assets")),
            ASSETS_DIRECTORY,
            update_deployed_count_and_size,
        )

    # Ensure the 'logs' directory exists
    if not os.path.isdir(os.path.join(POSIX_MOUNT_POINT_DIR, vex_disk_path, "logs")):
        os.mkdir(os.path.join(POSIX_MOUNT_POINT_DIR, vex_disk_path, "logs"))
        verbose_print(
            "[bold yellow]Created missing 'logs' directory on disk[/bold yellow]"
        )

    if not args.no_unmount:
        with console.status(
            "[yellow]Unmounting drive, please do not remove :warning:"
        ) as status:
            while True:
                try:
                    time.sleep(0.25)
                    unmount_drive(os.path.join(POSIX_MOUNT_POINT_DIR, vex_disk_path))
                    break
                except CalledProcessError:
                    console.print(
                        "[bold yellow]Failed to unmount drive, retrying...[/bold yellow]"
                    )
                    continue
            verbose_print("Syncing FS blocks")
            os.system("sync")
            verbose_print("[green]Unmounted drive successfully[/green]")

        console.print(
            f"[bold green]Completed in {round(time.perf_counter() - start_time, 2)} seconds[/bold green]"
        )
        console.print(
            "[bold green]You may now remove the drive :thumbsup:[/bold green]"
        )
    else:
        console.print(
            f"[bold green]Completed in {round(time.perf_counter() - start_time, 2)} seconds[/bold green]"
        )

    # Final stats with color
    console.print(
        f"[bold red]↑ Uploaded {total_files_deployed} files ({convert_size(total_bytes_uploaded)})[/bold red]"
    )
    console.print(
        f"[bold blue]↓ Downloaded {total_files_pulled} files ({convert_size(total_bytes_downloaded)})[/bold blue]"
    )


if __name__ == "__main__":
    main()
