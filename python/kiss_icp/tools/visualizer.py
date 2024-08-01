# MIT License
#
# Copyright (c) 2024 Luca Lobefaro, Ignazio Vizzo, Tiziano Guadagnino, Benedikt Mersch,
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.
import importlib
import os
from abc import ABC

import numpy as np

from kiss_icp.config.parser import KISSConfig

# Button names
START_BUTTON = " START\n[SPACE]"
PAUSE_BUTTON = " PAUSE\n[SPACE]"
NEXT_FRAME_BUTTON = "NEXT FRAME\n         [N]"
SCREENSHOT_BUTTON = "SCREENSHOT\n          [S]"
LOCAL_VIEW_BUTTON = "LOCAL VIEW\n         [G]"
GLOBAL_VIEW_BUTTON = "GLOBAL VIEW\n          [G]"
CENTER_VIEWPOINT_BUTTON = "CENTER VIEWPOINT\n               [C]"
QUIT_BUTTON = "QUIT\n  [Q]"

# Colors
BACKGROUND_COLOR = [0.0, 0.0, 0.0]
FRAME_COLOR = [0.8470, 0.1058, 0.3764]
KEYPOINTS_COLOR = [1, 0.7568, 0.0274]
LOCAL_MAP_COLOR = [0.0, 0.3019, 0.2509]
TRAJECTORY_COLOR = [0.1176, 0.5333, 0.8980]

# Size constants
FRAME_PTS_SIZE = 0.06
KEYPOINTS_PTS_SIZE = 0.2
MAP_PTS_SIZE = 0.08


class StubVisualizer(ABC):
    def __init__(self, config: KISSConfig):
        pass

    def update(self, source, keypoints, target_map, pose, last_time):
        pass


def start_pause_callback():
    button_name = PAUSE_BUTTON if Kissualizer.play_mode else START_BUTTON
    if Kissualizer.polyscope.imgui.Button(button_name) or Kissualizer.polyscope.imgui.IsKeyPressed(
        Kissualizer.polyscope.imgui.ImGuiKey_Space
    ):
        Kissualizer.play_mode = not Kissualizer.play_mode


def next_frame_callback():
    if Kissualizer.polyscope.imgui.Button(
        NEXT_FRAME_BUTTON
    ) or Kissualizer.polyscope.imgui.IsKeyPressed(Kissualizer.polyscope.imgui.ImGuiKey_N):
        Kissualizer.block_execution = not Kissualizer.block_execution


def screenshot_callback():
    # TODO: this is just for demo, set a more valid path
    if Kissualizer.polyscope.imgui.Button(
        SCREENSHOT_BUTTON
    ) or Kissualizer.polyscope.imgui.IsKeyPressed(Kissualizer.polyscope.imgui.ImGuiKey_S):
        image_filename = "screenshot.jpg"
        Kissualizer.polyscope.screenshot(image_filename)
        Kissualizer.polyscope.info(f"Screenshot save at: {image_filename}")


def fps_callback():
    if Kissualizer.play_mode:
        total_time_s = np.sum(Kissualizer.times) * 1e-9
        current_fps = float(len(Kissualizer.times) / total_time_s) if total_time_s > 0 else 0
        Kissualizer.polyscope.imgui.TextUnformatted(f"FPS: {int(np.floor(current_fps))}")


def center_viewpoint_callback():
    if Kissualizer.polyscope.imgui.Button(
        CENTER_VIEWPOINT_BUTTON
    ) or Kissualizer.polyscope.imgui.IsKeyPressed(Kissualizer.polyscope.imgui.ImGuiKey_C):
        Kissualizer.polyscope.reset_camera_to_home_view()


def toggle_buttons_andslides_callback():
    # FRAME
    changed, Kissualizer.frame_size = Kissualizer.polyscope.imgui.SliderFloat(
        "##frame_size", Kissualizer.frame_size, v_min=0.01, v_max=0.6
    )
    if changed:
        Kissualizer.polyscope.get_point_cloud("current_frame").set_radius(
            Kissualizer.frame_size, relative=False
        )
    Kissualizer.polyscope.imgui.SameLine()
    changed, Kissualizer.toggle_frame = Kissualizer.polyscope.imgui.Checkbox(
        "Frame Cloud", Kissualizer.toggle_frame
    )
    if changed:
        Kissualizer.polyscope.get_point_cloud("current_frame").set_enabled(Kissualizer.toggle_frame)

    # KEYPOINTS
    changed, Kissualizer.keypoints_size = Kissualizer.polyscope.imgui.SliderFloat(
        "##keypoints_size", Kissualizer.keypoints_size, v_min=0.01, v_max=0.6
    )
    if changed:
        Kissualizer.polyscope.get_point_cloud("keypoints").set_radius(
            Kissualizer.keypoints_size, relative=False
        )
    Kissualizer.polyscope.imgui.SameLine()
    changed, Kissualizer.toggle_keypoints = Kissualizer.polyscope.imgui.Checkbox(
        "Keypoints", Kissualizer.toggle_keypoints
    )
    if changed:
        Kissualizer.polyscope.get_point_cloud("keypoints").set_enabled(Kissualizer.toggle_keypoints)

    # LOCAL MAP
    changed, Kissualizer.map_size = Kissualizer.polyscope.imgui.SliderFloat(
        "##map_size", Kissualizer.map_size, v_min=0.01, v_max=0.6
    )
    if changed:
        Kissualizer.polyscope.get_point_cloud("local_map").set_radius(
            Kissualizer.map_size, relative=False
        )
    Kissualizer.polyscope.imgui.SameLine()
    changed, Kissualizer.toggle_map = Kissualizer.polyscope.imgui.Checkbox(
        "Local Map", Kissualizer.toggle_map
    )
    if changed:
        Kissualizer.polyscope.get_point_cloud("local_map").set_enabled(Kissualizer.toggle_map)


def background_color_callback():
    changed, Kissualizer.background_color = Kissualizer.polyscope.imgui.ColorEdit3(
        "Background Color",
        Kissualizer.background_color,
    )
    if changed:
        Kissualizer.polyscope.set_background_color(Kissualizer.background_color)


def register_trajectory():
    trajectory_curve = Kissualizer.polyscope.register_curve_network(
        "trajectory",
        np.asarray(Kissualizer.trajectory),
        np.asarray(Kissualizer.trajectory_edges),
        color=TRAJECTORY_COLOR,
    )
    trajectory_curve.set_radius(0.3, relative=False)


def unregister_trajectory():
    Kissualizer.polyscope.remove_curve_network("trajectory")


def global_view_callback():
    button_name = LOCAL_VIEW_BUTTON if Kissualizer.global_view else GLOBAL_VIEW_BUTTON
    if Kissualizer.polyscope.imgui.Button(button_name) or Kissualizer.polyscope.imgui.IsKeyPressed(
        Kissualizer.polyscope.imgui.ImGuiKey_G
    ):
        Kissualizer.global_view = not Kissualizer.global_view
        if Kissualizer.global_view:
            Kissualizer.polyscope.get_point_cloud("current_frame").set_transform(
                Kissualizer.last_pose
            )
            Kissualizer.polyscope.get_point_cloud("keypoints").set_transform(Kissualizer.last_pose)
            Kissualizer.polyscope.get_point_cloud("local_map").set_transform(np.eye(4))
            register_trajectory()
        else:
            Kissualizer.polyscope.get_point_cloud("current_frame").set_transform(np.eye(4))
            Kissualizer.polyscope.get_point_cloud("keypoints").set_transform(np.eye(4))
            Kissualizer.polyscope.get_point_cloud("local_map").set_transform(
                np.linalg.inv(Kissualizer.last_pose)
            )
            unregister_trajectory()
        Kissualizer.polyscope.reset_camera_to_home_view()


def configuration_callback():
    if Kissualizer.polyscope.imgui.TreeNode("Parameters"):
        config = Kissualizer.config
        Kissualizer.polyscope.imgui.TextUnformatted(f"Voxel size: {config.mapping.voxel_size}")
        Kissualizer.polyscope.imgui.TextUnformatted(
            f"# Points per voxel: {config.mapping.max_points_per_voxel}"
        )
        Kissualizer.polyscope.imgui.TextUnformatted(f"Max range: {config.data.max_range}")
        Kissualizer.polyscope.imgui.TextUnformatted(f"Min range: {config.data.min_range}")
        Kissualizer.polyscope.imgui.TreePop()


def quit_callback():
    if (
        Kissualizer.polyscope.imgui.Button(QUIT_BUTTON)
        or Kissualizer.polyscope.imgui.IsKeyPressed(Kissualizer.polyscope.imgui.ImGuiKey_Escape)
        or Kissualizer.polyscope.imgui.IsKeyPressed(Kissualizer.polyscope.imgui.ImGuiKey_Q)
    ):
        print("Destroying Visualizer")
        Kissualizer.polyscope.unshow()
        os._exit(0)


def main_gui_callback():
    start_pause_callback()
    if not Kissualizer.play_mode:
        Kissualizer.polyscope.imgui.SameLine()
        next_frame_callback()
    Kissualizer.polyscope.imgui.SameLine()
    screenshot_callback()
    Kissualizer.polyscope.imgui.SameLine()
    fps_callback()
    Kissualizer.polyscope.imgui.Separator()
    configuration_callback()
    Kissualizer.polyscope.imgui.Separator()
    toggle_buttons_andslides_callback()
    background_color_callback()
    global_view_callback()
    Kissualizer.polyscope.imgui.SameLine()
    center_viewpoint_callback()
    Kissualizer.polyscope.imgui.Separator()
    quit_callback()


class Kissualizer(StubVisualizer):
    # Static parameters
    polyscope = None
    config = None
    background_color = BACKGROUND_COLOR
    block_execution = True
    play_mode = False
    frame_size = FRAME_PTS_SIZE
    toggle_frame = True
    keypoints_size = KEYPOINTS_PTS_SIZE
    toggle_keypoints = True
    map_size = MAP_PTS_SIZE
    toggle_map = True
    global_view = False
    trajectory = []
    trajectory_edges = []
    times = []
    last_pose = np.eye(4)

    # Public Interface ----------------------------------------------------------------------------
    def __init__(self, config: KISSConfig):
        try:
            Kissualizer.polyscope = importlib.import_module("polyscope")
        except ModuleNotFoundError as err:
            print(f'polyscope is not installed on your system, run "pip install polyscope"')
            exit(1)

        # Initialize Visualizer
        Kissualizer.polyscope.set_program_name("KissICP Visualizer")
        Kissualizer.polyscope.init()
        self._initialize_visualizer()

        # Initialize parameters
        Kissualizer.config = config

    def update(self, source, keypoints, target_map, pose, last_time):
        Kissualizer.times.append(last_time)
        self._update_geometries(source, keypoints, target_map, pose)
        Kissualizer.last_pose = pose
        while Kissualizer.block_execution:
            Kissualizer.polyscope.frame_tick()
            if Kissualizer.play_mode:
                break
        Kissualizer.block_execution = not Kissualizer.block_execution

    # Private Interface ---------------------------------------------------------------------------
    def _initialize_visualizer(self):
        Kissualizer.polyscope.set_ground_plane_mode("none")
        Kissualizer.polyscope.set_background_color(BACKGROUND_COLOR)
        Kissualizer.polyscope.set_verbosity(0)
        Kissualizer.polyscope.set_user_callback(main_gui_callback)
        Kissualizer.polyscope.set_build_default_gui_panels(False)

    def _update_geometries(self, source, keypoints, target_map, pose):
        # CURRENT FRAME
        frame_cloud = Kissualizer.polyscope.register_point_cloud(
            "current_frame",
            source,
            color=FRAME_COLOR,
            point_render_mode="quad",
        )
        frame_cloud.set_radius(Kissualizer.frame_size, relative=False)
        if Kissualizer.global_view:
            frame_cloud.set_transform(pose)
        else:
            frame_cloud.set_transform(np.eye(4))
        frame_cloud.set_enabled(Kissualizer.toggle_frame)

        # KEYPOINTS
        keypoints_cloud = Kissualizer.polyscope.register_point_cloud(
            "keypoints", keypoints, color=KEYPOINTS_COLOR, point_render_mode="quad"
        )
        keypoints_cloud.set_radius(Kissualizer.keypoints_size, relative=False)
        if Kissualizer.global_view:
            keypoints_cloud.set_transform(pose)
        else:
            keypoints_cloud.set_transform(np.eye(4))
        keypoints_cloud.set_enabled(Kissualizer.toggle_keypoints)

        # LOCAL MAP
        map_cloud = Kissualizer.polyscope.register_point_cloud(
            "local_map",
            target_map.point_cloud(),
            color=LOCAL_MAP_COLOR,
            point_render_mode="quad",
        )
        map_cloud.set_radius(Kissualizer.map_size, relative=False)
        if Kissualizer.global_view:
            map_cloud.set_transform(np.eye(4))
        else:
            map_cloud.set_transform(np.linalg.inv(pose))
        map_cloud.set_enabled(Kissualizer.toggle_map)

        # TRAJECTORY (only visible in global view)
        Kissualizer.trajectory.append(pose[:3, 3])
        n_poses = len(Kissualizer.trajectory)
        if n_poses > 1:
            Kissualizer.trajectory_edges.append([n_poses - 2, n_poses - 1])
        else:
            Kissualizer.trajectory_edges.append([0, 0])
        if Kissualizer.global_view:
            register_trajectory()
