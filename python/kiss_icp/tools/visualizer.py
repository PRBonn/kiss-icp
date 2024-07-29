# MIT License
#
# Copyright (c) 2022 Ignacio Vizzo, Luca Lobefaro, Tiziano Guadagnino, Benedikt Mersch, Cyrill
# Stachniss.
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

BACKGROUND_COLOR = [0.8666, 0.8823, 0.8941]
FRAME_COLOR = [0.1451, 0.5568, 0.6509]
KEYPOINTS_COLOR = [0.8588, 0.3294, 0.3803]
LOCAL_MAP_COLOR = [0.9961, 0.8431, 0.4000]
TRAJECTORY_COLOR = [0.2078, 0.2078, 0.2078]


class StubVisualizer(ABC):
    def __init__(self):
        pass

    def update(self, source, keypoints, target_map, pose):
        pass


def start_pause_callback():
    button_name = "PAUSE" if Kissualizer.play_mode else "START"
    if Kissualizer.polyscope.imgui.Button(button_name):
        Kissualizer.play_mode = not Kissualizer.play_mode


def next_frame_callback():
    if Kissualizer.polyscope.imgui.Button("NEXT FRAME"):
        Kissualizer.block_execution = not Kissualizer.block_execution


def center_viewpoint():
    if Kissualizer.global_view:
        Kissualizer.polyscope.reset_camera_to_home_view()
    else:
        Kissualizer.polyscope.look_at((0.0, 0.0, 200.0), (0.0, 0.0, 0.0))


def screenshot_callback():
    # TODO: this is just for demo, set a more valid path
    if Kissualizer.polyscope.imgui.Button("SCREENSHOT"):
        image_filename = "screenshot.jpg"
        Kissualizer.polyscope.screenshot(image_filename)
        Kissualizer.polyscope.info(f"Screenshot save at: {image_filename}")


def center_viewpoint_callback():
    if Kissualizer.polyscope.imgui.Button("CENTER VIEWPOINT"):
        center_viewpoint()


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


def global_view_callback():
    button_name = "LOCAL VIEW" if Kissualizer.global_view else "GLOBAL VIEW"
    if Kissualizer.polyscope.imgui.Button(button_name):
        Kissualizer.global_view = not Kissualizer.global_view
        Kissualizer.polyscope.get_point_cloud("trajectory").set_enabled(Kissualizer.global_view)
        if Kissualizer.global_view:
            Kissualizer.polyscope.get_point_cloud("current_frame").set_transform(
                Kissualizer.last_pose
            )
            Kissualizer.polyscope.get_point_cloud("keypoints").set_transform(Kissualizer.last_pose)
            Kissualizer.polyscope.get_point_cloud("local_map").set_transform(np.eye(4))
            Kissualizer.polyscope.get_point_cloud("trajectory").set_enabled(True)
        else:
            Kissualizer.polyscope.get_point_cloud("current_frame").set_transform(np.eye(4))
            Kissualizer.polyscope.get_point_cloud("keypoints").set_transform(np.eye(4))
            Kissualizer.polyscope.get_point_cloud("local_map").set_transform(
                np.linalg.inv(Kissualizer.last_pose)
            )
            Kissualizer.polyscope.get_point_cloud("trajectory").set_enabled(False)
        center_viewpoint()


def quit_callback():
    if Kissualizer.polyscope.imgui.Button("QUIT"):
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
    Kissualizer.polyscope.imgui.Separator()
    toggle_buttons_andslides_callback()
    background_color_callback()
    global_view_callback()
    Kissualizer.polyscope.imgui.SameLine()
    center_viewpoint_callback()
    Kissualizer.polyscope.imgui.Separator()
    quit_callback()


# TODO: add screenshot
# New colorscheme
class Kissualizer(StubVisualizer):
    # Static parameters
    polyscope = None
    background_color = BACKGROUND_COLOR
    block_execution = True
    play_mode = False
    frame_size = 0.2
    toggle_frame = True
    keypoints_size = 0.3
    toggle_keypoints = True
    map_size = 0.1
    toggle_map = True
    global_view = False
    trajectory = []
    last_pose = np.eye(4)

    # Public Interface ----------------------------------------------------------------------------
    def __init__(self):
        try:
            Kissualizer.polyscope = importlib.import_module("polyscope")
        except ModuleNotFoundError as err:
            print(f'polyscope is not installed on your system, run "pip install polyscope"')
            exit(1)

        # Initialize Visualizer
        Kissualizer.polyscope.set_program_name("KissICP Visualizer")
        Kissualizer.polyscope.init()
        self._initialize_visualizer()

    def update(self, source, keypoints, target_map, pose):
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
        Kissualizer.polyscope.set_build_gui(False)
        Kissualizer.polyscope.set_open_imgui_window_for_user_callback(True)
        Kissualizer.polyscope.set_autoscale_structures(False)
        Kissualizer.polyscope.set_user_callback(main_gui_callback)

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
        trajectory_cloud = Kissualizer.polyscope.register_point_cloud(
            "trajectory",
            np.asarray(Kissualizer.trajectory),
            color=TRAJECTORY_COLOR,
            point_render_mode="sphere",
        )
        trajectory_cloud.set_radius(0.5, relative=False)
        trajectory_cloud.set_enabled(Kissualizer.global_view)
