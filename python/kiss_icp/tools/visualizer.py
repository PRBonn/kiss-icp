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
import datetime
import importlib
import os
from abc import ABC

import numpy as np

# Button names
START_BUTTON = " START\n[SPACE]"
PAUSE_BUTTON = " PAUSE\n[SPACE]"
NEXT_FRAME_BUTTON = "NEXT FRAME\n\t\t [N]"
SCREENSHOT_BUTTON = "SCREENSHOT\n\t\t  [S]"
LOCAL_VIEW_BUTTON = "LOCAL VIEW\n\t\t [G]"
GLOBAL_VIEW_BUTTON = "GLOBAL VIEW\n\t\t  [G]"
CENTER_VIEWPOINT_BUTTON = "CENTER VIEWPOINT\n\t\t\t\t[C]"
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
    def __init__(self):
        pass

    def update(self, source, keypoints, target_map, pose, vis_infos):
        pass


class Kissualizer(StubVisualizer):
    # Public Interface ----------------------------------------------------------------------------
    def __init__(self):
        try:
            self._ps = importlib.import_module("polyscope")
            self._gui = self._ps.imgui
        except ModuleNotFoundError as err:
            print(f'polyscope is not installed on your system, run "pip install polyscope"')
            exit(1)

        # Initialize GUI controls
        self._background_color = BACKGROUND_COLOR
        self._frame_size = FRAME_PTS_SIZE
        self._keypoints_size = KEYPOINTS_PTS_SIZE
        self._map_size = MAP_PTS_SIZE
        self._block_execution = True
        self._play_mode = False
        self._toggle_frame = True
        self._toggle_keypoints = True
        self._toggle_map = True
        self._global_view = False

        # Create data
        self._trajectory = []
        self._last_pose = np.eye(4)
        self._vis_infos = dict()
        self._selected_pose = ""

        # Initialize Visualizer
        self._initialize_visualizer()

    def update(self, source, keypoints, target_map, pose, vis_infos: dict):
        self._vis_infos = dict(sorted(vis_infos.items(), key=lambda item: len(item[0])))
        self._update_geometries(source, keypoints, target_map, pose)
        self._last_pose = pose
        while self._block_execution:
            self._ps.frame_tick()
            if self._play_mode:
                break
        self._block_execution = not self._block_execution

    # Private Interface ---------------------------------------------------------------------------
    def _initialize_visualizer(self):
        self._ps.set_program_name("KissICP Visualizer")
        self._ps.init()
        self._ps.set_ground_plane_mode("none")
        self._ps.set_background_color(BACKGROUND_COLOR)
        self._ps.set_verbosity(0)
        self._ps.set_user_callback(self._main_gui_callback)
        self._ps.set_build_default_gui_panels(False)

    def _update_geometries(self, source, keypoints, target_map, pose):
        # CURRENT FRAME
        frame_cloud = self._ps.register_point_cloud(
            "current_frame",
            source,
            color=FRAME_COLOR,
            point_render_mode="quad",
        )
        frame_cloud.set_radius(self._frame_size, relative=False)
        if self._global_view:
            frame_cloud.set_transform(pose)
        else:
            frame_cloud.set_transform(np.eye(4))
        frame_cloud.set_enabled(self._toggle_frame)

        # KEYPOINTS
        keypoints_cloud = self._ps.register_point_cloud(
            "keypoints", keypoints, color=KEYPOINTS_COLOR, point_render_mode="quad"
        )
        keypoints_cloud.set_radius(self._keypoints_size, relative=False)
        if self._global_view:
            keypoints_cloud.set_transform(pose)
        else:
            keypoints_cloud.set_transform(np.eye(4))
        keypoints_cloud.set_enabled(self._toggle_keypoints)

        # LOCAL MAP
        map_cloud = self._ps.register_point_cloud(
            "local_map",
            target_map.point_cloud(),
            color=LOCAL_MAP_COLOR,
            point_render_mode="quad",
        )
        map_cloud.set_radius(self._map_size, relative=False)
        if self._global_view:
            map_cloud.set_transform(np.eye(4))
        else:
            map_cloud.set_transform(np.linalg.inv(pose))
        map_cloud.set_enabled(self._toggle_map)

        # TRAJECTORY (only visible in global view)
        self._trajectory.append(pose[:3, 3])
        if self._global_view:
            self._register_trajectory()

    def _register_trajectory(self):
        trajectory_cloud = self._ps.register_point_cloud(
            "trajectory",
            np.asarray(self._trajectory),
            color=TRAJECTORY_COLOR,
        )
        trajectory_cloud.set_radius(0.3, relative=False)

    def _unregister_trajectory(self):
        self._ps.remove_point_cloud("trajectory")

    # GUI Callbacks ---------------------------------------------------------------------------
    def _start_pause_callback(self):
        button_name = PAUSE_BUTTON if self._play_mode else START_BUTTON
        if self._gui.Button(button_name) or self._gui.IsKeyPressed(self._gui.ImGuiKey_Space):
            self._play_mode = not self._play_mode

    def _next_frame_callback(self):
        if self._gui.Button(NEXT_FRAME_BUTTON) or self._gui.IsKeyPressed(self._gui.ImGuiKey_N):
            self._block_execution = not self._block_execution

    def _screenshot_callback(self):
        if self._gui.Button(SCREENSHOT_BUTTON) or self._gui.IsKeyPressed(self._gui.ImGuiKey_S):
            image_filename = "kisshot_" + (
                datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S") + ".jpg"
            )
            self._ps.screenshot(image_filename)

    def _vis_infos_callback(self):
        if self._gui.TreeNodeEx("Odometry Information", self._gui.ImGuiTreeNodeFlags_DefaultOpen):
            for key in self._vis_infos:
                self._gui.TextUnformatted(f"{key}: {self._vis_infos[key]}")
            if not self._play_mode and self._global_view:
                self._gui.TextUnformatted(f"Selected Pose: {self._selected_pose}")
            self._gui.TreePop()

    def _center_viewpoint_callback(self):
        if self._gui.Button(CENTER_VIEWPOINT_BUTTON) or self._gui.IsKeyPressed(
            self._gui.ImGuiKey_C
        ):
            self._ps.reset_camera_to_home_view()

    def _toggle_buttons_andslides_callback(self):
        # FRAME
        changed, self._frame_size = self._gui.SliderFloat(
            "##frame_size", self._frame_size, v_min=0.01, v_max=0.6
        )
        if changed:
            self._ps.get_point_cloud("current_frame").set_radius(self._frame_size, relative=False)
        self._gui.SameLine()
        changed, self._toggle_frame = self._gui.Checkbox("Frame Cloud", self._toggle_frame)
        if changed:
            self._ps.get_point_cloud("current_frame").set_enabled(self._toggle_frame)

        # KEYPOINTS
        changed, self._keypoints_size = self._gui.SliderFloat(
            "##keypoints_size", self._keypoints_size, v_min=0.01, v_max=0.6
        )
        if changed:
            self._ps.get_point_cloud("keypoints").set_radius(self._keypoints_size, relative=False)
        self._gui.SameLine()
        changed, self._toggle_keypoints = self._gui.Checkbox("Keypoints", self._toggle_keypoints)
        if changed:
            self._ps.get_point_cloud("keypoints").set_enabled(self._toggle_keypoints)

        # LOCAL MAP
        changed, self._map_size = self._gui.SliderFloat(
            "##map_size", self._map_size, v_min=0.01, v_max=0.6
        )
        if changed:
            self._ps.get_point_cloud("local_map").set_radius(self._map_size, relative=False)
        self._gui.SameLine()
        changed, self._toggle_map = self._gui.Checkbox("Local Map", self._toggle_map)
        if changed:
            self._ps.get_point_cloud("local_map").set_enabled(self._toggle_map)

    def _background_color_callback(self):
        changed, self._background_color = self._gui.ColorEdit3(
            "Background Color",
            self._background_color,
        )
        if changed:
            self._ps.set_background_color(self._background_color)

    def _global_view_callback(self):
        button_name = LOCAL_VIEW_BUTTON if self._global_view else GLOBAL_VIEW_BUTTON
        if self._gui.Button(button_name) or self._gui.IsKeyPressed(self._gui.ImGuiKey_G):
            self._global_view = not self._global_view
            if self._global_view:
                self._ps.get_point_cloud("current_frame").set_transform(self._last_pose)
                self._ps.get_point_cloud("keypoints").set_transform(self._last_pose)
                self._ps.get_point_cloud("local_map").set_transform(np.eye(4))
                self._register_trajectory()
            else:
                self._ps.get_point_cloud("current_frame").set_transform(np.eye(4))
                self._ps.get_point_cloud("keypoints").set_transform(np.eye(4))
                self._ps.get_point_cloud("local_map").set_transform(np.linalg.inv(self._last_pose))
                self._unregister_trajectory()
            self._ps.reset_camera_to_home_view()

    def _quit_callback(self):
        self._gui.SetCursorPosX(
            self._gui.GetCursorPosX() + self._gui.GetContentRegionAvail()[0] - 50
        )
        if (
            self._gui.Button(QUIT_BUTTON)
            or self._gui.IsKeyPressed(self._gui.ImGuiKey_Escape)
            or self._gui.IsKeyPressed(self._gui.ImGuiKey_Q)
        ):
            print("Destroying Visualizer")
            self._ps.unshow()
            os._exit(0)

    def _trajectory_pick_callback(self):
        if self._gui.GetIO().MouseClicked[0]:
            name, idx = self._ps.get_selection()
            if name == "trajectory" and self._ps.has_point_cloud(name):
                pose = self._trajectory[idx]
                self._selected_pose = f"x: {pose[0]:7.3f}, y: {pose[1]:7.3f}, z: {pose[2]:7.3f}>"
            else:
                self._selected_pose = ""

    def _main_gui_callback(self):
        # GUI callbacks
        self._start_pause_callback()
        if not self._play_mode:
            self._gui.SameLine()
            self._next_frame_callback()
        self._gui.SameLine()
        self._screenshot_callback()
        self._gui.Separator()
        self._vis_infos_callback()
        self._gui.Separator()
        self._toggle_buttons_andslides_callback()
        self._background_color_callback()
        self._global_view_callback()
        self._gui.SameLine()
        self._center_viewpoint_callback()
        self._gui.Separator()
        self._quit_callback()

        # Mouse callbacks
        self._trajectory_pick_callback()
