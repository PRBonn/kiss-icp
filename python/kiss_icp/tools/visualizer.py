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
SHOW_VOXEL_GRID_BUTTON = "SHOW VOXEL GRID\n\t\t\t\t[V]"
HIDE_VOXEL_GRID_BUTTON = "HIDE VOXEL GRID\n\t\t\t  [V]"
ORTHO_ON_BUTTON = "ORTHO VIEW ON\n\t\t\t [O]"
ORTHO_OFF_BUTTON = "ORTHO VIEW OFF\n\t\t\t  [O]"
QUIT_BUTTON = "QUIT\n  [Q]"

# Colors
BACKGROUND_COLOR = [0.0, 0.0, 0.0]
FRAME_COLOR = [0.8470, 0.1058, 0.3764]
KEYPOINTS_COLOR = [1, 0.7568, 0.0274]
LOCAL_MAP_COLOR = [0.0, 0.3019, 0.2509]
TRAJECTORY_COLOR = [0.1176, 0.5333, 0.8980]
VOXEL_GRID_COLOR = [0.9, 0.9, 0.9]

# Size constants
FRAME_PTS_SIZE = 0.06
KEYPOINTS_PTS_SIZE = 0.2
MAP_PTS_SIZE = 0.08


class StubVisualizer(ABC):
    def __init__(self):
        pass

    def update(self, source, keypoints, local_map, pose, vis_infos):
        pass

    def set_voxel_size(self, voxel_size):
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
        self._toggle_voxel_grid = False
        self._toggle_ortho = False
        self._global_view = False

        # Create data
        self._trajectory = []
        self._last_pose = np.eye(4)
        self._vis_infos = dict()
        self._selected_pose = ""
        self._voxel_grid_nodes = np.array([])
        self._voxel_grid_edges = np.array([])
        self._last_local_map = None
        self._voxel_size = 1.0

        # Initialize Visualizer
        self._initialize_visualizer()

    def update(self, source, keypoints, local_map, pose, vis_infos: dict):
        self._vis_infos = dict(sorted(vis_infos.items(), key=lambda item: len(item[0])))
        self._last_pose = pose
        self._update_geometries(source, keypoints, local_map, pose)
        while self._block_execution:
            self._ps.frame_tick()
            if self._play_mode:
                break
        self._block_execution = not self._block_execution

    def set_voxel_size(self, voxel_size):
        self._voxel_size = voxel_size

    # Private Interface ---------------------------------------------------------------------------
    def _initialize_visualizer(self):
        self._ps.set_program_name("KissICP Visualizer")
        self._ps.init()
        self._ps.set_ground_plane_mode("none")
        self._ps.set_background_color(BACKGROUND_COLOR)
        self._ps.set_verbosity(0)
        self._ps.set_user_callback(self._main_gui_callback)
        self._ps.set_build_default_gui_panels(False)
        self._ps.set_SSAA_factor(4)

    def _update_geometries(self, source, keypoints, local_map, pose):
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
            local_map.point_cloud(),
            color=LOCAL_MAP_COLOR,
            point_render_mode="quad",
        )
        map_cloud.set_radius(self._map_size, relative=False)
        if self._global_view:
            map_cloud.set_transform(np.eye(4))
        else:
            map_cloud.set_transform(np.linalg.inv(pose))
        map_cloud.set_enabled(self._toggle_map)

        # VOXEL GRID (only if toggled)
        self._last_local_map = local_map
        if self._toggle_voxel_grid:
            self._register_voxel_grid()

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

    def _register_voxel_grid(self):
        if self._last_local_map is None:
            return
        voxels = self._last_local_map.get_voxels()
        self._voxel_grid_nodes = np.zeros((voxels.shape[0] * 8, 3), dtype=np.float64)
        self._voxel_grid_edges = np.zeros((voxels.shape[0] * 12, 2), dtype=np.int64)

        for idx, voxel in enumerate(voxels):
            self._generate_new_voxel(voxel, idx)

        voxel_grid = self._ps.register_curve_network(
            "voxel_grid", self._voxel_grid_nodes, self._voxel_grid_edges, color=VOXEL_GRID_COLOR
        )
        voxel_grid.set_radius(0.01, relative=False)
        if self._global_view:
            voxel_grid.set_transform(np.eye(4))
        else:
            voxel_grid.set_transform(np.linalg.inv(self._last_pose))

    def _unregister_voxel_grid(self):
        if self._ps.has_curve_network("voxel_grid"):
            self._ps.remove_curve_network("voxel_grid")

    def _generate_new_voxel(self, voxel: np.ndarray, idx: int):
        verts = np.array(
            [
                [0, 0, 0],
                [1, 0, 0],
                [1, 1, 0],
                [0, 1, 0],
                [0, 0, 1],
                [1, 0, 1],
                [1, 1, 1],
                [0, 1, 1],
            ]
        ).astype(np.float64)
        edges = np.array(
            [
                [0, 1],
                [1, 2],
                [2, 3],
                [0, 3],
                [0, 4],
                [5, 4],
                [1, 5],
                [5, 6],
                [2, 6],
                [6, 7],
                [3, 7],
                [4, 7],
            ]
        ).astype(np.int64)
        verts = verts + voxel
        verts = verts * self._voxel_size
        edges += idx * 8
        self._voxel_grid_nodes[idx * 8 : idx * 8 + 8] = verts
        self._voxel_grid_edges[idx * 12 : idx * 12 + 12] = edges

    # GUI Callbacks ---------------------------------------------------------------------------
    def _start_pause_callback(self):
        button_name = PAUSE_BUTTON if self._play_mode else START_BUTTON
        if self._gui.Button(button_name) or self._gui.IsKeyPressed(self._gui.ImGuiKey_Space):
            self._play_mode = not self._play_mode
            if self._play_mode:
                self._toggle_voxel_grid = False
                self._unregister_voxel_grid()
                self._ps.set_SSAA_factor(1)
                if self._toggle_ortho:
                    self._ps.reset_camera_to_home_view()  # to reset FoV
                    self._ps.set_view_projection_mode("perspective")
                    self._ps.set_navigation_style("turntable")
                    self._toggle_ortho = False
            else:
                self._ps.set_SSAA_factor(4)

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
        f_key_pressed = self._gui.IsKeyPressed(self._gui.ImGuiKey_F)
        if f_key_pressed:
            self._toggle_frame = not self._toggle_frame
        changed, self._toggle_frame = self._gui.Checkbox("[F] Frame Cloud", self._toggle_frame)
        if changed or f_key_pressed:
            self._ps.get_point_cloud("current_frame").set_enabled(self._toggle_frame)

        # KEYPOINTS
        changed, self._keypoints_size = self._gui.SliderFloat(
            "##keypoints_size", self._keypoints_size, v_min=0.01, v_max=0.6
        )
        if changed:
            self._ps.get_point_cloud("keypoints").set_radius(self._keypoints_size, relative=False)
        self._gui.SameLine()
        k_key_pressed = self._gui.IsKeyPressed(self._gui.ImGuiKey_K)
        if k_key_pressed:
            self._toggle_keypoints = not self._toggle_keypoints
        changed, self._toggle_keypoints = self._gui.Checkbox(
            "[K] Keypoints", self._toggle_keypoints
        )
        if changed or k_key_pressed:
            self._ps.get_point_cloud("keypoints").set_enabled(self._toggle_keypoints)

        # LOCAL MAP
        changed, self._map_size = self._gui.SliderFloat(
            "##map_size", self._map_size, v_min=0.01, v_max=0.6
        )
        if changed:
            self._ps.get_point_cloud("local_map").set_radius(self._map_size, relative=False)
        self._gui.SameLine()
        m_key_pressed = self._gui.IsKeyPressed(self._gui.ImGuiKey_M)
        if m_key_pressed:
            self._toggle_map = not self._toggle_map
        changed, self._toggle_map = self._gui.Checkbox("[M] Local Map", self._toggle_map)
        if changed or m_key_pressed:
            self._ps.get_point_cloud("local_map").set_enabled(self._toggle_map)

    def _background_color_callback(self):
        changed, self._background_color = self._gui.ColorEdit3(
            "Background Color",
            self._background_color,
        )
        if changed:
            self._ps.set_background_color(self._background_color)

    def _inspection_callback(self):
        info_string = "Double-click on the trajectory to visualize it (only global view):"
        if self._gui.TreeNodeEx("Inspection", self._gui.ImGuiTreeNodeFlags_DefaultOpen):
            voxel_grid_button_name = (
                HIDE_VOXEL_GRID_BUTTON if self._toggle_voxel_grid else SHOW_VOXEL_GRID_BUTTON
            )
            if self._gui.Button(voxel_grid_button_name) or self._gui.IsKeyPressed(
                self._gui.ImGuiKey_V
            ):
                self._toggle_voxel_grid = not self._toggle_voxel_grid
                if self._toggle_voxel_grid:
                    self._register_voxel_grid()
                else:
                    self._unregister_voxel_grid()

            ortho_button_name = ORTHO_OFF_BUTTON if self._toggle_ortho else ORTHO_ON_BUTTON
            self._gui.SameLine()
            if self._gui.Button(ortho_button_name) or self._gui.IsKeyPressed(self._gui.ImGuiKey_O):
                self._toggle_ortho = not self._toggle_ortho
                if self._toggle_ortho:
                    self._ps.set_view_projection_mode("orthographic")
                    self._ps.set_navigation_style("planar")
                else:
                    self._ps.reset_camera_to_home_view()  # to reset Fov
                    self._ps.set_view_projection_mode("perspective")
                    self._ps.set_navigation_style("turntable")
                self._ps.reset_camera_to_home_view()

            self._gui.TextUnformatted(info_string)
            if self._selected_pose != "":
                self._gui.TextUnformatted(f"\t\tSelected Pose: {self._selected_pose}")
            self._gui.TreePop()

    def _global_view_callback(self):
        button_name = LOCAL_VIEW_BUTTON if self._global_view else GLOBAL_VIEW_BUTTON
        if self._gui.Button(button_name) or self._gui.IsKeyPressed(self._gui.ImGuiKey_G):
            self._global_view = not self._global_view
            if self._global_view:
                self._ps.get_point_cloud("current_frame").set_transform(self._last_pose)
                self._ps.get_point_cloud("keypoints").set_transform(self._last_pose)
                self._ps.get_point_cloud("local_map").set_transform(np.eye(4))
                self._register_trajectory()
                if self._toggle_voxel_grid:
                    self._ps.get_curve_network("voxel_grid").set_transform(np.eye(4))
            else:
                self._ps.get_point_cloud("current_frame").set_transform(np.eye(4))
                self._ps.get_point_cloud("keypoints").set_transform(np.eye(4))
                self._ps.get_point_cloud("local_map").set_transform(np.linalg.inv(self._last_pose))
                self._unregister_trajectory()
                if self._toggle_voxel_grid:
                    self._ps.get_curve_network("voxel_grid").set_transform(
                        np.linalg.inv(self._last_pose)
                    )
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
                self._selected_pose = f"x: {pose[0]:7.3f}, y: {pose[1]:7.3f}, z: {pose[2]:7.3f}"
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
        if not self._play_mode:
            self._gui.Separator()
            self._inspection_callback()
            self._gui.Separator()
            self._trajectory_pick_callback()
        self._global_view_callback()
        self._gui.SameLine()
        self._center_viewpoint_callback()
        self._gui.Separator()
        self._quit_callback()
