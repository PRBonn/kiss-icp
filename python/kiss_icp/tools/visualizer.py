# MIT License
#
# Copyright (c) 2022 Ignacio Vizzo, Tiziano Guadagnino, Benedikt Mersch, Cyrill
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
import copy
import importlib
import os
from abc import ABC
from functools import partial
from typing import Callable, List
import polyscope
import polyscope.imgui as polyscope_gui

import numpy as np

CYAN = np.array([0.24, 0.898, 1])
RED = np.array([128, 0, 0]) / 255.0
BLACK = np.array([0, 0, 0]) / 255.0
BLUE = np.array([0.4, 0.5, 0.9])
GRAY = np.array([0.4, 0.4, 0.4])
SPHERE_SIZE = 0.20

BLACK_COLOR = [0.0, 0.0, 0.0]
WHITE_COLOR = [1.0, 1.0, 1.0]
CYAN_COLOR = [0.24, 0.898, 1.0]
GRAY_COLOR = [0.4, 0.4, 0.4]


class StubVisualizer(ABC):
    def __init__(self):
        pass

    def update(self, source, keypoints, target_map, pose):
        pass


class RegistrationVisualizer2(StubVisualizer):
    # Public Interface ----------------------------------------------------------------------------
    def __init__(self):
        # Initialize Visualizer
        polyscope.init()
        self._initialize_visualizer()

        # Initialize GUI controls
        polyscope._background_color = BLACK_COLOR
        polyscope._block_execution = True
        polyscope._play_mode = False
        polyscope._toggle_frame = True
        polyscope._toggle_keypoints = True
        polyscope._toggle_map = True

    def update(self, source, keypoints, target_map, pose):
        frame_cloud = polyscope.register_point_cloud(
            "current_frame", source, color=CYAN_COLOR, radius=0.001, point_render_mode="quad"
        )
        frame_cloud.set_enabled(polyscope._toggle_frame)
        keypoints_cloud = polyscope.register_point_cloud(
            "keypoints", keypoints, color=CYAN_COLOR, radius=0.001, point_render_mode="quad"
        )
        keypoints_cloud.set_enabled(polyscope._toggle_keypoints)
        map_cloud = polyscope.register_point_cloud(
            "local_map",
            target_map.point_cloud(),
            color=GRAY_COLOR,
            radius=0.0005,
            point_render_mode="quad",
        )
        map_cloud.set_transform(np.linalg.inv(pose))
        map_cloud.set_enabled(polyscope._toggle_map)

        # Visualization loop
        self._update_visualizer()

    # Private Interface ---------------------------------------------------------------------------
    def _initialize_visualizer(self):
        polyscope.set_program_name("KissICP Visualizer")
        polyscope.set_ground_plane_mode("none")
        polyscope.set_background_color(BLACK_COLOR)
        polyscope.set_user_callback(RegistrationVisualizer2.gui_callback)
        polyscope.set_build_gui(True)

    def _update_visualizer(self):
        while polyscope._block_execution:
            polyscope.frame_tick()
            if polyscope._play_mode:
                break
        polyscope._block_execution = not polyscope._block_execution

    @staticmethod
    def gui_callback():
        # PROVA
        if polyscope_gui.Button("START/PAUSE"):
            polyscope._play_mode = not polyscope._play_mode

        # NEXT FRAME
        if not polyscope._play_mode:
            polyscope_gui.SameLine()
            if polyscope_gui.Button("NEXT FRAME"):
                polyscope._block_execution = not polyscope._block_execution

        # CENTER VIEWPOINT
        polyscope_gui.SameLine()
        if polyscope_gui.Button("CENTER VIEWPOINT"):
            polyscope.reset_camera_to_home_view()

        # TOGGLE BUTTONS
        changed, polyscope._toggle_frame = polyscope_gui.Checkbox(
            "Frame Cloud", polyscope._toggle_frame
        )
        if changed:
            polyscope.get_point_cloud("current_frame").set_enabled(polyscope._toggle_frame)
        changed, polyscope._toggle_keypoints = polyscope_gui.Checkbox(
            "Keypoints", polyscope._toggle_keypoints
        )
        if changed:
            polyscope.get_point_cloud("keypoints").set_enabled(polyscope._toggle_keypoints)
        changed, polyscope._toggle_map = polyscope_gui.Checkbox("Local Map", polyscope._toggle_map)
        if changed:
            polyscope.get_point_cloud("local_map").set_enabled(polyscope._toggle_map)

        # BACKGROUND COLOR
        changed, polyscope._background_color = polyscope_gui.ColorEdit3(
            "Background Color", polyscope._background_color
        )
        if changed:
            polyscope.set_background_color(polyscope._background_color)

        # QUIT
        if polyscope_gui.Button("QUIT"):
            print("Destroying Visualizer")
            polyscope.unshow()
            os._exit(0)


class RegistrationVisualizer(StubVisualizer):
    # Public Interface ----------------------------------------------------------------------------
    def __init__(self):
        try:
            self.o3d = importlib.import_module("open3d")
        except ModuleNotFoundError as err:
            print(f'open3d is not installed on your system, run "pip install open3d"')
            exit(1)

        # Initialize GUI controls
        self.block_vis = True
        self.play_crun = False
        self.reset_bounding_box = True

        # Create data
        self.source = self.o3d.geometry.PointCloud()
        self.keypoints = self.o3d.geometry.PointCloud()
        self.target = self.o3d.geometry.PointCloud()
        self.frames = []

        # Initialize visualizer
        self.vis = self.o3d.visualization.VisualizerWithKeyCallback()
        self._register_key_callbacks()
        self._initialize_visualizer()

        # Visualization options
        self.render_map = True
        self.render_source = True
        self.render_keypoints = False
        self.global_view = False
        self.render_trajectory = True
        # Cache the state of the visualizer
        self.state = (
            self.render_map,
            self.render_keypoints,
            self.render_source,
        )

    def update(self, source, keypoints, target_map, pose):
        target = target_map.point_cloud()
        self._update_geometries(source, keypoints, target, pose)
        while self.block_vis:
            self.vis.poll_events()
            self.vis.update_renderer()
            if self.play_crun:
                break
        self.block_vis = not self.block_vis

    # Private Interface ---------------------------------------------------------------------------
    def _initialize_visualizer(self):
        w_name = self.__class__.__name__
        self.vis.create_window(window_name=w_name, width=1920, height=1080)
        self.vis.add_geometry(self.source)
        self.vis.add_geometry(self.keypoints)
        self.vis.add_geometry(self.target)
        self._set_black_background(self.vis)
        self.vis.get_render_option().point_size = 1
        print(
            f"{w_name} initialized. Press:\n"
            "\t[SPACE] to pause/start\n"
            "\t  [ESC] to exit\n"
            "\t    [N] to step\n"
            "\t    [F] to toggle on/off the input cloud to the pipeline\n"
            "\t    [K] to toggle on/off the subsbampled frame\n"
            "\t    [M] to toggle on/off the local map\n"
            "\t    [V] to toggle ego/global viewpoint\n"
            "\t    [T] to toggle the trajectory view(only available in global view)\n"
            "\t    [C] to center the viewpoint\n"
            "\t    [W] to toggle a white background\n"
            "\t    [B] to toggle a black background\n"
        )

    def _register_key_callback(self, keys: List, callback: Callable):
        for key in keys:
            self.vis.register_key_callback(ord(str(key)), partial(callback))

    def _register_key_callbacks(self):
        self._register_key_callback(["Ā", "Q", "\x1b"], self._quit)
        self._register_key_callback([" "], self._start_stop)
        self._register_key_callback(["N"], self._next_frame)
        self._register_key_callback(["V"], self._toggle_view)
        self._register_key_callback(["C"], self._center_viewpoint)
        self._register_key_callback(["F"], self._toggle_source)
        self._register_key_callback(["K"], self._toggle_keypoints)
        self._register_key_callback(["M"], self._toggle_map)
        self._register_key_callback(["T"], self._toggle_trajectory)
        self._register_key_callback(["B"], self._set_black_background)
        self._register_key_callback(["W"], self._set_white_background)

    def _set_black_background(self, vis):
        vis.get_render_option().background_color = [0.0, 0.0, 0.0]

    def _set_white_background(self, vis):
        vis.get_render_option().background_color = [1.0, 1.0, 1.0]

    def _quit(self, vis):
        print("Destroying Visualizer")
        vis.destroy_window()
        os._exit(0)

    def _next_frame(self, vis):
        self.block_vis = not self.block_vis

    def _start_stop(self, vis):
        self.play_crun = not self.play_crun

    def _toggle_source(self, vis):
        if self.render_keypoints:
            self.render_keypoints = False
            self.render_source = True
        else:
            self.render_source = not self.render_source
        return False

    def _toggle_keypoints(self, vis):
        if self.render_source:
            self.render_source = False
            self.render_keypoints = True
        else:
            self.render_keypoints = not self.render_keypoints

        return False

    def _toggle_map(self, vis):
        self.render_map = not self.render_map
        return False

    def _toggle_view(self, vis):
        self.global_view = not self.global_view
        self._trajectory_handle()

    def _center_viewpoint(self, vis):
        vis.reset_view_point(True)

    def _toggle_trajectory(self, vis):
        if not self.global_view:
            return False
        self.render_trajectory = not self.render_trajectory
        self._trajectory_handle()
        return False

    def _trajectory_handle(self):
        if self.render_trajectory and self.global_view:
            for frame in self.frames:
                self.vis.add_geometry(frame, reset_bounding_box=False)
        else:
            for frame in self.frames:
                self.vis.remove_geometry(frame, reset_bounding_box=False)

    def _update_geometries(self, source, keypoints, target, pose):
        # Source hot frame
        if self.render_source:
            self.source.points = self.o3d.utility.Vector3dVector(source)
            self.source.paint_uniform_color(CYAN)
            if self.global_view:
                self.source.transform(pose)
        else:
            self.source.points = self.o3d.utility.Vector3dVector()

        # Keypoints
        if self.render_keypoints:
            self.keypoints.points = self.o3d.utility.Vector3dVector(keypoints)
            self.keypoints.paint_uniform_color(CYAN)
            if self.global_view:
                self.keypoints.transform(pose)
        else:
            self.keypoints.points = self.o3d.utility.Vector3dVector()

        # Target Map
        if self.render_map:
            target = copy.deepcopy(target)
            self.target.points = self.o3d.utility.Vector3dVector(target)
            if self.global_view:
                self.target.paint_uniform_color(GRAY)
            else:
                self.target.transform(np.linalg.inv(pose))
        else:
            self.target.points = self.o3d.utility.Vector3dVector()

        # Update always a list with all the trajectories
        new_frame = self.o3d.geometry.TriangleMesh.create_sphere(SPHERE_SIZE)
        new_frame.paint_uniform_color(BLUE)
        new_frame.compute_vertex_normals()
        new_frame.transform(pose)
        self.frames.append(new_frame)
        # Render trajectory, only if it make sense (global view)
        if self.render_trajectory and self.global_view:
            self.vis.add_geometry(self.frames[-1], reset_bounding_box=False)

        self.vis.update_geometry(self.keypoints)
        self.vis.update_geometry(self.source)
        self.vis.update_geometry(self.target)
        if self.reset_bounding_box:
            self.vis.reset_view_point(True)
            self.reset_bounding_box = False
