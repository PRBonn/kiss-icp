import json
from types import SimpleNamespace

import numpy as np
import pytest
import yaml
from typer.testing import CliRunner

from kiss_icp.config import KISSConfig, write_config
from kiss_icp.kiss_icp import KissICP
from kiss_icp.tools.cmd import app


def test_sparse_lidar_preserves_registration_points_and_map():
    # Six fine voxels, two coarse voxels, and one duplicate in a fine voxel.
    points = np.array([[x, 0.1, 0.1] for x in (0.1, 0.2, 0.6, 1.1, -0.1, -0.6, -1.1)])
    default = KissICP(KISSConfig(mapping={"voxel_size": 1.0}))
    sparse = KissICP(KISSConfig(data={"sparse_lidar": True}, mapping={"voxel_size": 1.0}))

    default_source, default_map = default.voxelize(points)
    sparse_source, sparse_map = sparse.voxelize(points)

    assert not default.config.data.sparse_lidar
    assert len(default_source) == 2
    assert len(default_map) == 6
    np.testing.assert_array_equal(sparse_source, sparse_map)
    np.testing.assert_array_equal(default_map, sparse_map)

    # The pipeline returns the retained keypoints and inserts the same points
    # into the local map in both modes.
    _, default_keypoints = default.register_frame(points, np.array([]))
    _, sparse_keypoints = sparse.register_frame(points, np.array([]))
    np.testing.assert_array_equal(default_keypoints, default_source)
    np.testing.assert_array_equal(sparse_keypoints, sparse_source)
    np.testing.assert_array_equal(default.local_map.point_cloud(), sparse.local_map.point_cloud())


@pytest.mark.parametrize("sparse_lidar", [False, True])
@pytest.mark.parametrize("points", [np.empty((0, 3)), np.array([[1.0, 2.0, 3.0]])])
def test_sparse_lidar_empty_and_single_point(sparse_lidar, points):
    odometry = KissICP(KISSConfig(data={"sparse_lidar": sparse_lidar}, mapping={"voxel_size": 1.0}))

    source, frame = odometry.voxelize(points)

    np.testing.assert_array_equal(source, points)
    np.testing.assert_array_equal(frame, points)


@pytest.mark.parametrize("config_source", ["yaml", "env"])
@pytest.mark.parametrize(
    "configured, flag, expected",
    [
        (False, None, False),
        (True, None, True),
        (False, "--sparse-lidar", True),
        (True, "--no-sparse-lidar", False),
    ],
)
def test_sparse_lidar_cli_config_precedence(
    tmp_path, monkeypatch, config_source, configured, flag, expected
):
    from kiss_icp import datasets
    from kiss_icp.pipeline import OdometryPipeline

    class Dataset:
        data_dir = str(tmp_path)

        def __len__(self):
            return 1

    monkeypatch.setattr(datasets, "dataset_factory", lambda **kwargs: Dataset())
    observed = {}

    def run(pipeline):
        observed["sparse_lidar"] = pipeline.odometry.config.data.sparse_lidar
        write_config(pipeline.config, str(tmp_path / "saved.yaml"))
        return SimpleNamespace(print=lambda: None)

    monkeypatch.setattr(OdometryPipeline, "run", run)
    monkeypatch.delenv("kiss_icp_data", raising=False)
    args = [str(tmp_path)]
    if config_source == "yaml":
        config_file = tmp_path / "config.yaml"
        config_file.write_text(yaml.safe_dump({"data": {"sparse_lidar": configured}}))
        args.extend(["--config", str(config_file)])
    else:
        monkeypatch.setenv("kiss_icp_data", json.dumps({"sparse_lidar": configured}))
    if flag:
        args.append(flag)

    result = CliRunner().invoke(app, args)

    assert result.exit_code == 0, result.output
    assert observed["sparse_lidar"] is expected
    saved = yaml.safe_load((tmp_path / "saved.yaml").read_text())
    assert saved["data"]["sparse_lidar"] is expected
