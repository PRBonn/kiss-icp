# KISS-ICP Config

**NOTE:** You **DO NOT NEED** to use these configuration files to run the system.

The configuration of our system is a sanitzed selection of parameters. In most of the cases it
should be enough to use the default parameters "as is".

## Additional configuration for KISS-ICP

You will need first need to install all the dependencies of the kiss_icp:

```sh
pip install "kiss-icp[all]"
```
If you need extra tunning then you can use the optionally provided yaml files from this folder. You
can override just 1 or all of the default parameters. To load this configuration you only need to
instruct the command line interface:

```sh
$ kiss_icp_pipeline <YOUR_DATASET> --config <path-to-config>/my_config.yaml
```

Big thanks to Markus Pielmeier who contributed PR #63 making this configuration module as flexible
as possible.

## Sparse LiDAR scans

For sparse scans, such as those from 16-beam or Livox sensors, the second downsampling step may
leave too few points for registration. You can skip it with:

```sh
kiss_icp_pipeline <YOUR_DATASET> --sparse-lidar
```

Or set it in your YAML configuration:

```yaml
data:
  sparse_lidar: true
```

This keeps the first downsampling step at `0.5 * voxel_size` and uses those points for both
registration and map updates. The default is `false`, which retains the second downsampling at
`1.5 * voxel_size`. Keeping more registration points can increase runtime.

The CLI flag overrides the configuration; `--no-sparse-lidar` explicitly disables it. Omitting
both flags keeps the YAML or environment setting. For ROS, set `data.sparse_lidar` in the node
parameters. For the C++ API, set `KISSConfig::sparse_lidar` to `true`.
