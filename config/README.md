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
