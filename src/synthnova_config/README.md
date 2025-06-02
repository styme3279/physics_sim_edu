# <img src="docs/images/icon.svg" alt="SynthNova Icon" width="40" height="40" style="vertical-align: middle; margin-right: 5px;"> SynthNova Config

<div align="right">
  <a href="README.md">English</a> | <a href="README.zh.md">中文</a>
</div>

SynthNova Config is a comprehensive configuration management package for the SynthNova simulation toolkit. It provides structured configuration handling with support for custom configurations, preset combinations, and convenient utilities for configuration operations.

## 🌟 Key Features

- **Flexible Configuration Management**
  - Custom configuration support
  - Pre-defined configuration combinations
  - Configuration inheritance and override capabilities
  - Dynamic configuration validation

- **Easy Configuration Operations**
  - Simple API for loading and saving configurations
  - Configuration validation and error checking
  - Template generation for new configurations
  - Configuration merging and updating

## ⚙️ Installation

### Install with pip

```bash
pip install .
```

### Install with conda

```bash
conda activate <your-conda-env>
pip install .
```

## 🔥 Quickstart

### Examples

Refer to the [examples](examples) directory for more detailed usage examples and best practices.

### Using as a Dependency

To include the SynthNova Config in your project, add the following dependency to your requirements:

```toml
"synthnova_config @ git+ssh://git@git.galbot.com:6043/synth_nova/synthnova_config.git@develop"
```

If you're using Hatch as your build tool, you'll need to enable direct references by adding this to your `pyproject.toml`:

```toml
[tool.hatch.metadata]
allow-direct-references = true
```
## Test

```bash
conda activate synthnova
python -m pytest tests -v
```

## 🙋 Troubleshooting

For any issues, questions, or contributions related to this package, please reach out to the [maintainers](pyproject.toml).

## 📜 License

```text
Copyright (c) 2023-2025 Galbot. All Rights Reserved.

This software contains confidential and proprietary information of Galbot, Inc.
("Confidential Information"). You shall not disclose such Confidential Information
and shall use it only in accordance with the terms of the license agreement you
entered into with Galbot, Inc.

UNAUTHORIZED COPYING, USE, OR DISTRIBUTION OF THIS SOFTWARE, OR ANY PORTION OR
DERIVATIVE THEREOF, IS STRICTLY PROHIBITED. IF YOU HAVE RECEIVED THIS SOFTWARE IN
ERROR, PLEASE NOTIFY GALBOT, INC. IMMEDIATELY AND DELETE IT FROM YOUR SYSTEM.
```
