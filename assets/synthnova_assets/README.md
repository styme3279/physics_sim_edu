# <img src="docs/source/_static/icon.svg" alt="SynthNova Assets Icon" width="40" height="40" style="vertical-align: middle; margin-right: 5px;"> SynthNova Assets

<div align="right">
  <a href="README.md">English</a> | <a href="README.zh.md">中文</a>
</div>

## Overview

SynthNova Assets is a collection of essential asset files required for running SynthNova demos. This repository contains the fundamental resources needed to get started with SynthNova.

## Installation

### Quick Install

You can install SynthNova Assets with a single command:

```bash
TEMP_DIR="$HOME/temp_git_download" && mkdir -p "$TEMP_DIR" && cd "$TEMP_DIR" && git archive --remote=ssh://git@git.galbot.com:6043/synth_nova/basic/synthnova_assets.git develop install.sh | tar -x && chmod +x install.sh && ./install.sh && cd - > /dev/null && rm -rf "$TEMP_DIR"
```

### Manual Installation

1. Clone this repository to your local SynthNova assets directory
2. The default assets directory is specified by the `SYNTHNOVA_ASSETS` environment variable
3. You can manually set the directory by running:
   ```bash
   export SYNTHNOVA_ASSETS="/path/to/your/assets"
   ```

### Non-Interactive Installation

For automated scripts, you can use the quiet installation mode:
```bash
bash install.sh quiet
```

⚠️ **Note**: Using quiet installation bypasses user confirmation prompts. Use with caution as it may perform actions without explicit permission.

## Quick Start

For detailed documentation, examples, and advanced usage, please refer to the documentation in the `docs` directory.

```bash
cd docs
bash view_docs.sh
```

## Support

For any issues, questions, or contributions, please contact the maintainers.

## License

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
