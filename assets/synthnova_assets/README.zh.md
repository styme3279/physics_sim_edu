# <img src="docs/source/_static/icon.svg" alt="SynthNova Assets 图标" width="40" height="40" style="vertical-align: middle; margin-right: 5px;"> SynthNova Assets

<div align="right">
  <a href="README.md">English</a> | <a href="README.zh.md">中文</a>
</div>

## 概述

SynthNova Assets 是运行 SynthNova 演示所需的必要资产文件集合。本仓库包含了开始使用 SynthNova 所需的基本资产。

## 安装

### 快速安装

您可以使用单个命令安装 SynthNova Assets：

```bash
TEMP_DIR="$HOME/temp_git_download" && mkdir -p "$TEMP_DIR" && cd "$TEMP_DIR" && git archive --remote=ssh://git@git.galbot.com:6043/synth_nova/basic/synthnova_assets.git develop install.sh | tar -x && chmod +x install.sh && ./install.sh && cd - > /dev/null && rm -rf "$TEMP_DIR"
```

### 手动安装

1. 将此仓库克隆到您的本地 SynthNova 资产目录
2. 默认资产目录由 `SYNTHNOVA_ASSETS` 环境变量指定
3. 您可以通过运行以下命令手动设置目录：
   ```bash
   export SYNTHNOVA_ASSETS="/path/to/your/assets"
   ```

### 非交互式安装

对于自动化脚本，您可以使用静默安装模式：
```bash
bash install.sh quiet
```

⚠️ **注意**：使用静默安装会绕过用户确认提示。请谨慎使用，因为它可能会在没有明确许可的情况下执行操作。

## 开始使用

有关详细文档、示例和用法，请参阅“docs”目录中的文档。

```bash
cd docs
bash view_docs.sh
```

## 支持

如有任何问题、疑问或想要贡献，请联系维护者。

## 许可证

```text
Copyright (c) 2023-2025 Galbot. All Rights Reserved.

本软件包含 Galbot, Inc. 的机密和专有信息（"机密信息"）。
您不得披露此类机密信息，并应仅按照您与 Galbot, Inc. 签订的许可协议的条款使用。

未经授权复制、使用或分发本软件或其任何部分或衍生作品均严格禁止。
如果您错误收到本软件，请立即通知 Galbot, Inc. 并从您的系统中删除。
```