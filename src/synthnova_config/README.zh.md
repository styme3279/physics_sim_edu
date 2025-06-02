# <img src="docs/images/icon.svg" alt="SynthNova 图标" width="40" height="40" style="vertical-align: middle; margin-right: 5px;"> SynthNova 配置管理

<div align="right">
  <a href="README.md">English</a> | <a href="README.zh.md">中文</a>
</div>

SynthNova Config 是 SynthNova 仿真工具包的综合性配置管理包。它提供了结构化的配置处理功能，支持自定义配置、预设组合，以及便捷的配置操作工具。

## 🌟 主要特性

- **灵活的配置管理**
  - 支持自定义配置
  - 预定义配置组合
  - 配置继承和覆盖功能
  - 动态配置验证

- **便捷的配置操作**
  - 简单的配置加载和保存 API
  - 配置验证和错误检查
  - 新配置模板生成
  - 配置合并和更新

## ⚙️ 安装

### 使用 pip 安装

```bash
pip install .
```

### 使用 conda 安装

```bash
conda activate <your-conda-env>
pip install .
```

## 🔥 快速开始

### 示例

请参考 [examples](examples) 目录获取更详细的使用示例和最佳实践。

### 作为依赖项使用

要在您的项目中包含 SynthNova Config，请在您的依赖项中添加以下内容：

```toml
"synthnova_config @ git+ssh://git@git.galbot.com:6043/synth_nova/synthnova_config.git@develop"
```

如果您使用 Hatch 作为构建工具，需要在 `pyproject.toml` 中启用直接引用：

```toml
[tool.hatch.metadata]
allow-direct-references = true
```

## 🙋 故障排除

如有任何问题、疑问或贡献，请联系 [维护者](pyproject.toml)。

## 📜 许可证

```text
版权所有 (c) 2023-2025 Galbot。保留所有权利。

本软件包含 Galbot, Inc. 的机密和专有信息（"机密信息"）。您不得披露此类机密信息，
并且只能按照您与 Galbot, Inc. 签订的许可协议的条款使用。

未经授权复制、使用或分发本软件或其任何部分或衍生作品，均严格禁止。如果您错误地收到了
本软件，请立即通知 Galbot, Inc. 并从您的系统中删除。
```
