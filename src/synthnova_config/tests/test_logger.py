import pytest
from pathlib import Path
import tempfile
import os
from synthnova_config.logger.logger import LoggerConfig

def test_default_config():
    """Test default configuration values."""
    config = LoggerConfig()
    assert config.log_level == "info"
    assert config.use_file_log is True
    assert config.log_path is None

def test_custom_config():
    """Test custom configuration values."""
    config = LoggerConfig(
        log_level="debug",
        use_file_log=False,
        log_path="/custom/path"
    )
    assert config.log_level == "debug"
    assert config.use_file_log is False
    assert config.log_path == "/custom/path"

def test_invalid_log_level():
    """Test invalid log level raises error."""
    with pytest.raises(ValueError):
        LoggerConfig(log_level="invalid_level")

def test_extra_fields():
    """Test that extra fields are not allowed."""
    with pytest.raises(ValueError):
        LoggerConfig(extra_field="value")

def test_field_validation():
    """Test field validation on assignment."""
    config = LoggerConfig()
    config.log_level = "warning"  # Valid value
    assert config.log_level == "warning"
    
    with pytest.raises(ValueError):
        config.log_level = "invalid_level"  # Invalid value

def test_str_strip_whitespace():
    """Test that string fields are stripped of whitespace."""
    config = LoggerConfig(log_path="  /path/to/logs  ")
    assert config.log_path == "/path/to/logs"

def test_log_path_validation():
    """Test log path validation."""
    # Test with valid path
    with tempfile.TemporaryDirectory() as temp_dir:
        config = LoggerConfig(log_path=temp_dir)
        assert config.log_path == temp_dir

    # Test with None (valid)
    config = LoggerConfig(log_path=None)
    assert config.log_path is None

    # Test with non-existent path (should still be valid as path might be created later)
    config = LoggerConfig(log_path="/non/existent/path")
    assert config.log_path == "/non/existent/path"

def test_to_dict():
    """Test conversion to dictionary."""
    config = LoggerConfig(
        log_level="error",
        use_file_log=True,
        log_path="/test/path"
    )
    config_dict = config.to_dict()
    assert isinstance(config_dict, dict)
    assert config_dict["log_level"] == "error"
    assert config_dict["use_file_log"] is True
    assert config_dict["log_path"] == "/test/path"

def test_from_dict():
    """Test creation from dictionary."""
    data = {
        "log_level": "debug",
        "use_file_log": False,
        "log_path": "/test/path"
    }
    config = LoggerConfig.from_dict(data)
    assert config.log_level == "debug"
    assert config.use_file_log is False
    assert config.log_path == "/test/path"

def test_from_dict_invalid_data():
    """Test creation from dictionary with invalid data."""
    invalid_data = {
        "log_level": "invalid_level",
        "use_file_log": False,
        "log_path": "/test/path"
    }
    with pytest.raises(ValueError):
        LoggerConfig.from_dict(invalid_data)

def test_all_log_levels():
    """Test all valid log levels."""
    valid_levels = ["debug", "info", "warning", "error"]
    for level in valid_levels:
        config = LoggerConfig(log_level=level)
        assert config.log_level == level

def test_use_file_log_types():
    """Test use_file_log with different boolean values."""
    # Test with True
    config = LoggerConfig(use_file_log=True)
    assert config.use_file_log is True

    # Test with False
    config = LoggerConfig(use_file_log=False)
    assert config.use_file_log is False

def test_log_path_edge_cases():
    """Test log path with various edge cases."""
    # Test with empty string
    config = LoggerConfig(log_path="")
    assert config.log_path == ""

    # Test with relative path
    config = LoggerConfig(log_path="./logs")
    assert config.log_path == "./logs"

    # Test with home directory
    home_path = str(Path.home())
    config = LoggerConfig(log_path=home_path)
    assert config.log_path == home_path

def test_config_immutability():
    """Test that config fields can be modified after creation."""
    config = LoggerConfig()
    original_level = config.log_level
    
    # Modify the log level
    config.log_level = "error"
    assert config.log_level == "error"
    assert config.log_level != original_level

def test_config_serialization_roundtrip():
    """Test that config can be serialized and deserialized correctly."""
    original_config = LoggerConfig(
        log_level="warning",
        use_file_log=True,
        log_path="/test/path"
    )
    
    # Convert to dict and back
    config_dict = original_config.to_dict()
    new_config = LoggerConfig.from_dict(config_dict)
    
    # Compare all fields
    assert new_config.log_level == original_config.log_level
    assert new_config.use_file_log == original_config.use_file_log
    assert new_config.log_path == original_config.log_path 