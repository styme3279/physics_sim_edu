#####################################################################################
# Copyright (c) 2023-2025 Galbot. All Rights Reserved.
#
# This software contains confidential and proprietary information of Galbot, Inc.
# ("Confidential Information"). You shall not disclose such Confidential Information
# and shall use it only in accordance with the terms of the license agreement you
# entered into with Galbot, Inc.
#
# UNAUTHORIZED COPYING, USE, OR DISTRIBUTION OF THIS SOFTWARE, OR ANY PORTION OR
# DERIVATIVE THEREOF, IS STRICTLY PROHIBITED. IF YOU HAVE RECEIVED THIS SOFTWARE IN
# ERROR, PLEASE NOTIFY GALBOT, INC. IMMEDIATELY AND DELETE IT FROM YOUR SYSTEM.
#####################################################################################
#          _____             _   _       _   _
#         / ____|           | | | |     | \ | |
#        | (___  _   _ _ __ | |_| |__   |  \| | _____   ____ _
#         \___ \| | | | '_ \| __| '_ \  | . ` |/ _ \ \ / / _` |
#         ____) | |_| | | | | |_| | | | | |\  | (_) \ V / (_| |
#        |_____/ \__, |_| |_|\__|_| |_| |_| \_|\___/ \_/ \__,_|
#                 __/ |
#                |___/
#
#####################################################################################
#
# Description: Example for creating custom configurations
#
# Author: Herman Ye@Galbot
# Date: 2025-04-18
#
# This example demonstrates:
# 1. How to create custom configuration classes using Pydantic
# 2. How to save and load configurations
# 3. How to handle configuration validation and type checking
#
#####################################################################################


from pydantic import BaseModel, Field, ConfigDict
from synthnova_config.utils import (
    save_config,
    load_config,
    delete_config,
    create_temp_config_file,
)
from synthnova_config.logger import LoggerConfig


class Foo(BaseModel):
    """Example configuration class demonstrating basic Pydantic model usage"""
    a: int = Field(default=1, description="First integer parameter")
    b: int = Field(default=2, description="Second integer parameter")

    model_config = ConfigDict(
        validate_assignment=True,  # Enable runtime validation of field assignments
        extra="forbid",  # Prevent accidental addition of undefined fields
        frozen=False,  # Allow modification of configuration after initialization
    )


class CustomConfig(BaseModel):
    """Main configuration class containing system and custom configurations"""

    logger_config: LoggerConfig = Field(
        default_factory=LoggerConfig,
        description="System-wide logging configuration, including log levels, "
        "output destinations, and formatting options",
    )

    foo: Foo = Field(
        default_factory=Foo,
        description="Example configuration object",
    )

    model_config = ConfigDict(
        validate_assignment=True,  # Enable runtime validation of field assignments
        extra="forbid",  # Prevent accidental addition of undefined fields
        frozen=False,  # Allow modification of configuration after initialization
    )


def main():
    """Main function demonstrating configuration creation, saving, loading, and deletion"""
    # Create a custom config instance
    config = CustomConfig()
    print("\nLoaded configuration:")
    print(config)
    print("\nConfiguration as JSON:")
    print(config.model_dump_json(indent=4))

    # Save to temporary file
    filepath = create_temp_config_file(suffix=".json")
    save_config(config=config, filepath=filepath)
    print(f"\nConfiguration saved to {filepath}")

    # Load configuration from file
    config_dict = load_config(filepath=filepath, config_class=CustomConfig)
    print(f"Type of config_dict: {type(config_dict)}")
    print(f"\nConfiguration loaded from {filepath}")

    # Delete temporary file
    delete_config(filepath=filepath)
    print(f"\nDeleted {filepath}")


if __name__ == "__main__":
    main()
