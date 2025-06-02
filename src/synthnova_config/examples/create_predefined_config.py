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
# Description: Example for creating simulator configurations
# Author: Herman Ye@Galbot
# Date: 2025-03-03
#
#####################################################################################
def demo_create_config():
    """Demonstrate how to create a new predefined config with default values.

    This function shows how to instantiate a new RenderSimulatorConfig object
    with its default values. The configuration will be printed to console in
    both string and JSON formats.

    Returns:
        None

    Example:
        >>> demo_create_config()
        Loaded configuration:
        RenderSimulatorConfig(...)

        Configuration as JSON:
        {
            ...
        }
    """
    from synthnova_config import RenderSimulatorConfig

    config = RenderSimulatorConfig()

    print("\nLoaded configuration:")
    print(config)
    print("\nConfiguration as JSON:")
    print(config.model_dump_json(indent=4))


def demo_save_config():
    """Demonstrate how to save a configuration to a JSON file.

    This function creates a new RenderSimulatorConfig with default values and
    saves it to a temporary JSON file. The file path is generated automatically
    using the create_temp_config_file utility.

    Returns:
        str: The path to the saved configuration file.

    Example:
        >>> filepath = demo_save_config()
        Configuration saved to /tmp/config_12345.json
    """
    from synthnova_config import RenderSimulatorConfig
    from synthnova_config.utils import save_config, create_temp_config_file

    # Create a config to save
    config = RenderSimulatorConfig()

    # Define save path
    filepath = create_temp_config_file(suffix=".json")

    # Save to file
    save_config(config=config, filepath=filepath)
    print(f"\nConfiguration saved to {filepath}")
    return filepath


def demo_load_config():
    """Demonstrate how to load configuration from a JSON file.

    This function shows how to load a RenderSimulatorConfig from a JSON file.
    It first creates and saves a sample configuration using demo_save_config(),
    then loads it back to demonstrate the loading process.

    The loaded configuration is displayed in both object and JSON formats.

    Returns:
        None

    Example:
        >>> demo_load_config()
        Type of config_dict: <class 'RenderSimulatorConfig'>
        Configuration loaded from /tmp/config_12345.json
        Configuration as JSON:
        {
            ...
        }
    """
    from synthnova_config import RenderSimulatorConfig
    from synthnova_config.utils import load_config

    example_file_path = demo_save_config()

    # Load from file
    config_dict = load_config(
        filepath=example_file_path, config_class=RenderSimulatorConfig
    )
    print(f"Type of config_dict: {type(config_dict)}")
    print(f"\nConfiguration loaded from {example_file_path}")
    print("\nConfiguration as JSON:")
    print(config_dict.model_dump_json(indent=4))


def demo_delete_config():
    """Demonstrate how to delete a configuration file.

    This function shows how to safely delete a configuration file from the filesystem.
    It first creates a temporary configuration file using demo_save_config(),
    then deletes it to demonstrate the deletion process.

    Returns:
        None

    Example:
        >>> demo_delete_config()
        Deleted /tmp/config_12345.json
    """
    from synthnova_config.utils import delete_config

    filepath = demo_save_config()
    delete_config(filepath=filepath)
    print(f"\nDeleted {filepath}")


def main():
    """Run all configuration demonstrations.

    This function serves as the entry point for running all configuration
    demonstrations. It shows the complete workflow of:
    1. Creating a configuration
    2. Saving it to a file
    3. Loading it from the file (optional)
    4. Deleting the configuration file (optional)

    Note:
        Some demo functions are commented out by default to prevent
        unintended file operations. Uncomment them to test specific
        functionality.

    Returns:
        None
    """
    demo_create_config()

    # Uncomment the following lines to test the other demos
    # demo_save_config()
    # demo_load_config()
    # demo_delete_config()


if __name__ == "__main__":
    main()
