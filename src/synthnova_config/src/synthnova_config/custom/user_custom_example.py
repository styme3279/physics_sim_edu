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
# Module: User Custom Configuration Example
# Description: Example configuration for user-defined custom settings
# Author: Awesome User@Galbot
# Date: 2025-04-16
#
#####################################################################################

# This example demonstrates how to create custom configuration classes using Pydantic
# You can use this as a template to build your own configuration classes
# and combine them with other configurations in your project.

from pydantic import BaseModel, Field, ConfigDict
from typing import List, Dict, Optional, Union
from enum import Enum


class DatabaseType(str, Enum):
    """
    Enumeration of supported database types.
    
    Usage Example:
        # You can use this enum to specify database types in your configuration
        db_type = DatabaseType.POSTGRESQL
        if db_type == DatabaseType.MYSQL:
            # Handle MySQL specific configuration
            pass
    """

    POSTGRESQL = "postgresql"
    MYSQL = "mysql"
    SQLITE = "sqlite"
    MONGODB = "mongodb"


class CustomAPIConfig(BaseModel):
    """
    Configuration for custom API settings.
    
    This class demonstrates how to create a configuration class for API settings.
    You can extend this class or create similar ones for different components.
    
    Key Features:
    1. Type safety with Pydantic
    2. Default values
    3. Field descriptions
    4. Validation
    
    Usage Example:
        # Create a configuration instance
        api_config = CustomAPIConfig(
            base_url="https://your-api.com",
            timeout=60,
            retry_attempts=5
        )
        
        # Access configuration values
        print(api_config.base_url)
        print(api_config.timeout)
        
        # The configuration is automatically validated
        # Invalid values will raise validation errors
    """

    base_url: str = Field(
        default="https://api.example.com",
        description="Base URL for API endpoints. This is the root URL for all API calls."
    )

    timeout: int = Field(
        default=30,
        description="Request timeout in seconds. After this time, the request will be cancelled.",
        ge=1,  # Greater than or equal to 1
        le=300  # Less than or equal to 300
    )

    retry_attempts: int = Field(
        default=3,
        description="Number of times to retry failed requests before giving up.",
        ge=0,  # Greater than or equal to 0
        le=10  # Less than or equal to 10
    )

    headers: Dict[str, str] = Field(
        default_factory=lambda: {
            "Content-Type": "application/json",
            "Accept": "application/json",
        },
        description="Default headers to be included in all API requests."
    )

    # You can add more fields as needed
    # Example of adding an optional field with validation
    api_key: Optional[str] = Field(
        default=None,
        description="API key for authentication. If not provided, requests will be unauthenticated.",
        min_length=32,  # Minimum length of API key
        max_length=128  # Maximum length of API key
    )

    # Example of adding a list field
    allowed_methods: List[str] = Field(
        default=["GET", "POST"],
        description="List of allowed HTTP methods for API requests."
    )

    # Example of adding a nested configuration
    class RateLimitConfig(BaseModel):
        """
        Nested configuration for rate limiting settings.
        This shows how to create complex nested configurations.
        """
        requests_per_minute: int = Field(
            default=60,
            description="Maximum number of requests allowed per minute."
        )
        burst_size: int = Field(
            default=10,
            description="Maximum number of requests allowed in a burst."
        )

    rate_limit: RateLimitConfig = Field(
        default_factory=RateLimitConfig,
        description="Rate limiting configuration for API requests."
    )

    # Example of model configuration
    model_config = ConfigDict(
        # Add any Pydantic model configuration here
        # For example:
        extra="forbid",  # Forbid extra fields
        validate_assignment=True,  # Validate on assignment
    )
