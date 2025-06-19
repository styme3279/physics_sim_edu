# Physics Simulator Tests

This directory contains comprehensive test suite for the physics_simulator package.

## Test Structure

```
tests/
├── __init__.py              # Test package initialization
├── conftest.py             # Pytest fixtures and configuration
├── test_base_sim.py        # Tests for abstract base class
├── test_mujoco_simulator.py # Tests for MuJoCo simulator
├── test_robot_model.py     # Tests for robot model functionality
├── test_world.py           # Tests for world management
├── test_integration.py     # Integration tests for complete workflows
└── README.md              # This file
```

## Running Tests

### Quick Start

```bash
# Run all tests
python -m pytest tests/

# Run with coverage
python -m pytest --cov=src/physics_simulator tests/

# Run specific test file
python -m pytest tests/test_mujoco_simulator.py

# Run with verbose output
python -m pytest -v tests/
```

### Using the Test Runner Script

```bash
# Run all tests
python run_tests.py

# Run only unit tests
python run_tests.py --type unit

# Run integration tests
python run_tests.py --type integration

# Generate coverage report
python run_tests.py --type coverage

# Run tests without slow tests
python run_tests.py --fast
```

## Test Categories

Tests are organized into categories using pytest markers:

- `@pytest.mark.unit` - Unit tests for individual components
- `@pytest.mark.integration` - Integration tests for workflows
- `@pytest.mark.slow` - Tests that take longer to run
- `@pytest.mark.mujoco` - Tests requiring MuJoCo dependencies

### Running Specific Categories

```bash
# Run only unit tests
python -m pytest -m unit

# Run only integration tests
python -m pytest -m integration

# Skip slow tests
python -m pytest -m "not slow"
```

## Dependencies

The test suite requires the following additional packages:

```bash
pip install pytest pytest-cov pytest-mock
```

## Configuration

Test configuration is managed through:

- `pytest.ini` - Main pytest configuration
- `conftest.py` - Shared fixtures and test setup

## Coverage

Coverage reports are generated in HTML format:

```bash
python -m pytest --cov=src/physics_simulator --cov-report=html tests/
```

Open `htmlcov/index.html` in your browser to view the detailed coverage report.

## Mocking Strategy

Tests extensively use mocking to:

- Isolate components under test
- Avoid dependencies on external hardware/files
- Speed up test execution
- Ensure reproducible test results

Key mocking patterns:
- MuJoCo functions and classes
- File system operations
- External dependencies
- Hardware interfaces

## Writing New Tests

When adding new tests:

1. Place unit tests in the appropriate `test_*.py` file
2. Add integration tests to `test_integration.py`
3. Use appropriate pytest markers
4. Follow the existing mocking patterns
5. Keep tests focused and independent
6. Add docstrings explaining what is being tested

### Example Test Structure

```python
class TestNewFeature:
    """Test suite for new feature."""
    
    def test_basic_functionality(self, basic_config):
        """Test basic feature functionality."""
        # Arrange
        # Act
        # Assert
        
    @pytest.mark.slow
    def test_performance_intensive_feature(self):
        """Test performance-intensive functionality."""
        # Test implementation
``` 