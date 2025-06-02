class PhysicsSimulatorError(Exception):
    """Base exception class for all errors in the Physics Simulator.
    
    All custom exceptions in the physics simulator package should inherit from this class
    to enable proper error handling and categorization.
    """

    pass


class XMLError(PhysicsSimulatorError):
    """Exception raised for errors related to XML processing.
    
    This exception is raised when there are issues with parsing, validating,
    or manipulating XML files used for model definitions, configurations, etc.
    """

    pass


class SimulationError(PhysicsSimulatorError):
    """Exception raised for errors occurring during simulation runtime.
    
    This includes physics calculation errors, state inconsistencies,
    or other issues that prevent the simulation from progressing correctly.
    """

    pass


class RandomizationError(PhysicsSimulatorError):
    """Exception raised for errors in randomization processes.
    
    This can occur when random number generation fails or produces
    values that would create an invalid simulation state.
    """

    pass
