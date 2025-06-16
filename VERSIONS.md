# Version Descriptions

| Version | Description |
| - | - |
| 0.12.0a0 | Original master version |
| 0.12.0a1 | Modified master version that contains the constant velocity, constant congestion case. This version also contains several modifications that might be necessary or unnecessary for the functioning of additional cases. Use this version as the base for any further expansions. |


# What to do for new versions?
1. Make the changes in the CostEvaluator.h to incude the implementation for any additional costs.
    i. When adding costs, add the declaration, add the implementation, and then include it in the penalisedCost function.
    ii. Modify the deltaCost to subtract the cost from the original cost and add the proposal cost.
    iii. If any new class variable are created, update the CostEvaluator.cpp, and bindings.cpp to allow it to be made available in Python.
    iv. Update the Python type hints to ensure proper support and functionality.
2. Run the build in debug mode.