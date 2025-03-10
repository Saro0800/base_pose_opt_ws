import numpy as np
from pymoo.core.problem import ElementwiseProblem
from pymoo.algorithms.moo.nsga2 import NSGA2
from pymoo.optimize import minimize
from pymoo.visualization.scatter import Scatter

# Define a Multi-Objective Optimization Problem with 3 objectives
class MyProblem(ElementwiseProblem):
    def __init__(self):
        super().__init__(n_var=3,   # Number of decision variables
                         n_obj=3,   # Number of objectives
                         n_constr=0,  # No constraints
                         xl=np.array([-2, -2, -2]),  # Lower bounds
                         xu=np.array([2, 2, 2]))  # Upper bounds

    def _evaluate(self, x, out, *args, **kwargs):
        f1 = x[0]**2 + x[1]**2 + x[2]**2  # First objective function
        f2 = (x[0] - 1)**2 + (x[1] - 1)**2 + (x[2] - 1)**2  # Second objective function
        f3 = (x[0] + 1)**2 + (x[1] + 1)**2 + (x[2] + 1)**2  # Third objective function
        out["F"] = np.array([f1, f2, f3])

# Define the NSGA-II Algorithm
algorithm = NSGA2()

# Solve the problem
problem = MyProblem()
res = minimize(problem,
               algorithm,
               ('n_gen', 100),  # Run for 100 generations
               seed=1,
               verbose=True)

# Plot the Pareto Front in 3D
plot = Scatter(title="Pareto Front (3 Objectives)", labels=["F1", "F2", "F3"], angle=(45,45))
plot.add(res.F, label="Pareto Front")
plot.show()
