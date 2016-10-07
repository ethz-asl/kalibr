#!/bin/env ipython

# Roslib gives us an easy way to pull in python libraries
import roslib
# load_manifest() looks for python export tags in the manifest files.
roslib.load_manifest("aslam_backend_tutorial")

# Once the manifest is loaded, the tutorial python package and all dependencies
# can be seen by python.
import aslam_backend as aslam
# sm is the Schweizer Messer library
import sm
# This is our tutorial package
import aslam_backend_tutorial as abt
# Numpy is the linear algebra library
import numpy as np
import pylab as pl

K = 1000

# The true wall position
true_w = -5.0;

# The noise properties.
sigma_n = 0.01;
sigma_u = 0.1;
sigma_x = 0.01;

# Create random odometry
true_u_k = np.random.random(K);
u_k = true_u_k + sigma_u * np.random.randn(K)      

# Integrate the odometry
x_k = np.cumsum(u_k)
true_x_k = np.cumsum(true_u_k)

# Create the noisy measurments
y_k = np.zeros(K)
for k in range(0,K):
    y_k[k] = 1.0 / (true_w - true_x_k[k]) + sigma_n * np.random.randn()


# Now we can build an optimization problem.
problem = aslam.OptimizationProblem()

#  Create a design variable for the wall position
dv_w = abt.ScalarDesignVariable(true_w + np.random.randn())
# Setting this active means we estimate it.
dv_w.setActive(True)
# Add it to the optimization problem.
problem.addDesignVariable(dv_w)

# Now we add the initial state.
dv_x_km1 = abt.ScalarDesignVariable(x_k[0])
# Setting this active means we estimate it.
dv_x_km1.setActive(True)
# Add it to the optimization problem.
problem.addDesignVariable(dv_x_km1)

# Now create a prior for this initial state.
prior = abt.ErrorTermPrior(dv_x_km1, true_x_k[0], sigma_x * sigma_x)
# and add it to the problem.
problem.addErrorTerm(prior)

# Now march through the states creating design variables,
# odometry error terms and measurement error terms.
for k in range(1,K):
	# Create the design variable at time k
	dv_x_k = abt.ScalarDesignVariable(x_k[k])
	dv_x_k.setActive(True)
	problem.addDesignVariable(dv_x_k)
	
	# Create odometry error
	em = abt.ErrorTermMotion(dv_x_km1, dv_x_k, u_k[k], sigma_u * sigma_u)
	problem.addErrorTerm(em)
	
	# Create observation error
	eo = abt.ErrorTermObservation(dv_x_k, dv_w, y_k[k], sigma_n * sigma_n)
	problem.addErrorTerm(eo)
	
	# Move this design variable to the x_{k-1} position for use in the next loop.
	dv_x_km1 = dv_x_k;

# Now we have a valid optimization problem full of design variables and error terms.
# Create some optimization options.
options = aslam.OptimizerOptions()
options.verbose = True
options.linearSolver = "cholmod"
options.levenbergMarquardtLambdaInit = 10
options.doSchurComplement = False
options.doLevenbergMarquardt = True
# Force it to over-optimize
options.convergenceDeltaX = 1e-12
options.convergenceDeltaJ = 1e-12
# Then create the optimizer and go!
optimizer = aslam.Optimizer(options)
optimizer.setProblem( problem )
optimizer.optimize()

