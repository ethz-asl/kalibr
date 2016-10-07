#include <aslam/backend/ScalarDesignVariable.hpp>
#include <aslam/backend/ErrorTermObservation.hpp>
#include <aslam/backend/ErrorTermMotion.hpp>
#include <aslam/backend/ErrorTermPrior.hpp>
#include <aslam/backend/OptimizationProblem.hpp>
#include <aslam/backend/Optimizer.hpp>
#include <iostream>
// Bring in some random number generation from Schweizer Messer.
#include <sm/random.hpp>
#include <vector>
#include <algorithm>
#include <boost/foreach.hpp>


int main(int argc, char ** argv)
{
  if(argc != 2)
    {
      std::cout << "Usage: example K\n";
      std::cout << "The argument K is the number of timesteps to include in the optimization\n";
      return 1;
    }

  const int K = atoi(argv[1]);

  try 
    {
      // The true wall position
      const double true_w = -5.0;
      
      // The noise properties.
      const double sigma_n = 0.01;
      const double sigma_u = 0.1;
      const double sigma_x = 0.01;

      // Create random odometry
      std::vector<double> true_u_k(K);
      BOOST_FOREACH(double & u, true_u_k)
	{
	  u = sm::random::uniform();
	}
      
      // Create the noisy odometry
      std::vector<double> u_k(K);
      for(int k = 0; k < K; ++k)
	{
	  u_k[k] = true_u_k[k] + sigma_u * sm::random::normal();
	}

      // Create the states from noisy odometry.
      std::vector<double> x_k(K);
      std::vector<double> true_x_k(K);
      x_k[0] = 0.0;
      true_x_k[0] = 0.0;
      for(int k = 1; k < K; ++k)
	{
	  true_x_k[k] = true_x_k[k-1] + true_u_k[k];
	  x_k[k] = x_k[k-1] + u_k[k];
	}


      // Create the noisy measurments
      std::vector<double> y_k(K);
      for(int k = 0; k < K; ++k)
	{
	  y_k[k] = (1.0 / (true_w - true_x_k[k])) + sigma_n * sm::random::normal();
	}
      
      // Now we can build an optimization problem.
      boost::shared_ptr<aslam::backend::OptimizationProblem> problem( new aslam::backend::OptimizationProblem);
      
      // First, create a design variable for the wall position.
      boost::shared_ptr<aslam::backend::ScalarDesignVariable> dv_w(new aslam::backend::ScalarDesignVariable(true_w + sm::random::normal()));
      // Setting this active means we estimate it.
      dv_w->setActive(true);
      // Add it to the optimization problem.
      problem->addDesignVariable(dv_w);

      // Now we add the initial state.
      boost::shared_ptr<aslam::backend::ScalarDesignVariable> dv_x_km1(new aslam::backend::ScalarDesignVariable(x_k[0]));
      // Setting this active means we estimate it.
      dv_x_km1->setActive(true);
      // Add it to the optimization problem.
      problem->addDesignVariable(dv_x_km1);

      // Now create a prior for this initial state.
      boost::shared_ptr<aslam::backend::ErrorTermPrior> prior(new aslam::backend::ErrorTermPrior(dv_x_km1.get(), true_x_k[0], sigma_x * sigma_x));
      // and add it to the problem.
      problem->addErrorTerm(prior);
      
      // Now march through the states creating design variables,
      // odometry error terms and measurement error terms.
      for(int k = 1; k < K; ++k)
	{
	  boost::shared_ptr<aslam::backend::ScalarDesignVariable> dv_x_k(new aslam::backend::ScalarDesignVariable(x_k[k]));
	  dv_x_k->setActive(true);
	  problem->addDesignVariable(dv_x_k);

	  // Create odometry error
	  boost::shared_ptr<aslam::backend::ErrorTermMotion> em(new aslam::backend::ErrorTermMotion(dv_x_km1.get(), dv_x_k.get(), u_k[k], sigma_u * sigma_u));
	  problem->addErrorTerm(em);
	  
	  // Create observation error
	  boost::shared_ptr<aslam::backend::ErrorTermObservation> eo(new aslam::backend::ErrorTermObservation(dv_x_k.get(), dv_w.get(), y_k[k], sigma_n * sigma_n));
	  problem->addErrorTerm(eo);
	  
	  // Move this design variable to the x_{k-1} position for use in the next loop.
	  dv_x_km1 = dv_x_k;
	}

      // Now we have a valid optimization problem full of design variables and error terms.
      // Create some optimization options.
      aslam::backend::OptimizerOptions options;
      options.verbose = true;
      options.linearSolver = "cholmod";
      options.levenbergMarquardtLambdaInit = 10;
      options.doSchurComplement = false;
      options.doLevenbergMarquardt = true;
      // Force it to over-optimize
      options.convergenceDeltaX = 1e-12;
      options.convergenceDeltaJ = 1e-12;
      // Then create the optimizer and go!
      aslam::backend::Optimizer optimizer(options);
      optimizer.setProblem( problem );
      optimizer.optimize();

      
    }
  catch(const std::exception & e)
    {
      std::cout << "Exception during processing: " << e.what();
      return 1;
    }

  std::cout << "Processing completed successfully\n";
  return 0;
}
