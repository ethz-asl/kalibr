#ifndef ASLAM_CAMERAS_FINITE_DIFFERENCES_HPP
#define ASLAM_CAMERAS_FINITE_DIFFERENCES_HPP

#define ASLAM_CAMERAS_ESTIMATE_JACOBIAN(classPointer, functionName, x0, step, outJ) \
  {																		\
																		\
	Eigen::VectorXd fx0;												\
	classPointer->functionName(x0,fx0);									\
	outJ.resize(fx0.size(), x0.size());									\
	Eigen::VectorXd fxp, fxm;											\
	Eigen::VectorXd x = x0;												\
	for(unsigned c = 0; c < x0.size(); c++)								\
	  {																	\
		x(c) += step;													\
		classPointer->functionName(x,fxp);								\
																		\
		x(c) -= 2.0*step;												\
		classPointer->functionName(x,fxm);								\
																		\
		x(c) += step;													\
		outJ.col(c) = (fxp - fxm)/(step*2.0);							\
																		\
	  }																	\
																		\
  }																		\
	  

#define ASLAM_CAMERAS_ESTIMATE_INTRINSIC_JACOBIAN(functionName, projectionClass, updateClass, x0, step, outJ) \
  {																		\
																		\
		Eigen::VectorXd fx0;											\
		projectionClass.functionName(x0,fx0);							\
		Eigen::VectorXd fxa, fxb, fxc, fxd;								\
		Eigen::MatrixXd I, IO;											\
		updateClass.getParameters(I);									\
		IO = I;															\
																		\
		outJ.resize(fx0.size(), I.size());								\
		for(unsigned c = 0; c < I.size(); c++)							\
		{																\
			I(c,0) += 2.0 * step;					 					\
			updateClass.setParameters( I );								\
			projectionClass.functionName(x0,fxa);						\
																		\
			I(c,0) -= step;											    \
			updateClass.setParameters(I);								\
			projectionClass.functionName(x0,fxb);						\
																		\
			I(c,0) -= 2.0 * step;										\
			updateClass.setParameters(I);								\
			projectionClass.functionName(x0,fxc);						\
																		\
			I(c,0) -= step;												\
			updateClass.setParameters(I);								\
			projectionClass.functionName(x0,fxd);						\
																		\
			updateClass.setParameters(IO);								\
			fx0 = ((8.0*fxb) + fxd - fxa - (8.0*fxc))/(step*12.0);		\
																		\
			for(int r = 0; r < outJ.rows(); ++r)						\
			  outJ(r,c) = fx0(r);										\
		}																\
																		\
  }																		\

#define ASLAM_CAMERAS_ESTIMATE_DISTORTION_JACOBIAN(functionName, distortionClass, x0, step, outJ) \
  {																		\
																		\
	    Eigen::VectorXd fx0 = x0;										\
		Eigen::VectorXd fxa = x0, fxb = x0, fxc = x0, fxd = x0;			\
		distortionClass.functionName(fx0);								\
		Eigen::MatrixXd I, IO;											\
		distortionClass.getParameters(I);								\
		IO = I;															\
																		\
																		\
        outJ.resize(fx0.size(), I.size());								\
		for(int c = I.size() - 1; c >= 0; --c)							\
		  {																\
			fxa = x0;													\
			I(c,0) += 2.0 * step;					 					\
			distortionClass.setParameters( I );							\
			distortionClass.functionName(fxa);							\
																		\
			fxb = x0;													\
			I(c,0) -= step;											    \
			distortionClass.setParameters(I);							\
			distortionClass.functionName(fxb);							\
																		\
			fxc = x0;													\
			I(c,0) -= 2.0 * step;										\
			distortionClass.setParameters(I);							\
			distortionClass.functionName(fxc);							\
																		\
			fxd = x0;													\
			I(c,0) -= step;												\
			distortionClass.setParameters(I);							\
			distortionClass.functionName(fxd);							\
																		\
																		\
			distortionClass.setParameters(IO);							\
			fx0 = ( (8.0*fxb) + fxd - fxa - (8.0*fxc))/(step*12.0);		\
																		\
			for(int r = 0; r < outJ.rows(); ++r)						\
			  outJ(r,c) = fx0(r);										\
		}																\
																		\
  }																		\


#endif /* ASLAM_CAMERAS_FINITE_DIFFERENCES_HPP */
