Thoughts on how to implement the spline traj gen and following:

1) Build a list of essential waypoints with derivative constraints:
- start point, orientation specifies velocity vector
- point directly in front of gate center (by some distance, configurable)
- point in middle of gate
- point behind gate center (by same distance)
(PointArrayType)

2) Generate initial spline connecting waypoints.  Use an interpolating spline.  
Figure out how to best parameterize this spline (KnotVector, dimension) and 
whether any intermediate waypoints are needed.  

SplineType Eigen::SplineFitting< SplineType >::Interpolate 	( 	const PointArrayType &  	pts,
		DenseIndex  	degree,
		const KnotVectorType &  	knot_parameters 
	) 	
https://eigen.tuxfamily.org/dox/unsupported/structEigen_1_1SplineFitting.html#adc80b6f0dd0dbbea28130fb254626874

3) Determine the max centripetal acceleration of the spline.  We need to 
determine the metric value for kappa (the rate of change of curvature of 
the spline).  

Define Kappa => rate of change of curvature in metric units
Define Kappa_s => rate of change of curvature of spline (in non-dimensional units)

Kappa = Kappa_s * l/u

Where l has units of the metric you want to use (meters?) and u is the
parameter defining the spline (0 at beginning of spline, 1 at end)

Once we know Kappa, we need to find the max kappa along the spline and
then determine the max centripetal acceleration of the spline as follows:

a_c = v^2/R

Kappa = 1/R

So:

a_c = v^2/R

max(a_c) = max(Kappa) * v^2

There are two free parameters we need to work with: v (the average velocity/nominal velocity we want to follow)
and a_c_allowed (the max centripetal acceleration our vehicle can support or that we're willing to allow)

4) Now, we need to schedule our tangential acceleration/velocity along the
spline.  We will know what the initial velocity and final velocity will need to be.
We can impose some contraints and decide on an acceleration profile that we
like.  I think we'll want to start with constant acceleration, and limiting
the tangential acceleration to 1/2 the vehicle's acceleration limit.  This
will allow the other 1/2 to be used for centripetal acceleration.  

5) Once we have the spline fully parameterized, (which I think will be done 
by finishing step 4), now we need to determine the desired attitude at
each point along the spline.  This is done by first aligning the body x-vector
with the first derivative of the spline.  Then, the body z-axis will be aligned
with the acceleration vector (the centripetal acceleration, or acceleration 
of the spline/second derivative of the spline).  

To get the body y-axis, we simply take the cross product of the x and z axes.  

Now, we have a 3x3 matrix that represents the desired attitude of the vehicle at each point
along the spline.  

6) Convert this desired attitude from step 5 to a quaternion.   
Find the error between the vehicle's current attitude and this desired attitude.

Then, convert this error quaternion to an angle-axis formulation.  
We can apply gains to each component of the axis proportional to the angle*axis.  
This could be a simple Proportional loop, or might use a gain function of
some sort (like a sigmoid).  The outputs will be the desired angular rates.  

How do we get the thrust from this?
