Thoughts on how to implement the spline traj gen and following:

1) Build a list of essential waypoints with derivative constraints:
- start point, orientation specifies velocity vector
- point directly in front of gate center (by some distance, configurable)
- point in middle of gate
- point behind gate center (by same distance)
(PointArrayType)

2) Generate initial spline connecting waypoints

SplineType Eigen::SplineFitting< SplineType >::Interpolate 	( 	const PointArrayType &  	pts,
		DenseIndex  	degree,
		const KnotVectorType &  	knot_parameters 
	) 	
https://eigen.tuxfamily.org/dox/unsupported/structEigen_1_1SplineFitting.html#adc80b6f0dd0dbbea28130fb254626874

3) Evaluate smoothness?

4) Compute derivatives of spline.  Use as feedforwards?

5) Compute max velocity across the spline?

6) Look ahead by some distance.  Calculate desired attitude and velocity.

7) Compute desired angular rates and throttle based on quaternion error between desired and actual (error quaternion)
