### Reflections

## The model

The implementation of Model Predictive Control is a kinematic model. The state consists of positions in x and y, velocity and orientation angle [x, y, v, psi]. The actuations, [a, steer_val] are the acceleration, in the range <-1, 1> and steering angle in the range <-25 deg, 25 deg>. The model is implemented in the mpc.cpp file and is outlined below:

	x[t+1] = x(t) + v(t) * cos(psi(t)) * dt;
	y[t+1] = y(t) + v(t) * sin(psi(t)) * dt;
	psi[t+1] = psi(t) + v(t) * delta(t) / Lf * dt;
	v[t+1] = v(t) + a(t) * dt;
	cte[t+1] = (f(t) - y(t)) + (v(t) * sin(epsi(t)) * dt);
	epsi[t+1] = (psi(t) - psides(t)) + v(t) * delta(t) / Lf * dt;

As can be seen in the model outline, state variables are calculated based on the state values from the previous timestep.

## Timestep Length and Elapsed Duration (N & dt)

To tune the parameters I began with dt equal to the simulated latency (0.1s). For large values of N the car would not predict the trajectory well, often cutting corners and hitting the curb during turns. For small values of N the car swiveled because it reacted too quickly to changes in road curvature. Eventually I settled on N = 6 which gave me a somewhat stable trajectory. However the car would still swivel on sharper turns.

To ammend that, I increased the dt to 0.2 but that resulted in car reacting too slowly to changes of curvature. Ultimately I settled on dt of 0.15. With these parameters I was able to drive the car safely and without oscillations around the track. 

I started out with a reference velocity of 25[m/s] which resulted in car speed of around 55 mph. I gradually raised it to see how fast I can drive the car around the track without causing unsafe situations. I was able to raise it to 30[m/s] which results in car speed around 65 mph.

## Polynomial Fitting and MPC Preprocessing

To obtain a second degree polynomial in map coordinates I apply a transformation to the waypoint data: 

	double transf_x, transf_y;
	transf_x =     (wayp_x- car_x)*cos(car_theta) + (wayp_y- car_y)*sin(car_theta);
	transf_y =  -1*(wayp_x- car_x)*sin(car_theta) + (wayp_y- car_y)*cos(car_theta);

The waypoints are transformed from map's coordinate system to car's. In this form they can provide useful coefficients that are used by the MPC algorithm and to visualize the waypoints on the simulator with the yellow line.

## Model Predictive Control with Latency

Latecy is solved by predicting state variables at a moment 0.1s in the future and inputting them into the solver. The latency adjustment in code is shown below:

	// adjust for latency
	double dt = 0.1;
	px = v*dt;
	psi = -v*str_ang*dt/2.67;
	// calculate cross-track and psi errors
	double cte = -polyeval(coeffs, px) - py;
	double epsi = -atan(coeffs[1]+2*coeffs[2]*px);
	// Calculate steer and throttle value using MPC
	Eigen::VectorXd state(6);
	state << px, 0, psi, v, cte, epsi;
	auto vars = mpc.Solve(state, coeffs);
