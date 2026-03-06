#Conic State Extrapolation Routine (Spherical) / Precise Predict Extrapolation (Non-Spherical) WIP / Rodrigues Rotation formula / Instantaneous Impact Point
#GinGin 2023



#Conic State Extrapolation Routine to be used in PEG guidance for (r,v) gravity influence (Spherical Gravity)
#Based on Space Shuttle GNC equation document No.25 / Conic State Extrapolation by Shepperd and Robertson

##### CSE global variables and constants (foo_t is for tilde/normalized by r0 parameters) #####

var CSE = {

### Kepler Routine (Main function) ###
#Program constants
imax: 100, #Iterator max value
kmax: 100, #U1 series iterator max value
smin: 0, 
smax: 0,
epsilon_a: 0, #Tolerance for near parabolic orbits (not used)
epsilon_w: 0, #Tolerance for Q calculation (not used)
espilon_t_tilde: 0.000001, #Primary convergence criterion / normalized
epsilon_t: 0.000001, #Secondary convergence criterion / Secant iterator

#Input variables
#r0: [0,0,0],
#v0: [0,0,0],
#dt: 0,
x: 0,
dtc_prev: 0,
xc_prev: 0,

#Output variables
r_out: [0,0,0],
v_out: [0,0,0],
xc: 0,
dtc: 0,
e_flag: 0,


### Subroutine ###
#Kepler Transfer Time Interval 
dt_arg_t: 0,
A: 0, #value of Battin function
D: 0,
E: 0,

#U Continued Fraction
u1: 0,

#Q Continued Fraction Subroutine
Q: 0,

#Kepler Iteration Loop
x_guess_t: 0,
dt_guess_t: 0,

#Secant iterator
dx_t: 0,
xmin_t: 0,
dt_min_t: 0,
xmax_t: 0,
dt_max_t: 0,

};


##### Conic State Extrapolation as a function of Transfer Time - Kepler Routine (page 24 to 32) #####

#Main function (r0 and v0 are initial state vector parameters / dt time at which we need extrapolation) (5.1)
var conic_state_extrapolation = func (r0, v0, dt) {

##5-2
var f0 = 1;
if (dt < 0) {f0 = -1;}

var n = 0;
var r0_norm = SpaceShuttle.norm(r0);

var f1 = (f0 * math.sqrt(r0_norm)) / math.sqrt(SpaceShuttle.PEG_jsbsim.mu); #ft
var f2 = 1 / f1;
var f3 = f2 / r0_norm;
var f4 = f1 * r0_norm;
var f5 = f0 / math.sqrt(r0_norm);
var f6 = f0 * math.sqrt(r0_norm);

var ir0 = SpaceShuttle.scalar_product(1 / r0_norm, r0);
var v0_t = SpaceShuttle.scalar_product(f1, v0);
var sigma0_t = SpaceShuttle.dot_product(ir0, v0_t);
var b0 = SpaceShuttle.dot_product(v0_t, v0_t) - 1;
var a_t = 1 - b0; #1/a normalized


##5-3
CSE.x_guess_t = f5 * CSE.x; #Initial guess 
var xlast_t = f5 * CSE.xc_prev;
CSE.xmin_t = 0;
var dt_tilde = f3 * dt;
var dt_last_t = f3 * CSE.dtc_prev;
CSE.dt_min_t = 0;

#Orbit considered not parabolic for Shuttle operations (1/a greater than epsilon_a)
if (debug.isnan(a_t) == 0) {CSE.xmax_t = 2 * math.pi / math.sqrt(math.abs(a_t));}

var xP_t = CSE.xmax_t; #x over one orbit
var P_t = CSE.dt_max_t; #Period

if (a_t > 0)
    {
    CSE.dt_max_t = CSE.xmax_t / a_t;
    xP_t = CSE.xmax_t;
    P_t = CSE.dt_max_t;
    
    while (dt_tilde >= P_t)
        {
        n = n + 1;
        dt_tilde = dt_tilde - P_t;
        dt_last_t = dt_last_t - P_t;
        CSE.x_guess_t = CSE.x_guess_t - xP_t;
        xlast_t = xlast_t - xP_t;
        }
    }

else
    {
    kepler_transfer_time_interval(CSE.xmax_t, sigma0_t, a_t, CSE.epsilon_w, CSE.smax, CSE.kmax);
    CSE.dt_max_t = CSE.dt_arg_t; #output of above function is dtarg tilde

    while (CSE.dt_max_t < dt_tilde)
        {
        CSE.dt_min_t = CSE.dt_max_t;
        CSE.xmin_t = CSE.xmax_t;
        CSE.xmax_t = 2 * CSE.xmax_t;

        kepler_transfer_time_interval(CSE.xmax_t, sigma0_t, a_t, CSE.epsilon_w, CSE.smax, CSE.kmax);
        CSE.dt_max_t = CSE.dt_arg_t; 
        }
    }


##5-4
if ((CSE.xmin_t < CSE.x_guess_t) and (CSE.x_guess_t < CSE.xmax_t)) {CSE.x_guess_t = CSE.x_guess_t;}
else {CSE.x_guess_t = (CSE.xmin_t + CSE.xmax_t) / 2;}

kepler_transfer_time_interval(CSE.x_guess_t, sigma0_t, a_t, CSE.epsilon_w, CSE.smax, CSE.kmax);
CSE.dt_guess_t = CSE.dt_arg_t;

if (dt_tilde < CSE.dt_guess_t)
    {
    if ((CSE.x_guess_t < xlast_t) and (xlast_t < CSE.xmax_t) and (CSE.dt_guess_t < dt_last_t) and (dt_last_t < CSE.dt_max_t))
        {
        CSE.xmax_t = xlast_t;
        CSE.dt_max_t = dt_last_t;
        }
    }

else
    {
    if ((CSE.xmin_t < xlast_t) and (xlast_t < CSE.x_guess_t) and (CSE.dt_min_t < dt_last_t) and (dt_last_t < CSE.dt_guess_t))
        {
        CSE.xmin_t = xlast_t;
        CSE.dt_min_t = dt_last_t;
        }
    }

#Iteration loop
kepler_iteration_loop(CSE.imax, CSE.espilon_t_tilde, CSE.epsilon_t, dt_tilde, CSE.x_guess_t, CSE.dt_guess_t, CSE.xmin_t, CSE.dt_min_t, CSE.xmax_t, CSE.dt_max_t, sigma0_t, a_t, CSE.smax, CSE.kmax);


##5-5
var r_t = 1 + 2 * (b0 * CSE.A + sigma0_t * CSE.D * CSE.E);
var b4 = 1 / r_t;

#Final outputs
CSE.xc = f6 * (CSE.x_guess_t + n * xP_t);
CSE.dtc = f4 * (CSE.dt_guess_t + n * P_t);
extrapolated_state_vector(f2, b4, sigma0_t, r0_norm, CSE.A, CSE.D, CSE.E, ir0, v0_t);

#print("Universal eccentric anomaly converged is : ", CSE.xc, " Kepler routine transfer time is : ", CSE.dtc, " convergence error number is : ", CSE.e_flag);

#Previous converged values for transfer time and universal eccentric anomaly are stored
CSE.dtc_prev = CSE.dtc;
CSE.xc_prev = CSE.xc;

};


##### Subroutines (page 32 and beyond) #####

#Kepler Transfer Time (5.3.1)
var kepler_transfer_time_interval = func (xarg_t, sigma0_t, a_t, epsilon_w, smax, kmax) {

CSE.e_flag = 0;

U1_series_summation(xarg_t, a_t, kmax);
var z_t = 2 * CSE.u1;
if (debug.isnan(z_t) == 0) {CSE.E = 1 - 0.5 * a_t * math.pow(z_t, 2);}
var w = math.sqrt((1 + CSE.E) / 2);
CSE.D = w * z_t;
if (debug.isnan(z_t) == 0) {CSE.A = math.pow(CSE.D, 2);}
var B = 2 * (CSE.E + sigma0_t * CSE.D);

Q_continued_fraction(w, epsilon_w, smax);
CSE.dt_arg_t = CSE.D * (B + CSE.A * CSE.Q);

#print("Transfer time interval is : ", CSE.dt_arg_t);

};


#U1 series Summation (5.3.2)
var U1_series_summation = func (xarg_t, a_t, kmax) {

var delta_u1 = xarg_t / 4;
CSE.u1 = delta_u1;
var f7 = -a_t * math.pow(delta_u1, 2);

var k = 3;
while (k < kmax)
    {
    delta_u1 = (f7 * delta_u1) / (k * (k-1));
    var u1_old = CSE.u1;
    CSE.u1 = CSE.u1 + delta_u1;
    
    if (CSE.u1 == u1_old) {return;}
    k = k + 2;
    }

if (k >= kmax) {CSE.e_flag = 2;} #No convergence

#Output is u1 value
#print("u1 value is : ", CSE.u1, " eflag is : ", CSE.e_flag);

};


#Q Continued Fraction (5.3.3)
var Q_continued_fraction = func (w, epsilon_w, smax) {

var xq = 0;

if (w < epsilon_w) #w < 0
    {
    CSE.Q = 0;    
    return;
    }

else if ((w > epsilon_w) and (w < 1)) {xq = 21.04 - 13.04 * w;}
else if ((w >= 1) and (w < 4.625)) {xq = (5 * (2 * w + 5)) / 3;}
else if ((w >= 4.625) and (w < 13.846)) {xq = (10 * (w + 12)) / 7;}
else if ((w >= 13.846) and (w < 44)) {xq = (w + 60) / 2;}
else if ((w >= 44) and (w < 100)) {xq = (w + 164) / 4;}
else {xq = 70;}

var b = 0;
var y = (w - 1) / (w + 1);

var j = math.floor(xq);
var b = y / (1 + (1 - b) * ((j - 1) / (j + 2))); #Defined here / not a global var

while (j > 2)
    {
    b = y / (1 + (1 - b) * ((j - 1) / (j + 2)));
    j = j - 1;
    }

#Output is Q value
CSE.Q = (1 / math.pow(w, 2)) * (1 + (2 * (1 - b / 4)) / (3 * w * (w + 1)));
#print("j value is : ", j, " Q value is : ",  CSE.Q);

};


#Kepler Iteration Loop (5.3.4)
var kepler_iteration_loop = func (imax, espilon_t_tilde, epsilon_t, dt_tilde, x_guess_t, dt_guess_t, xmin_t, dt_min_t, xmax_t, dt_max_t, sigma0_t, a_t, smax, kmax) {

var i = 1;
while (i < imax)
    {
    var dt_error_t = dt_tilde - CSE.dt_guess_t;

    #10^-6 value to be checked
    if (math.abs(dt_error_t) < espilon_t_tilde) {return;}

    #Secant Iterator function
    secant_iterator(epsilon_t, dt_error_t, CSE.x_guess_t, CSE.dt_guess_t, CSE.xmin_t, CSE.dt_min_t, CSE.xmax_t, CSE.dt_max_t);
    var xold_t = CSE.x_guess_t;
    CSE.x_guess_t = CSE.x_guess_t + CSE.dx_t;
    if (CSE.x_guess_t == xold_t) {return;}

    #Kepler Transfer Time Interval
    var dt_old_t = CSE.dt_guess_t;
    kepler_transfer_time_interval(CSE.x_guess_t, sigma0_t, a_t, CSE.epsilon_w, smax, kmax);
    if (CSE.dt_guess_t == dt_old_t) {return;}

    i = i + 1;
    }

#No iteration convergence for normalized x and dt 
CSE.e_flag = CSE.e_flag + 1;

};



#Secant Iterator (5.3.5)
var secant_iterator = func (epsilon_t, dt_error_t, x_guess_t, dt_guess_t, xmin_t, dt_min_t, xmax_t, dt_max_t) {

var dt_min_prev = dt_guess_t - dt_min_t;
var dt_max_prev = dt_guess_t - dt_max_t;

if ((math.abs(dt_min_prev) < epsilon_t) or (math.abs(dt_max_prev) < epsilon_t)) 
    {
    CSE.dx_t = 0;
    }

else
    {
    if (dt_error_t < 0)
        {
        CSE.dx_t = ((x_guess_t - xmax_t) / dt_max_prev) * dt_error_t;
        if ((x_guess_t + CSE.dx_t) <= xmin_t) {CSE.dx_t = ((x_guess_t - xmin_t) / dt_min_prev) * dt_error_t;}

        #Output
        CSE.xmax_t = x_guess_t;
        CSE.dt_max_t = dt_guess_t;
        }

    else
        {
        CSE.dx_t = ((x_guess_t - xmin_t) / dt_min_prev) * dt_error_t;
        if ((x_guess_t + CSE.dx_t) >= xmin_t) {CSE.dx_t = ((x_guess_t - xmax_t) / dt_max_prev) * dt_error_t;}

        #Output
        CSE.xmin_t = x_guess_t;
        CSE.dt_min_t = dt_guess_t;
        }
    }

#print("dx is : ", CSE.dx_t, " xmin is : ", CSE.xmin_t, " dt min is : ", CSE.dt_min_t, " x max is : ", CSE.xmax_t, " dt max is : ", CSE.dt_max_t);

};



#Extrapolated State Vector (5.3.6) // i_r0 and v0_t are vectors
var extrapolated_state_vector = func (f2, b4, sigma0_t, r0_norm, A, D, E, ir0, v0_t) {

var F = 1 - 2 * A;
var G_tilde = 2 * (D * E + sigma0_t * A);
var Ft_tilde = -2 * b4 * D * E;
var Gt = 1 - 2 * b4 * A;

var temp1 = SpaceShuttle.scalar_product(F, ir0);
var temp2 = SpaceShuttle.scalar_product(G_tilde, v0_t);
var temp3 = SpaceShuttle.scalar_product(Ft_tilde, ir0);
var temp4 = SpaceShuttle.scalar_product(Gt, v0_t);

#Output is extrapolated state vector at time dt
CSE.r_out = SpaceShuttle.scalar_product(r0_norm, SpaceShuttle.add_vector(temp1, temp2));
CSE.v_out = SpaceShuttle.scalar_product(f2, SpaceShuttle.add_vector(temp3, temp4));

#print ("r out x is : ", CSE.r_out[0], " r out y is : ", CSE.r_out[1], "r out z is : ", CSE.r_out[2]);
#print ("v out x is : ", CSE.v_out[0], " v out y is : ", CSE.v_out[1], "v out z is : ", CSE.v_out[2]);

};



#Rodrigues Rotation formula for PEG initial guess: https://en.wikipedia.org/wiki/Rodrigues%27_rotation_formula
##### Rodrigues rotation formula #####

var rodrigues_rotation_formula = func (v, k, theta) {

#V vector to be rotated / k is a unit vector describing an axis of rotation about which v rotates by theta angle (degrees)

var outvec = [0,0,0];
k = SpaceShuttle.normalize(k);
theta = theta * 0.0174533;

var temp1 = SpaceShuttle.scalar_product(math.cos(theta), v);
outvec = temp1;

temp1 = SpaceShuttle.cross_product(k, v);
var temp2 = SpaceShuttle.scalar_product(math.sin(theta), temp1);
outvec = SpaceShuttle.add_vector(outvec, temp2);

temp1 = SpaceShuttle.dot_product(k, v);
temp2 = SpaceShuttle.scalar_product((1 - math.cos(theta)) * temp1, k);
outvec = SpaceShuttle.add_vector(outvec, temp2);

#print("outvec norm is : ", SpaceShuttle.norm(outvec));

return outvec;

};


#Instantaneous Impact Point computations (IIP) for IIP guidance law
#Guidance Law for Rocket from Jo and Ahn
##### Calculate IIP in an ECI and ECEF frame #####

var instantaneous_impact_point  = func {

#ECI intial state vector
var eci_r0 = [getprop("/fdm/jsbsim/position/eci-x-ft"), getprop("/fdm/jsbsim/position/eci-y-ft"), getprop("/fdm/jsbsim/position/eci-z-ft")];
var eci_v0 = [getprop("/fdm/jsbsim/velocities/eci-x-fps"), getprop("/fdm/jsbsim/velocities/eci-y-fps"), getprop("/fdm/jsbsim/velocities/eci-z-fps")];
var r0_norm = SpaceShuttle.norm(eci_r0);
var v0_norm = SpaceShuttle.norm(eci_v0);

#ECI iip state vector
var eci_rp = [0, 0, 0];
var rp_norm = 20898950; #Mean earth radius ft (6370km)

#LVLH frame based on initial inertial vector 
var ir = SpaceShuttle.normalize(eci_r0); #radial
var ih = SpaceShuttle.normalize(SpaceShuttle.cross_product(eci_r0, eci_v0)); #momentum direction
var i_theta = SpaceShuttle.cross_product(ih, ir); #prograde 


#Flight path angle
var temp1 = SpaceShuttle.dot_product(eci_r0, eci_v0);
var temp2 = r0_norm * v0_norm;
var temp3 = SpaceShuttle.MIDVAL(-1, temp1 / temp2, 1);
var gamma = math.asin(temp3);

#print("gamma is : ", gamma * D2R);

#Angle of flight of the rocket
var mu = 3.986004418e14 * 35.3147; #ft3.s-2
var h_norm = SpaceShuttle.norm(SpaceShuttle.cross_product(eci_r0, eci_v0)); #angular momentum moment
var c1 = - temp1 * h_norm / (mu * r0_norm);
var c2 = -1 + math.pow(h_norm, 2) / (mu * r0_norm);
var c3 = -1 + math.pow(h_norm, 2) / (mu * rp_norm);
var c1_pow = math.pow(c1, 2);
var c2_pow = math.pow(c2, 2);
var c3_pow = math.pow(c3, 2);

#print("c1 is :", c1, " c2 is  :", c2, " c3 is :", c3, " temp 1 is :", c1_pow * c3_pow - (c1_pow + c2_pow) * (c3_pow - c2_pow));

temp1 = math.sqrt(math.abs(c1_pow * c3_pow - (c1_pow + c2_pow) * (c3_pow - c2_pow)));
temp2 = (c1 * c3 + temp1) / (c1_pow + c2_pow);
temp3 = SpaceShuttle.MIDVAL(-1, temp2, 1);
var phi = math.asin(temp3);

#Distance of iip (Earth mean perimeter is 21598 nm)
var iip_distance = phi * 21598 / (2 * math.pi);

#print("angle of flight is : ", phi * R2D, " Distance of iip in Nm is : ", iip_distance);

return iip_distance;

}
