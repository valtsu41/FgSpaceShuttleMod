#Ascent Power Explicit Guidance and Unified Power Flight Guidance for the Space Shuttle 
#GinGin 2021
#Based on Space Shuttle GNC equation document No. 24 / Unified Powered Flight Guidance



###### JSB sim variables for Ascent #####
var PEG_jsbsim = {

### Property nodes ###
#ECI
eci_x_node: 0,
eci_y_node: 0,
eci_z_node: 0,
eci_norm_node: 0,
eci_xdot_node: 0,
eci_ydot_node: 0,
eci_zdot_node: 0,
v_eci_norm_node: 0,
altitude_node: 0,
pitch_node: 0,
roll_node: 0,

#Time
time_node: 0,

#Thrust
fbx_prop_node: 0,
fby_prop_node: 0,
fbz_prop_node: 0,
weight_node: 0,

#FCS
pitch_ssme1_node: 0,
pitch_ssme2_node: 0,
pitch_ssme3_node: 0,

#Others
throttle_node: 0,
eng_op_node: 0,
lan_node: 0,


### Updated variables from node ###
r: [0,0,0],
v: [0,0,0],
r_norm: 0,
v_norm: 0,
earth_radius: 0,
t: 0, #SV time
f_prop_norm: 0,
f_prop: [0,0,0],
m: 0,
m_prev: 0,
m_dot: 101, 
mu: 0,
inc_tgt: 0, #rad
Rx: [0,0,0], #Rot matrix
lan: 0, #rad
throttle_factor: 0,
eng_op: 0,
theta: 0,
phi: 0,
pitch_ssme_mean: 0.0, #rad

node_init: 0,

};



##### PEG Nomenclature: global variables and constants #####
var PEG = {

#i-loaded targets (rd,vd,gamma,inc,LAN)
rd_height: 0, 
rd_norm: 0, #rd_norm = radius + rd_height
vd_norm: 0,
gamma_d: 0, #degrees
iy: [0,0,0], #Normal desired traj plane f(inc,LAN)
iy_init: [0,0,0], #Targeted orbit plane for HSI computations in pfd.nas
n: 2, #Number of phases 

#Constants
aL: 95.14, #3G constant acc
DT: 1.92, #Theoretical Update cycle
omega: 0.000072921159, #Angular rotation velocity of Earth rad.s

#Variables 
s_pass1: 1,
s_pre: 1, #Pre thrust
s_engoff: 0,
s_mode: 1,
s_phase: [0,0,1,0], #0 for constant F, 1 for constant a // length = n+1
k: 1, #Phase number
r_bias: [0,0,0],
r_grav: [0,0,0],
v_grav: [0,0,0],
rd: [0,0,0],
rd_init: [0,0,0], #First rd computation at peg init
tgo_total: 1,
tgo_total_prev: 1,
tgo: [0,1,1],
tb: [0,10,0],
rho: 1, #Vgo damping factor
vgo: [0,0,0],
vgo_undamped: [0,0,0],
rgo_xy: [0,0,0], #Rgo projected on normal plane (norm is xtrack)
v_bias: [0,0,0], #V miss
r_bias: [0,0,0],
dv_sensed: [0,0,0],
v_prev: [0,0,0],
V_ex: 14424, 
tau: 429,
t_prev: 0,
tig: 0, #time of ignition for active guidance (srb sep + 10s)
DT_real: 1.92, #Real update cycle
rthu_gain: 0, #Bias for RTHU transition

#Convergence criterion
bias_criterion: 0.05, #Vbias/Rbias filtered criterion
tgo_criterion: 5, #delta tgo
vmiss_criterion: 0.05, #delta Vmiss/go
release_criterion: 50, #tgo where position constraints are released (near MECO)
fine_count_criterion: 15, #End of PEG guidance (6s for RTLS)
phi_max: 10, #Lambdat dot * Tau ref clamp value 0.15

#Flags
peg_convergence: 0,
peg_loop_init: 0,
peg_constant_accel: 0, #Constant accel phase
peg_fine_count: 0, #Tgo - 10s
peg_meco: 0,
peg_converged_tgo: "No",
peg_converged_vmiss: "No",
vmiss_flag: 0, #Counter for Vmiss convergence
tmiss_flag: 0,
insertion_type: 0, #0 for direct / 1 for standard
abort_flag: "nominal",
rthu_flag: 0,
reset_flag: 0,
ksc_lan_flag: 0, #Correct forecasted LAN for KSC only
final_steering_flag: 0,
advanced_peg_flag: 0, #Advanced meco parameter
SPLANE: "OFF", #No constraint on iY target (current orbital plane for iY)

#TAL variables
RTHETA: 0, #Surface range between predicted MECO and aim point /ft
RT: [0,0,0], #HACEF in ECI coord 
CRNG_D: 0, #TAL Cross range in feet
CR_MAX: 500, #Cross range max in Nm
#A: [0,0,0,0,0,0,0,0,2054,2.2382,-3.4446e-4,1.8723e-8], #TAL MECO inertial velocity coefficient (A8-11) / meters !
A: [0,0,0,0,0,0,0,0,2174,2.2382,-3.4446e-4,1.8723e-8], #TAL MECO inertial velocity coefficient (A8-11) /A8 adjusted to have 23700ft/s for ZZA / meters  !
#Vd_norm_recomputation_flag: "OFF",

#Output
theta_thrust: 0,
tau_R: 0, #Ref time where iF and lambda are parallels (mid course)
phi: 0, #Thrust attitude angle (iF.lambda)
phi_dot: 0,
lambda: [0,0,0], #ref thrust direction (unit)
lambda_dot: [0,0,0], #turning rate
lambda_dot_unit: [0,0,0],
lamdba_dot_norm: 0, 
iF: [0,0,0], #commanded thrust vector f(lambda, lambda_dot)
PEG_theta: 20, #Theta output for jsbsim guidance
PEG_theta_prev: 20,

};


##### Unfied Powered Flight Guidance (improved PEG) main exec function / Computations done for a Shuttle two stages only ascent after SRB sep (n = 2) #####
var UPFG_main_routine = func {

#First pass node init
if (PEG_jsbsim.node_init == 0)
    {
    PEG_jsbsim.eci_x_node = props.globals.getNode("/fdm/jsbsim/position/eci-x-ft", 1);
    PEG_jsbsim.eci_y_node = props.globals.getNode("/fdm/jsbsim/position/eci-y-ft", 1);
    PEG_jsbsim.eci_z_node = props.globals.getNode("/fdm/jsbsim/position/eci-z-ft", 1);
    PEG_jsbsim.eci_norm_node = props.globals.getNode("/fdm/jsbsim/position/eci-norm-ft", 1); #Radius from Earth center to Shuttle
    PEG_jsbsim.altitude_node = props.globals.getNode("/position/altitude-ft", 1);
    PEG_jsbsim.pitch_node = props.globals.getNode("/fdm/jsbsim/systems/navigation/state-vector/pitch-deg", 1);
    PEG_jsbsim.roll_node = props.globals.getNode("/fdm/jsbsim/systems/navigation/state-vector/roll-deg", 1);

    PEG_jsbsim.eci_xdot_node = props.globals.getNode("/fdm/jsbsim/velocities/eci-x-fps", 1);
    PEG_jsbsim.eci_ydot_node = props.globals.getNode("/fdm/jsbsim/velocities/eci-y-fps", 1);
    PEG_jsbsim.eci_zdot_node = props.globals.getNode("/fdm/jsbsim/velocities/eci-z-fps", 1);
    PEG_jsbsim.v_eci_norm_node = props.globals.getNode("/fdm/jsbsim/velocities/eci-velocity-mag-fps", 1); #Vi
    PEG_jsbsim.time_node = props.globals.getNode("/fdm/jsbsim/sim-time-sec", 1);

    PEG_jsbsim.fbx_prop_node = props.globals.getNode("/fdm/jsbsim/forces/fbx-prop-lbs", 1);
    PEG_jsbsim.fby_prop_node = props.globals.getNode("/fdm/jsbsim/forces/fby-prop-lbs", 1);
    PEG_jsbsim.fbz_prop_node = props.globals.getNode("/fdm/jsbsim/forces/fbz-prop-lbs", 1);
    PEG_jsbsim.weight_node = props.globals.getNode("/fdm/jsbsim/inertia/mass-slugs", 1);
    PEG_jsbsim.eng_op_node = props.globals.getNode("/fdm/jsbsim/systems/mps/number-engines-operational", 1);

    #CoG Pitch
    PEG_jsbsim.pitch_ssme1_node = props.globals.getNode("/fdm/jsbsim/propulsion/engine[0]/pitch-to-CoG", 1);
    PEG_jsbsim.pitch_ssme2_node = props.globals.getNode("/fdm/jsbsim/propulsion/engine[1]/pitch-to-CoG", 1);
    PEG_jsbsim.pitch_ssme3_node = props.globals.getNode("/fdm/jsbsim/propulsion/engine[2]/pitch-to-CoG", 1);

    #Constant parameters
    PEG_jsbsim.mu = 3.986004418e14 * 35.3147; #ft3.s-2
    PEG_jsbsim.inc_tgt = getprop("/fdm/jsbsim/systems/ap/launch/inclination-target") * 0.0174533;
    PEG_jsbsim.Rx = [1,0,0,0,math.cos(PEG_jsbsim.inc_tgt),math.sin(PEG_jsbsim.inc_tgt),0,-math.sin(PEG_jsbsim.inc_tgt),math.cos(PEG_jsbsim.inc_tgt)];

    #Orbital parameters
    PEG_jsbsim.lan_node = props.globals.getNode("/fdm/jsbsim/systems/orbital/ascending-node-lon-rad", 1);

    #MECO i-loaded parameters if no already fed in mission file (km in feet) 
    var ap_tgt = getprop("/fdm/jsbsim/systems/ap/launch/apoapsis-target") * 3280.84;

    #Standard Insertion forced for low Ap (< 100 Nm)
    if ((ap_tgt < 610612) and (PEG.insertion_type == 0)) {PEG.insertion_type = 1;}
    #else if ((ap_tgt >= 610612) and (PEG.insertion_type == 1)) {PEG.insertion_type = 0;}


    #Advanced meco parameters from mission file (independant of ap targeted)
    if (getprop("/mission/post-meco/advanced-meco-peg-parameters"))
        {
        SpaceShuttle.advanced_meco_parameters();
        }

    #User friendly meco parameters computation
    else
        {
        if (PEG.insertion_type == 0) 
            {
            #Direct (Pe = 30 Nm) / MECO radius 345kft
            UFPG_MECO_parameters(345000, ap_tgt, 182283);
            SpaceShuttle.callout.make("Direct Insertion selected with a forecasted Apogee of "~sprintf("%3.0f",ap_tgt * 0.000164579)~" Nautical Miles", "help");
            }
        else 
            {
            #Standard (Pe = 10 Nm) / Ap max 100 Nm / MECO radius 360 kft
            var ap_tgt_standard = math.min(ap_tgt, 607612); 
            setprop("/fdm/jsbsim/systems/ap/launch/apoapsis-target", ap_tgt_standard / 3280.84);
            UFPG_MECO_parameters(360000, ap_tgt_standard, 60000);
            setprop("/fdm/jsbsim/systems/ap/launch/rthu-enable", 0); #No RTHU 
            SpaceShuttle.callout.make("Standard Insertion selected with a forecasted Apogee of "~sprintf("%3.0f",ap_tgt_standard * 0.000164579)~" Nautical Miles", "help");
            }
        
        print("Insertion type is : ", PEG.insertion_type);
        }
    PEG_jsbsim.node_init = 1;
    }



#Loop timer init for AutoLaunch nasal code
if (PEG.peg_loop_init == 0) {PEG.peg_loop_init = 1;}

#UFPG halted at regular MECO 
if (PEG.peg_meco == 1) 
    {
    PEG.peg_loop_init == 0;
    return;
    }

### Update of jsbsim hash ###

#SSME pitch for thrust vector computation degrees
PEG_jsbsim.pitch_ssme_mean = 57.29578 * math.avg(PEG_jsbsim.pitch_ssme1_node.getValue(), PEG_jsbsim.pitch_ssme2_node.getValue(), PEG_jsbsim.pitch_ssme3_node.getValue());

#State vector / Norm / Time
PEG_jsbsim.r = [PEG_jsbsim.eci_x_node.getValue(), PEG_jsbsim.eci_y_node.getValue(), PEG_jsbsim.eci_z_node.getValue()];
PEG_jsbsim.v = [PEG_jsbsim.eci_xdot_node.getValue(), PEG_jsbsim.eci_ydot_node.getValue(), PEG_jsbsim.eci_zdot_node.getValue()];
PEG_jsbsim.r_norm = SpaceShuttle.norm(PEG_jsbsim.r);
PEG_jsbsim.v_norm = SpaceShuttle.norm(PEG_jsbsim.v);
PEG_jsbsim.t = PEG_jsbsim.time_node.getValue();
PEG_jsbsim.earth_radius = PEG_jsbsim.r_norm - PEG_jsbsim.altitude_node.getValue(); #For better rd target
PEG_jsbsim.theta = PEG_jsbsim.pitch_node.getValue();
PEG_jsbsim.phi = PEG_jsbsim.roll_node.getValue();

#Total thrust (OMS + SSME + RCS + any prop things) 
PEG_jsbsim.f_prop = [PEG_jsbsim.fbx_prop_node.getValue(),PEG_jsbsim.fby_prop_node.getValue(),PEG_jsbsim.fbz_prop_node.getValue()];
PEG_jsbsim.f_prop_norm = SpaceShuttle.norm(PEG_jsbsim.f_prop);
PEG_jsbsim.eng_op = PEG_jsbsim.eng_op_node.getValue();

#Mass / Mass dot over one guidance cycle
PEG_jsbsim.m = PEG_jsbsim.weight_node.getValue();
PEG_jsbsim.m_dot = - (PEG_jsbsim.m - PEG_jsbsim.m_prev) / PEG.DT;
PEG_jsbsim.m_prev = PEG_jsbsim.m;

#Orbital parameters
PEG_jsbsim.lan = PEG_jsbsim.lan_node.getValue();


#print("Radius from Earth to Shuttle is : ", PEG_jsbsim.r_norm, " Inertial velocity is : ", PEG_jsbsim.v_norm, " State vector time is ; ", PEG_jsbsim.t);
#print("Total thrust is : ", PEG_jsbsim.f_prop_norm);
#print("Mass is : ", PEG_jsbsim.m, " Mass dot is : ", PEG_jsbsim.m_dot);
#print("Mu is : ", PEG_jsbsim.mu);
#print("Earth radius km is : ", PEG_jsbsim.earth_radius * 0.0003048);
#print("Mean SSME thrust pitch gimbal is : ", PEG_jsbsim.pitch_ssme_mean);


#Prethrust phase (No active guidance // Stage 1 opened loop before SRB sep)
if (PEG.s_pre > 0)
    {    
    ### Block 1 - Initialization (5-4) ###

    PEG.s_mode = 1; #PEG1 for ascent
    PEG.k = 1;
    PEG.s_pass1 = 1;
    PEG.s_engoff = 0;
    PEG.r_bias = [0,0,0];
    PEG.tgo_total = 1;
    PEG.r_grav = SpaceShuttle.scalar_product(-0.5 * PEG_jsbsim.mu / math.pow(PEG_jsbsim.r_norm,3), PEG_jsbsim.r);
    PEG.tig = PEG_jsbsim.t; #Time at PEG init is stored for Tau ref
    PEG_jsbsim.m_dot = 101; #3 engines 104% (3249 lbs/s or 101 slugs/s for initial tb computation)

    PEG.V_ex = PEG_jsbsim.f_prop_norm / PEG_jsbsim.m_dot;
    PEG.aT = PEG_jsbsim.f_prop_norm / PEG_jsbsim.m;
    PEG.tau = PEG.V_ex / PEG.aT;
    PEG.t_prev = PEG_jsbsim.t;
    
    #### To be activated when a proper LAN forecast will be ok ####
    
    #Orbital plan targeted // unit vector anti momentum h direction
    var lan = getprop("/fdm/jsbsim/systems/ap/launch/asc-nd-lon-tgt") * 0.0174533; #Forecasted LAN accurate for KSC and Northerly Launch
    #Current lan stored in jsbsim hash
    
    #Current Anti-momentum
    var temp1 = SpaceShuttle.cross_product(PEG_jsbsim.v, PEG_jsbsim.r);
    var iy_current = SpaceShuttle.normalize(temp1);

    var z_eci_unit = [0,0,-1]; #Pointing towards Earth Center
    var Rz = [math.cos(lan), math.sin(lan),0,-math.sin(lan),math.cos(lan),0,0,0,1];
    var temp1 = SpaceShuttle.matrix_3d_product(Rz,PEG_jsbsim.Rx);

    #Precise LAN Orbital Plane targeted only for KSC launch 
    if ((getprop("/mission/launch-site/launch-site-iata") == "KSC") and (getprop("/sim/gui/dialogs/SpaceShuttle/auto_launch/select-north") == 1))
        {
        PEG.iy = SpaceShuttle.scalar_product(1, SpaceShuttle.matrix_vector_product(temp1,z_eci_unit));
        PEG.ksc_lan_flag = 1;
        PEG.SPLANE = "ON";
        }

    #Current Orbital Plane (yaw steering done by low level jsbsim)
    else {PEG.iy = SpaceShuttle.normalize(iy_current);}

    

    #print("iy test 0 is : ", iy_test[0], " iy 1 test is : ", iy_test[1], " iy 2 test is : ", iy_test[2]);
    #print("iy 0 is : ", PEG.iy[0], " iy 1 is : ", PEG.iy[1], " iy 2 is : ", PEG.iy[2]);
    #print("norm iy is : ", SpaceShuttle.norm(PEG.iy));
    
    #PEG.iy = [-math.sin(lan) * math.sin(PEG_jsbsim.inc_tgt), math.cos(lan) * math.sin(PEG_jsbsim.inc_tgt), math.cos(PEG_jsbsim.inc_tgt)];

    #Initial computations for Rd and Vgo for better initial convergence (from SRB sep to MECO / 700Nm of downrange /13°ish of arc)
    #rd    
    temp1 = SpaceShuttle.rodrigues_rotation_formula(SpaceShuttle.normalize(PEG_jsbsim.r), SpaceShuttle.scalar_product(-1, PEG.iy), 13);
    PEG.rd_init = SpaceShuttle.scalar_product(PEG.rd_norm, temp1);
    PEG.rd = PEG.rd_init;
    #PEG.rd = [6486583, -15684600, 12899708]; #Perfect rd for ISS mission file

    #Vd
    var temp2 = SpaceShuttle.cross_product(SpaceShuttle.normalize(PEG.rd), PEG.iy);
    var vd = SpaceShuttle.scalar_product(PEG.vd_norm, temp2);
    #vd = [14916, 16360, 13036]; #Perfect vd for ISS test mission file

    #Vgo
    PEG.vgo = SpaceShuttle.subtract_vector(vd, PEG_jsbsim.v);

    

    SpaceShuttle.callout.make("Power Explicit Guidance is initialized", "help");

    
    #print("rd init norm is : ", SpaceShuttle.norm(PEG.rd), " Vgo init norm is : ", SpaceShuttle.norm(PEG.vgo));
    #print("rd x is : ", PEG.rd[0], " rd y is : ", PEG.rd[1], " rd z is : ", PEG.rd[2]);
    #print("vd x is : ", vd[0], " vd y is : ", vd[1], " vd z is : ", vd[2]);

    }

else
    { 
    ### Block 2 - Update (5-5) ###

    #Rd norm updated with current earth radius
    PEG.rd_norm = PEG_jsbsim.earth_radius + PEG.rd_height;

    #Read accelerometers for total vel change accumulated since last cycle
    PEG.dv_sensed = SpaceShuttle.subtract_vector(PEG_jsbsim.v, PEG.v_prev);
    PEG.v_prev = [PEG_jsbsim.v[0], PEG_jsbsim.v[1], PEG_jsbsim.v[2]];

    PEG.DT_real = PEG_jsbsim.t - PEG.t_prev;
    PEG.t_prev = PEG_jsbsim.t;

    if (PEG.s_pass1 == 1)
        {
        PEG.dv_sensed = [0,0,0];
        PEG.s_pass1 = 0;
        }
    
    else {PEG.vgo = SpaceShuttle.subtract_vector(PEG.vgo, PEG.dv_sensed);}

    #for (var i = PEG.k; i < (PEG.n + 1); i = i + 1)
    #    {
    #    PEG.tgo[i] = PEG.tgo[i] - PEG.DT_real;
    #    }

    PEG.tb[PEG.k] = PEG.tb[PEG.k] - PEG.DT_real;
    
    #Change of phase condition (Constant T to Constant A)
    if (((PEG.tb[1] <= 0) or (PEG.peg_constant_accel == 1)) and (PEG.k < PEG.n))
        {
        PEG.tgo[1] = 0;
        PEG.tb[1] = 0;
        PEG.k = math.min(PEG.k + 1, PEG.n); #Phase can't be higher than max required to avoid NaN
        }

    #print("Real delta is : ", PEG.DT_real);
    #print("dv sensed x is : ", PEG.dv_sensed[0], " dv sensed y is : ", PEG.dv_sensed[1], " dv sensed z is : ", PEG.dv_sensed[2]);
    #print("Phase number is : ", PEG.k, " Time to go for current phase is : ", PEG.tgo[PEG.k]);

    }


#Manual MECO exit condition
if (PEG_jsbsim.eng_op == 0) 
    {
    PEG.s_engoff = 1;
    return;
    }


### Block 3 - Time to go (5-7) ###

if (PEG_jsbsim.m_dot != 0)
    {
    PEG.aT = PEG_jsbsim.f_prop_norm / PEG_jsbsim.m;
    PEG.V_ex = PEG_jsbsim.f_prop_norm / PEG_jsbsim.m_dot;
    PEG.tau = PEG.V_ex / PEG.aT;
    }


var Li = [0,0,0]; #Thrust Integral / Vthrust predicted

if (PEG.peg_fine_count == 0)
    {
    if (PEG.k < PEG.n)
        {
        #Computation of Time to burn up to 3 G's (aL) for correct convergence
        PEG.tb[1] = PEG_jsbsim.m / PEG_jsbsim.m_dot - PEG.V_ex / PEG.aL;

        #Constant Thrust
        var temp = PEG.tau / (PEG.tau - PEG.tb[1]);

        #In case of very low thrust condition (2EO) / Vgo might be less than V predicted for phase 1 / Need to be carefull there !
        if (temp > 0) {Li[1] = math.min(PEG.V_ex * math.ln(temp), SpaceShuttle.norm(PEG.vgo));}
        else {Li[1] = 0;}

        #Time to burn recomputed in case of Vpredicted limited by Vgo
        PEG.tb[1] = PEG.tau * (1 - math.exp(-Li[1] / PEG.V_ex));
        }

    else
        {
        PEG.tb[1] = 0;
        PEG.tgo[1] = 0;
        Li[1] = 0;
        }

    #Constant Accel  
    Li[2] = math.max((SpaceShuttle.norm(PEG.vgo) - Li[1]), 0); #Avoid a negative value in case of no constant A phase (abort)
    PEG.tb[2] = Li[2] / PEG.aL;
    }

#In case of very low thrust condition (2EO) / Vgo might be less than V predicted for phase 1 / 
#if (Li[1] > (SpaceShuttle.norm(PEG.vgo))
#    {
#    Li[1] = (SpaceShuttle.norm(PEG.vgo);
#    PEG.tb[1] = PEG.tau * (1 - math.exp(-Li[1] / PEG.V_ex));
#    }


for (var i = PEG.k; i < (PEG.n + 1); i = i + 1)
    {
    PEG.tgo[i] = PEG.tgo[i-1] + PEG.tb[i];
    }

PEG.tgo_total_prev = PEG.tgo_total;
PEG.tgo_total = PEG.tgo[PEG.n];

#First convergence test (Newer Tgo and older Tgo - DT // Goes unconverged when 5 pass are above criterion)
if (math.abs(PEG.tgo[PEG.n] - (PEG.tgo_total_prev - PEG.DT_real)) < PEG.tgo_criterion) 
    {
    if (PEG.tmiss_flag != 0) {PEG.tmiss_flag = 0;}
    if (PEG.peg_converged_tgo == "No")
        {
        SpaceShuttle.callout.make("Ascent guidance time to go converged", "help");
        PEG.peg_converged_tgo = "Yes";
        }
    }


else 
    {
    if (PEG.peg_converged_tgo == "Yes")
        {
        PEG.tmiss_flag = PEG.tmiss_flag + 1;
        if (PEG.tmiss_flag == 5)
            {
            PEG.tmiss_flag = 0;
            PEG.peg_converged_tgo = "No";
            SpaceShuttle.callout.make("Ascent guidance time to go unconverged", "help");
            }
        }
    }

#Test for fine count flag (Tgo - 10s to freeze computations)
if ((PEG.tgo_total < PEG.fine_count_criterion) and (PEG.peg_constant_accel == 1) and (PEG.peg_fine_count == 0)) 
    {
    PEG.peg_fine_count = 1;
    if (PEG.SPLANE == "ON") {PEG.SPLANE = "OFF";}
    SpaceShuttle.callout.make(""~sprintf("%2.0f",PEG.fine_count_criterion)~" seconds to MECO, Fine count phase", "help");
    }



#First convergence test (Newer Tgo and older Tgo - DT)
#if (math.abs(PEG.tgo_total - PEG.tgo_total_prev) < PEG.tgo_criterion) {PEG.peg_converged_tgo = "Yes";}
#else {PEG.peg_converged_tgo = "No";}

#print ("L phase 1 is : ", Li[1], " L Phase 2 is : ", Li[2], " L total before block 4 is : ", Li[1] + Li[2]);
#print("Time to burn phase 1 is : ", PEG.tb[1], " Time to burn phase 2 is : ", PEG.tb[2], " time to go total is : ", PEG.tgo_total);
#print("Accel is : ", PEG.aT, " Exhaust velocity is : ", PEG.V_ex, " Tau is : ", PEG.tau);
#print("Tgo PEG converged is : ", PEG.peg_converged_tgo);
#print("Delta TGO is : ", math.abs(PEG.tgo[PEG.n] - (PEG.tgo_total_prev - PEG.DT_real)));
#print("Tgo for constant thrust phase is : ", PEG.tgo[1]);



### Block 4 - Integrals of Thrust (5-5) ###

var L_total = 0;
var J_total = 0;
var S_total = 0;
var Q_total = 0;
var H_total = 0;
var P_total = 0;

var Ji = [0,0,0];
var Si = [0,0,0];
var Qi = [0,0,0];
var Pi = [0,0,0];


for (var i = PEG.k; i < (PEG.n + 1); i = i + 1)
    {
    
    #Phase 1 constant thrust
    if (PEG.s_phase[i] == 0)
        {
        Ji[i] = PEG.tau * Li[i] - PEG.V_ex * PEG.tb[i];
        Si[i] = -Ji[i] + PEG.tb[i] * Li[i];
        Qi[i] = Si[i] * (PEG.tau + PEG.tgo[i-1]) - 0.5 * PEG.V_ex * math.pow(PEG.tb[i], 2);
        Pi[i] = Qi[i] * (PEG.tau + PEG.tgo[i-1]) - 0.5 * PEG.V_ex * math.pow(PEG.tb[i], 2) * ((1/3) * PEG.tb[i] + PEG.tgo[i-1]);
        }
    
    #Phase 2 constant accel
    else
        {
        Ji[i] = 0.5 * Li[i] * PEG.tb[i];
        Si[i] = Ji[i];
        Qi[i] = Si[i] * ((1/3) * PEG.tb[i] + PEG.tgo[i-1]);
        Pi[i] = (1/6) * Si[i] * (math.pow(PEG.tgo[i], 2) + 2 * PEG.tgo[i] * PEG.tgo[i-1] + 3 * math.pow(PEG.tgo[i-1], 2));
        }
    

    Ji[i] = Ji[i] + Li[i] * PEG.tgo[i-1];
    Si[i] = Si[i] + L_total * PEG.tb[i];
    Qi[i] = Qi[i] + J_total * PEG.tb[i];
    Pi[i] = Pi[i] + H_total * PEG.tb[i];

    L_total = L_total + Li[i];
    J_total = J_total + Ji[i];
    S_total = S_total + Si[i];
    Q_total = Q_total + Qi[i];
    P_total = P_total + Pi[i];
    H_total = J_total * PEG.tgo[i] - Q_total;


    }

#print("L total is (V to go predicted) : ", L_total, " J total is : ", J_total, " S total is (Range to go predicted) : ", S_total , " Q total is : ", Q_total, " P total is : ", P_total, " H total is : ", H_total);



### Block 5 - Turning Rate (5-10) ###

PEG.lambda = SpaceShuttle.normalize(PEG.vgo);

var temp1 = 0;
var temp2 = 0;
var temp3 = 0;
var temp4 = 0;
var rgo = [0,0,0];

if (PEG.s_mode < 6)
    {
    #Turning Rate computations are frozen close to Meco to avoid iF high rate of change
    if (PEG.tgo_total > PEG.fine_count_criterion)
        {
        #rgrav
        temp1 = math.pow(PEG.tgo_total / PEG.tgo_total_prev, 2);
        PEG.r_grav = SpaceShuttle.scalar_product(temp1, PEG.r_grav);

        #rgo
        temp1 = SpaceShuttle.scalar_product(PEG.tgo_total, PEG_jsbsim.v);
        temp2 = SpaceShuttle.add_vector(temp1, PEG.r_grav);
        temp3 = SpaceShuttle.add_vector(PEG_jsbsim.r, temp2);
        rgo = SpaceShuttle.subtract_vector(PEG.rd, temp3);

        #print("rgo norm km first computation is : ", SpaceShuttle.norm(rgo) * 0.0003048);

        #Downrange lvlh unit vector (iy defined as orbital plan targeted = f(lan,inc))
        temp1 = SpaceShuttle.cross_product(PEG.rd, PEG.iy);
        var iz = SpaceShuttle.normalize(temp1);

        #Rgo projection on xy plan (norm is xtrack)
        temp1 = SpaceShuttle.scalar_product(SpaceShuttle.dot_product(iz, rgo), iz);
        PEG.rgo_xy = SpaceShuttle.subtract_vector(rgo, temp1);

        #rgo iz component (prograde lvlh projection)
        temp1 = SpaceShuttle.dot_product(PEG.lambda, PEG.rgo_xy);
        temp2 = SpaceShuttle.dot_product(PEG.lambda, iz);
        var rgo_z = (S_total - temp1) / temp2;

        #Updated rgo
        temp1 = SpaceShuttle.add_vector(SpaceShuttle.scalar_product(rgo_z, iz), PEG.r_bias);
        rgo = SpaceShuttle.add_vector(PEG.rgo_xy, temp1);
        
        #Thrust turning rate vector
        temp1 = SpaceShuttle.subtract_vector(rgo, SpaceShuttle.scalar_product(S_total, PEG.lambda));
        temp2 = 1 / (Q_total - S_total * (J_total / L_total));
        #temp2 = 1 / (Q_total - S_total * (PEG.tgo_total / 2));
        PEG.lambda_dot = SpaceShuttle.scalar_product(temp2, temp1);

        #Lambda Norm stored
        PEG.lambda_dot_norm = SpaceShuttle.norm(PEG.lambda_dot);

        #Lambda dot normalized (Not mandatory)
        if (PEG.lambda_dot_norm != 0) {PEG.lambda_dot_unit = SpaceShuttle.normalize(PEG.lambda_dot);}
        else {PEG.lambda_dot_unit = [0,0,0];}


        #print("rgo component prograde is : ", rgo_z * 0.0003048); 
        #print("rgo prjected on radial/off plane norm is : ", SpaceShuttle.norm(PEG.rgo_xy) * 0.0003048);
        #print("rgrav norm km is : ", SpaceShuttle.norm(PEG.r_grav) * 0.0003048);
        #print("iz 0 is : ", iz[0], " iz 1 is : ", iz[1], " iz 2 is : ", iz[2]);
        #print("rgo norm km is : ", SpaceShuttle.norm(rgo) * 0.0003048); #downrange distance to MECO
        }

    #Turning rate independant of rgo close to MECO
    #else
    #    {
    #    temp1 = SpaceShuttle.cross_product(PEG.lambda, PEG_jsbsim.r);
    #    temp2 = SpaceShuttle.cross_product(temp1, PEG.lambda);
    #    PEG.lambda_dot = SpaceShuttle.scalar_product(PEG.lambda_dot_norm, temp2);

        #Lambda dot normalized (Not mandatory)
    #    if (SpaceShuttle.norm(PEG.lambda_dot) != 0) {PEG.lambda_dot_unit = SpaceShuttle.normalize(PEG.lambda_dot);}
    #    else {PEG.lambda_dot_unit = [0,0,0];}
    #    }


    #print("lambda dot 0 is : ", PEG.lambda_dot[0], " lambda dot 1 is : ", PEG.lambda_dot[1], "lambda dot 2 is : ", PEG.lambda_dot[2]);
    }

#Tau ref (mid course centroid time // Lamdba = iF) / JOL
if (L_total != 0) {PEG.tau_R = J_total / L_total;}
else {PEG.tau_R = PEG.tgo_total / 2;}

#phi_max reduced
#if ((PEG.phi_max == 10) and (PEG.tgo_total < (2 * PEG.release_criterion)))
#    {
#    PEG.phi_max = 0.30;
#    SpaceShuttle.callout.make("Lambdot clamped", "help");
#    }

#Lamdba dot clamped to avoid high rate of changes close to MECO
if ((PEG.tau_R * PEG.lambda_dot_norm) > PEG.phi_max)
    {
    PEG.lambda_dot_norm = PEG.phi_max / PEG.tau_R;
    PEG.lambda_dot = SpaceShuttle.scalar_product(PEG.lambda_dot_norm, PEG.lambda_dot_unit);
    }
temp1 = SpaceShuttle.scalar_product(PEG.tau_R, PEG.lambda_dot);

#Unit thrust direction (commanded thrust direction)
temp2 = SpaceShuttle.subtract_vector(PEG.lambda, temp1);
PEG.iF = SpaceShuttle.normalize(temp2);

#print("lambda dot * tau norm is : ", PEG.lambda_dot_norm * PEG.tau_R);
#print("lambda dot norm is : ", PEG.lambda_dot_norm);


#Thrust attitude angle
temp1 = SpaceShuttle.MIDVAL(SpaceShuttle.dot_product(PEG.iF, PEG.lambda), -1, 1);
if (debug.isnan(temp1) == 0) {PEG.phi = math.acos(temp1);}
if (PEG.tau_R != 0) {PEG.phi_dot = -(1 / PEG.tau_R) * PEG.phi;}

#New Vthrust (L_total) and Rthrust (J_total) ie. predicted effect of thrust with phi/ phi dot
temp1 = L_total - 0.5 * L_total * math.pow(PEG.phi, 2) - J_total * PEG.phi * PEG.phi_dot - 0.5 * H_total * math.pow(PEG.phi_dot, 2);
temp2 = SpaceShuttle.scalar_product(temp1, PEG.lambda);
temp3 = L_total * PEG.phi + J_total * PEG.phi_dot;
temp4 = SpaceShuttle.scalar_product(temp3, PEG.lambda_dot_unit);
var v_thrust = SpaceShuttle.subtract_vector(temp2, temp4);

temp1 = S_total - 0.5 * S_total * math.pow(PEG.phi, 2) - Q_total * PEG.phi * PEG.phi_dot - 0.5 * P_total * math.pow(PEG.phi_dot, 2);
temp2 = SpaceShuttle.scalar_product(temp1, PEG.lambda);
temp3 = S_total * PEG.phi + Q_total * PEG.phi_dot;
temp4 = SpaceShuttle.scalar_product(temp3, PEG.lambda_dot_unit);
var r_thrust = SpaceShuttle.subtract_vector(temp2, temp4);

#New Vthrust (L_total) and Rthrust (J_total) ie. predicted effect of thrust from PEG4/7 routine
temp1 = H_total - J_total * PEG.tau_R;
temp2 = L_total - 0.5 * math.pow(PEG.lambda_dot_norm, 2) * temp1;
var v_thrust_bis = SpaceShuttle.scalar_product(temp2, PEG.lambda);

var Q_bis = Q_total - S_total * PEG.tau_R;
#temp1 = P_total - PEG.tau_R * (Q_total - Q_bis);
#temp2 = S_total - 0.5 * math.pow(SpaceShuttle.norm(PEG.lambda_dot), 2) * temp1;
#temp3 = SpaceShuttle.scalar_product(temp2, PEG.lambda);
#temp4 = SpaceShuttle.scalar_product(Q_bis, PEG.lambda_dot);
#var r_thrust_bis = SpaceShuttle.add_vector(temp3, temp4);


temp1 = P_total - 2 * Q_total * PEG.tau_R + S_total * math.pow(PEG.tau_R, 2);
temp2 = S_total - math.pow(SpaceShuttle.norm(PEG.lambda_dot), 2) * temp1;
temp3 = SpaceShuttle.scalar_product(temp2, PEG.lambda);
temp4 = SpaceShuttle.scalar_product(Q_bis, PEG.lambda_dot);
var r_thrust_bis = SpaceShuttle.add_vector(temp3, temp4);



#Delta V and R between required and predicted effect of thrust (Vmiss and Rmiss for convergence criterion)

var v_bias_unfiltered = SpaceShuttle.subtract_vector(PEG.vgo, v_thrust);
var r_bias_unfiltered = SpaceShuttle.subtract_vector(rgo, r_thrust);

if (math.abs((SpaceShuttle.norm(PEG.vgo) - SpaceShuttle.norm(v_thrust))) < (PEG.bias_criterion * SpaceShuttle.norm(PEG.vgo))) {PEG.v_bias = v_bias_unfiltered;}
if (math.abs((SpaceShuttle.norm(rgo) - SpaceShuttle.norm(r_thrust))) < (PEG.bias_criterion * SpaceShuttle.norm(rgo))) {PEG.r_bias = r_bias_unfiltered;}

#if (PEG.tgo_total < PEG.release_criterion) {PEG.r_bias = [0,0,0];}

##print("tau mid course is : ", PEG.tau_R);
#print("V thrust predicted J is : ", SpaceShuttle.norm(v_thrust));
#print("R thrust predicted S km is : ", SpaceShuttle.norm(r_thrust) * 0.0003048);
#print("V thrust predicted delta is : ", SpaceShuttle.norm(v_thrust) - SpaceShuttle.norm(v_thrust_bis));
#print("R thrust predicted delta is : ", SpaceShuttle.norm(r_thrust) - SpaceShuttle.norm(r_thrust_bis));
#print("Thrust attitude angle is : ", PEG.phi * 57);
#print("V miss is : ", SpaceShuttle.norm(v_bias_unfiltered));
#print("R miss is : ", SpaceShuttle.norm(r_bias_unfiltered));



### Block 6 - Steering Command (5-11) ###


#LVLH frame (different convention than the one used in UFPG / x is prograde and z radial)
var local_z = SpaceShuttle.normalize(PEG_jsbsim.r); #radial

temp1 = SpaceShuttle.cross_product(PEG_jsbsim.r, PEG_jsbsim.v); 
temp2 = SpaceShuttle.cross_product(temp1, PEG_jsbsim.r); 
#temp1 = SpaceShuttle.cross_product([0,0,1], local_z);
var local_x = SpaceShuttle.normalize(temp2);

var local_y = SpaceShuttle.cross_product(local_x, local_z); #-h

#print("local y 0 is : ", local_y[0], "local y 1 is : ", local_y[1], "local y 2 is : ", local_y[2]);
#print("local z 0 is : ", local_z[0], "local z 1 is : ", local_z[1], "local z 2 is : ", local_z[2]);

#Unit thrust commanded direction LVLH Z axis angle (Theta Euler angle)
temp1 = SpaceShuttle.MIDVAL(SpaceShuttle.dot_product(PEG.iF, local_z), -1, 1);
var theta_iF_euler = 90 - 57.29578 * math.acos(temp1);

#Current Inertial Flight Path for fine count steering
temp1 = SpaceShuttle.normalize(PEG_jsbsim.v);
temp2 = SpaceShuttle.MIDVAL(SpaceShuttle.dot_product(temp1, local_z), -1, 1);
var gamma_euler = 90 - 57.29578 * math.acos(temp2);



############## Test values ###################
#Test values (Lambda / Angle of Attack / Inertial gamma)


#temp1 = SpaceShuttle.MIDVAL(SpaceShuttle.dot_product(PEG.lambda, local_z), -1, 1);
#var lambda_lvlh = 90 - 57.29578 * math.acos(temp1);

#temp1 = SpaceShuttle.normalize(PEG_jsbsim.v);
#temp2 = SpaceShuttle.MIDVAL(SpaceShuttle.dot_product(temp1, local_x), -1, 1);
#var vv_lvlh = 57.29578 * math.acos(temp2);
#temp2 = SpaceShuttle.MIDVAL(SpaceShuttle.dot_product(PEG.iF, temp1), -1, 1);
#var if_vv = 57.29578 * math.acos(temp2);

#temp1 = SpaceShuttle.MIDVAL(SpaceShuttle.dot_product(PEG.lambda_dot, PEG.lambda), -1, 1);
#if (debug.isnan(temp1) == 0) {var lambda_ortho = 57.29578 * math.acos(temp1);}

#temp1 = SpaceShuttle.normalize(PEG_jsbsim.r);
#temp2 = SpaceShuttle.MIDVAL(SpaceShuttle.dot_product(local_x, PEG.lambda), -1, 1);
#var lambda_r = 57.29578 * math.acos(temp2);

############## Test values ###################





#Commanded theta (SSME yaw to be added)
var theta_commanded = theta_iF_euler + math.cos(PEG_jsbsim.phi / 57.29578) * (16 + PEG_jsbsim.pitch_ssme_mean) + PEG.rthu_gain;

#Theta to jsbsim via autolaunch.nas if PEG converged (Vmiss, Rmiss, Tgo criterions)
#Theta frozen when Tgo < 10s 
if (PEG.tgo_total > (PEG.fine_count_criterion))
    {
    if (math.abs(PEG.PEG_theta_prev - PEG.PEG_theta) < 5)
        {
        if ((PEG.peg_converged_tgo == "Yes") and (PEG.peg_converged_vmiss == "Yes")) 
            {
            PEG.PEG_theta = theta_commanded;
            }
        }
    else 
        {
        PEG.PEG_theta = PEG.PEG_theta_prev;
        UFPG_reset("reset");
        SpaceShuttle.callout.make("UFPG reset", "help");

        #Ascent Cue Card interpolation to be done there
        }
    
    PEG.PEG_theta_prev = PEG.PEG_theta;
    }

#Inertial gamma MECO targeting
else if ((PEG.final_steering_flag == 0) and (PEG.tgo_total < (PEG.fine_count_criterion)))
    {
    #var delta_gamma = -PEG.gamma_d + gamma_euler;
    #PEG.PEG_theta = PEG.PEG_theta_prev - 4 * delta_gamma;
    #PEG.PEG_theta = PEG.PEG_theta_prev;

    UFPG_final_steering();
    }



#print("Inertial FPA is : ", gamma_euler);
#print("Theta LVLH is : ", PEG_jsbsim.theta);
#print("iF theta euler is : ", theta_iF_euler);
#print("Theta commanded is : ", PEG.PEG_theta);
#print("Theta lambda is : ", lambda_lvlh, " Gamma real is : ", vv_lvlh); 
#print("Orthogonality check : ", lambda_ortho);
#print("Lambda and local x angle is : ", lambda_r);
#print("iF and vv angles is : ", if_vv);
#print("lvlh ortho check : ", 57.29 * math.acos(SpaceShuttle.MIDVAL(SpaceShuttle.dot_product(local_x, local_z), -1, 1)));
#print("RTHU gain is : ", PEG.rthu_gain);



### Block 7 - Gravity Effects (5-12) ###

#Initial state vector at present loop time
temp1 = SpaceShuttle.scalar_product(-1/10, r_thrust);
temp2 = SpaceShuttle.scalar_product(-1/30 * PEG.tgo_total, v_thrust);
var delta_rc = SpaceShuttle.add_vector(temp1, temp2);

temp1 = SpaceShuttle.scalar_product(6 / (5 * PEG.tgo_total), r_thrust);
temp2 = SpaceShuttle.scalar_product(-1/10, v_thrust);
var delta_vc = SpaceShuttle.add_vector(temp1, temp2);

var rc1 = SpaceShuttle.add_vector(PEG_jsbsim.r, delta_rc);
var vc1 = SpaceShuttle.add_vector(PEG_jsbsim.v, delta_vc);


#Propagated state vector due to gravity alone at MECO forecasted time (Precise J2 prediction to be done)
SpaceShuttle.conic_state_extrapolation(rc1, vc1, PEG.tgo_total);
var rc2 = SpaceShuttle.CSE.r_out;
var vc2 = SpaceShuttle.CSE.v_out;

#Updated state vector grav
PEG.v_grav = SpaceShuttle.subtract_vector(vc2, vc1);

temp1 = SpaceShuttle.scalar_product(PEG.tgo_total, vc1);
temp2 = SpaceShuttle.add_vector(rc1, temp1);
PEG.r_grav = SpaceShuttle.subtract_vector(rc2, temp2);

#print("r grav norm block 7 km is : ", SpaceShuttle.norm(PEG.r_grav) * 0.0003048);



### Block 8 - Velocity To Be Gained (5-13) ###

if (PEG.tgo_total > PEG.fine_count_criterion)
    {
    #Predicted radius vector at MECO (RP)
    temp1 = SpaceShuttle.add_vector(PEG.r_grav, r_thrust);
    temp2 = SpaceShuttle.add_vector(PEG_jsbsim.r, SpaceShuttle.scalar_product(PEG.tgo_total, PEG_jsbsim.v));
    var r_pred = SpaceShuttle.add_vector(temp2, temp1);

    #Predicted velocity vector at MECO (VP)
    temp1 = add_vector(PEG_jsbsim.v, PEG.v_grav);
    var v_vpred = add_vector(temp1, v_thrust);


    #Rp projected into the desired orbital plane if orbit plane is constrained
    if (PEG.SPLANE == "ON")
        {
        temp1 = SpaceShuttle.scalar_product(SpaceShuttle.dot_product(r_pred, PEG.iy), PEG.iy);
        r_pred = SpaceShuttle.subtract_vector(r_pred, temp1);
        }

    if ((PEG.tgo_total > PEG.release_criterion) and (PEG.reset_flag < 3))
        {
        #Desired radius vector at MECO / S ALT to be introduced
        temp1 = SpaceShuttle.normalize(r_pred);
        PEG.rd = SpaceShuttle.scalar_product(PEG.rd_norm, temp1);
        }

    #At least 3 reset and no convergence or TAL after a droop // we need to release the rd constraint
    else if (((PEG.reset_flag > 0) or (SpaceShuttle.droop_flag == 1)) and (PEG.tgo_total > PEG.release_criterion)) {PEG.rd = r_pred;}

    #Radius constraint released
    else if (PEG.tgo_total < PEG.release_criterion)
        {
        #Constraints released close to MECO on rd to avoid lambda dot high rate of change
        PEG.rd = r_pred;
        if (PEG.SPLANE == "ON") {PEG.SPLANE = "OFF";}

        #phi_max reduced
        if (PEG.phi_max != 0.15) 
            {
            PEG.phi_max = 0.15;
            SpaceShuttle.callout.make("Time to MECO is "~sprintf("%2.0f",PEG.release_criterion)~"  seconds, Guidance MECO radius constraint is released", "help");
            }

        #Phi_max linear function until MECO
        #PEG.phi_max = PEG.tgo_total * (0.15 / PEG.release_criterion);

        }

    

    #LVLH frame at desired cutoff position (x is radial, z downrange, y normal defined in i-loaded parameters)
    var ix = SpaceShuttle.normalize(PEG.rd);
    var iz = SpaceShuttle.cross_product(ix, PEG.iy);

    #TAL Vdmag recomputed and Xrange computation for iy steering (A4.8.1.A15) 
    #if ((PEG.abort_flag == "TAL") and (PEG.Vd_norm_recomputation_flag == "OFF"))
    if (PEG.abort_flag == "TAL") 
        {
        #HAC center in EarthFixed coordinates 
        if (rwy_coord.RLS[0] == 0) 
            {
            compute_TAEM_guidance_targets();
            #print("TAL initial RLS computation is :", "YES");
            }
        
        #ECI coord of aim point at predicted MECO time
        PEG.RT = matrix_vector_product(M_CONV_EARTH_FIXED_TO_ECI(PEG.tgo_total), rwy_coord.RLS);
        PEG.RT = normalize(PEG.RT);

        #Surface range at MECO in feet
        PEG.RTHETA = norm(rwy_coord.RLS) * math.acos(safe_acos(dot_product(ix, PEG.RT)));

        #TIME from closest approache to TAL aim point
        var T_CA = PEG.RTHETA / PEG.vd_norm;

        #Surface range at MECO in km
        PEG.RTHETA = PEG.RTHETA * FT2M / 1000;
        #print("Norm of RLS in Nm is : ", norm(rwy_coord.RLS) * 0.000164579, " ARC between cutoff and aim point in degrees is : ", math.acos(safe_acos(dot_product(ix, PEG.RT))) * R2D);
        
        #Desired velocity vector at MECO in ft/s
        PEG.vd_norm = (PEG.A[8] + PEG.A[9] * PEG.RTHETA + PEG.A[10] * math.pow(PEG.RTHETA, 2) + PEG.A[11] * math.pow(PEG.RTHETA, 3)) * M2FT ;
        #print("TAL range from MECO to target in km is :", PEG.RTHETA, " Desired velocity at MECO for TAL is : ", PEG.vd_norm);

        #Vd_norm clamped for min and max (18400 for Canadian site and 25200 for Diego Garcia)
        PEG.vd_norm = MIDVAL(PEG.vd_norm, 18400, 25200);
        
        #No more constraint on targeted plane
        if (PEG.SPLANE == "ON") {PEG.SPLANE = "OFF";}

        #TIME from closest approache to TAL aim point
        #var T_CA = PEG.RTHETA / PEG.vd_norm;
        var RT_AIM = matrix_vector_product(M_CONV_EARTH_FIXED_TO_ECI(PEG.tgo_total + T_CA), rwy_coord.RLS);
        #print("Time to TAL aim point from MECO is ", T_CA);

        #Current target orbit plane for TAL
        var iy_current = SpaceShuttle.cross_product(PEG_jsbsim.v, PEG_jsbsim.r);
        PEG.iy = SpaceShuttle.normalize(iy_current);

        #Current crossrange
        PEG.CRNG_D = dot_product(RT_AIM, PEG.iy);
        #print("X range from PEG guidance in Nm is ", PEG.CRNG_D / 6076.12);

        #Crossrange max is 500 Nm, above that yaw steering available (To be done in ap.xml)
        if (PEG.CRNG_D > (PEG.CR_MAX * 6076.12))
            {
            temp1 = cross_product(v_vpred, r_pred);
            PEG.iy = normalize(temp1);

            #TAL aim point adjusted for max crossrange
            temp1 = scalar_product(math.sgn(PEG.CRNG_D) * PEG.CR_MAX * 6076.12, PEG.iy);
            RT_AIM = subtract_vector(RT_AIM, temp1);

            #New target iY
            temp1 = cross_product(RT_AIM, r_pred);
            PEG.iy = normalize(temp1);
            }
        }



    #Desired velocity vector at MECO
    temp1 = [ix[0], ix[1], ix[2], PEG.iy[0], PEG.iy[1], PEG.iy[2], iz[0], iz[1], iz[2]]; 

    temp2 = [math.sin(PEG.gamma_d * 0.0174533), 0, math.cos(PEG.gamma_d * 0.0174533)];
    temp3 = SpaceShuttle.matrix_vector_product(temp1, temp2);
    var vd = SpaceShuttle.scalar_product(PEG.vd_norm, temp3);
    
    #Plane is not constrained
    #if (PEG.SPLANE == "OFF") {PEG.iy = cross_product(vd, PEG.rd);}
    if ((PEG.SPLANE == "OFF") and (PEG.abort_flag != "TAL"))    
        {
        var iy_current = SpaceShuttle.cross_product(PEG_jsbsim.v, PEG_jsbsim.r);
        PEG.iy = SpaceShuttle.normalize(iy_current);
        }

    #print("Plane is constrained : ", PEG.SPLANE);

    #Second convergence test before new Vgo is computed (Goes unconverged when 5 pass are above criterion)
    if (math.abs(SpaceShuttle.norm(SpaceShuttle.subtract_vector(PEG.vgo, v_thrust))) < math.abs(PEG.vmiss_criterion * SpaceShuttle.norm(PEG.vgo))) 
        {
        if (PEG.vmiss_flag != 0) {PEG.vmiss_flag = 0;} 
        if (PEG.peg_converged_vmiss == "No")
            {
            SpaceShuttle.callout.make("Ascent guidance converged, auto steering available", "help");
            PEG.peg_converged_vmiss = "Yes";
            #PEG.reset_flag = 0;
            } 
        } 
    else 
        {
        if (PEG.peg_converged_vmiss == "Yes")
            {
            PEG.vmiss_flag = PEG.vmiss_flag + 1;
            if (PEG.vmiss_flag == 5)
                {
                PEG.vmiss_flag = 0;
                PEG.peg_converged_vmiss = "No";
                SpaceShuttle.callout.make("Ascent guidance unconverged, be ready for manual takeover", "help");
                UFPG_reset("reset");
                }
            }
        }

    #Velocity to be gained update (rho factor to be determined / zero for Ascent / rho matrix for RTLS)
    temp1 = SpaceShuttle.subtract_vector(SpaceShuttle.scalar_product(0.5, PEG.v_bias), PEG.v_grav);
    temp2 = SpaceShuttle.subtract_vector(vd, PEG_jsbsim.v);
    PEG.vgo_undamped = SpaceShuttle.add_vector(temp2, temp1);
    PEG.vgo = PEG.vgo_undamped;
    #if (PEG.peg_fine_count == 0) {PEG.vgo = PEG.vgo_undamped;}



    #print("r pred norm is : ", SpaceShuttle.norm(r_pred), " desired radius norm is : ", SpaceShuttle.norm(PEG.rd));
    #print("delta r pred norm is : ", math.abs(SpaceShuttle.norm(r_pred) - PEG.rd_norm));
    #print("v desired at MECO is : ", SpaceShuttle.norm(vd));
    



    #print("Peg converged second test is : ", PEG.peg_converged_vmiss);
    #print("Peg unconverged flag number is :", PEG.vmiss_flag);
    #print("Vmiss / Vgo norm is : ", 100 * SpaceShuttle.norm(v_bias_unfiltered) / SpaceShuttle.norm(PEG.vgo));

    #print("Vd coord are: ", vd[0], " ", vd[1], " ", vd[2]);
    #print("Rd coord are: ", PEG.rd[0], " ", PEG.rd[1], " ", PEG.rd[2]);
    #print("iy 0 is : ", PEG.iy[0], " iy 1 is : ", PEG.iy[1], " iy 2 is : ", PEG.iy[2]);

    #print("fine count flag is : ", PEG.peg_fine_count);

    }

#print("vgo norm is : ", SpaceShuttle.norm(PEG.vgo));

#Active guidance init flag
if (PEG.s_pre > 0) {PEG.s_pre = 0;}


settimer( func {UPFG_main_routine(); }, PEG.DT);

};


##### Reset function in case PEG is going unconverged / Abort MECO recomputations (ATO / TAL / DROOP) ####
var UFPG_reset = func (type) {

if (type == "reset")
    {
    #Arbitrary manner of self starting the algorithm // Explicit solution [...] for rocket propelled vehicles / Jaggers AIAA
    PEG.lambda_dot = [0,0,0];
    PEG.lambda_dot_unit = [0,0,0];
    PEG.lambda = SpaceShuttle.normalize(PEG_jsbsim.v);
    PEG.r_grav = SpaceShuttle.scalar_product(-0.5 * PEG_jsbsim.mu / math.pow(PEG_jsbsim.r_norm,3), PEG_jsbsim.r);

    #Bias reset
    PEG.v_bias = [0,0,0];
    PEG.r_bias = [0,0,0];

    #rd
    PEG.rd = PEG.rd_init;

    #Vd
    var temp2 = SpaceShuttle.cross_product(SpaceShuttle.normalize(PEG.rd), PEG.iy);
    var vd = SpaceShuttle.scalar_product(PEG.vd_norm, temp2);
        
    #Vgo
    PEG.vgo = SpaceShuttle.subtract_vector(vd, PEG_jsbsim.v);

    #Constraint release on plane for better convergence
    if (PEG.SPLANE == "ON") {PEG.SPLANE = "OFF";}

    PEG.reset_flag = PEG.reset_flag + 1;

    SpaceShuttle.callout.make("Ascent guidance unconverged, Reset", "help");
    }

else if (type == "TAL")
    {
    #Nominal TAL
    if (SpaceShuttle.droop_flag == 0)
        {
        #Radius height at MECO is 360 kft / Shallow FPA / AP 55Nm / PE -500Nm
        #UFPG_MECO_parameters(350000, 360000 , -607612 * 5);

        PEG.rd_height = 360000;
        PEG.rd_norm = PEG.rd_height + getprop("/fdm/jsbsim/metrics/terrain-radius");
        PEG.vd_norm = 23700; #51.6 ° Inc
        PEG.gamma_d = 0.15;
        }

    #TAL after droop (Ascent/Abort Handbook)
    else
        {
        #Radius height at MECO is 330 kft / Shallow FPA 
        #UFPG_MECO_parameters(340000, 350000 , -607612 * 5);

        PEG.rd_height = 350000;
        PEG.rd_norm = PEG.rd_height + getprop("/fdm/jsbsim/metrics/terrain-radius");
        PEG.vd_norm = 23700; #51.6 ° Inc
        PEG.gamma_d = 0.15;
        }


    #Clamped Gamma to have a shallow FPA
    #PEG.gamma_d = math.min(PEG.gamma_d, 0.15);
    
    #Orbital plane targeted is the current one at TAL designation
    if (PEG.SPLANE == "ON") {PEG.SPLANE = "OFF";}

    #Fine count criterion extended to 20
    PEG.fine_count_criterion = 20;

    #Convergence criterion wider
    PEG.vmiss_criterion = 0.1;

    PEG.abort_flag = "TAL";
    SpaceShuttle.callout.make("TAL MECO targets are recomputed", "help");

    print("Rd TAL is : ", PEG.rd_height, " gamma TAL is : ", PEG.gamma_d, " Vd TAL is : ", PEG.vd_norm);
    }

else if (type == "ATO")
    {
    #To define an i-loaded Abort target there via mission file
    PEG.abort_flag = "ATO";
    }

};



##### MECO desired parameters (Vd, Rd, gamma) as a function of Apogee targeted  #####
var UFPG_MECO_parameters = func (Rd_height, Ap, Pe) {

#Pe = 30 Nm for Direct Insertion and 10 Nm (even 0) for direct insertion (Ap below 100 Nm)

#Inputs
var mu = 3.986004418e14 * 35.3147;
var earth_radius = getprop("/fdm/jsbsim/metrics/terrain-radius");
Rd = Rd_height + earth_radius + SpaceShuttle.auto_launch_traj_loft;
Ap = Ap + earth_radius;
Pe = Pe + earth_radius;

#Desired Meco velocity
var temp1 = (2 / Rd) - (2 / (Ap + Pe));
var Vd = math.sqrt(mu * temp1);

#Desired Meco Flight Path Angle
var Sma = (Ap + Pe) / 2;
var z = 2 * Pe - math.pow(Pe, 2) / Sma;
var x = 2 * Rd - math.pow(Rd, 2) / Sma;
temp1 = SpaceShuttle.MIDVAL(-1, math.sqrt(z/x), 1);
var gamma_d = 57.29578 * math.acos(temp1); #deg

#Outputs to UFPG
PEG.rd_height = Rd_height + SpaceShuttle.auto_launch_traj_loft;
PEG.rd_norm = Rd;
if (PEG.abort_flag == "nominal") {PEG.vd_norm = Vd;}
PEG.gamma_d = gamma_d + SpaceShuttle.auto_launch_mps_climbout_bias;

print("Rd height from tool is :", PEG.rd_height);
print("Traj loft is : ", SpaceShuttle.auto_launch_traj_loft);
print("Rd norm from tool is : ", Rd);
print("Vd norm from tool is : ", Vd);
print("Gamma d from tool is : ", gamma_d);

};


##### Final steering function #####
var UFPG_final_steering = func  {



#jsbsim parameters Faster update speed
PEG_jsbsim.r = [PEG_jsbsim.eci_x_node.getValue(), PEG_jsbsim.eci_y_node.getValue(), PEG_jsbsim.eci_z_node.getValue()];
PEG_jsbsim.v = [PEG_jsbsim.eci_xdot_node.getValue(), PEG_jsbsim.eci_ydot_node.getValue(), PEG_jsbsim.eci_zdot_node.getValue()];
var altitude_agl = PEG_jsbsim.altitude_node.getValue();

#Update of MECO target for real current radius (contraints released)
var ap_tgt = getprop("/fdm/jsbsim/systems/ap/launch/apoapsis-target") * 3280.84;

#TAL
if (PEG.abort_flag == "TAL")
    {
    #Nominal TAL
    if (SpaceShuttle.droop_flag == 0)
        {
        #Radius height at MECO is 360 kft / Shallow FPA / AP 55Nm / PE -500Nm
        UFPG_MECO_parameters(altitude_agl, altitude_agl + 10000 , -607612 * 5);
        }

    #TAL after droop (Ascent/Abort Handbook)
    else
        {
        #Radius height at MECO is 330 kft / Shallow FPA 
        UFPG_MECO_parameters(altitude_agl, altitude_agl + 10000 , -607612 * 5);
        }
    }
#ATO/ Nominal
else
    {
    if (PEG.advanced_peg_flag == 0)
        {
        if (PEG.insertion_type == 0) {UFPG_MECO_parameters(altitude_agl, ap_tgt, 182283);}
        else {UFPG_MECO_parameters(altitude_agl, ap_tgt, 60000);}

        print("Final steering standard meco parameters");
        }
    else
        {
        print("Final steering ADVANCED");    
        }
    }



var local_z = SpaceShuttle.normalize(PEG_jsbsim.r); #radial
var temp1 = SpaceShuttle.normalize(PEG_jsbsim.v);
var temp2 = SpaceShuttle.MIDVAL(SpaceShuttle.dot_product(temp1, local_z), -1, 1);
var gamma_euler = 90 - 57.29578 * math.acos(temp2);

var delta_gamma = -(PEG.gamma_d - 0.2) + gamma_euler;
PEG.PEG_theta = math.min(PEG.PEG_theta_prev - 30 * delta_gamma, PEG.PEG_theta_prev);
#PEG.PEG_theta_prev = PEG.PEG_theta;

#Null rates before MECO 
if ((PEG.peg_meco == 1) or (PEG.s_engoff == 1) or (PEG.tgo_total < 2)) {return;}

#print("final theta commanded is : ", PEG.PEG_theta);

if (PEG.final_steering_flag == 0) {PEG.final_steering_flag = 1;}

settimer( func {UFPG_final_steering(); }, 0.1);

};



