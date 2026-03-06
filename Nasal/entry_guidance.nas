#Entry guidance computer for the Space Shuttle
#Thorsten Renk 2016 // GinGin 2023
#Simulation of Original Entry Guidance is based on: Entry Guidance and Entry AP / 80-FM-23 and 24 / KSC-16515


# we ought to have an organized collection of site data, not everything scattered across the dialogs

io.include("landing_sites.nas");

#Main Site
var landing_site = geo.Coord.new();
landing_site.index = 0;
landing_site.rwy_pri = "";
landing_site.rwy_sec = "";
landing_site.tacan = "";
landing_site.rwy_sel = 0;

#Alternate sites logic (OI-33 / Spec 54)
var landing_alt_site_one = geo.Coord.new();
var landing_alt_site_two = geo.Coord.new();
var alt_sites = {
	
	#Shortname only
	one_shortname: "",
	two_shortname: "",

	#Shortname with rwy ID
	one_iata: "", 
	two_iata: "",

	#Index
	one_index: 0,
	two_index: 0,

	#Flag for logic activation
	entry_flag: 0,
};



############## Original Entry Guidance Simulation WIP started 20/06/2021  ##############

# Entry jsbsim and runway frame variables and computations ###########################################################
var entry_jsbsim = {

	#Property nodes are done in TAEM_jsbsim
	ALPHA: 0,
	DRAG: 0,
	DRAG_last: 0,
	DRAG_dot: 0,
	CD: 0,
	CD_last: 0,
	CD_dot: 0,
	HLS: 0, #Height above runway
	LOD: 0,
	QBAR: 0,
	RDOT: 0,
	ROLL: 0, #bank in rad
	VE: 0,
	VI: 0,
	XLFAC: 0, #total accel
	M: 0, #slugs
	XYZE: [0,0,0],
	XYZE_last: [0,0,0],
	XYZED: [0,0,0],
	LAT_geo: 0,
	LON_geo: 0,
	radius_geo: 0,

};

#Entry computations function
var entry_input_parameters_calculations = func {

#JSB sim values that need to be stored 
entry_jsbsim.ALPHA = SpaceShuttle.TAEM_jsbsim.alpha_node.getValue();
entry_jsbsim.DRAG = SpaceShuttle.TAEM_jsbsim.drag_accel_node.getValue();
entry_jsbsim.CD = SpaceShuttle.TAEM_jsbsim.cd_node.getValue();
entry_jsbsim.HLS = SpaceShuttle.TAEM_jsbsim.altitude_qnh_node.getValue() - SpaceShuttle.rwy_coord.RTE1;
entry_jsbsim.LOD = SpaceShuttle.TAEM_jsbsim.lod_node.getValue();
entry_jsbsim.QBAR = SpaceShuttle.TAEM_jsbsim.qbar_node.getValue();
entry_jsbsim.RDOT = -SpaceShuttle.TAEM_jsbsim.hdot_node.getValue();
entry_jsbsim.ROLL = SpaceShuttle.TAEM_jsbsim.phi_node.getValue() * 0.0174533; #rad
entry_jsbsim.VE = SpaceShuttle.TAEM_jsbsim.vtrue_node.getValue();
entry_jsbsim.VI = SpaceShuttle.TAEM_jsbsim.vi_node.getValue();
entry_jsbsim.XLFAC = SpaceShuttle.TAEM_jsbsim.load_factor_node.getValue();
entry_jsbsim.M = SpaceShuttle.TAEM_jsbsim.weight_node.getValue();

#Geodetic to ECEF vector
entry_jsbsim.LAT_geo = SpaceShuttle.TAEM_jsbsim.lat_geo_node.getValue();
entry_jsbsim.LON_geo = SpaceShuttle.TAEM_jsbsim.lon_geo_node.getValue();
entry_jsbsim.radius_geo = SpaceShuttle.TAEM_jsbsim.radius_geo_node.getValue();
entry_jsbsim.XYZE = SpaceShuttle.ECEF_converter(entry_jsbsim.LAT_geo, entry_jsbsim.LON_geo, entry_jsbsim.radius_geo);

#Geodetic to Rwy frame position vector
SpaceShuttle.rwy_coord.XYZ_rwy = SpaceShuttle.matrix_vector_product(SpaceShuttle.rwy_coord.REC, SpaceShuttle.subtract_vector(entry_jsbsim.XYZE, SpaceShuttle.rwy_coord.RLS));

#ECEF Velocity
if (EGRT_data.IFP == 0) #First pass instantaneous V not relevant
	{
	entry_jsbsim.XYZED[0] = 0;
	entry_jsbsim.XYZED[1] = 0;
	entry_jsbsim.XYZED[2] = 0;
	}
else
	{
	entry_jsbsim.XYZED[0] = (entry_jsbsim.XYZE[0] - entry_jsbsim.XYZE_last[0]) / EGD.DTEGD;
	entry_jsbsim.XYZED[1] = (entry_jsbsim.XYZE[1] - entry_jsbsim.XYZE_last[1]) / EGD.DTEGD;
	entry_jsbsim.XYZED[2] = (entry_jsbsim.XYZE[2] - entry_jsbsim.XYZE_last[2]) / EGD.DTEGD;
	}


#Store last XYZE position
entry_jsbsim.XYZE_last[0] = entry_jsbsim.XYZE[0];
entry_jsbsim.XYZE_last[1] = entry_jsbsim.XYZE[1];
entry_jsbsim.XYZE_last[2] = entry_jsbsim.XYZE[2];

#print ("XE  is: ", entry_jsbsim.XYZE[0], " YE  is: ", entry_jsbsim.XYZE[1], " ZE  is: ", entry_jsbsim.XYZE[2]);
#print ("XE dot is: ", entry_jsbsim.XYZED[0], " YE dot is: ", entry_jsbsim.XYZED[1], " ZE dot is: ", entry_jsbsim.XYZED[2]);

#print("Current drag is : ", entry_jsbsim.DRAG, " current LOD is : ", entry_jsbsim.LOD, " current total G is : ", entry_jsbsim.XLFAC);
#print("Current vertical speed is : ", entry_jsbsim.RDOT);
}



# Entry targeting routine - EGRT - Frame conversion / Distance to Runway / Delta Azimuth functions ###########################################################

#Variables
var EGRT_data = {

	#Input parameters
	SRAZ: 0,
	OVHD: 1, #0 for STIN
	RWID: 1, #Rwy ID
	IFP: 0, #Init flag
	VTOGL: 0, #Automatic toggle between STIN/OVHD

	#Input constants
	RX22: 0.9933065782, #Earth radius ratio
	RECX: 20925196, #Earth equatorial radius a
	XHACL: -35245,
	XHACH: -35245,
	WTGS1: 8000, #slugs
	RI: 20000,
	PSHARS: 270,
	RFO: 14000,
	R1: 0,
	R2: 0.093,
	A3TOL: -0.003,
	
	#Variables
	VE_test: 0,
	RC: [0,0,0], 
	HACEF: [0,0,0],
	RCCEF: [0,0,0],
	
	#Outputs
	VREL_fps: 0,
	TRANGE_last: 0,
	TRANGE: 0,
	DELAZ: 0,
	RCHMAG: 0, #radius landing site
	PSHAT: 0, #HTA
	RTURNT: 0, #HAC radius
	YSGNT: 0, #R/L HAC
	RWID0: 0, #Current runway targeted
	
	reset_EGRT_data: func {

		me.OVHD = 1;
		me.RWID = 1;
		me.IFP = 0;
		me.VTOGL = 0;
		me.TRANGE = 0;
		me.DELAZ = 0;
		me.RCHMAG = 0;
		me.PSHAT = 0;
		me.RTURNT = 0;
		me.YSGNT = 0;
	},

};



#EGRT routine main loop
var EGRT_EXEC = func {

#Either Pri or Sec runway targeted
if (EGRT_data.IFP == 0) {EGRT_data.RWID0 = EGRT_data.RWID;}

#Redesignation and mid point computations done here (mid range between 2 runways)
#Not done for now

#VE from norm function (not used)
EGRT_data.VE_test = SpaceShuttle.norm(entry_jsbsim.XYZED);

#EGRT sequence function
EGRT_SEQ();

#First pass flag
if (EGRT_data.IFP == 0) {EGRT_data.IFP = 1;}

#Runway redesignation and EGRT flag reset
if (EGRT_data.RWID0 != EGRT_data.RWID) {EGRT_data.IFP == 0;}

#print ("VE from norm is : ", EGRT_data.VE_test, " IFP is : ", EGRT_data.IFP);
#print("RWID0 is : ", EGRT_data.RWID0, " RWID current is : ", EGRT_data.RWID);

};



#EGRT sequence function
var EGRT_SEQ = func {



#Shuttle position in runway frame
#var RG = SpaceShuttle.rwy_coord.XYZ_rwy;
var SIGNY = math.sgn(SpaceShuttle.rwy_coord.XYZ_rwy[1]);

#Automatic toggle between OVHD and STIN to be done there


#Variables that are Runway ID dependant (one time computations)
if (EGRT_data.IFP == 0)
	{
	#Initial Rturn is equal to Ri
	EGRT_data.RTURNT = EGRT_data.RI;

	#HAC turn (L/R) done in TAEM site computations under TAEM_guidance_GTP.YSGN
	EGRT_data.YSGNT = TAEM_guidance_GTP.YSGN;

	#EGRT-CHACRC - Center heading alinement cone in runway frame 
	EGRT_data.RC[0] = SpaceShuttle.TAEM_guidance_XHAC.XHAC;
	EGRT_data.RC[1] = TAEM_guidance_GTP.YSGN * EGRT_data.RFO;
	EGRT_data.RC[2] = 0;

	#EGRT-CHACEFC - Center heading alinement circle - ECEF
	EGRT_data.HACEF = SpaceShuttle.matrix_vector_product(SpaceShuttle.rwy_coord.REC_inv, EGRT_data.RC);

	#RCCEF and norm
	EGRT_data.RCCEF = SpaceShuttle.add_vector(SpaceShuttle.rwy_coord.RLS, EGRT_data.HACEF);
	EGRT_data.RCHMAG = SpaceShuttle.norm(EGRT_data.RCCEF);
	}



#EGRT-BV - Bearing of vehicle
var VNORM = SpaceShuttle.cross_product(entry_jsbsim.XYZE, EGRT_data.RCCEF);
var RVEHMG = SpaceShuttle.norm(entry_jsbsim.XYZE);

var T3 = VNORM[2] * RVEHMG;
var T4 = VNORM[0] * entry_jsbsim.XYZE[1] - VNORM[1] * entry_jsbsim.XYZE[0];
var BARVEH = math.atan2(T3, T4);
var SINB = EGRT_data.RI / EGRT_data.RCHMAG; #RF <= Rturn <= RI


#EGRT-BVCHAC - Bearing to center of alinement circle
var T5 = VNORM[2] * EGRT_data.RCHMAG;
var T6 = EGRT_data.RCCEF[1] * VNORM[0] - VNORM[1] * EGRT_data.RCCEF[0];
var BARCC = math.atan2(T5, T6);
if (BARCC < 0) {BARCC = BARCC + 2 * math.pi;}


#EGRT-COSTHETA - Great circle arc
var T7 = SpaceShuttle.dot_product(entry_jsbsim.XYZE, EGRT_data.RCCEF);
var CTHVC = T7 / (RVEHMG * EGRT_data.RCHMAG);


#EGRT-DWP - Distance to WP1
var STHVC = math.sqrt(1 - math.pow(CTHVC, 2));
var CTVWP1 = CTHVC + 1.5 * CTHVC * math.pow(SINB, 2); #MCRT02 = 0.5 ?
var SBARCR = SINB / STHVC;

var TEMP = CTVWP1 * SBARCR;
TEMP = math.min(1, math.max(TEMP, -1));
var A2 = math.acos(TEMP);

TEMP = CTVWP1;
TEMP = math.min(1, math.max(TEMP, -1));
var DVEWP1 = math.acos(TEMP) * EGRT_data.RCHMAG;


#EGRT-DVNEP - Range to treshold point
TEMP = SBARCR;
TEMP = math.min(1, math.max(TEMP, -1));
var T8 = math.asin(TEMP);
var BARWP1 = BARVEH - EGRT_data.YSGNT * T8;

var A3 = 0.5 * math.pi - A2 + EGRT_data.YSGNT * (SpaceShuttle.rwy_coord.RAZ - BARCC);
if (math.abs(A3) > math.pi) {A3 = A3 - 2 * math.pi * math.sgn(A3);}
if ((EGRT_data.PSHAT > (EGRT_data.PSHARS + 1)) or (A3 < EGRT_data.A3TOL) or (EGRT_data.YSGNT != SIGNY)) {A3 = A3 + 2 * math.pi;}

EGRT_data.PSHAT = A3 * 57.29578;
EGRT_data.RTURNT = EGRT_data.RFO + EGRT_data.R1 * EGRT_data.PSHAT + EGRT_data.R2 * math.pow(EGRT_data.PSHAT, 2);
var DARC = (EGRT_data.RFO * EGRT_data.PSHAT + 0.5 * EGRT_data.R1 * math.pow(EGRT_data.PSHAT, 2) + 0.3333333 * EGRT_data.R2 * math.pow(EGRT_data.PSHAT, 3)) * 0.0174533;

EGRT_data.TRANGE = 1.645788e-4 * (DVEWP1 + DARC - EGRT_data.RC[0]); #Nm
setprop("/fdm/jsbsim/systems/entry_guidance/remaining-distance-nm", EGRT_data.TRANGE);

#print("Entry Range to treshold in Nm is : ", EGRT_data.TRANGE);


#EGRT-DELAZ - Azimuth error
var H = SpaceShuttle.cross_product(entry_jsbsim.XYZE, entry_jsbsim.XYZED);
var PSI = math.atan2(RVEHMG * H[2], entry_jsbsim.XYZE[1] * H[0] - entry_jsbsim.XYZE[0] * H[1]);

EGRT_data.DELAZ = PSI - BARWP1;
if (math.abs(EGRT_data.DELAZ) > math.pi) {EGRT_data.DELAZ = EGRT_data.DELAZ - 2 * math.pi * math.sgn(EGRT_data.DELAZ);}

setprop("/fdm/jsbsim/systems/entry_guidance/delta-azimuth-deg", EGRT_data.DELAZ * 57.29578);
setprop("/fdm/jsbsim/systems/entry_guidance/target-azimuth-deg", geo.aircraft_position().course_to(landing_site)); #To be done with global variable PSI (Track)


#print("Bearing WP 1 is :", BARVEH * 57.29578, " PSI is : ", PSI * 57.2957, " Entry Delaz is : ", EGRT_data.DELAZ * 57.29578);


#VREL and target azimuth computation for RTLS loop (done in TAEM.nas / December 2024)
EGRT_data.VREL_fps = ((EGRT_data.TRANGE - EGRT_data.TRANGE_last) / EGD.DTEGD) / EGD.CNMFS;

setprop("/fdm/jsbsim/systems/entry_guidance/vrel-fps", EGRT_data.VREL_fps);

#Vrel sign
var vrel_sign = getprop("/fdm/jsbsim/systems/entry_guidance/vrel-sign");

if ((EGRT_data.VREL_fps > 0.0) and (vrel_sign != 1))
	{
	setprop("/fdm/jsbsim/systems/entry_guidance/vrel-sign", 1);
	}
else if ((EGRT_data.VREL_fps < 0.0) and (vrel_sign != -1))
	{
	setprop("/fdm/jsbsim/systems/entry_guidance/vrel-sign", -1);
	}

EGRT_data.TRANGE_last = EGRT_data.TRANGE;
#print("Vrel fps is : ", EGRT_data.VREL_fps);


};





# Entry Guidance variables  ###########################################################


#Variables hash (only one hash with constants, Inputs, Outputs for better readability) / Entry Guidance Data (EGD)
var EGD = {

	#Input parameters (Most of them are done in entry_jsbsim) // page 16

	EGFLG: 0, #Guidance mode flag
	START: 0, #Init flag
	START_LOOP: 0, #Init flag for Entry loop
	RTLS_START_LOOP: 0,#Init flag for RTLS loop
	MM304_PHI: 0, #Pre entry values from i-load
	MM304_ALPHA: 0,
	
	#Misc Flag

	RR_flag: 0, #Roll reversal flag
	SB_flag: 0,
	TAL_pullout_flag: 0, #TAL first pullout
	
	#Input constants (First value of arrays is always 0 to have correct index vs NASA formalism where arrays start at placeholder[1]) // page 17

	ACLAM_C: [0, 15, 0.0025], #_C for constant to avoid mix up between containers
	ACLIM_C: [0, 37, 0, 7.66666667, 0.00223333],
	ACN1: 50,
	AK: -3.4573,
	AK1: -4.76,
	ALFM: 33,
	ALIM: 70.84,
	ALMN: [0, 0.7986355, 0.9659258, 0.93969, 1.0],
	ASTART: 5.66,
	#CALPO: [0, 19.45, -4.074, -4.2778, 16.398, 4.476, -9.9339, 40, 40, 40, 40], #Alpha commanded constant terms // OTT 38/28° profile (never flown)
	CALPO: [0, 5.424505, 5.424505, -4.2778, 16.398, 4.476, -9.9339, 40, 40, 40, 40], #STS-1 40° profile
	#CALP1: [0, -0.776388e-2, 8.74771e-3, 0.8875002e-2, -0.3143109e-3, 3.1875e-3, 6.887436e-3, 0, 0, 0, 0], #AC rate terms // OTT 38/28°
	CALP1: [0, 0.003430198, 0.003430198, 0.8875002e-2, -0.3143109e-3, 3.1875e-3, 6.887436e-3, 0, 0, 0, 0],
	#CALP2: [0, 0.2152776e-5, -7.44e-7, -0.7638891e-6, 0.2571456e-6, 0, -2.374978e-7, 0, 0, 0, 0], #AC quadratic terms	// OTT 38/28°
	CALP2: [0, 0, 0, -0.7638891e-6, 0.2571456e-6, 0, -2.374978e-7, 0, 0, 0, 0],
	#CDDOT: [0, 1500, 2000, 0.15, 0.0783, -8.165e-3, 6.833e-4, 7.5e-5, 13.666e-4, -8.165e-3], #OTT
	CDDOT: [0, 1500, 2000, 0.18, 0.0783, -8.165e-3, 6.833e-4, 9.0e-5, 13.666e-4, -8.165e-3], #STS1
	CNMFS: 1.645788e-4,
	CRDEAF: 4.0,
	CT16: [0, 0.1354, -0.10, 0.006],
	CT17: [0, 1.537e-2, -5.8146e-1],
	CT16MN: 0.025,
	CT16MX: 0.35,
	CT17MN: 0.0025,
	CT17MX: 0.014,
	CT17M2: 0.00133, #CT17 min with alpha modulation
	CY0: -0.1309,
	CY1: 1.0908e-4,
	C17MP: 0.75,
	C21: 0.06,
	C22: -0.001,
	C23: 4.25e-6,
	C24: 0.01,
	C25: 0.01,
	C26: 0,
	C27: 0,
	DDLIM: 3.0,
	DDMIN: 0.15, #Alpha mod termination 0.15
	DELV: 2300, #2300
	DF: 19.0, #20.81
	DLALLM: 43,
	DLAPLM: 2.0,
	D23C: 19.38,
	D230: 19.38,
	DRDDL: -1.5,
	DTEGD: 1.92, #Update cycle rate
	DT2MIN: 0.008,
	EEF4: 2.0e-6,
	ETRAN: 5.998473e7, #STS1
	E1: 0.01,
	GS_ft: 32.174, #G != GS[index]
	GS: [0, 0.02, 0.02, 0.03767, 0.03], #Smoothing roll command OTT
	#GS: [0, 0, 0.0001, 0, 0], #STS1
	HSMIN: 20500,
	HS0: [0, 18075, 27000, 45583.5],
	HS1: [0, 0.725, 0, -0.9445],
	LODMIN: 0.5,
	NALP: 9,
	MM304_PHI0: 0, #Pre entry commanded presets (phi and alpha)
	MM304_ALPHA0: 40,
	RDMAX: 12.0,
	#RLMC: [0, 70, 70, 0, 70, 0, 70], 
	RLMC: [0, 70, 70, 0, -370, 0, 30], #STS1
	RPT1: 8, #STS 1 29.44 #OTT 22.4 ########### Transition Bias ##########
	#VA_initial: 27637, 
	VA_initial: 23163, #STS1 
	VALMOD: 23000, #Alpha modulation start flag
	VALP: [0, 2850, 3563.87, 4500, 6809, 7789, 14500, 14500, 14500, 14500], #STS1 40°
	VA: [0, 21000, 27637], #STS1
	#VA: [0, 22000, 27197], #OTT
	VB1: 19000,
	VC16: 23000,
	VC20: 2500,
	#VELMN: 8000,
	VELMN: 9500, #STS1
	VEROLC: 8000,
	VHS: [0, 12310, 19675.5],
	VNOALP: 25000, #STS1 // Alpha modulation
	VQ: 10499, #Velocity at the end of constant drag phase (5000 intitially)
	VRLMC: 2500,
	VSAT: 25766.2,
	VS1: 23283.5,
	VRDT: 23000,
	V_TAEM: 2500,
	VTRAN: 10500,
	VYLMAX: 23000,
	YLMIN: 0.03,
	YLMIN2: 0.07,
	Y: [0, 0.1832596, 0.1745329, 0.3054326], #RR heading deadband in rad
	ZK1: 0.6, #STS1



	#Internal Parameters (storage / used across several functions) // Page 26

	A: [0, 0, 0], #Temp variable for range computation
	ACLAM: 0,
	ACLIM: 0,
	ACMD1: 0,
	ALDCO: 0,
	ALDREF: 0,
	#ALPCMD: 0,
	ALPDOT: 0,
	ARG: [0, 0, 0, 0],
	#A2: 0, #Done in an array above
	CAG: 0,
	CQ1: [0, 0, 0],
	CQ2: [0, 0, 0],
	CQ3: [0, 0, 0],
	C1: 0,
	C16: 0,
	C17: 0,
	C2: 0,
	C4: 0,
	C20: 0, 
	DD: 0,
	DDS: 0,
	DDP: 0,
	DELALF: 0,
	DELALP: 0,
	DLRDOT: 0,
	DLIM: 0,
	DLZRL: 0,
	DRDD: 0,
	DREF: [0, 0, 0],
	DREFP: 0,
	DREFPT: 0,
	#DREFP: [0, 0, 0, 0, 0, 0],
	DRF: 0,
	DX: [0, 0, 0],
	DZOLD: 0,
	DZSGN: 0,
	D231: 0,
	EEF: 0,
	HDTRF: [0, 0, 0],
	HS: 0,
	IALP: 0,
	ICT: 0,
	ISLECP: 0,
	#ISLECT: 0,
	ITRAN: "", #string
	LMFLG: 0,
	LMN: 0,
	LODV: 0,
	LODX: 0,
	Q: [0, 0, 0, 0],
	RCG: 0,
	RCG1: 0,
	RDEALF: 0,
	RDTREF: 0,
	REQ1: 0,
	RER1: 0,
	RF: [0, 0, 0],
	RFF1: 0,
	RDTRF: 0,
	ROLLC: [0, 0, 0, 0],
	RPT: 0,
	R231: 0,
	START: 0,
	T1: 0,
	T2: 0,
	T2DOT: 0,
	T2OLD: 0,
	RK2ROL: 1,
	V: [0, 0, 0, 0],
	VB2: 0,
	#VCG: 0,
	VE2: 0,
	VF: [0, 0, 0],
	VSAT2: 0,
	VTRB: 0,
	VO: [0, 0, 0],
	VF: [0, 0, 0],
	VX: [0, 0, 0],
	XLOD: 0,
	YL: 0,
	ZK: 0,


	#Outputs

	#AP
	ALPCMD: 0, 
	ROLLCMD: 0,

	#Display
	DREFP: 0, 
	DRAG: 0, #entry_jsbsim.DRAG
	ROLREF: 0, #ROLLC[3]
	ISLECT: 1,
	VCG: 0, #Velocity at Constant Drag phase init
	VRR: 0,
	EOWD: 0,
	EEI: 0,
	RC176G: 0,
	RDOTREF_display: 0,

	reset: func {

		me.ECFLG = 0;
		me.START = 0;

	},


};



# Entry Guidance main execution loop - EGEXEC entry guidance executive (Appendix A1) ###########################################################


var EGEXEC = func {

#Computations for ECEF and rwy frame coordinates
entry_input_parameters_calculations();

#EGRT routine main loop
EGRT_EXEC();

#EGSCALHT 
EGSCALHT();

#EGINIT at first pass or rwy redesignation
if (EGD.START == 0) {EGINIT();}

#EGCOMN
EGCOMN();


#Make transition tests I
if ((EGD.ISLECT == 1) and ((entry_jsbsim.XLFAC >= EGD.ASTART) and (entry_jsbsim.DRAG >= 3)))
	{
	EGD.ISLECT = 2;
	EGD.DTEGD = 1.92;

	if (entry_jsbsim.VE < EGD.VTRAN) {EGD.ISLECT = 5;}
	}

if ((EGD.ISLECT == 2) and (entry_jsbsim.VE < EGD.VB1)) {EGD.ISLECT = 3;}

if(((EGD.ISLECT == 2) or (EGD.ISLECT == 3)) and (EGD.T2 > EGD.ALFM)) {EGD.ISLECT = 4;}

if ((EGD.EGFLG > 0) and (EGD.ISLECT > 2)) {EGD.ISLECT = 2;} #Canned MCC mode


#Compute vertical L/D during preentry phase
if (EGD.ISLECT == 1) {EGPEP();} 

#Compute Ref parameters during Temp control and Equilibrium glide phases
else if ((EGD.ISLECT == 2) or (EGD.ISLECT == 3))
	{
	EGRP();
	EGREF();
	}

#Compute reference parameters during constant drag phase
else if (EGD.ISLECT == 4) {EGREF4();}

#Compute reference parameters during transition phase
else if (EGD.ISLECT == 5) {EGTRAN();}

#EGALPCMD
EGALPCMD();

#Compute vertical L/D command following preentry phase
if (EGD.ISLECT > 1)
	{
	EGNNSLCT();
	EGLODVCMD();
	}

#Compute roll command
EGROLCMD();

#Output to JSBsim
DAP3D();

#Make transition tests II
if (EGD.ISLECT == 2)
	{
	if ((entry_jsbsim.VE < EGD.VA_initial) and (EGD.DREFP < EGD.DREFP3)) {EGD.ISLECT = 3;}
	if ((entry_jsbsim.VE < (EGD.VCG + EGD.DELV)) and (EGD.DREFP > EGD.DREFP4)) {EGD.ISLECT = 4;}
	}

else if (EGD.ISLECT == 3)
	{
	if ((entry_jsbsim.VE < (EGD.VCG + EGD.DELV)) and (EGD.DREFP > EGD.DREFP4)) {EGD.ISLECT = 4;}
	if ((entry_jsbsim.VE < (EGD.VTRAN + EGD.DELV)) and (EGD.DREFP > EGD.DREFP5) and (EGD.VCG < EGD.VTRAN)) {EGD.ISLECT = 5;}
	}

else if ((EGD.ISLECT == 4) and (entry_jsbsim.VE < (EGD.VTRAN + EGD.DELV)) and (EGD.DREFP > EGD.DREFP5)) {EGD.ISLECT = 5;}



### Functions from previous Entry Code (trailers, etc) ###

trailer_set.update(SpaceShuttle.EGRT_data.TRANGE);
trailer_set.updatebox(SpaceShuttle.EGRT_data.TRANGE - SpaceShuttle.EGD.DRDD * (SpaceShuttle.entry_jsbsim.DRAG - SpaceShuttle.EGD.DREFP)); #Guidance box
body_flap_management();

#Trailer update speed goes to 15 seconds below 14000 ft/s (Entry traj 3) 
if ((entry_jsbsim.VE < 14000) and (trailer_set.time_limit == 29)) {trailer_set.time_limit = 15;}
		
#SB 81% at Mach 10 for Cm considerations 
if ((entry_jsbsim.VE < 10000) and (EGD.SB_flag == 0)) {Entry_Speedbrake_control();}
	

### EGEXEC loop flag and exit conditions ###

#Exit the loop at TAEM init
if (SpaceShuttle.TAEM_guidance_TGINIT.LOOP == 1) {return;}

#Flag to avoid several EGEXEC loops when runway is changed
if (EGD.START_LOOP == 0) {EGD.START_LOOP = 1;}

#print("ISLECT is: ", EGD.ISLECT, " Start flag is : ", EGD.START, " START LOOP flag is : ", EGD.START_LOOP);

#Entry loop timer (1.92s)
settimer( func {EGEXEC(); }, EGD.DTEGD)

};


#RTLS function for parameters needed by RTLS code (Older function / GRTLS loop in TAEM.nas since December 2024)
var RTLSEXEC = func {

#Computations for ECEF and rwy frame coordinates
entry_input_parameters_calculations();

#EGRT routine main loop
EGRT_EXEC();

#Exit the loop at TAEM init
if (SpaceShuttle.TAEM_guidance_TGINIT.LOOP == 1) {return;}

#Flag to avoid several EGEXEC loops when runway is changed
if (EGD.RTLS_START_LOOP == 0) {EGD.RTLS_START_LOOP = 1;}

settimer( func {RTLSEXEC();}, EGD.DTEGD)
}




# Entry Guidance subfunctions called by EGEXEC - Appendix A2 to A13 ###########################################################


#EGSCALHT - Scale height / Compute altitude scale height HS
var EGSCALHT = func {

if (entry_jsbsim.VE < EGD.VHS[1]) {EGD.HS = EGD.HS0[1] + EGD.HS1[1] * entry_jsbsim.VE;}
else
	{
	if (entry_jsbsim.VE < EGD.VHS[2]) {EGD.HS = EGD.HS0[2];}
	else {EGD.HS = EGD.HS0[3] + EGD.HS1[3] * entry_jsbsim.VE;}
	}

if (EGD.HS < EGD.HSMIN) {EGD.HS = EGD.HSMIN;}

#print("Scale height HS is: ", EGD.HS);

};






#EGINIT - Initialization
var EGINIT = func {

#Initialize variables and flags
EGD.CZOLD = 0;
EGD.IVRR = 0;
EGD.ITRAN = "OFF"; 
EGD.ISLECT = 1;
EGD.ICT = 0;
EGD.IDBCHG = 0;
EGD.T2 = 0;
EGD.DREFP = 0;
EGD.VQ2 = math.pow(EGD.VQ, 2);
EGD.RK2ROL = -math.sgn(EGRT_data.DELAZ);
EGD.DLRDOT = 0;
EGD.LMFLG = 0;
EGD.VTRB = 60000;
EGD.DDP = 0;
EGD.Y[1] = 0.1832596; #First roll reversal at 10° of Daz
EGD.RK2RLP = EGD.RK2ROL;
EGD.RR_flag = 0;
EGD.SB_flag = 0;

#Pre entry I-loaded values (No Pre Bank)
EGD.MM304_PHI = EGD.MM304_PHI0; 
EGD.MM304_ALPHA = EGD.MM304_ALPHA0;

#Compute desired transition range (RPT)
var TEMP1 = -((EGD.ETRAN - EGD.EEF4) * math.ln(EGD.DF / EGD.ALFM)) / (EGD.ALFM - EGD.DF);
var TEMP2 = (math.pow(EGD.VTRAN, 2) - EGD.VQ2) / (2 * EGD.ALFM);

EGD.RPT = (TEMP1 + TEMP2) * EGD.CNMFS + EGD.RPT1;
#print("Transition range RPT is: ", EGD.RPT);

EGD.VSAT2 = math.pow(EGD.VSAT, 2);
EGD.VSIT2 = math.pow(EGD.VS1, 2);
EGD.VCG = EGD.VQ;
EGD.D23 = EGD.D230;

EGD.DX[1] = 1;
EGD.VO[1] = EGD.VB1;
EGD.VX[1] = EGD.VA_initial;
EGD.VF[1] = EGD.VA[1];
EGD.A[1] = EGD.AK;
EGD.VO[2] = EGD.VA[1];
EGD.VX[2] = EGD.VA[2];
EGD.A[2] = EGD.AK1;
EGD.VB2 = math.pow(EGD.VB1, 2);

#Compute component of constant drag phase range (RCG)
EGD.RCG1 = EGD.CNMFS * (EGD.VSIT2 - EGD.VQ2) / (2 * EGD.ALFM);
#print("Constant drag range RCG is: ", EGD.RCG1);

#First pass flag
EGD.IALP = EGD.NALP;
EGD.START = 1;

};





#EGCOMN - Common computations
var EGCOMN = func {

#Compute common variables
EGD.XLOD = math.max(entry_jsbsim.LOD, EGD.LODMIN);
EGD.T1 = EGD.GS_ft * (1 - math.pow(entry_jsbsim.VI, 2) / EGD.VSAT2); #Equilibrium lift 
EGD.T2OLD = EGD.T2; 
EGD.VE2 = math.pow(entry_jsbsim.VE, 2);
EGD.EEF = EGD.GS_ft * entry_jsbsim.HLS + EGD.VE2 / 2;
EGD.EOWD = EGD.EEF / EGD.GS_ft;
EGD.CAG = 2 * EGD.GS_ft * EGD.HS + EGD.VE2;

if (EGD.ISLECT < 5)
	{
	EGD.T2 = EGD.CNMFS * (EGD.VE2 - EGD.VQ2) / (2 * (EGRT_data.TRANGE - EGD.RPT));
	EGD.T2DOT = (EGD.T2 - EGD.T2OLD) / EGD.DTEGD;

	#print("Eq glide Nz T1 is: ", EGD.T1);
	#print("Constant drag level to reach target T2 is : ", EGD.T2);

	#Compute altitude rate reference for transition
	if (entry_jsbsim.VE < (EGD.VTRAN + EGD.DELV))
		{
		EGD.C1 = (EGD.T2 - EGD.DF) / (EGD.ETRAN - EGD.EEF4);
		EGD.RDTRFT = -(EGD.C1 * (EGD.GS_ft * entry_jsbsim.HLS - EGD.EEF4) + EGD.DF) * 2 * entry_jsbsim.VE * EGD.HS / EGD.CAG;
		EGD.DREFP5 = EGD.DF + (EGD.EEF - EGD.EEF4) * EGD.C1 + EGD.GS[4] * (EGD.RDTREF - EGD.RDTRFT);

		#print("Altitude rate ref RDTRFT is : ", EGD.RDTRFT);
		}
	}


### Display parameters ###

#Display Hdot ref parameters
entry_jsbsim.DRAG_dot = (entry_jsbsim.DRAG - entry_jsbsim.DRAG_last) / EGD.DTEGD;
entry_jsbsim.CD_dot = (entry_jsbsim.CD - entry_jsbsim.CD_last) / EGD.DTEGD;

#Stored parameters
entry_jsbsim.DRAG_last = entry_jsbsim.DRAG;
entry_jsbsim.CD_last = entry_jsbsim.CD;

#Hdot ref from Entry Workbook for RDTREF benchmarking
EGD.RDOTREF_display = -EGD.HS * (2 * entry_jsbsim.DRAG / entry_jsbsim.VE + entry_jsbsim.DRAG_dot / entry_jsbsim.DRAG - entry_jsbsim.CD_dot / entry_jsbsim.CD);

#print("Hdot ref entry worbook is : ", EGD.RDOTREF_display, " Drag dot is : ", entry_jsbsim.DRAG_dot, " CD dot is : ", entry_jsbsim.CD_dot);

};




#EGPEP - Preentry phase
var EGPEP = func {

#Compute vertical L/D during preentry phase
EGD.LODX = EGD.XLOD * math.cos(EGD.MM304_PHI * 0.0174533);
EGD.LODV = EGD.LODX;
EGD.ALDREF = EGD.LODX;


#print("L/D commanded is : ", EGD.LODX);
};


#EGRP - Range Prediction
var EGRP = func {

var K = 0;

#Compute ref parameters for temperature control and equilibrium glide phases (ISLECT 2 or 3)
if (entry_jsbsim.VE > 27637) #VA[1] for first quadratic but too much drag // Second quadratic is better all the way to equilibrium phase
	{
	K = 2;
	EGD.N = 2;
	}
else
	{
	K = 1;
	EGD.N = 1;
	EGD.RF[2] = 0;
	}

EGD.VF[EGD.N] = entry_jsbsim.VE;

if (EGD.START == 1)
	{
	K = 1;
	EGD.START = 2;

	for (var i = 1; i < 3; i = i + 1)
		{
		if (i == 2) {EGD.DX[2] = EGD.CQ1[1] + EGD.VA[1] * (EGD.CQ2[1] + EGD.CQ3[1] * EGD.VA[1]);}

		EGD.CQ3[i] = -EGD.A[i] * EGD.DX[i] / (2 * EGD.VO[i] * (EGD.VX[i] - EGD.VO[i]));
		EGD.CQ2[i] = -2 * EGD.VX[i] * EGD.CQ3[i];
		EGD.CQ1[i] = EGD.DX[i] - EGD.VO[i] * (EGD.CQ2[1] + EGD.CQ3[1] * EGD.VO[1]);

		#print("DX2 is : ", EGD.DX[2]);
		}

	}

for (var i = K; i < (EGD.N + 1); i = i + 1)
	{
	if (entry_jsbsim.VE < EGD.VO[i]) {EGD.RF[i] = 0;}
	else
		{
		EGD.V[1] = (8 * EGD.VO[i] + EGD.VF[i]) / 9;
		EGD.V[2] = (EGD.VO[i] + EGD.VF[i]) / 2;
		EGD.V[3] = EGD.VO[i] + EGD.VF[i] - EGD.V[1];

		for (var j = 1; j < 4; j = j + 1) {EGD.Q[j] = (EGD.CQ1[i] / EGD.V[j]) + EGD.CQ2[i] + EGD.CQ3[i] * EGD.V[j];}

		EGD.RF[i] = (27 / EGD.Q[1] + 44 / EGD.Q[2] + 27 / EGD.Q[3]) * (EGD.VF[i] - EGD.VO[i]) / 98;

		#print("Q1 is : ", EGD.Q[1], " Q2 is : ", EGD.Q[2], " Q3 is : ", EGD.Q[3]);
		}
	}

EGD.RFF1 = EGD.CNMFS * (EGD.RF[1] + EGD.RF[2]);

#print("Temp control range x D23 (RFF1) is: ", EGD.RFF1);


#Update reference drag level D23 at VB1
if ((EGD.T2DOT > EGD.DT2MIN) or (entry_jsbsim.VE > (EGD.VCG + EGD.DELV)))
	{
	if (entry_jsbsim.VE < EGD.VB1) {EGD.VB2 = EGD.VE2;}

	EGD.VCG = EGD.VQ;
	EGD.D23L = EGD.ALFM * (EGD.VSIT2 - EGD.VB2) / (EGD.VSIT2 - EGD.VQ2);

	if (EGD.D23 > EGD.D23L) {EGD.VCG = math.sqrt(EGD.VSIT2 - EGD.D23L * (EGD.VSIT2 - EGD.VQ2) / EGD.D23);}
	else {EGD.D23 = EGD.D23L;}

	EGD.A[2] =  EGD.CNMFS * (EGD.VSIT2 - EGD.VB2) / 2;
	EGD.REQ1 = EGD.A[2] * math.ln(EGD.ALFM / EGD.D23);
	EGD.RCG = EGD.RCG1 - EGD.A[2] / EGD.D23;
	EGD.R231 = EGD.RFF1 + EGD.REQ1;
	EGD.R23 = EGRT_data.TRANGE - EGD.RCG - EGD.RPT;
	EGD.D231 = EGD.R231 / EGD.R23;
	EGD.DRDD = -EGD.R23 / EGD.D231;

	if (EGD.D231 >= EGD.D23L)
		{
		EGD.D23 = EGD.D231 + EGD.A[2] * math.pow(1 - EGD.D23 / EGD.D231, 2) / (2 * EGD.R23);
		}
	else {EGD.D23 = math.max(EGD.D231, EGD.E1);}

	#MCC canned mode 2
	if (EGD.EGFLG > 1) {EGD.D23 = EGD.D23C;}

	#print(" Velocity at Constant drag is : ", EGD.VCG, " D23 value is : ", EGD.D23, " Range to D23 is R23 : ", EGD.R23, " DRDD is : ", EGD.DRDD);
	}

};




#EGREF - Reference parameters computations for temperature control and equilibrium phases
var EGREF = func {

#Temperature control phase
if (entry_jsbsim.VE > EGD.VB1)
	{
	for (var i = 1; i < (EGD.N + 1); i = i + 1)
		{
		EGD.DREF[i] = EGD.CQ1[i] + entry_jsbsim.VE * (EGD.CQ2[i] + EGD.CQ3[i] * entry_jsbsim.VE);
		EGD.HDTRF[i] = -EGD.HS * (2 * (EGD.DREF[i] / entry_jsbsim.VE) - EGD.CQ2[i] - 2 * EGD.CQ3[i] * entry_jsbsim.VE);
		}

	if (entry_jsbsim.VE > EGD.VA[1])
		{
		EGD.DRF = EGD.DREF[2] - EGD.DREF[1];
		EGD.DRF = EGD.DRF * (EGD.DRF + (EGD.HDTRF[1] - EGD.HDTRF[2]) * EGD.GS[1]);

		if (EGD.DRF < 0) {EGD.N = 1;}

		#print("DRF is : ", EGD.DRF);
		}

	EGD.DREFP = EGD.D23 * EGD.DREF[EGD.N];
	EGD.RDTREF = EGD.D23 * EGD.HDTRF[EGD.N];
	EGD.C2 = EGD.CQ2[EGD.N] * EGD.D23;

	#print("N is : ", EGD.N, " DREF 1 is : ", EGD.DREF[1], " DREF 2 is : ", EGD.DREF[2]);
	#print("H dot ref 1 is : ", EGD.HDTRF[1], " H dot ref 2 is ; ", EGD.HDTRF[2]);
	
	}


#Equilibrium glide phase
if (entry_jsbsim.VE < EGD.VA_initial) 
	{
	EGD.ALDCO = (1 - EGD.VB2 / EGD.VSIT2) / EGD.D23;
	EGD.DREFP1 =  (1 - EGD.VE2 / EGD.VSIT2) / EGD.ALDCO;
	EGD.RDTRF1 = -2 * EGD.HS / (entry_jsbsim.VE * EGD.ALDCO);
	EGD.DREFP3 = EGD.DREFP1 + EGD.GS[2] * (EGD.RDTREF - EGD.RDTRF1);

	if ((EGD.DREFP3 > EGD.DREFP) or (entry_jsbsim.VE < EGD.VB1))
		{
		EGD.DREFP = EGD.DREFP1;
		EGD.RDTREF = EGD.RDTRF1;
		EGD.C2 = 0;
		}

	#print("DREFP3 is : ", EGD.DREFP3);
	}


#Compute test value for DREFP for transition to constant drag phase (ISLECT 4)
EGD.DREFP4 = EGD.GS[3] * (EGD.RDTREF + 2 * EGD.HS * EGD.T2 / entry_jsbsim.VE) + EGD.T2;
EGD.ITRAN = "ON";

#print("Drag ref is : ", EGD.DREFP, " Hdot ref is : ", EGD.RDTREF);
#print("DREFP4 is : ", EGD.DREFP4);

};




#EGREF4 - Constant drag phase
var EGREF4 = func {

#Compute reference parameters during constant drag phase
EGD.DREFP = EGD.T2; #Nominally 33ft/s²
EGD.RDTREF = -2 * EGD.HS * EGD.T2 / entry_jsbsim.VE;
EGD.DRDD = -(EGRT_data.TRANGE - EGD.RPT) / EGD.T2;
EGD.C2 = 0;

EGD.ITRAN = "ON";

#print("Drag ref is : ", EGD.DREFP, " H dot ref is : ", EGD.RDTREF, " DRDD is : ", EGD.DRDD, " ITRAN is : ", EGD.ITRAN);

};




#EGTRAN - Transition phase
var EGTRAN = func {

#Compute reference parameters during transition phase
if (EGD.ITRAN == "OFF")
	{
	EGD.DREFP = EGD.ALFM; #33ft/s²
	EGD.ITRAN = "ON";
	}

EGD.DREFPT = EGD.DREFP - EGD.DF;

if (math.abs(EGD.DREFPT) < EGD.E1) {EGD.DREFP = EGD.DF + EGD.E1 * math.sgn(EGD.DREFPT);}

if (EGD.DREFP < EGD.E1) {EGD.DREFP = EGD.E1;}

EGD.DREFPT = EGD.DREFP - EGD.DF;
EGD.C1 = EGD.DREFPT / (EGD.EEF - EGD.EEF4);
EGD.RER1 = EGD.CNMFS * math.ln(EGD.DREFP / EGD.DF) / EGD.C1;
EGD.DRDD = math.min(EGD.CNMFS / (EGD.C1 * EGD.DREFP) - EGD.RER1 / EGD.DREFPT, EGD.DRDDL);
EGD.DREFP = EGD.DREFP + (EGRT_data.TRANGE - EGD.RER1 - EGD.RPT1) / EGD.DRDD;
EGD.DLIM = EGD.ALIM * entry_jsbsim.DRAG / entry_jsbsim.XLFAC;

if (EGD.DREFP > EGD.DLIM)
	{
	EGD.DREFP = EGD.DLIM;
	EGD.C1 = 0;
	}

if (EGD.DREFP < EGD.E1) {EGD.DREFP = EGD.E1;}

EGD.RDTREF = -EGD.HS * entry_jsbsim.VE * (2 * EGD.DREFP - EGD.C1 * EGD.VE2) / EGD.CAG;
EGD.C2 = 4 * entry_jsbsim.VE * EGD.DREFP * (1 - math.pow(EGD.VE2 / EGD.CAG, 2)) / EGD.CAG;

#print("Drag limit is : ", EGD.DLIM);
#print("Drag ref is : ", EGD.DREFP, " H dot ref is : ", EGD.RDTREF, " DRDD is : ", EGD.DRDD, " ITRAN is : ", EGD.ITRAN);

};




#EGALPCMD - Angle of attack command
var EGALPCMD = func {

#Compute angle of attack command (based on STS1 40 ° profile instead of OTT 38/28 profile never used)
if ((entry_jsbsim.VE < EGD.VALP[EGD.IALP]) and (EGD.IALP > 0)) {EGD.IALP = EGD.IALP - 1;}

#Quadratic function for AOA
var j = EGD.IALP + 1;
EGD.ALPCMD = EGD.CALPO[j] + entry_jsbsim.VE * (EGD.CALP1[j] + EGD.CALP2[j] * entry_jsbsim.VE);

if (EGD.ISLECT == 1) {EGD.ALPCMD = EGD.MM304_ALPHA;}

EGD.ALPDOT = (EGD.ALPCMD - EGD.ACMD1) / EGD.DTEGD;
EGD.ACMD1 = EGD.ALPCMD;

#print("IALP is : ", EGD.IALP, " Commanded Alpha without mod is : ", EGD.ALPCMD);

};




#EGGNSLCT - Gain select
var EGNNSLCT = func {

#Compute controller gains
EGD.C16 = EGD.CT16[1] * math.pow(entry_jsbsim.DRAG, EGD.CT16[2]);
if (entry_jsbsim.VE < EGD.VC16) {EGD.C16 = EGD.C16 + EGD.CT16[3] * (entry_jsbsim.DRAG - EGD.DREFP);}
EGD.C16 = SpaceShuttle.MIDVAL(EGD.C16, EGD.CT16MN, EGD.CT16MX);

if (EGD.ICT == 1) {EGD.CT17MN = EGD.CT17M2;} #Alpha mod
EGD.C17 = SpaceShuttle.MIDVAL(EGD.CT17[1] * math.pow(entry_jsbsim.DRAG, EGD.CT17[2]), EGD.CT17MN, EGD.CT17MX);
if (EGD.ICT == 1) {EGD.C17 = EGD.C17MP * EGD.C17;}

#print("C16 delta drag factor is : ", EGD.C16, " C17 Hdot factor is : ", EGD.C17);

};




#EGLODVCMD - lateral logic and vertical L/D command
var EGLODVCMD = func {

#Compute vertical L/D command (LODV)
EGD.A44 = math.exp(-(entry_jsbsim.VE - EGD.CDDOT[1]) / EGD.CDDOT[2]);
EGD.CDCAL = EGD.CDDOT[4] + EGD.ALPCMD * (EGD.CDDOT[5] + EGD.CDDOT[6] * EGD.ALPCMD) + EGD.CDDOT[3] * EGD.A44;
EGD.CDDOTC = EGD.CDDOT[7] * (entry_jsbsim.DRAG + EGD.GS_ft * entry_jsbsim.RDOT / entry_jsbsim.VE) * EGD.A44 + EGD.ALPDOT * (EGD.CDDOT[8] * EGD.ALPCMD + EGD.CDDOT[9]);
EGD.C4 = EGD.HS * EGD.CDDOTC / EGD.CDCAL;

#Alpha modulation
if (entry_jsbsim.VE < EGD.VNOALP)
	{
	if ((entry_jsbsim.DRAG >= EGD.DREFP) or (entry_jsbsim.VE < EGD.VALMOD) or (EGD.ICT == 1))
		{
		EGD.ICT = 1;
		EGD.C20 = SpaceShuttle.MIDVAL(EGD.C22 + EGD.C23 * entry_jsbsim.VE, EGD.C24 ,EGD.C21);

		if (entry_jsbsim.VE < EGD.VC20) {EGD.C20 = math.max(EGD.C25 + entry_jsbsim.VE * EGD.C26, EGD.C27);}

		#Delta alpha based on max Drag of 33 ft/s²
		EGD.DELALP = SpaceShuttle.MIDVAL(EGD.CDCAL * ((math.min(EGD.DREFP, EGD.ALFM) / math.min(entry_jsbsim.DRAG, EGD.ALFM)) - 1) / EGD.C20, -EGD.DLAPLM, EGD.DLAPLM);

		if (math.abs(entry_jsbsim.DRAG - EGD.DREFP) < EGD.DDMIN) {EGD.DELALP = 0;}

		#print("Delta alpha modulation is : ", EGD.DELALP);
		#print("C20 drag mod coeff is : ", EGD.C20);
		#print("CDCAL is : ", EGD.CDCAL);
		}
	}

if (EGD.ISLECT == 5) {EGD.T1 = EGD.GS_ft * (1 - EGD.VE2 / EGD.VSAT2);}

EGD.ALDREF = EGD.T1 / EGD.DREFP + (2 * EGD.RDTREF + EGD.C2 * EGD.HS) / entry_jsbsim.VE; #LOD ref
EGD.RDTRF = EGD.RDTREF + EGD.C4;
EGD.DD = entry_jsbsim.DRAG - EGD.DREFP;

#Hdot feedback
if (entry_jsbsim.VE < EGD.VRDT)
	{
	EGD.DDS = SpaceShuttle.MIDVAL(EGD.DD, -EGD.DDLIM, EGD.DDLIM);
	EGD.ZK = EGD.ZK1;

	if (EGD.RK2RLP * EGD.RK2ROL < 0) {EGD.VTRB = entry_jsbsim.VE - EGD.ACN1 * EGD.DREFP;}

	if ((math.abs(EGD.DD) <= math.abs(EGD.DDP)) or (entry_jsbsim.VE > EGD.VTRB) or (EGD.LMFLG > 0)) {EGD.ZK = 0;}

	#H dot feedback (H dot bias for display/guidance; can be resetted // Clamped (Handbook Entry)
	EGD.DLRDOT = SpaceShuttle.MIDVAL(EGD.DLRDOT + EGD.ZK * EGD.DDS, -150, 150); 
	}

EGD.DDP = EGD.DD;
EGD.RK2RLP = EGD.RK2ROL;

#Delta drag correction based on ALFM max (33ft/s²) // Rdotref unclamped for High energy correction
EGD.DD_LOD = math.min(entry_jsbsim.DRAG, EGD.ALFM) - math.min(EGD.DREFP, EGD.ALFM);

EGD.LODX = EGD.ALDREF + EGD.C16 * EGD.DD_LOD + EGD.C17 * (EGD.RDTRF + EGD.DLRDOT - entry_jsbsim.RDOT); #LOD commanded
EGD.LODV = EGD.LODX;

EGD.YL = SpaceShuttle.MIDVAL(EGD.CY0 + EGD.CY1 * entry_jsbsim.VE, EGD.Y[2], EGD.Y[1]);
EGD.LMN = EGD.ALMN[2];
EGD.DZSGN = math.abs(EGRT_data.DELAZ) - math.abs(EGD.DZOLD);
EGD.DZOLD = EGRT_data.DELAZ;

#if ((entry_jsbsim.VE > EGD.VREF) and (entry_jsbsim.DRAG > (EGD.DREFP + 2))) {EGD.EEI = 3;} # VREF not defined ? EEI is entry evaluation indicator

if (EGD.DZSGN > 0)
	{
	if ((EGD.YL - EGD.YLMIN) < math.abs(EGRT_data.DELAZ)) {EGD.LMN = EGD.ALMN[1];}
	}
else
	{
	if ((EGD.YL - EGD.YLMIN2) < math.abs(EGRT_data.DELAZ)) {EGD.LMN = EGD.ALMN[1];}
	}

if (entry_jsbsim.VE > EGD.VYLMAX) {EGD.LMN = EGD.ALMN[4];}
else {EGD.RK2ROL =  math.sgn(entry_jsbsim.ROLL);}

if (entry_jsbsim.VE < EGD.VELMN) {EGD.LMN = EGD.ALMN[3];}

EGD.LMN = EGD.XLOD * EGD.LMN;
EGD.DLZRL = EGRT_data.DELAZ * EGD.RK2ROL;

#First roll reversal at Daz = 10° then 17° above mach 4
if (((EGD.RK2ROL * EGD.RK2RLP) > 0) and (EGD.IDBCHG == 1)) 
	{
	if (entry_jsbsim.VE > 4000) {EGD.Y[1] = EGD.Y[3];} #17°
	else {EGD.Y[1] = 0.1832596;} #10°
	}

if ((math.abs(EGD.LODV) >= EGD.LMN) and (EGD.DLZRL <= 0))
	{
	EGD.LMFLG = 1;
	EGD.LODV = EGD.LMN * math.sgn(EGD.LODV);
	}
else
	{
	EGD.LMFLG = 0;
	EGD.LMN = EGD.XLOD;

	### Roll Reversal ###

	#RR will not start if DELAZ condition is not met 
	if ((EGD.DLZRL >= EGD.YL) and (math.abs(EGRT_data.DELAZ) > EGD.Y[1]))
		{
		EGD.RK2ROL = -EGD.RK2ROL; #Roll reversal
		EGD.IDBCHG = 1;

		if (EGD.RR_flag == 0)
			{
			#Roll reversal jsbsim init
			#print("Initiating roll reversal!");
			setprop("/fdm/jsbsim/systems/ap/entry/roll-reversal-init", 1);

			EGD.RR_flag = 1;
			}
		}

	else if ((math.abs(EGRT_data.DELAZ) < EGD.Y[1]) and (EGD.RR_flag == 1))
		{
		#Roll reversal ending
		setprop("/fdm/jsbsim/systems/ap/entry/roll-reversal-init", 0);
		#print("Ending roll reversal!");
		EGD.RR_flag = 0;
		}
	}
	

#print("Delta drag is : ", EGD.DD, " DLRDOT is ", EGD.DLRDOT, " LOD commanded is : ", EGD.LODV, " LOD ref is : ", EGD.ALDREF);
#print("Max LODV is : ", EGD.LMN, " roll sign is : ", EGD.RK2ROL);
#print("C4 is : ", EGD.C4);
#print("Delta drag for LOD is : ", EGD.DD_LOD);
#print("Roll reversal flag is : ", EGD.RR_flag);

};



#EGROLCMD - roll command
var EGROLCMD = func {

#Compute roll commands and reference angles
if ((EGD.RK2ROL < 0) and (EGD.IVRR == 0))
	{
	EGD.VRR = entry_jsbsim.VE;
	EGD.IVRR = 1;
	}

EGD.ARG[1] = EGD.LODV / EGD.XLOD; #Roll commanded
EGD.ARG[2] = EGD.LODX / EGD.XLOD;
EGD.ARG[3] = EGD.ALDREF / EGD.XLOD; #Roll ref for display

for(var i = 1; i < 4; i = i+1)
	{
	if (math.abs(EGD.ARG[i]) >= 1) {EGD.ARG[i] = math.sgn(EGD.ARG[i] * EGD.XLOD);}
	EGD.ROLLC[i] = EGD.RK2ROL * math.acos(EGD.ARG[i]) * 57.29578;
	}


#Alpha modulation drag bias
if (EGD.ICT == 1)
	{
	EGD.DELALF = entry_jsbsim.ALPHA - EGD.ACMD1;
	EGD.RDEALF = SpaceShuttle.MIDVAL(EGD.CRDEAF * EGD.DELALF, -EGD.RDMAX, EGD.RDMAX);
	EGD.ALMNXD = math.acos (EGD.LMN / EGD.XLOD) * 57.29578;
	EGD.ROLLC[1] = (math.abs(EGD.ROLLC[2]) + SpaceShuttle.MIDVAL(EGD.RDEALF, -EGD.ALMNXD, EGD.ALMNXD)) * EGD.RK2ROL;
	
	EGD.ACLAM = math.min(EGD.DLALLM, EGD.ACLAM_C[1] + EGD.ACLAM_C[2] * entry_jsbsim.VE);
	EGD.ACLIM = math.min(EGD.ACLIM_C[1] + EGD.ACLIM_C[2] * entry_jsbsim.VE, EGD.ACLIM_C[3] + EGD.ACLIM_C[4] * entry_jsbsim.VE);

	#Alpha mod commanded (Alpha vs Acmd ?)
	EGD.ALPCMD = SpaceShuttle.MIDVAL(EGD.ACMD1 + EGD.DELALP, EGD.ACLIM, EGD.ACLAM);

	#print("Max alpha mod is : ", EGD.ACLAM, " Min alpha mod is : ", EGD.ACLIM);
	}


#Limit roll command
if (entry_jsbsim.VE > EGD.VRLMC) {EGD.RLM = math.min(EGD.RLMC[1], EGD.RLMC[2] + EGD.RLMC[3] * entry_jsbsim.VE);} #Entry(70°)
else {EGD.RLM = math.max(EGD.RLMC[6], EGD.RLMC[4] + EGD.RLMC[5] * entry_jsbsim.VE);} #TAEM(30°)
if ((math.abs(EGD.ROLLC[1]) > EGD.RLM) and (entry_jsbsim.VE < EGD.VEROLC)) {EGD.ROLLC[1] = EGD.RLM * math.sgn(EGD.ROLLC[1]);} #Max 70 ° below 8000 ft/s

#First roll
if ((EGD.ISLECT == 2) and (EGD.ISLECP == 1)) {EGD.RC176G = EGD.ROLLC[1];} 
EGD.ISLECP = EGD.ISLECT;

#Roll commanded and filtered deg
EGD.ROLLCMD = EGD.ROLLC[1];

#Roll clamped for AP (Minimum 20° / Max 180° during closed loop)
if (EGD.ISLECT > 1) {EGD.ROLLCMD = EGD.RK2ROL * SpaceShuttle.MIDVAL(math.abs(EGD.ROLLCMD), 20, 180);}

#print("First roll velocity is : ", EGD.VRR, " Roll ref is : ", EGD.ROLLC[3], " Roll commanded is : ", EGD.ROLLC[1], " Alpha mod commanded is : ", EGD.ALPCMD);
#print("ICT is : ", EGD.ICT, " Delta alpha is : ", EGD.DELALF, " Roll sign is : ", EGD.RK2ROL);

};



# Entry Guidance Autopilot Output - IBM Autopilot WIP (Appendix D) ###########################################################


var DAP3D = func {

### Jsbsim outputs // to be done later in a proper function with deadband logic ###

#Low energy logic  bypass
if (getprop("/fdm/jsbsim/systems/ap/entry/low-energy-logic") == 1) 
	{
	EGD.ALPCMD = EGD.ACMD1 - 3;
	EGD.ROLLCMD = EGD.RK2ROL * SpaceShuttle.MIDVAL(math.abs(EGD.ROLLCMD), 10, math.abs(2.2 * 57.29578 * EGRT_data.DELAZ))

	#Low energy automatically exited when Nominal path is reached WIP
	#if ()
	 #{
	 #setprop("/fdm/jsbsim/systems/ap/entry/low-energy-logic", 0);
	 #}
	}

#Difference done there to be have same refresh rate between Roll and RollC
#setprop("/fdm/jsbsim/systems/ap/entry/bank-diff", EGD.ROLLCMD - entry_jsbsim.ROLL * 57.29578);

#TAL Alpha 43° before first pullout
if ((getprop("/fdm/jsbsim/systems/abort/abort-mode") == 2) and (EGD.TAL_pullout_flag == 0))
	{
	EGD.ALPCMD = 43;

	#Normal Alpha logic after first pullout (400 ft/s) or Equilibrium phase reached
	if (((entry_jsbsim.RDOT > -400) and (EGD.ISLECT == 2)) or (EGD.ISLECT > 2)) {EGD.TAL_pullout_flag = 1;}
	}

#Bank commanded output (Minimum Bank commanded of 20° // Entry Workbook)
setprop("/fdm/jsbsim/systems/ap/entry/bank-cmd", EGD.ROLLCMD);

#Jsbsim output in rad
setprop("/fdm/jsbsim/systems/ap/entry/alpha-commanded", EGD.ALPCMD * 0.0174533);



};



# Entry Speedbrake control #

var Entry_Speedbrake_control = func {

EGD.SB_flag = 1;

#Control of the SB out of the TGSBC function to work at the correct rate (10.9 deg/s)
var sb_state = getprop("/controls/shuttle/speedbrake");

if (getprop("/fdm/jsbsim/systems/ap/automatic-sb-control") == 1)	
		{		
		if (sb_state > 0.82) {SpaceShuttle.decrease_speedbrake();}
		else if (sb_state <= 0.80) {SpaceShuttle.increase_speedbrake();}
		}

#End of SB control after 80% reached or TAEM init
if ((sb_state > 0.795) or (TAEM_guidance_available == 1)) {return;}
	
#dT = 0.1 / it gives 10°/s for closing and 5°/s for opening
settimer( func {Entry_Speedbrake_control(); }, 0.1);

};




### Various functions for Entry Code ###


var entry_guidance_available = 0;

var entry_interface = geo.Coord.new();
var distance_last = 0.0;

var radius_set = [];

#Variable for entry history path 
var trailer_set = {

	entry: [[0,0], [0,0], [0,0], [0,0], [0,0], [0,0]],
	entry_box: [[0,0], [0,0], [0,0], [0,0], [0,0], [0,0]],
	timer: 0,
	time_limit: 29,
	update: func (distance) {

	
	if (me.timer == 0)
		{
		me.timer = me.timer + 1;
		me.create_entry(distance);
		}
	else
		{me.timer = me.timer + 1;}
	if (me.timer >= me.time_limit) {me.timer = 0;}
	
	},

	updatebox: func (distance) {

	
	if (me.timer == 0)
		{
		me.timer = me.timer + 1;
		me.create_entry_box(distance);
		}
	else
		{me.timer = me.timer + 1;}
	if (me.timer >= me.time_limit) {me.timer = 0;}
	
	},

	

	create_entry: func (distance) {
		
	#6  markers for shuttle path history 
	me.entry[5][0] = me.entry[4][0];
	me.entry[5][1] = me.entry[4][1];

	me.entry[4][0] = me.entry[3][0];
	me.entry[4][1] = me.entry[3][1];

	me.entry[3][0] = me.entry[2][0];
	me.entry[3][1] = me.entry[2][1];

	me.entry[2][0] = me.entry[1][0];
	me.entry[2][1] = me.entry[1][1];

	me.entry[1][0] = me.entry[0][0];
	me.entry[1][1] = me.entry[0][1];

	me.entry[0][0] = distance;
	me.entry[0][1] = entry_jsbsim.VE;
	},

	create_entry_box: func (distance) {

	#6  markers for guidance box history 
	me.entry_box[5][0] = me.entry_box[4][0];
	me.entry_box[5][1] = me.entry_box[4][1];

	me.entry_box[4][0] = me.entry_box[3][0];
	me.entry_box[4][1] = me.entry_box[3][1];

	me.entry_box[3][0] = me.entry_box[2][0];
	me.entry_box[3][1] = me.entry_box[2][1];

	me.entry_box[2][0] = me.entry_box[1][0];
	me.entry_box[2][1] = me.entry_box[1][1];

	me.entry_box[1][0] = me.entry_box[0][0];
	me.entry_box[1][1] = me.entry_box[0][1];

	me.entry_box[0][0] = distance;
	me.entry_box[0][1] = entry_jsbsim.VE;
	},

};


# manage body flap trim #################################################

var body_flap_management = func {

var q_bar = getprop("/fdm/jsbsim/aero/qbar-psf");
var pod_pitch_up = getprop("/fdm/jsbsim/systems/rcs/pod2-up-raw-pitch");
var pod_pitch_down = getprop("/fdm/jsbsim/systems/rcs/pod2-down-raw-pitch");



if (getprop("/fdm/jsbsim/systems/ap/automatic-bodyflap-control") == 0) {return;}


var elevator_trim = getprop("/fdm/jsbsim/fcs/elevator-pos-deg");
var altitude = getprop("/position/altitude-ft");
var roll_velocity_rad = getprop("/fdm/jsbsim/velocities/p-rad_sec");
var roll_reversal = getprop("/fdm/jsbsim/systems/ap/entry/roll-reversal-init");


#For q bar below 20 in ops 3, Body flaps trims the RCS pitch up/down (based on Up/Down firing) to alleviate RCS pitch load ( and excessive fuel consumption)
#Then above q bar of 20 , Body flaps trim the elevons when Pitch Jets are deactivated

#Q bar 0.5, start of aerosurfaces trim
if (q_bar < 0.5) {return;}

else if ((q_bar > 0.5) and (q_bar < 20)) #Pitch jets trimming outside roll reversal to avoid BF oscillations
	{
	#if (roll_velocity_rad < 0.0087)
	if (roll_reversal == 0)
		{
		if (pod_pitch_up > 0) {SpaceShuttle.bodyflap_up();}
		else if (pod_pitch_down > 0) {SpaceShuttle.bodyflap_down();}
		}
	else {return;}
	}
else
	{
	if (altitude > 10000) #Elevons trimming
		{ 
		if (elevator_trim < -5.0) {SpaceShuttle.bodyflap_up();}
		else if (elevator_trim > 3.0) {SpaceShuttle.bodyflap_down();}
		}

	#Bodyflap in Trail position in final (0° on final // 0 of CmBF) // Neutral Cl/Cd/Cm BF 
	else if (altitude < 10000) 
		{
		var bodyflap_state = getprop("/controls/shuttle/bodyflap-pos-rad");
		if (bodyflap_state != 0.0) 
			{
			bodyflap_state = 0.0;
			setprop("/controls/shuttle/bodyflap-pos-rad", bodyflap_state);
			SpaceShuttle.callout.make("Body Flaps Trail.", "info");
			return;
			}
		}
	}
}


var create_radius_set = func {

var base = geo.Coord.new();
var dist = getprop("/sim/gui/dialogs/SpaceShuttle/entry_guidance/EI-radius") * 1853.0;
var point = [];

var npoints = 20;

var course = 330;
var step = (course -180.0)/(npoints-1);

for (var i = 0; i< npoints; i=i+1)
	{
	base.set_xyz(landing_site.x(), landing_site.y(), landing_site.z());
	base.apply_course_distance(course - i*step, dist);
	point = [SpaceShuttle.lon_to_x(base.lon()), SpaceShuttle.lat_to_y(base.lat())];
	append(radius_set, point);
	}


}

var compute_entry_guidance_target = func {

var pos = geo.aircraft_position();

var distance = pos.distance_to(landing_site)/ 1853.0;
var course = pos.course_to(landing_site);

setprop("/sim/gui/dialogs/SpaceShuttle/entry_guidance/site-dist", 0.0);


# now we compute the desired entry interface
# make that 4100 miles to site

setsize(radius_set, 0);
create_radius_set();

setprop("/sim/gui/dialogs/SpaceShuttle/entry_guidance/site-dist", distance);
setprop("/sim/gui/dialogs/SpaceShuttle/entry_guidance/site-string", "active");


var mode_string = getprop("/sim/gui/dialogs/SpaceShuttle/entry_guidance/entry-mode");

if (mode_string == "normal")
	{
	setprop("/fdm/jsbsim/systems/entry_guidance/guidance-mode",1);
	}
else if (mode_string == "TAL")
	{
	setprop("/fdm/jsbsim/systems/entry_guidance/guidance-mode",2);
	}
else if (mode_string == "RTLS")
	{
	SpaceShuttle.init_rtls();
	}

entry_guidance_available = 1;

# usually we would compute a TAEM guidance target at TAEM interface, but if the Shuttle is
# initialized at TAEM interface, no target is selected yet, so if distance to site is
# within TAEM range, we compute it now

if ((distance < 100.0) and (mode_string != "RTLS")){SpaceShuttle.compute_TAEM_guidance_targets();}


}


#Function to compute delta azimuth of alternate landing sites (Spec 54) / landing_site is a coord variable
var delta_azimuth_entry = func(landing_site_coord) {

var ground_track = getprop("/fdm/jsbsim/systems/entry_guidance/groundtrack-course-deg");
var pos = SpaceShuttle.state_vector_position();
var delta_az = 0;

#For RTLS and TAEM (delta_az = Bearing to Wp1 - Heading) / for entry (delta_az = Heading - Bearing to Wp1)
if ((GRTLS.CONT == "ON") or (GRTLS.PRTLS == "ON")) {delta_az = -ground_track + pos.course_to(landing_site_coord);}
else {delta_az = ground_track - pos.course_to(landing_site_coord);}

if (delta_az < -180) {delta_az = delta_az + 360;}
else if(delta_az > 180) {delta_az = delta_az - 360;}

return delta_az
};

# this is Vandenberg  we update later upon selection

landing_site.set_latlon(34.722, -120.567);




############## Older Entry Guidance Main function ##############

var update_entry_guidance =  func {

var pos = geo.aircraft_position();
var mm = getprop("/fdm/jsbsim/systems/dps/major-mode");

if (SpaceShuttle.bfs_in_control == 1)
	{
	mm = getprop("/fdm/jsbsim/systems/dps/major-mode-bfs");
	}

var course = pos.course_to(landing_site);
var v_eci = getprop("/fdm/jsbsim/velocities/eci-velocity-mag-fps");
var v_true_fps = getprop("/fdm/jsbsim/velocities/vtrue-fps");
var distance = pos.distance_to(landing_site);
var altitude = getprop("/position/altitude-ft");
var v_rel_fps = (distance - distance_last) /0.3048;
var drag_fps = getprop("fdm/jsbsim/systems/entry_guidance/aero-drag-deceleration-fts");
var drag_ratio = getprop("/fdm/jsbsim/systems/entry_guidance/drag-guidance-box");

setprop("/fdm/jsbsim/systems/entry_guidance/vrel-fps", v_rel_fps);
if (v_rel_fps > 0.0)
	{
	setprop("/fdm/jsbsim/systems/entry_guidance/vrel-sign", 1);
	}
else
	{
	setprop("/fdm/jsbsim/systems/entry_guidance/vrel-sign", -1);
	}

distance_last = distance;

distance = distance/ 1853.0;


setprop("/fdm/jsbsim/systems/entry_guidance/target-azimuth-deg", course);
setprop("/fdm/jsbsim/systems/entry_guidance/remaining-distance-nm", distance);


if (mm == 304)
	{
	var v_error = SpaceShuttle.get_entry_drag_deviation(v_true_fps, distance);
	#var v_error = SpaceShuttle.get_entry_drag_deviation(drag_fps, distance); #Delta drag done in xml
	setprop("/fdm/jsbsim/systems/entry_guidance/v-error-fps", v_error);

	trailer_set.update(SpaceShuttle.EGRT_data.TRANGE);
	trailer_set.updatebox(SpaceShuttle.EGRT_data.TRANGE - SpaceShuttle.EGD.DRDD * (SpaceShuttle.entry_jsbsim.DRAG - SpaceShuttle.EGD.DREFP)); #Guidance box
	roll_reversal_management();
	body_flap_management();

	#Trailer update speed goes to 15 seconds below 14000 ft/s (Entry traj 3) 
	if ((v_true_fps < 14000) and (trailer_set.time_limit == 29))
		{
		trailer_set.time_limit = 15;
		#trailer_set.update(distance);
		}

	#SB 81% at Mach 10 for Cm considerations (same SB logic than in TAEM guidance.nas)
	#DeadBand to keep the SB in a constant position
	var sb_state = getprop("/controls/shuttle/speedbrake");
	if ((v_true_fps < 10000) and (v_true_fps > 2500) and (sb_state < 0.75)) 
		{
		var sb_max = 0.80;

		if (getprop("/fdm/jsbsim/systems/ap/automatic-sb-control") == 1)	
			{
			if (sb_state > sb_max + 0.02) {SpaceShuttle.decrease_speedbrake();}
			else if (sb_state < sb_max - 0.02) {SpaceShuttle.increase_speedbrake();}
			}
		}


	# cease banking and alpha management in the transition to TAEM (Mid value, End of traj 5 between 50 and 70 Nm)
	#No bank, max L/D alpha at TAEM transition
	

	if (distance < 65.0) #or (v_true_fps < 3000) or (altitude < 90000))
		{
		if (getprop("/fdm/jsbsim/systems/ap/entry/taem-transit-init") == 0)
			{
			#print("Preparing transition to TAEM guidance!");
			setprop("/fdm/jsbsim/systems/ap/entry/taem-transit-init",1);
			}
		}
	}
}


# manage roll reversals #################################################

var roll_reversal_management = func {

var current_bank = getprop("/orientation/roll-deg");
var roll_direction = getprop("/fdm/jsbsim/systems/ap/entry/roll-sign");
var v_true_fps = getprop("/fdm/jsbsim/velocities/vtrue-fps");

# if a roll reversal is on, we need to check whether to end it

if (getprop("/fdm/jsbsim/systems/ap/entry/roll-reversal-init") == 1)
	{
	var commanded_bank = getprop("/fdm/jsbsim/systems/ap/entry/reversal-bank-angle-target-deg");

	if (math.abs(current_bank - commanded_bank) < 5.0)
		{

		roll_direction = - roll_direction;
		setprop("/fdm/jsbsim/systems/ap/entry/roll-sign", roll_direction);
		setprop("/fdm/jsbsim/systems/ap/entry/roll-reversal-init", 0);
		#print("Ending roll reversal!");
		return;
		}

	}


var delta_az = getprop("/fdm/jsbsim/systems/entry_guidance/delta-azimuth-deg");


var drag_bank = getprop("/fdm/jsbsim/systems/ap/entry/drag-bank-angle-target-deg");

if (getprop("/fdm/jsbsim/systems/ap/entry/roll-reversal-init") == 0)
{

#Roll reversal at Daz = 17 ° above mach 4 and 10 ° below

if (v_true_fps > 4000)
	{
	if (math.abs(delta_az) < 17.0) {return;}
	if (((delta_az > 17.0) and (roll_direction == 1)) or ((delta_az < -17.0) and (roll_direction == -1)))
		{
		
		setprop("/fdm/jsbsim/systems/ap/entry/reversal-bank-angle-target-deg", -current_bank);
		#print("Initiating roll reversal!");
		setprop("/fdm/jsbsim/systems/ap/entry/roll-reversal-init", 1);
		}
	}

else 
	{
	if (math.abs(delta_az) < 10.0) {return;}
	if (((delta_az > 10.0) and (roll_direction == 1)) or ((delta_az < -10.0) and (roll_direction == -1)))
		{
		setprop("/fdm/jsbsim/systems/ap/entry/reversal-bank-angle-target-deg", -current_bank);
		#print("Initiating roll reversal!");
		setprop("/fdm/jsbsim/systems/ap/entry/roll-reversal-init", 1);
		}
	}
}


}






