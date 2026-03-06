# GRTLS / ECAL / TAEM and AutoLand guidance functionalities for the Space Shuttle
# Thorsten Renk 2015 // Gingin 2024

#TAEM original guidance simulation links
#Shuttle TAEM guidance and Flight Control (OTT) JSC-16522
#Space Shuttle Autoland design AIAA paper 82-1604-CP
#Orbiter Autoland conference AIAA 92-1273

#TAEM Iphase (1 to 3) GRTLS (4 to 6) PRTLS (8) // App trainer (7)

# This file has been modified as part of the Utility mod

var TAEM_WP_1 = geo.Coord.new();
var TAEM_WP_2 = geo.Coord.new();
var TAEM_AP = geo.Coord.new();
var TAEM_threshold = geo.Coord.new();
var TAEM_HAC_center = geo.Coord.new();
var TAEM_rwy_nl = geo.Coord.new();
var TAEM_rwy_nr = geo.Coord.new();
var TAEM_rwy_fl = geo.Coord.new();
var TAEM_rwy_fr = geo.Coord.new();
var TAEM_guidance_available = 0;
var TAEM_TACAN_available = 0;
var TAEM_guidance_phase = 0;
var TAEM_guidance_string = "";
var TAEM_loop_running = 0;

var TAEM_units = {

	feet_to_meters: 0.3048,
	RTD: 57.29578,
	DTR: 0.0174533,

};

var TAEM_MCC_flags = {
	one_eighty: 0,
	ninety: 0
};

#JSB sim variables used in several functions
var TAEM_jsbsim = {

	#Property nodes (TAEM and entry)
	mach_node: 0,
	vtrue_node: 0,
	vh_node: 0,
	vi_node: 0,
	altitude_qnh_node: 0,
	qbar_node: 0,
	hdot_node: 0,
	hdot_dot_node: 0,
	weight_node: 0,
	phi_node: 0,
	theta_node: 0,
	alpha_node: 0,
	yaw_rate_node: 0,
	roll_rate_node: 0,
	pitch_rate_node: 0,
	nz_node: 0,
	gamma_node: 0,
	lat_geo_node: 0,
	lon_geo_node: 0,
	radius_geo_node: 0,
	lod_node: 0, #L/D
	load_factor_node: 0, #total G
	drag_accel_node: 0,
	cd_node: 0,
	auto_pitch_node: 0,
	auto_roll_node: 0,

	#TAEM guidance
	mach: 0,
	V_true: 0,
	V_ground: 0,
	V_equivalent: 0,
	VH: 0,
	QBAR: 0,
	H_qfe: 0,
	H_agl: 0,
	HDOT: 0,
	HDOT_DOT: 0,
	weight: 0, #slugs
	gamma: 0, #deg
	XYZE: [0,0,0],
	#XYZED: [0,0,0],
	LAT_geo: 0, #deg
	LON_geo: 0, 
	radius_geo: 0,
	auto_pitch: 0,
	auto_roll: 0,
	
	#TAEM DAP 
	PHIR: 0,
	THETAR: 0,
	ALPHA: 0,
	R: 0, #yaw rate deg/s
	P: 0, #roll rate
	Q: 0, #pitch rate
	NZ: 0,

};

#Shuttle coordinates in runway centered frame (used by TAEM and entry)
var rwy_coord = {

	#State vector position
	POS: 0,

	#Runway Lat/Lon/Az (rad)
	TLATD: 0,
	TLONG: 0,
	RAZ: 0, #AZRW
	RTE1: 0, #runway alt
	RLS: [0,0,0], #Rwy ECEF 

	#Conversion frame matrix
	REC: [0,0,0,0,0,0,0,0,0], #ECEF /rwy conversion mat
	REC_inv: [0,0,0,0,0,0,0,0,0],
	XYZ_rwy: [0,0,0], #Position in rwy frame // [RG] in Entry formalism

	#Coordinates rwy frame
	X: 0,
	Y: 0,
	Z: 0,
	X_sign: 1,
	Y_sign: 1,
	X_last: 0,
	Y_last: 0,
	Z_last: 0,

	#Velocity rwy frame
	X_dot: 0,
	Y_dot: 0,
	Z_dot: 0,

	#Course with respect to centerline
	PSD: 0,

	reset_rwy_coord: func {

		me.TLATD = 0;
		me.TLONG = 0;
		me.RAZ = 0;
		me.RLS = [0,0,0];
		me.REC = [0,0,0,0,0,0,0,0,0];
		me.REC_inv = [0,0,0,0,0,0,0,0,0];
		me.XYZ_rwy = [0,0,0];
		me.X = 0;
		me.Y = 0;
		me.Z = 0;
		me.X_sign = 1;
		me.Y_sign = 1;
		me.X_last = 0;
		me.Y_last = 0;
		me.X_dot = 0;
		me.Y_dot = 0;
		me.Z_dot = 0;
		me.PSD = 0;

	},

};

#GRTLS variables for OPS6 RTLS and ECAL from PRTLS to alpha transition (Phase 6 to 4)
var GRTLS = {

	#Flags
	CONT: "OFF",
	ECAL: "OFF",
	PRTLS: "OFF",
	INIT_PASS: 0,
	S_CONT_YAW: "OFF", #Yaw steering
	ISTP4: 1, #Phase 4 S-turn flag
	BAILOUT_FLAG: 0,
	DPSAC_INIT: "OFF", #ECAL High energy heading error init flag (ECAL phase 4 S turns)
	#DTG: 1.92,
	SPDBRK_FLAG: 0,

	#GRCOMP ECAL 
	EOW: 0,
	EN: 0,
	ES: 0,
	EST: 0,
	EMEP: 0,
	EMAX: 0,
	EMIN: 0,
	EN_C1: 0,
	EN_C2: 0, 
	EMEP_C1: 0,
	EMEP_C2: 0,
	ES_C1: 0,
	ES_C2: 0,
	EN_C1_ECAL: -30000,
	EN_C1_RTLS: -2400, #ECAL improvement source
	#EN_C1_RTLS: -2400,
	EN_C2_ECAL: 0.534,
	EN_C2_RTLS: 0.306, #ECAL improvement source
	#EN_C2_RTLS: 0.306,
	EMEP_C1_ECAL: -40000,
	EMEP_C1_RTLS: -2500, #ECAL improvement source
	#EMEP_C1_RTLS: -2500,
	EMEP_C2_ECAL: 0.534,
	EMEP_C2_RTLS: 0.294, #ECAL improvement source
	#EMEP_C2_RTLS: 0.294,
	ES_C1_ECAL: -35000,
	ES_C1_RTLS: -45000, #ECAL improvement source
	#ES_C1_RTLS: -45000,
	ES_C2_ECAL: 0.582,
	ES_C2_RTLS: 0.71, #ECAL improvement source
	#ES_C2_RTLS: 0.71,

	compute_EN_C: func{
	
		if ((me.CONT == "ON") and (me.PRTLS == "OFF"))
			{
			me.EN_C1 = me.EN_C1_ECAL;	
			me.EN_C2 = me.EN_C2_ECAL;	
			me.EMEP_C1 = me.EMEP_C1_ECAL;	
			me.EMEP_C2 = me.EMEP_C2_ECAL;
			#me.EMEP_C1 = me.EMEP_C1_RTLS;	
			#me.EMEP_C2 = me.EMEP_C2_RTLS;
			me.ES_C1 = me.ES_C1_ECAL;
			me.ES_C2 = me.ES_C2_ECAL;
			}
		else if (me.PRTLS == "ON")
			{
			me.EN_C1 = me.EN_C1_RTLS;	
			me.EN_C2 = me.EN_C2_RTLS;	
			me.EMEP_C1 = me.EMEP_C1_RTLS;	
			me.EMEP_C2 = me.EMEP_C2_RTLS;
			me.ES_C1 = me.ES_C1_RTLS;
			me.ES_C2 = me.ES_C2_RTLS;
			}

	},

	#GRTRN
	NZSW1: 1.75, #G initial transition for Nz Hold (1.85)
	HDTRN: -320, #RTLS transition to phase 4
	HDTRNA: -320, #CONT transition to phase 4
	HDTBNK: -600, #ECAL  Bank initiation during Nz hold
	GRALPR: 0,
	TLFMX1: 2.5,
	EN_DELTA: 0,
	EN_DELTA_OLD: 0,
	ENLIMHI: 0,
	ENLIMLO: 0,
	ALPHA_MIN: 0,
	ALPHA_MAX: 0,
	DPSAC1: 20,
	DPSAC2: 25,
	EN_ALPHA_BIAS: 0,
	EN_BIAS1: 0,
	EN_BIAS2: -0.5,
	EN_BIAS3: 0,
	EN_BIAS4: 0.15,
	ENALPL: -1,
	ENALPU: 0.5,
	ALPPRT: 3.5, #Pitch channel load factor protection limit
	
	#GRNZC 
	NZSW: 0,
	SMNZ1: 0, #Exp Nz lead term
	#SMNZC1A: 1.1, #Initial value of SMNZ1 (1.1)
	SMNZC1A: 0.13, #Initial value of SMNZ1
	SMNZC1: 0.13, #Initial value of SMNZ1
	SMNZ2: 0, #Linear Nz lead term
	SMNZC2: 0.6, #Initial value of SMNZ2 (0.95)
	SMNZC2A: 1.0, #Initial value of SMNZ2
	SMNZC3A: 0.7314,
	SMNZC3: 0.7314,
	#SMNZC4A: 0.35,
	SMNZC4A: 0.1,
	SMNZC4: 0.023,
	SMNCZ5: 0.065,
	SMNZ2L: -0.05,
	SMNZ2LA: -0.05,
	GRNZC1: 1.5, #NZ target i-loaded for RTLS (1.2)
	GRNZC1A: 2.5, #NZ target i-loaded for CONT 
	ALPE1: 25, #Alpha to Nz conversion
	ALPE2: 0.005,

	#GRALPC
	ALPCMD: 0,
	DGRALP: 0, #Alpha decrement for alpha transition
	ALPREC: 50,
	ALPREC_CONT: 58,
	ALPTRN_INIT: 0, #Alpha at Alpha transition init
	HDMAX: 0,
	DGRNZT: 0,
	DGRNZ: 0, #Between 0.2 and -0.3
	DHDNZ: 0.002,
	DHDLL: 0.2,
	DHDUL: -0.3,
	HDMAX: 0,
	HDNOM: - 1558,
	DNZB: 0.65,
	DHDNZ: 0.001,
	DNZMIN: 2,
	DNZMAX: 3.9,
	DNZ1: 0.2,
	DNZMX1: 3.0, #Max G's for Ecal Nz Prebank logic (3)
	ITGTNZ: "OFF", #ECAL prebank initiation flag (Nz Hold)
	IGRA: 0, #Flag for Alpha decrement at alpha transition
	GRALL: -0.25, #limit on DGRALP (0.5)
	GRALU: 0.25,
	GRALU2: 0.5, #Upper limit on DGRALP for ECAL and G load below 2.5 G's (higher rate allowed)
	
	#GRSBC
	SBQ: 20, #Qbar mini for opening
	DEL1SB: 3.125, #SB increment for phase 6 and 5
	GRSBL1: 80.6,
	GRSBL2: 65,
	MACHSBS: 19.5,
	MACHSBI: 2.6,
	HDSBC: 0, #Contigency hdot speedbrake
	MACHSB: 0, #Contigency mach speedbrake
	DSBOFB: 0, #Current speed brake position

	#GRPHIC
	MSW3: 7.0, #ISTP4 mach flag (Iphase 4 S turn)
	DPSACI: 0, #DPSAC initial for ECAL S turn limitation
	PHISTN: 60, #ECAL sturn roll limit
	PHISS: 45, #ECAL roll limit
	DPSACLMT: 0, #ECAL sturn DPSAC limit
	DPSACT: 0,
	DPSACLMT_MAX: 30, #ECAL max value of DPSACLMT
	DPHISLP: 3.125, 
	DPHIINT: 5,
	IBNK: "OFF", #ECAL roll logic flag (roll activation at phase 4)
	IRLPTF: "OFF", #ECAL G's roll protection flag
	PHINZ2: 0,  #ECAL max load roll
	DNZMX2: 3.9, #ECAL max g phase 4 turn 
	

};

#TGINIT Function variables that change with IPHASE (limits)
var TAEM_guidance_TGINIT = {

	#First pass init
	LOOP: 0,
	SB_LOOP: 0,
	
	DTG: 1.92, #Guidance cycle interval (2.08Hz) / 1.92 for PRTLS / 0.96 for GRTLS / 0.48 for TAEM
	IRESET: 0,
	ISR: 50.0, #Roll fader time constant (5 / DTG)
	MEP: 0,
	RF: 14000,
	RF_last: 14000, #RF shrinking
	PHILIM: 50,
	DNZUL: 0.5,
	DNZLL: -0.5,
	DSBI: 0,
	QBARF: 0,
	QBD: 0,
	TG_END: 0,
	AL_END: 0,
	GRTLS_TAEM_INIT: 0, #GRTLS flag / TAEM from OPS6
	IPHASE: 1, #TAEM phase
	PMODE: 0, #Autoland A/L phase
	FMODE: 0, #Autoland sub PMODE 3 (Flare)

	reset_TGINIT: func {
		
		me.LOOP = 0;
		#me.DTG = 0.48;
		me.ISR = 50.0;
		me.MEP = 0;
		me.RF = 14000;
		me.RF_last = 14000;
		me.PHILIM = 50;
		me.DNZUL = 0.5;
		me.DNZLL = -0.5;
		me.DSBI = 0;
		#me.QBARF = 0; 
		me.QBD = 0;
		#me.IPHASE = 1;
		me.PMODE = 0;
		me.FMODE = 0;
		#me.GRTLS_TAEM_INIT = 0;
		me.TG_END = 0;
		me.AL_END = 0;

		#Reset of some conditions for app trainer 
		TAEM_guidance_TGPHIC.FLATTURN = 0; 
		TAEM_guidance_TGSBC.first_retract_flag = 0;
		TAEM_guidance_TGSBC.second_retract_flag = 0;
	}, 

};


#XHAC Function variables (TGXHAC)
var TAEM_guidance_XHAC = {

	#Aim Point (Nominal / Close-in)
	IGI: 1,
	XA: -7500,

	#Inner Glide Slope (20°/18° weight dependant done at TAEM initialisation) /Tangent
	IGS: 1,
	TGGS: -0.36397023,
	GAMMA: -20,

	#Autoland variables 
	XK: -2780, #X coord of circular flare stage
	XE: -4322, #Exp decay boundary (FMODE3)

	#Heights of NEP/MEP and A/L interface (fixed)
	HFTC: 12018,
	HALI: 10018,
	HMEP: 6000,

	#Distance between threshold and 20° / AL / 18° final intercept (6.6Nm, 5.8Nm, 4Nm) XFTC = XA + HFTC/TGGS
	XFTC: 0, #6.6N to 7.2 Nm
	XALI: 0, #5.8Nmish
	XMEP: 0, #3.95Nmish

	#TAEM final parameters
	XHAC: 0,
	HHAC: 0,
	
	#Distance for Phase 3 transition (Pre-Final)
	DR3: 6000,
	RPRED3: 0,

	#HAC shrink logic
	hac_shrink_flag: 0,
	rw_perp_vec_lat: 0,
	rw_perp_vec_lon: 0,

	construct_hac_center: func {

		TAEM_HAC_center.set_latlon(TAEM_WP_2.lat() + me.rwy_perp_vec_lat * TAEM_guidance_TGINIT.RF , TAEM_WP_2.lon() + me.rwy_perp_vec_lon * TAEM_guidance_TGINIT.RF);
		TAEM_HAC_center.radius = TAEM_guidance_TGINIT.RF * 0.3048;

	},

	compute_TGXHAC: func {
		
		#Aim-point
		if (me.IGI == 1) {me.XA = -7500;}
		else {me.XA = -6500;}

		#Final GS
		if (me.IGS == 1) 
			{
			me.TGGS = -0.36397023;
			me.GAMMA = -20;
			me.XK = -2780;
			me.XE = -4322;
			TAEM_guidance_TGNZC.HDOT_TD1 = 8; #Open loop final flare 
			} 
		else 
			{
			me.TGGS = -0.32491969;
			me.GAMMA = -18;
			me.XK = -3350;
			me.XE = -5322;
			TAEM_guidance_TGNZC.HDOT_TD1 = 6;
			} 

		#Distance to A/L points
		me.XFTC = me.XA + (me.HFTC / me.TGGS);
		me.XALI = me.XA + (me.HALI / me.TGGS);
		me.XMEP = me.XA + (me.HMEP / me.TGGS);
		
	},


};

#Groundtrack Computation Function variables(GTP)
var TAEM_guidance_GTP = {

	#Right/Left turn into HAC
	YSGN: 0,
	
	#HAC transition factor (10% of Rturn initially)
	HAC_transition: 1.05,

	#Final radius Max/Min for Low Energy shrinking
	RFMX: 14000,
	RFMN: 7000, #5000

	#HAC center relative coordinates
	XCIR: 0,
	YCIR: 0,
	RCIRC: 0,

	#Angle relative to HAC Center
	PSC: 0,
	PST: 0,

	#Hac Turn Angle (initially set to 270°)
	PSHAN: 0,
	PSHA: 270,

	#Delta Azimuth to Tangency Point
	DPSAC: 0,
	
	#Distance to HAC Tangency Point
	RTAN: 0,

	#HAC radius
	RTURN: 0,

	#Distance into the HAC
	RPRED2: 0,

	#Direct distance to runway
	RPREDF: 0, 

	#Total distance to runway
	RPRED: 0,
	RPRED_last: 0,
	RPRED_nm: 0,

	#Vrel for RTLS
	VREL_fps: 0,
	
};	

#TAEM General Computation Function variables (TGCOMP)
var TAEM_guidance_TGCOMP = {
	 
	INIT: 0,

	#Actual E/W
	DRPRED: 0,
	EOW: 0,

	#Reference E/W and i-loaded variables
	EN: 0,
	EN_shift: 0,
	IEL: 1,

	EN_C1: 0,
	EN_C2: 0,
	EOWSPT: 90000, #97217 #117718 STS1 (choosen one is 19 Nm / DR2Max)
	EN_C1_IEL_1: -3000, #949
	EN_C1_IEL_2: 15000, #15360
	EN_C2_IEL_1: 0.5625, #0.5500275
	EN_C2_IEL_2: 0.375, #0.3890240
	R2MAX: 115000,
	ESHFMX: 10000, #20000 // Empirical Shift max for large HAC energy dump

	compute_EN_C: func {

		if (me.IEL == 1)
			{
			me.EN_C1 = me.EN_C1_IEL_1;
			me.EN_C2 = me.EN_C2_IEL_1;
			}
		else
			{
			me.EN_C1 = me.EN_C1_IEL_2;
			me.EN_C2 = me.EN_C2_IEL_2;
			}

	},	

	#Reference Altitude and i-loaded variables (Latest IGS adjusted)
	HREF: 0,
	PBHC: 0,
	CUBIC_C3: 0,
	CUBIC_C4: 0,
	PBRC: 0,
	PBGC: 0.1125953, #Gamma 6.4°

	#Reference Altitude and i-loaded variables for Autoland flare (IGS depending ones are done in XHAC)
	HK: 28932,
	XC: 1700, #H of circular flare closed loop init
	XA2: 1000, #IGS intercept (1000 feet past runway threshold)
	TGGAMMAREF2: -0.026185, #Gamma 1.5° (Inner GS)
	R_flare: 28802,
	HD: 29,  #Exp decay 
	sigma_EXP: 920,

	#Values from STS 1 (20° path) extended to 18° for IGS 2 (Same cubic part)
	PBRC_IGS_1: 308109.5, 
	PBRC_IGS_2: 308109.5, 
	PBHC_IGS_1: 84821.29,
	PBHC_IGS_2: 84821.29,
	CUBIC_C3_IGS_1: -0.3641168e-6,
	CUBIC_C3_IGS_2: -0.3641168e-6,
	CUBIC_C4_IGS_1: -0.9481026e-13,
	CUBIC_C4_IGS_2: -0.9481026e-13,

	#Values for a lower 18° cubic alti profile 
		#PBRC: 256527.82, 
		#PBHC_IGS_1: 71056.205,
		#PBHC_IGS_2: 64596.550,
		#CUBIC_C3_IGS_1: -4.3377079e-7,
		#CUBIC_C3_IGS_2: -3.9433708e-7,
		#CUBIC_C4_IGS_1: -2.2083206e-13,
		#CUBIC_C4_IGS_2: -2.0075642e-13,

	compute_HREF_C: func {

		if (TAEM_guidance_XHAC.IGS == 1)
			{
			me.PBRC = me.PBRC_IGS_1;
			me.PBHC = me.PBHC_IGS_1;
			me.CUBIC_C3 = me.CUBIC_C3_IGS_1;
			me.CUBIC_C4 = me.CUBIC_C4_IGS_1;
			}
		else
			{
			me.PBRC = me.PBRC_IGS_2;
			me.PBHC = me.PBHC_IGS_2;
			me.CUBIC_C3 = me.CUBIC_C3_IGS_2;
			me.CUBIC_C4 = me.CUBIC_C4_IGS_2;
			}

	},	
	

	#Reference Qbar and i-loaded variables 
	QBREF: 0,
	QBRUL: 305, #old Upper Limit 285psf / new 300kts
	QBRLL: 225, #old Lower Limit 180psf / new 258kts
	QBRML: 247, #old Middle Limit 220psf new 270kts
	PBRCQ: 121522, #old limit 89971.082 Segment boundary

	#Final Radius HAC shrinking
	HREFOH: 0,
	DRF: 0,

	#Altitude /Range error
	HERROR: 0,
	DHDRRF: 0,
	DELRNG: 0,

	#QBAR / QBAR dot filtered / QBAR error (QBARF/QBD in TGINIT)
	QBARD: 0,
	QBERR: 0,
	EAS_CMD: 0,
	CQG: 0.5583958,

};

#Taem Transition/Limits variables (TGTRAN)
var TAEM_guidance_TGTRAN = {

	#S-turn Energy and i-loaded variables
	ES: 1,
	ES1: 15000, #4523
	EDRS: 0.635416, #0.6089492
	ENBIAS: 10000,

	#MEP Energy and i-loaded variables
	EMEP: 0,
	EMEP_C1: 0,
	EMEP_C2: 0,
	EMEP_C1_IEL_1: -10000, #-3263
	EMEP_C1_IEL_2: 15000, #12088
	EMEP_C2_IEL_1: 0.44375, #0.4168274
	EMEP_C2_IEL_2: 0.29168, #0.2821187

	compute_EMEP_C: func {

		if (TAEM_guidance_TGCOMP.IEL == 1)
			{
			me.EMEP_C1 = me.EMEP_C1_IEL_1;
			me.EMEP_C2 = me.EMEP_C2_IEL_1;
			}
		else
			{
			me.EMEP_C1 = me.EMEP_C1_IEL_2;
			me.EMEP_C2 = me.EMEP_C2_IEL_2;
			}
	
	},

	#STIN Energy and i-loaded variables
	EMOH: 0,
	EMOH_C1: -7575,
	EMOH_C2: 0.4717993,

	#S-turns variables
	ENBIAS: 10000,
	sturn_threshold: 0,
	S_sign: 0,
	SPSI: 0,

};


#Taem Nz commanded variables (TGNZC)
var TAEM_guidance_TGNZC = {

#Nz command output
DNZC: 0, #NZC Unlimited
DNZCL: 0, #NZC Limited
DNZCD: 0, #NZC dot
NZC: 0, #NZC sent to FCS

#Altitude variables and gains TAEM
GDH: 0,
HDREF: 0,
HDERR: 0,
GDHC: 2.0,
GDHS: 7e-5,
GDHLL: 0.3,
GDHUL: 1.0,
HDREQG: 0.1,
DNZCG: 0.01,
HDREQG: 0.1,

#Altitude variables Autoland OGS (Lower gains in Tsikalas paper)
H_int: 0,
KH: 0.0036,
KHDOT: 0.0109,
KHINT: 0.05,

#Altitude variables Autoland Flare Nz Open Loop
NZCOM5: 0,
NZ_max_openloop: 0,
GAMMAERR_filtered: 0,
theta_dot_max: 0,
GAMMASYNC: 0,
TPRED: 2,
GAMMAREF2: -1.5, #IGS shallow slope

#Altitude variables Autoland Flare closed loop
NZCOM3: 0,
NZCOM4: 0,
HERREXP: 0, #Alti exp decay
V_true_filtered: 0,
KHDOTERR: 0.0120, #Same than OGS KHDOT
KH_pullup: 0.0046, #0.0046
KI_pullup: 0.05,
A13: 1, #Lag filter constant C1
A40: 1,

#Altitude variables Autoland Final Flare
NZCOM6: 0,
NZCOM7: 0,
HDREFF: 0, #HDREF filtered
HDEST: 0, #Hdot at ff initiate
HDOT_openloop: 0,
K_slope: 0,
slope_factor: 0.0, #linear coefficient for open loop (0.7 sweet spot)
H_NOACC: 2,
HDOT_TD1: 8, #6 for 18° #10 for 20° (?) / done in compute_TGXHAC
HDOT_TD2: -3,
KTCHDN: -4.88,
KHDOT: 0.0120, #0.02
KIFLR: 0.0, #0.1 
TFLR: 5,
TAU_TD2: 5,
KFLR: 0.025, #0.004 #0.0250
A3: 10, #rad/s

#Max/Min dynamic pressure profile variables
QBMXNZ: 0,
QBMNNZ: 0,
MXQBWT: 0,
QBLL: 0,
QBM1: 1.05,
QBM2: 1.7,
QBMX1: 340,
QBMX2: 300,
QBMX3: 300,
QBMXS1: -400,
QMACH1: 0.89,
QMACH2: 1.15,
QBWT1: 0.0233521,
QBWT2: 0.01902763,
QBWT3: 0.03113613,
QBMSL1: -0.0288355,
QBMSL2: 0.00570829,

#Pull up manoeuver variables
EQLOWL: 60000,
EQLOWU: 85000,
PSOHQB: 0,
QBREF2: 225, #QBREF at HAC 270° (TGCOMP.QBRLL)
PQBWRR: 0.006,
PEWRR: 0.52,

#QBAR upper/lower limits gains
QBNZUL: 0,
QBNZLL: 0,
QBG1: 0.1,
QBG2: 0.125,

#Energy over Weight Upper/ lower limits and gains
EMAX: 0,
EMIN: 0,
EOWNZUL: 0,
EOWNZLL: 0,
EDELNZU: 10000, #8000
EDELNZL: 4000,
GEUL: 0.1,
GEHDUL: 0.01,
GELL: 0.1,
GEHDLL: 0.01,
DNZCDL: 0.1,


};


#Taem Roll value commanded variables (TGPHIC)
var TAEM_guidance_TGPHIC = {

#Roll limits
PHILIMIT: 0,
PHIMIN: 30, #Supersonic roll limit (30° for TAEM vs 45° for GRTLS)

#Phase 1
TTH: 0, #time to hac

#Phase 2 HAC
RERRC: 0,
RDOT: 0,
PHIP2C: 0,
RDOTRF: 0,

#Phase 3 Pre Final
YINTERRC: 0,
YERRC: 0,
DPHI: 0,
PHIO: 0,

#Phase 3 variables (Max 2500 feet Xrange)
GY: 0.05, #0.05
GYDOT: 0.6, #0.6
GYINT: 0.01, #A3
YERRLM: 120, #Position error limit (YLIMft = YLIMerr° / GY)
YERRINTLM: 50, #Integral error limit 

#Autoland variables
YERR: 0,
YINTERR: 0,

#Autoland gains (Max 1000 feet Xrange)
FLATTURN: 0, #WOW
KYDOT: 0.7, #10
KY1: 0.07, #YDOT
AINT: 0, #Int not used for stability
YLIM: 1000, 
YINTLIM: 50,

#Final Roll commanded (unlimited and limited)
PHIC: 0,
PHIC_AT: 0,

};

#Taem Speedbrake logic variables (TGSBC)
var TAEM_guidance_TGSBC = {

#Limits
DSBCLL: 0,
DSBCUL: 0,
DSBLIM: 100,
DSBSUP: 65,
DSBNOM: 65,
DSBIL: 20, #Integral limit
DSBCM: 0.95,

#Energy i-loaded variables
SB_energy_UL: 0,
DSBCE2: 7000,
DSBCE1: 0.015,

#Qbar i-loaded variables
DSBE: 0,

#DSBI: 0, Integral loop value in TGINIT
GSBE: 1.5,
GSBI: 0.1,

#EAS i-loaded variables 
V_equivalent_filtered: 0,
EASERR: 0,
GSB: 2,
A14: 1, #EAS filter C1
#GSBI same than for Qbar int

#Fixed setting flag variables
first_retract_flag: 0,
second_retract_flag: 0,

#Stored SB values for pre-flare settings
#windspeed_int: 0,
#windspeed_int_last: 0,
windspeed_3000: 0,
timestamp_3000: 0,
windspeed_500: 0,
timestamp_500: 0,
windshift_500: 0,

#Deflection output
DSBC: 0,
DSBC_AT: 0,

};


#TDAP Function variables
var TAEM_guidance_TDAP = {

#Pitch channel variables
RTANP: 0,
DNZCMP: 0,
NZERR: 0,
QC: 0,
BCSL: 0,

#Pitch channel constants / Gains
TPLIM: 1,
CPMIN: 0.5, #Cos Phi min (60°) 
VCO: 549.125,
GQN: 3.36,

#Roll channel variables
BANKER: 0,
BANKER_prev: [0, 0, 0, 0, 0],
BANKER_sum: 0,
BRATE: 0,

#Roll channel constants / Gains
GPS: -3.25,
GPI: 4.4,
GPLL: 0.5,
GPUL: 1.8,
PCS: -16.667,
PCI: 30,
PCLL: 5,
PCUL: 20,

#AOA limits variables
ALPMIN: 0,
ALPMAX: 0,
AMNS: 20,
AMNI: -16,
AMXM2: 1.5,

#TDAP filter main input
XIN: 0,

#Filters input/output parameters 
XI_1: 0,
XI_2: 0,
XI_3: 0,
XO_1: 0,
XO_2: 0,
XO_3: 0,

#Fresh values in case of TAEM recomputation
reset_TDAP: func {

me.RTANP = 0;
me.DNZCMP = 0;
me.NZERR = 0;
me.QC = 0;
me.BCSL = 0;
TAEM_guidance_TGNZC.NZC = 0; #Final NZC sent to FCS

},



};

var TAEM_lag_filter_variables = {

#Filter 1
XI_1: 0,
XO_1: 0,

#Filter 2
XI_2: 0,
XO_2: 0,

#Filter 3
XI_3: 0,
XO_3: 0,

#Filter 4
XI_4: 0,
XO_4: 0,

#Filter 5
XI_5: 0,
XO_5: 0,

};

TAEM_threshold.MLS_available = 0;

var HUD_data_set = {

	vangle_aim: 0,
	hangle_aim: 0,
	vangle_threshold: 0,
	hangle_threshold: 0,
	vangle_guidance: 0,
	vangle_nr: 0,
	hangle_nr: 0,
	vangle_nl: 0,
	hangle_nl: 0,
	vangle_fr: 0,
	hangle_fr: 0,
	vangle_fl: 0,
	hangle_fl: 0,
	MLS_acquired: 0,
};


var area_nav_set = {

	TACAN_locked: 0,
	MLS_locked: 0,
	MLS_processing: 0,
	air_data_available: 0,
	gps_available: 0,
	source: 0,

	TACAN_aut: 0,
	TACAN_inh: 1,
	TACAN_for: 0,

	air_data_h_aut: 0,
	air_data_h_inh: 1,
	air_data_h_for: 0,

	air_data_gc_aut: 0,
	air_data_gc_inh: 1,
	air_data_gc_for: 0,

	drag_h_aut: 1,
	drag_h_inh: 0,
	drag_h_for: 0,

	gps_aut: 0,
	gps_inh: 1,
	gps_for: 0,

	gps_gc_aut: 0,
	gps_gc_inh: 1,
	gps_gc_for: 0,

	accuracy_lat : 0.1,
	accuracy_lon: 0.1,
	accuracy_alt: 15000.0,

	acc_x: 5000.0,
	acc_y: 5000.0,
	acc_z: 5000.0,

	offset_x: 0.0,
	offset_y: 0.0,
	offset_z: 0.0,

	offset_lat: 0.0,
	offset_lon: 0.0,
	
	baro_alt_m: 0.0,

	tacan_acc_dist: 0,
	tacan_acc_az: 0,

	tacan_offset_dist: 0,
	tacan_offset_az: 0,
	tacan_offset_az_deg: 0,

	dist_m: 0,

	m_to_lat: 1.0/110952.0,
	m_to_lon: 1.0/110952.0,

	nav_bearing_tacan: 0,
	nav_dist_tacan: 0,

	tac_resid_range: 0.0,
	tac_resid_bearing: 0.0,
	tac_ratio_range: 0.0,
	tac_ratio_bearing: 0.0,

	adta_resid_h: 0.0,
	adta_ratio_h: 0.0,

	drag_h_resid: 0.0,
	drag_h_ratio: 0.0,

	gps_resid: 0.0,
	gps_ratio: 0.0,

	drag_h_atm_model : 0,
	drag_h_offset: 	0.0,

	gps_update_cycle_count: 0,

	init: func {
		me.true_pos = geo.aircraft_position();
		me.nav_pos = state_vector_position();
		
		var acc_factor = 1.0;	

		if (me.true_pos.lat() > 55.0)
			{
			if (me.drag_h_atm_model == 0) {acc_factor = 1.3;}
			else if (me.drag_h_atm_model == 2) {acc_factor = 1.2;}
			}
		else if (me.true_pos.lat() < -55.0)
			{
			if (me.drag_h_atm_model == 0) {acc_factor = 1.3;}
			else if (me.drag_h_atm_model == 1) {acc_factor = 1.2;}
			}
		else
			{
			if (me.drag_h_atm_model == 1) {acc_factor = 1.2;}
			else if (me.drag_h_atm_model == 2){acc_factor = 1.2;}
			}
	

		me.drag_h_offset = acc_factor * 800.0 * (rand() - 0.5);
		#print("Init area nav, drag alt offset is: ", me.drag_h_offset);
	},


	update_entry: func {

		me.update_pos();
		me.update_signals();

		if (me.TACAN_locked == 1) 
			{
			if (TAEM_TACAN_available == 0)
				{set_TAEM_TACAN();}
			else
				{me.update_nav();}
			}

		if (me.air_data_available == 1)
			{
			me.compute_baro_alt_error();
			}
		me.compute_drag_alt_error();
		me.update_sv_by_gps();
	
		SpaceShuttle.air_data_system.update();
	},

	update_taem: func {

		me.update_pos();
		me.update_signals();
		me.update_nav();

		if ((me.MLS_locked == 1) and (me.TACAN_for == 0))
			{
			me.compute_MLS_error_set();
			}
		else if ((me.TACAN_locked == 1) and (me.TACAN_inh == 0))
			{
			me.compute_tacan_error_set();
			}	

		if (me.air_data_available == 1)
			{
			me.compute_baro_alt_error();
			}
		me.compute_drag_alt_error();	
		me.update_sv_by_gps();

		SpaceShuttle.air_data_system.update();
		
	},

	update_pos: func {
		me.m_to_lon = 1.0/(math.cos(getprop("/position/latitude-deg")*math.pi/180.0) * 110952.0);
		me.true_pos = geo.aircraft_position();
		me.nav_pos = state_vector_position();
		

	},

	update_nav: func {

		if ((TAEM_guidance_available == 1) or (TAEM_TACAN_available == 1))
			{
			me.nav_bearing_tacan = me.nav_pos.course_to(TAEM_threshold);
			me.nav_dist_tacan = me.nav_pos.distance_to(TAEM_threshold);

			me.tac_bearing_tacan = me.true_pos.course_to(TAEM_threshold) + me.tacan_offset_az_deg;
			me.tac_dist_tacan = me.true_pos.distance_to(TAEM_threshold) + me.tacan_offset_dist * 1853.0;


			me.tac_resid_range = (me.tac_dist_tacan - me.nav_dist_tacan)/1853.0;
			me.tac_resid_bearing = (me.tac_bearing_tacan - me.nav_bearing_tacan);

			#Resid max for range 1.5 Nm and 3° for bearing
			me.tac_ratio_range = math.abs(me.tac_resid_range)/1.5;
			me.tac_ratio_bearing = math.abs(me.tac_resid_bearing)/3;

			#Resid max for GPS 
			me.gps_resid = me.true_pos.direct_distance_to(me.nav_pos)/1853.0;
			me.gps_ratio = me.gps_resid/0.2;

			SpaceShuttle.tacan_system.redundancy_management();
			}

		if (me.air_data_available == 1)
			{

			#Resid max for Probes Height is 3000 feet		
			me.baro_alt_m = getprop("/instrumentation/altimeter/indicated-altitude-ft") * 0.3048;
			me.adta_resid_h = (me.baro_alt_m - me.nav_pos.alt())/0.3058;
			#me.adta_ratio_h = 0.5 * math.abs(me.adta_resid_h)/(me.nav_pos.alt() * 0.016404);
			me.adta_ratio_h = math.abs(me.adta_resid_h)/3000;
			}
	},

	update_sv_by_gps: func {

		if (me.gps_aut == 0)  {return;}

		if (me.gps_update_cycle_count < 40)
			{
			me.gps_update_cycle_count = me.gps_update_cycle_count + 1;
			return;
			}
		else
			{
			me.gps_update_cycle_count = 0;
			#print("State vector written by GPS");
			SpaceShuttle.GPS_to_prop();
			}


	},

	update_signals: func {

		me.dist_m = 0;	
		if (TAEM_guidance_available == 1)
			{
			me.dist_m = me.true_pos.distance_to(TAEM_threshold);
			}
		else if (SpaceShuttle.entry_guidance_available == 1)
			{
			me.dist_m = getprop("/fdm/jsbsim/systems/entry_guidance/remaining-distance-nm") * 1853.0;
			}
		else 
			{
			me.dist_m = 1000000.0;
			}



		var dist_norm = me.dist_m/741200.0;

		#print ("Range: ", me.dist_m, " norm: ", dist_norm);
		#print ("LOS alt: ", 48000.0 * dist_norm * dist_norm);
		
		if ((me.dist_m < 741200.0) and (me.true_pos.alt() > 48000.0 * dist_norm * dist_norm)) # TACAN range
			{
			
			if (getprop("/fdm/jsbsim/systems/navigation/tacan-available") == 1)
				{
				if (me.TACAN_locked == 0) {print("TACAN signal acquired");}	
				me.TACAN_locked = 1;
				}
			else
				{
				if (me.TACAN_locked == 1) {print("TACAN signal lost");}	
				me.TACAN_locked = 0;
				TAEM_TACAN_available = 0;
				}
			}
		else 
			{
			if (me.TACAN_locked == 1) {print("TACAN signal lost");}	
			me.TACAN_locked = 0;
			TAEM_TACAN_available = 0;
			}



		if ((TAEM_threshold.MLS_available == 1) and (me.dist_m < 37060.0) and (me.true_pos.alt() < 0.5 * me.dist_m)) # MLS range
			{

			
			var heading = getprop("/orientation/heading-deg");
			var delta_az = math.abs(heading - TAEM_threshold.heading);

			if (delta_az > 180.0) {delta_az = delta_az - 360.0;}
			if (delta_az < -180.0) {delta_az = delta_az + 360.0;}

			var channel_match = 0;

			for (var i=0; i<3; i=i+1)
				{
				if (SpaceShuttle.mls_system.receiver[i].channel == TAEM_threshold.MLS_channel)
					{
					channel_match = 1;
					}
				}


			if ((delta_az < 55.0) and (channel_match == 1))
				{
				if (getprop("/fdm/jsbsim/systems/navigation/mls-available") == 1)
					{
					if (me.MLS_locked == 0) {print("MLS signal acquired");}	
					me.MLS_locked = 1;
					}
				else
					{
					if (me.MLS_locked == 1) {print("MLS signal lost");}	
					me.MLS_locked = 0;
					}
				}
			else	
				{
				if (me.MLS_locked == 1) {print("MLS signal lost");}	
				me.MLS_locked = 0;
				}
			}
			
		else
			{
			if (me.MLS_locked == 1) {print("MLS signal lost");}	
			me.MLS_locked = 0;
			}

		if (getprop("/fdm/jsbsim/systems/navigation/air-data-available") == 1)
			{
			me.air_data_available = 1;
			}
		else
			{
			me.air_data_available = 0;
			}

		if (getprop("/fdm/jsbsim/systems/navigation/gps-available") == 1)
			{
			me.gps_available = 1;
			}	
		else
			{
			me.gps_available = 0;
			}
		

	},


	compute_tacan_error_set : func {


		me.tacan_acc_dist = SpaceShuttle.tacan_system.acc_dist(me.dist_m);
		me.tacan_offset_dist = SpaceShuttle.tacan_system.offset_range();

		me.tacan_acc_az = SpaceShuttle.tacan_system.acc_az(me.dist_m);
		me.tacan_offset_az_deg = SpaceShuttle.tacan_system.offset_az();
		me.tacan_offset_az = me.tacan_offset_az_deg * me.dist_m * math.pi/180.0;

		var bearing_rad = me.true_pos.course_to(TAEM_threshold) * math.pi/180.0;

		var cb = math.cos(bearing_rad);
		var sb = math.sin(bearing_rad);

		var cabs = math.abs(cb);
		var sabs = math.abs(sb);

		if ((me.TACAN_for == 1) or ((me.TACAN_aut == 1) and (me.tac_ratio_range < 1.0) and (me.tac_ratio_bearing < 1.0)))
			{
			me.acc_x = cabs * me.tacan_acc_dist + sabs * me.tacan_acc_az;
			me.acc_y = sabs * me.tacan_acc_dist + cabs * me.tacan_acc_az;

			me.offset_x = cb * me.tacan_offset_dist - sb *  me.tacan_offset_az;
			me.offset_y = sb * me.tacan_offset_dist + cb *  me.tacan_offset_az;

			me.accuracy_lat = me.acc_x * me.m_to_lat;
			me.accuracy_lon = me.acc_y * me.m_to_lon;

			me.offset_lat = me.offset_x * me.m_to_lat;	
			me.offset_lon = me.offset_y * me.m_to_lon;
			}
		else
			{
			me.acc_x = 5000.0;
			me.acc_y = 5000.0;
			me.offset_x = 0.0;
			me.offset_y = 0.0;
			me.offset_lat = 0.0;
			me.offset_lon = 0.0;

			me.accuracy_lat = me.acc_x * me.m_to_lat;
			me.accuracy_lon = me.acc_y * me.m_to_lon;
			
			}

	},

	compute_MLS_error_set : func {

		# according to SCOM, typical errors are 5 ft alt, 21 ft downtrack and 17 ft crosstrack

		me.accuracy_x = 20.0 * 0.3058;
		me.accuracy_y = 20.0 * 0.3058;
		me.accuracy_z = 5.0 * 0.3058;

		me.accuracy_lat = 20.0 * 0.3058 * me.m_to_lat;
		me.accuracy_lon = 20.0 * 0.3085 * me.m_to_lon;
		me.accuracy_alt = 5.0 * 0.3085 ;
		me.acc_z = 5.0;
		me.offset_z = 0.0;
		me.offset_lat = 0.0;
		me.offset_lon = 0.0;

		me.MLS_processing = 1;



	},

	compute_baro_alt_error : func {

		# FAA allows an 80 ft tolerance at 10.000 ft in barometric altitude
		# so assume the error margin is some 50 ft every 10.000 ft
		
		# in addition we have an offset of the QNH is entered wrong



		if ((me.air_data_h_for == 1) or ((me.air_data_h_aut == 1) and (me.adta_ratio_h < 1.0)))
			{
			me.offset_z = me.baro_alt_m - me.true_pos.alt();
			me.acc_z = me.true_pos.alt() * 0.005;
			}

		else
			{
			me.offset_z = 0.0;
			me.acc_z = 5000.0;
			}

		


	},

	compute_drag_alt_error: func {

		me.drag_h_resid = (me.true_pos.alt() + me.drag_h_offset - me.nav_pos.alt())/0.3058;
		me.drag_h_ratio = math.abs(me.drag_h_resid / 1640.0);

	},


};


#Final segment ( 7nm NEP / 4 Nm MEP)
var final_approach_reserve = 7.0;



var TAEM_predictor_set = {

	entry: [[0.0, 0.0], [0.0,0.0], [0.0,0.0]],
	x: 0.0,
	y: 0.0,
	angle: 0.0,
	
	update: func {

	var groundspeed = getprop("/velocities/groundspeed-kt") * 0.51444;
	var rate = getprop("/orientation/yaw-rate-degps");	

	me.x = 0;
	me.y = 0;
	me.angle = 0;

	me.evolve(groundspeed, rate);

	me.entry[0][0] = math.sqrt(me.x * me.x + me.y * me.y);
	me.entry[0][1] = math.asin(me.x/me.entry[0][0]);	
	
	me.evolve(groundspeed, rate);

	me.entry[1][0] = math.sqrt(me.x * me.x + me.y * me.y);
	me.entry[1][1] = math.asin(me.x/me.entry[1][0]);

	me.evolve(groundspeed, rate);

	me.entry[2][0] = math.sqrt(me.x * me.x + me.y * me.y);
	me.entry[2][1] = math.asin(me.x/me.entry[2][0]);




	},

	evolve: func (groundspeed, rate) {

	for (var i=0; i < 20; i=i+1)
		{
		me.x = me.x + math.sin(me.angle * math.pi/180.0) * groundspeed;
		me.y = me.y + math.cos(me.angle * math.pi/180.0) * groundspeed;
		me.angle = me.angle + rate;
		}

	},

};


# helper function to get simulated state vector position rather than 
# true aircraft position

var state_vector_position = func {
	var lat = getprop("/fdm/jsbsim/systems/navigation/state-vector/latitude-deg");
	var lon = getprop("/fdm/jsbsim/systems/navigation/state-vector/longitude-deg");
	var alt = getprop("/fdm/jsbsim/systems/navigation/state-vector/altitude-ft") * 0.3048;
	return geo.Coord.new().set_latlon(lat, lon, alt);
}


# we pick up TACAN from ~400 miles out, at that time we have no TAEM guidance but would like
# to be able to display something, so we create a 'lite' guidance target

var set_TAEM_TACAN = func {

if (TAEM_TACAN_available == 1) {return;}

# first check whether we have a valid runway / site specified

var site_string = getprop("/sim/gui/dialogs/SpaceShuttle/entry_guidance/site");
var runway_string = getprop("/sim/gui/dialogs/SpaceShuttle/entry_guidance/runway");

set_TAEM_threshold(site_string, runway_string);

if (TAEM_threshold.lat() == 0.0)
	{return;}

print("Site TACAN data available");

TAEM_TACAN_available = 1;

}


# TAEM and Entry Landing Site compuations  ###########################################################

var compute_TAEM_guidance_targets = func {

var major_mode = getprop("/fdm/jsbsim/systems/dps/major-mode");

#TG_END 
#TAEM_guidance_available = 0;
TAEM_guidance_TGINIT.TG_END = 0;

#IPHASE 
#TAEM_guidance_phase = 0;

#Reset of TGINIT and Rwy coord if it is not the first compute_TAEM iteration (Condition /runway changes etc)
#if (TAEM_guidance_TGINIT.IRESET == 1) 
	#{
	#TAEM_guidance_TGINIT.reset_TGINIT();
	#rwy_coord.reset_rwy_coord();
	#TAEM_guidance_TDAP.reset_TDAP();
	#}

#Master reset for app trainer mainly
TAEM_guidance_TGINIT.reset_TGINIT();
rwy_coord.reset_rwy_coord();
TAEM_guidance_TDAP.reset_TDAP();

#Reset of Entry functions
SpaceShuttle.EGRT_data.reset_EGRT_data();


var lat_to_m = 110952.0; 
var lon_to_m  = math.cos(getprop("/position/latitude-deg")*math.pi/180.0) * lat_to_m;
var m_to_lon = 1.0/lon_to_m;
var m_to_lat = 1.0/lat_to_m;

# first check whether we have a valid runway / site specified

var site_string = getprop("/sim/gui/dialogs/SpaceShuttle/entry_guidance/site");
var runway_string = getprop("/sim/gui/dialogs/SpaceShuttle/entry_guidance/runway");


set_TAEM_threshold(site_string, runway_string);

if (TAEM_threshold.lat() == 0.0)
	{return;}

#print("TAEM site data available");

#var pos = geo.aircraft_position();
var pos = state_vector_position();
var abort_mode = getprop("/fdm/jsbsim/systems/abort/abort-mode");

#TAEM computation conditions ( below 120 Nm// 240 km) // 305 and 603 // Avoid breakage for Abort (TAL mainly)
#(major_mode == 603)) and (abort_mode < 5)) 
#if ((pos.distance_to(TAEM_threshold) > 240000.0) and ((major_mode == 305) or (SpaceShuttle.TAEM_guidance_TGINIT.GRTLS_TAEM_INIT == 1)))
#	{
#	#setprop("/sim/messages/copilot", "No TAEM guidance to site possible.");
#	SpaceShuttle.callout.make("No TAEM guidance to site possible.", "help");
#	return;
#	}


#Entry computation conditions ( below 220 Nm// 407 km) // 304 Only
#else if ((pos.distance_to(TAEM_threshold) > 407440.0) and (major_mode == 304))
#	{
#	#setprop("/sim/messages/copilot", "No TAEM guidance to site possible.");
#	SpaceShuttle.callout.make("No Entry TAEM guidance to site possible.", "help");
#	return;
#	}

# if the threshold is defined and within range, we can construct a valid solution

#print("TAEM guidance available");

#TAEM init 
if (((major_mode == 305) or (TAEM_guidance_TGINIT.GRTLS_TAEM_INIT == 1)) and (TAEM_guidance_available == 0))
	{
	TAEM_guidance_available = 1;
	setprop("/fdm/jsbsim/systems/ap/taem/auto-taem-master",1);

	if (TAEM_guidance_TGINIT.GRTLS_TAEM_INIT == 1) 
		{
		#Tell gliding RTLS AP that we're done with alpha transition
		setprop("/fdm/jsbsim/systems/ap/grtls/taem-transition-init", 1);

		#DPS Transition to TAEM GRTLS
		setprop("/fdm/jsbsim/systems/dps/major-mode", 603);
		setprop("/fdm/jsbsim/systems/dps/major-mode-bfs", 603); #It forces the listening BFS into mm  602 when bfs display select recycled
		SpaceShuttle.ops_transition_auto("p_vert_sit");

		#print("603 test is : ", "OK");
		}
	}

#Reset of Sturns legality in case of TAEM recomputation 
setprop("/fdm/jsbsim/systems/ap/taem/s-turn-treshold", 0);
GRTLS.ISTP4 = 1;

### XHAC Function ###

#IGS gamma (Approach/Landing Handbook B3)
if (getprop("/fdm/jsbsim/inertia/weight-lbs") > 220000) {TAEM_guidance_XHAC.IGS = 2;}
else {TAEM_guidance_XHAC.IGS = 1;}

 

var entry_point_string = getprop("/fdm/jsbsim/systems/taem-guidance/entry-point-string");
var aim_point_string = getprop("/fdm/jsbsim/systems/approach-guidance/aim-point-string");

#IGI (Aimpoint Nominal / Close-in)
if (aim_point_string == "CLSE") {TAEM_guidance_XHAC.IGI = 2;}
else {TAEM_guidance_XHAC.IGI = 1;}

#NEP/MEP
if (entry_point_string == "MEP") {TAEM_guidance_TGINIT.MEP = 1;}
else {TAEM_guidance_TGINIT.MEP = 0;}


#TAEM init functions for Initial condition changes (IGS/IGI dependant)
TAEM_guidance_XHAC.compute_TGXHAC();

#Compute IGS dependant linear function constants (Altitude Ref)
TAEM_guidance_TGCOMP.compute_HREF_C();


	#print("IGS: ", TAEM_guidance_XHAC.IGS, " IGI: ", TAEM_guidance_XHAC.IGI, " MEP: ", TAEM_guidance_TGINIT.MEP);
	#print("XA: " ,TAEM_guidance_XHAC.XA, " TGGS: ", TAEM_guidance_XHAC.TGGS);
	#print("HFTC: ", TAEM_guidance_XHAC.HFTC, " HALI: ", TAEM_guidance_XHAC.HALI, " HMEP: ", TAEM_guidance_XHAC.HMEP);
	#print("XFTC: ", TAEM_guidance_XHAC.XFTC, " XALI: ", TAEM_guidance_XHAC.XALI, " XMEP: ", TAEM_guidance_XHAC.XMEP);


#A/L distances
if (TAEM_guidance_TGINIT.MEP == 0) 
	{
	TAEM_guidance_XHAC.XHAC = TAEM_guidance_XHAC.XFTC;
	TAEM_guidance_XHAC.HHAC = TAEM_guidance_XHAC.HFTC;
	} 
else 
	{
	TAEM_guidance_XHAC.XHAC = TAEM_guidance_XHAC.XMEP;
	TAEM_guidance_XHAC.HHAC = TAEM_guidance_XHAC.HMEP;
	} 

#Boundary between HDG and Pre Final Phase
TAEM_guidance_XHAC.RPRED3 = TAEM_guidance_XHAC.DR3 - TAEM_guidance_XHAC.XHAC;

#print("XHAC is: ", TAEM_guidance_XHAC.XHAC, " HHAC is: ", TAEM_guidance_XHAC.HHAC, " RPRED3 is: ", TAEM_guidance_XHAC.RPRED3);


### Autoland XHAC dependant variables (done in compute XHAC) ###

#Z coordinate of pull up circle (Hk = 1700 + radius * Cos(OGS))
#TAEM_guidance_TGCOMP.HK = TAEM_guidance_TGCOMP.XC + TAEM_guidance_TGCOMP.R_flare * math.cos(TAEM_guidance_XHAC.GAMMA * 0.0174533);

#X coordinate of exponential decay / center of circular close in Aim Point adjustement
if (TAEM_guidance_XHAC.XA == -6500) 
	{
	TAEM_guidance_XHAC.XE = TAEM_guidance_XHAC.XE + 1000;
	TAEM_guidance_XHAC.XK = TAEM_guidance_XHAC.XK + 1000;
	}

#Sigma convergence coeff of exponential function
#TAEM_guidance_TGCOMP.sigma_EXP = TAEM_guidance_TGCOMP.HD / (TAEM_guidance_TGCOMP.TGGAMMAREF2 - math.tan(math.asin((TAEM_guidance_TGCOMP.XE - TAEM_guidance_TGCOMP.XK) / TAEM_guidance_TGCOMP.R_flare)));

#print("XE is : ", TAEM_guidance_XHAC.XE, "XK is : ", TAEM_guidance_XHAC.XK);


### Initial Ground Track Computation (GTP) for Runway centered frame ###

var runway_dir_vec = [math.sin(TAEM_threshold.heading * math.pi/180.0), math.cos(TAEM_threshold.heading * math.pi/180.0)];


##print (TAEM_threshold.heading, " ", runway_dir_vec[0], " ", runway_dir_vec[1]);

TAEM_WP_2.set_latlon(TAEM_threshold.lat() - m_to_lat * runway_dir_vec[1] * (-TAEM_guidance_XHAC.XHAC * 0.3048),  TAEM_threshold.lon() - m_to_lon *runway_dir_vec[0] * (-TAEM_guidance_XHAC.XHAC * 0.3048));
TAEM_WP_2.set_alt(TAEM_guidance_XHAC.HHAC * 0.3048);

# store the aim point

TAEM_AP.set_latlon(TAEM_threshold.lat() - m_to_lat * runway_dir_vec[1] * (-TAEM_guidance_XHAC.XA * 0.3048),  TAEM_threshold.lon() - m_to_lon *runway_dir_vec[0] * (-TAEM_guidance_XHAC.XA * 0.3048));
TAEM_AP.set_alt(TAEM_threshold.alt());


# construct the edges of the HUD virtual runway

TAEM_rwy_nl.set_latlon(TAEM_threshold.lat(), TAEM_threshold.lon(), TAEM_threshold.alt());
TAEM_rwy_nl.apply_course_distance( (TAEM_threshold.heading - 180.0), 200.0); 
TAEM_rwy_nl.apply_course_distance( (TAEM_threshold.heading - 90.0), 50.0); 

TAEM_rwy_nr.set_latlon(TAEM_threshold.lat(), TAEM_threshold.lon(), TAEM_threshold.alt());
TAEM_rwy_nr.apply_course_distance( (TAEM_threshold.heading - 180.0), 200.0); 
TAEM_rwy_nr.apply_course_distance( (TAEM_threshold.heading + 90.0), 50.0); 

TAEM_rwy_fl.set_latlon(TAEM_threshold.lat(), TAEM_threshold.lon(), TAEM_threshold.alt());
TAEM_rwy_fl.apply_course_distance( TAEM_threshold.heading, TAEM_threshold.rwy_length); 
TAEM_rwy_fl.apply_course_distance( (TAEM_threshold.heading - 90.0), 50.0);

TAEM_rwy_fr.set_latlon(TAEM_threshold.lat(), TAEM_threshold.lon(), TAEM_threshold.alt());
TAEM_rwy_fr.apply_course_distance( TAEM_threshold.heading, TAEM_threshold.rwy_length); 
TAEM_rwy_fr.apply_course_distance( (TAEM_threshold.heading + 90.0), 50.0);

# now construct the center of the HAC

var approach_dir = pos.course_to(TAEM_WP_2);
var approach_vec = [math.sin(approach_dir * math.pi/180.0), math.cos(approach_dir * math.pi/180.0)];

#print (approach_dir, " ", approach_vec[0], " ", approach_vec[1]);

var runway_perp_vec = [math.sin((TAEM_threshold.heading + 90) * math.pi/180.0), math.cos((TAEM_threshold.heading + 90) * math.pi/180.0)];

var approach_dot_rwyperp = SpaceShuttle.dot_product_2d(approach_vec, runway_perp_vec);
var approach_mode = getprop("/fdm/jsbsim/systems/taem-guidance/approach-mode-string");

if ((approach_mode == "OVHD") and (approach_dot_rwyperp < 0.0)) # we need to flip direction
	{
	runway_perp_vec = [-runway_perp_vec[0], -runway_perp_vec[1]];
	}
if ((approach_mode == "STRT") and (approach_dot_rwyperp > 0.0)) # we need to flip direction
	{
	runway_perp_vec = [-runway_perp_vec[0], -runway_perp_vec[1]];
	}

#HAC entry direction saved for HAC shrink logic
TAEM_guidance_XHAC.rwy_perp_vec_lat = m_to_lat * runway_perp_vec[1] * 0.3048;
TAEM_guidance_XHAC.rwy_perp_vec_lon = m_to_lon * runway_perp_vec[0] * 0.3048;

#print(runway_perp_vec[0], " ", runway_perp_vec[1]);


#Initial HAC center is based on Final Radius (Rf = 14000 feet /4300m/ 2.3Nm) 
TAEM_guidance_XHAC.construct_hac_center();

# now, determine how much we have to turn and store the info
var turn_degrees = math.abs(approach_dir - TAEM_threshold.heading);

if ((turn_degrees < 180.0) and (approach_mode == "OVHD"))
	{
	turn_degrees = 360.0 - turn_degrees;
	}
else if ((turn_degrees > 180.0) and (approach_mode == "STRT"))
	{
	turn_degrees = 360.0 - turn_degrees;
	}

#Turn degrees stored for WP 1 construction depending on Turn degrees as radial will vary with it.

#2.1 to 2.7 Initial Hac Turn Angle
TAEM_guidance_GTP.PSHA = turn_degrees; #Inital PSHA for first pass

#print("PSHA is: ",TAEM_guidance_GTP.PSHA);




# WP-1 is the tangent on the HAC which leads, after the turn, to a tangent pointing at the runway

var wp1_vec = [-approach_vec[1], approach_vec[0]];
var wp1_dot_rwy = SpaceShuttle.dot_product_2d(wp1_vec, runway_dir_vec);



if ((wp1_dot_rwy < 0.0) and (approach_mode == "OVHD"))
	{wp1_vec = [-wp1_vec[0], -wp1_vec[1]];}
if ((wp1_dot_rwy > 0.0) and (approach_mode == "STRT"))
	{wp1_vec = [-wp1_vec[0], -wp1_vec[1]];}



#2.8 Inital HAC radius at tangent HAC point
TAEM_guidance_GTP.RTURN = TAEM_guidance_TGINIT.RF + 0.093 * math.pow(TAEM_guidance_GTP.PSHA, 2); 
TAEM_guidance_GTP.RCIRC = TAEM_guidance_GTP.RTURN; #Spec 50

#print("RTURN is: ", TAEM_guidance_GTP.RTURN);


# the WP 1 aim HAC entry point depends of the angle we have to fly at entry of the HAC (radius from 4.5 Nm to 2.3 Nm) 
TAEM_WP_1.set_latlon(TAEM_HAC_center.lat() + m_to_lat * wp1_vec[1] * TAEM_guidance_GTP.RTURN * 0.3048, TAEM_HAC_center.lon() + m_to_lon * wp1_vec[0] * TAEM_guidance_GTP.RTURN * 0.3048);



#Formula based on TAEM guidance equations that takes into account the spiral inward (From 4.3 Nm max radius to 2.3Nm mini radius once NEP is reached)
#2.9 Initial in HAC distance computation
TAEM_guidance_GTP.RPRED2 = (TAEM_guidance_TGINIT.RF * TAEM_guidance_GTP.PSHA + 0.093 * math.pow(TAEM_guidance_GTP.PSHA,3.0) / 3) * 0.0174533 - TAEM_guidance_XHAC.XHAC; 

TAEM_WP_1.turn_deg = TAEM_guidance_GTP.PSHA;
TAEM_WP_1.approach_dir = approach_dir;
TAEM_WP_1.distance_to_runway_m = TAEM_guidance_GTP.RPRED2 * 0.3048;
TAEM_WP_1.hac_radius = TAEM_guidance_GTP.RTURN * 0.3048;
#print("RPRED2 is: ", TAEM_guidance_GTP.RPRED2);


# now figure out what direction to turn onto the HAC (YSGN)
var test_vec = [runway_perp_vec[1], -runway_perp_vec[0]];



var turn_direction = "right";
TAEM_guidance_GTP.YSGN = 1;

if (SpaceShuttle.dot_product_2d(runway_dir_vec, test_vec) > 0.0)
	{
	turn_direction = "left";
	TAEM_guidance_GTP.YSGN = -1;
	}

TAEM_WP_1.turn_direction = turn_direction;
#setprop("/fdm/jsbsim/systems/ap/taem/set-bank-target", TAEM_guidance_GTP.YSGN * 30.0);



# tell light manager to switch runway lights on

SpaceShuttle.light_manager.set_theme("RUNWAY");

#ACQ phase 
TAEM_guidance_phase = 1;

#Property nodes for improved perfomances (Shared by Entry and TAEM guidance)

#Geodetic 
TAEM_jsbsim.lat_geo_node = props.globals.getNode("/fdm/jsbsim/systems/navigation/state-vector/latitude-deg", 1);
TAEM_jsbsim.lon_geo_node = props.globals.getNode("/fdm/jsbsim/systems/navigation/state-vector/longitude-deg", 1);
TAEM_jsbsim.radius_geo_node = props.globals.getNode("/fdm/jsbsim/position/radius-to-vehicle-ft", 1);

#TAEM
TAEM_jsbsim.mach_node = props.globals.getNode("/fdm/jsbsim/velocities/mach", 1);
TAEM_jsbsim.vtrue_node = props.globals.getNode("/fdm/jsbsim/velocities/vtrue-fps", 1);
TAEM_jsbsim.vh_node = props.globals.getNode("/fdm/jsbsim/velocities/u-fps", 1);
TAEM_jsbsim.altitude_qnh_node = props.globals.getNode("/position/altitude-ft", 1);
TAEM_jsbsim.qbar_node = props.globals.getNode("/fdm/jsbsim/aero/qbar-psf", 1);
TAEM_jsbsim.hdot_node = props.globals.getNode("/fdm/jsbsim/velocities/v-down-fps", 1);
TAEM_jsbsim.hdot_dot_node = props.globals.getNode("/fdm/jsbsim/accelerations/hdotdot-ft_s2", 1);
TAEM_jsbsim.weight_node = props.globals.getNode("/fdm/jsbsim/inertia/mass-slugs", 1);
TAEM_jsbsim.gamma_node = props.globals.getNode("/fdm/jsbsim/flight-path/gamma-deg", 1);

#TDAP
TAEM_jsbsim.phi_node = props.globals.getNode("/fdm/jsbsim/systems/navigation/state-vector/roll-deg", 1);
TAEM_jsbsim.theta_node = props.globals.getNode("/fdm/jsbsim/systems/navigation/state-vector/pitch-deg", 1);
TAEM_jsbsim.alpha_node = props.globals.getNode("/fdm/jsbsim/aero/alpha-deg", 1);
TAEM_jsbsim.yaw_rate_node = props.globals.getNode("/fdm/jsbsim/velocities/r-rad_sec", 1);
#TAEM_jsbsim.roll_rate_node = props.globals.getNode("/fdm/jsbsim/velocities/p-rad_sec", 1);
TAEM_jsbsim.pitch_rate_node = props.globals.getNode("/fdm/jsbsim/velocities/q-rad_sec", 1);
TAEM_jsbsim.nz_node = props.globals.getNode("/fdm/jsbsim/accelerations/Nz", 1);
TAEM_jsbsim.auto_pitch_node = props.globals.getNode("/fdm/jsbsim/systems/ap/automatic-pitch-control", 1);
TAEM_jsbsim.auto_roll_node = props.globals.getNode("/fdm/jsbsim/systems/ap/automatic-roll-control", 1);


#Entry and PRTLS (EGRT targeting function) only
if ((major_mode == 304) or (major_mode == 601))
	{
	TAEM_jsbsim.vi_node = props.globals.getNode("/fdm/jsbsim/velocities/eci-velocity-mag-fps", 1);
	TAEM_jsbsim.lod_node = props.globals.getNode("/fdm/jsbsim/aero/coefficient/LD-full-ratio-rockwell-interpolated", 1);
	TAEM_jsbsim.load_factor_node = props.globals.getNode("/fdm/jsbsim/accelerations/a-pilot-ft_sec2", 1);
	TAEM_jsbsim.drag_accel_node = props.globals.getNode("/fdm/jsbsim/systems/entry_guidance/aero-drag-deceleration-fts", 1);
	TAEM_jsbsim.cd_node = props.globals.getNode("/fdm/jsbsim/aero/coefficient/CD-basic", 1);
	}


#Runway / ECEF frame conversion matrix (TAEM and Entry guidance)

#TAEM runway treshold parameters
rwy_coord.TLATD = TAEM_threshold.lat() * 0.0174533;
rwy_coord.TLONG = TAEM_threshold.lon() * 0.0174533;
rwy_coord.RAZ = TAEM_threshold.heading * 0.0174533;
rwy_coord.RTE1 = TAEM_threshold.elevation;

var cos_TLATD = math.cos(rwy_coord.TLATD);
var sin_TLATD = math.sin(rwy_coord.TLATD);
var cos_TLONG = math.cos(rwy_coord.TLONG);
var sin_TLONG = math.sin(rwy_coord.TLONG);
var cos_RAZ = math.cos(rwy_coord.RAZ);
var sin_RAZ = math.sin(rwy_coord.RAZ);

#Conversion matrix coeff
rwy_coord.REC[0] = -cos_RAZ * sin_TLATD * cos_TLONG - sin_RAZ * sin_TLONG;
rwy_coord.REC[1] = sin_RAZ * sin_TLATD * cos_TLONG - cos_RAZ * sin_TLONG;
rwy_coord.REC[2] = -cos_TLATD * cos_TLONG;
rwy_coord.REC[3] = -cos_RAZ * sin_TLATD * sin_TLONG + sin_RAZ * cos_TLONG;
rwy_coord.REC[4] = sin_RAZ * sin_TLATD * sin_TLONG + cos_RAZ * cos_TLONG;
rwy_coord.REC[5] = -cos_TLATD * sin_TLONG;
rwy_coord.REC[6] = cos_RAZ * cos_TLATD;
rwy_coord.REC[7] = -sin_RAZ * cos_TLATD;
rwy_coord.REC[8] = -sin_TLATD;

#Inverse
rwy_coord.REC_inv = SpaceShuttle.matrix_inverse(rwy_coord.REC);

#Earth radius factor at treshold position
var RLSTC = math.sqrt(math.pow(cos_TLATD, 2) + SpaceShuttle.EGRT_data.RX22 * math.pow(sin_TLATD, 2)); 

#Runway threshold ECEF vector 
var radius_to_treshold = SpaceShuttle.EGRT_data.RECX / RLSTC + rwy_coord.RTE1;
rwy_coord.RLS = SpaceShuttle.ECEF_converter(TAEM_threshold.lat(), TAEM_threshold.lon(), radius_to_treshold);

#print(" Langing site earth radius_km is  : ", SpaceShuttle.EGRT_data.RECX / RLSTC * 0.00030);


#Start of Main TAEM Loop (Including GRTLS TAEM after alpha transition)
if ((TAEM_loop_running == 0) and ((major_mode == 305) or (TAEM_guidance_TGINIT.GRTLS_TAEM_INIT == 1)))
	{
	#Flag for Rwy redesig
	TAEM_loop_running = 1;
	TAEM_guidance_TGINIT.IRESET = 1;	

	#Start of TAEM Main Loop
	TAEM_TGEXEC();

	print("TAEM loop is: ", TAEM_loop_running);
	}

#Start of GRTLS loop (PRTLS, alpha recovery to alpha transition)
else if ((GRTLS.PRTLS == "ON") or (GRTLS.CONT == "ON"))
	{
	#Start of PRTLS loop
	if (GRTLS.INIT_PASS == 0) 
		{
		#PRTLS phase at first call
		TAEM_guidance_TGINIT.IPHASE = 8; 
		GREXEC();
		}
	}

#Start of Main Entry loop 
else if (major_mode == 304)
	{
	if (SpaceShuttle.EGD.START_LOOP == 0) {SpaceShuttle.EGEXEC();}
	}

}

# the central TAEM / GRTLS guidance execution loop #########################################################

var GREXEC = func { 

#Exit the loop at TAEM init (stage.nas) or if OPS 6 is manually exited
if ((TAEM_guidance_TGINIT.GRTLS_TAEM_INIT == 1) or (getprop("/fdm/jsbsim/systems/dps/ops") != 6)) 
	{
	TAEM_guidance_TGINIT.GRTLS_TAEM_INIT = 1;
	#Initial NZC for Iphase 1 (avoid a transient with last NZC from Nz hold phase)
	TAEM_guidance_TGNZC.NZC = 0;
	#TAEM_guidance_TGNZC.NZC = TAEM_jsbsim.NZ - math.cos(TAEM_jsbsim.THETAR) / SpaceShuttle.MIDVAL(math.cos(TAEM_jsbsim.PHIR), 0.5, 1) * (TAEM_jsbsim.V_true / (TAEM_jsbsim.V_true + TAEM_guidance_TDAP.VCO));                       
	#TAEM_guidance_TGNZC.NZC = SpaceShuttle.MIDVAL(TAEM_guidance_TGNZC.NZC, -1, 1);
	TAEM_guidance_TGINIT.IPHASE = 1;

	#print("CORNZC  is: ", TAEM_guidance_TGNZC.NZC);	
	return;
	}

#Computations for coordinates in Rwy frame and stored jsbsim variables
TAEM_input_parameters_calculations();

#XGAC done in initial TAEM computations

#GTP / Handles distance computations
TAEM_GTP();

#Energy computation (Iphase 4 only)
if (TAEM_guidance_TGINIT.IPHASE < 5) {GRCOMP();}

#Transition
GRTRN();

#Nz hold guidance
if (TAEM_guidance_TGINIT.IPHASE == 5) {GRNZC();}
else {GRALPC();}


if (TAEM_guidance_TGINIT.IPHASE != 8) 
	{
	#Speedbrake
	GRSBC();

	#Roll 
	GRPHIC();
	}


#TDAP function for Pitch/Roll/SB output to FCS
if (TAEM_guidance_TGINIT.IPHASE < 6) {TAEM_TDAP();}


#Init flag (GRINIT)
if (GRTLS.INIT_PASS == 0) 
	{
	GRTLS.INIT_PASS = 1;

	#Nz hold coefficients
	if(GRTLS.CONT == "OFF")
		{
		GRTLS.SMNZ1 = GRTLS.SMNZC1;
		GRTLS.SMNZ2 = GRTLS.SMNZC2;
		}
	else
		{
		GRTLS.SMNZ1 = GRTLS.SMNZC1A;
		GRTLS.SMNZ2 = GRTLS.SMNZC2A;
		GRTLS.SMNZC3 = GRTLS.SMNZC3A;	
		GRTLS.SMNZC4 = GRTLS.SMNZC4A;
		GRTLS.SMNZ2L = GRTLS.SMNZ2LA;	
		}
		
	GRTLS.NZSW = GRTLS.NZSW1; #Transition to Nz Hold
	GRTLS.compute_EN_C();

	#if (GRTLS.CONT == "OFF") {GRTLS.ECAL = "OFF";}
	#else
	#	{
	#	GRTLS.ECAL = "ECAL_INIT";		
	#	if (GRTLS.S_CONT_YAW = "ON") {GRTLS.ECAL = "ON";}
	#	}

	}

#Various functions
SpaceShuttle.body_flap_management();

settimer( func {GREXEC(); }, TAEM_guidance_TGINIT.DTG);

#print("Update rate is : ", TAEM_guidance_TGINIT.DTG, " ECAL is : ", GRTLS.ECAL, " CONT is : ", GRTLS.CONT, " PRTLS is : ", GRTLS.PRTLS);
};



var TAEM_TGEXEC = func {
#Order for TAEM functions: XHAC / GTP / TGCOMP / TGTRAN / TGNZC / TGSBC / TGPHIC then Nz Pitch and Roll commanded into jsbsim

#print("Treshold elevation is : ", rwy_coord.RTE1);

if (TAEM_guidance_available == 0)
	{
	TAEM_loop_running = 0;
	return;
	} 

if (TAEM_guidance_TGINIT.AL_END == 1)  {return;}

#Computations for coordinates in Rwy frame and stored jsbsim variables
TAEM_input_parameters_calculations();

#XGAC done in initial TAEM computations

#GTP / Handles distance computations
TAEM_GTP();

#TGCOMP for reference functions
TAEM_TGCOMP();

#TGTRAN for phase boundaries and limits
TAEM_TGTRAN();

#TGNZC 
TAEM_TGNZC();

#TGSBC 
TAEM_TGSBC();

#TGPHIC for Roll commanded
TAEM_TGPHIC();

#TDAP function for Pitch/Roll/SB output to FCS
TAEM_TDAP();

#print ("Treshold elevation is : ", rwy_coord.RTE1);

#First loop flag
if (TAEM_guidance_TGINIT.LOOP == 0) 
	{
	
	TAEM_guidance_TGINIT.LOOP = 1;
	TAEM_guidance_TGINIT.IPHASE = 1; #Switch from Alpha transition to ACQ for GRTLS
	TAEM_guidance_TGINIT.DTG = 0.48; #Switch from previous GRTLS rate
	#TAEM threshold elevation based on geo nasal function (more precise for a better Autoland alt QFE variable) 
	settimer( func {

	#settimer to avoid a nil case due to terrain not loaded
	rwy_coord.RTE1 = geo.elevation(TAEM_threshold.lat(), TAEM_threshold.lon()) * 3.280840;

	}, 20.0);
	
	}


SpaceShuttle.body_flap_management();
TAEM_predictor_set.update();
update_HUD_symbology(rwy_coord.POS);

if (TAEM_guidance_phase == 2) {
	var error_word = "";

	if (abs(TAEM_guidance_TGCOMP.HERROR) < 1000) error_word = "on";
	else {
		if (abs(TAEM_guidance_TGCOMP.HERROR) >= 1000 and abs(TAEM_guidance_TGCOMP.HERROR) <= 2000) error_word = "slightly ";

		if (TAEM_guidance_TGCOMP.HERROR < 0) error_word = error_word~"high";
		else if (TAEM_guidance_TGCOMP.HERROR > 0) error_word = error_word~"low";
	}

	if (TAEM_guidance_GTP.PSHA < 180 and !TAEM_MCC_flags.one_eighty) {
		SpaceShuttle.callout.make("Atlantis, "~error_word~" at the 180", "real");
		TAEM_MCC_flags.one_eighty = 1;
	}
	if (TAEM_guidance_GTP.PSHA < 90 and !TAEM_MCC_flags.ninety) {
		SpaceShuttle.callout.make("Atlantis, "~error_word~" at the 90", "real");
		TAEM_MCC_flags.ninety = 1;
	}
}


#TAEM A/L main loop timer
settimer( func {TAEM_TGEXEC(); }, TAEM_guidance_TGINIT.DTG);

#print("Update rate is : ", TAEM_guidance_TGINIT.DTG);
};



# TAEM guidance and flight control input parameter calculations (Appendix) - runway frame computations / JSBsim hash values  ###########################################################

var TAEM_input_parameters_calculations = func {

#JSB sim values that need to be stored 
TAEM_jsbsim.mach = TAEM_jsbsim.mach_node.getValue();
TAEM_jsbsim.V_true = TAEM_jsbsim.vtrue_node.getValue();
TAEM_jsbsim.VH = TAEM_jsbsim.vh_node.getValue();
TAEM_jsbsim.QBAR = TAEM_jsbsim.qbar_node.getValue();
TAEM_jsbsim.HDOT = TAEM_jsbsim.hdot_node.getValue();
TAEM_jsbsim.HDOT_DOT = TAEM_jsbsim.hdot_dot_node.getValue();
TAEM_jsbsim.weight = TAEM_jsbsim.weight_node.getValue();
TAEM_jsbsim.gamma = TAEM_jsbsim.gamma_node.getValue();

#TDAP
TAEM_jsbsim.PHIR = TAEM_jsbsim.phi_node.getValue() * 0.0174533; #Radians
TAEM_jsbsim.THETAR = TAEM_jsbsim.theta_node.getValue() * 0.0174533;
TAEM_jsbsim.ALPHA = TAEM_jsbsim.alpha_node.getValue();
TAEM_jsbsim.R = TAEM_jsbsim.yaw_rate_node.getValue() * 57.29578; #Deg
#TAEM_jsbsim.P = TAEM_jsbsim.roll_rate_node.getValue() * 57.29578; #Deg
TAEM_jsbsim.Q = TAEM_jsbsim.pitch_rate_node.getValue() * 57.29578; #Deg
TAEM_jsbsim.NZ = TAEM_jsbsim.nz_node.getValue(); #g
TAEM_jsbsim.auto_pitch = TAEM_jsbsim.auto_pitch_node.getValue();
TAEM_jsbsim.auto_roll = TAEM_jsbsim.auto_roll_node.getValue();

#JSB sim Pre-Final and Autoland only variables
if ((TAEM_guidance_TGINIT.IPHASE == 3) or (TAEM_guidance_TGINIT.IPHASE == 7))
	{
	TAEM_jsbsim.V_equivalent = getprop("/fdm/jsbsim/velocities/ve-fps");
	TAEM_jsbsim.V_ground = getprop("/fdm/jsbsim/velocities/vg-fps");
	TAEM_jsbsim.H_agl = getprop("/position/altitude-agl-ft") - 17; #Gear extended, CoG is 17 feet above ground when WOW
	}

#Geodetic to ECEF vector
TAEM_jsbsim.LAT_geo = TAEM_jsbsim.lat_geo_node.getValue();
TAEM_jsbsim.LON_geo = TAEM_jsbsim.lon_geo_node.getValue();
TAEM_jsbsim.radius_geo = TAEM_jsbsim.radius_geo_node.getValue();
TAEM_jsbsim.XYZE = SpaceShuttle.ECEF_converter(TAEM_jsbsim.LAT_geo, TAEM_jsbsim.LON_geo, TAEM_jsbsim.radius_geo);

#Geodetic to Rwy frame position vector
rwy_coord.XYZ_rwy = SpaceShuttle.matrix_vector_product(rwy_coord.REC, SpaceShuttle.subtract_vector(TAEM_jsbsim.XYZE, rwy_coord.RLS));


#Hqfe compensated for nominal touchdown point (2500 feet) sloped runway
TAEM_jsbsim.H_qfe = TAEM_jsbsim.altitude_qnh_node.getValue() - (rwy_coord.RTE1 + 29 + 2500 * math.tan(TAEM_threshold.slope * 0.0174533));


#H_qfe with a correction for sloped runway during final flare phase once threshold is passed // On threshold, 29 feet of diff 
	#if ((TAEM_guidance_TGINIT.PMODE > 3) and (rwy_coord.X_sign == 1))
	#	{
	#	var slope_correction = math.tan(TAEM_threshold.slope * 0.0174533) * rwy_coord.X;
	#	TAEM_jsbsim.H_qfe = TAEM_jsbsim.altitude_qnh_node.getValue() - (rwy_coord.RTE1 + 29 + slope_correction);
	#
	#	#print ("slope correction is : ", slope_correction);
	#	}
	#else {TAEM_jsbsim.H_qfe = TAEM_jsbsim.altitude_qnh_node.getValue() - (rwy_coord.RTE1 + 29);}

#print ("H qfe is : ", TAEM_jsbsim.H_qfe, " X ECEF is : ", TAEM_jsbsim.XYZE[0], " Y ECEF is : ", TAEM_jsbsim.XYZE[1], " Z ECEF is : ", TAEM_jsbsim.XYZE[2]);


#State vector 
rwy_coord.POS = state_vector_position();
TAEM_guidance_GTP.RPREDF = rwy_coord.POS.distance_to(TAEM_threshold) * 3.280839;

#Sign of coordinates (Used one time in TAEM)
	#Sign of X
	#rwy_coord.X_sign = math.sgn(rwy_coord.XYZ_rwy[0]);
	#Sign of Y
	#rwy_coord.Y_sign = math.sgn(rwy_coord.XYZ_rwy[1]);

#X Position
rwy_coord.X = rwy_coord.XYZ_rwy[0];

#Y Position 
rwy_coord.Y = rwy_coord.XYZ_rwy[1];


#Velocity
if ((TAEM_guidance_TGINIT.LOOP == 0) and (GRTLS.INIT_PASS == 0)) #First pass instantaneous V not relevant
	{
	rwy_coord.X_dot = 0;
	rwy_coord.X_dot = 0;
	}
else 
	{
	rwy_coord.X_dot = (rwy_coord.X - rwy_coord.X_last) / TAEM_guidance_TGINIT.DTG;
	rwy_coord.Y_dot = (rwy_coord.Y - rwy_coord.Y_last) / TAEM_guidance_TGINIT.DTG;
	}
	
#Store last position
rwy_coord.X_last = rwy_coord.X;
rwy_coord.Y_last = rwy_coord.Y;

#Course relative to centerline
rwy_coord.PSD = math.atan2(rwy_coord.Y_dot,rwy_coord.X_dot) * 57.29578;

#print(" X is: ", rwy_coord.X , " Rpred is: ", TAEM_guidance_GTP.RPREDF  ," Y is: ", rwy_coord.Y , " X_dot is: ", rwy_coord.X_dot, " Y_dot is: ", rwy_coord.Y_dot, " PSD is: ", rwy_coord.PSD, " X sign is: ", rwy_coord.X_sign, " Y sign is: ", rwy_coord.Y_sign);

	
};


# GRTLS function (Transition, Alpha and Nz commanded, ECAL E/W) ###########################################################


var GRCOMP = func {

#2.12 to 2.18 ECAL/RTLS energy computations
TAEM_guidance_TGCOMP.EOW = TAEM_jsbsim.H_qfe + math.pow(TAEM_jsbsim.V_true,2) / 64.348;

#RTLS and CONT (Nominal TAEM coefficient for WIP purpose)
if ((GRTLS.PRTLS == "ON") or (GRTLS.CONT == "ON"))
	{
	TAEM_guidance_TGCOMP.DRPRED = TAEM_guidance_GTP.RPRED + TAEM_guidance_XHAC.XALI;
	GRTLS.EN = TAEM_guidance_TGCOMP.EN_C1_IEL_1 + TAEM_guidance_TGCOMP.EN_C2_IEL_1 * TAEM_guidance_TGCOMP.DRPRED;
	GRTLS.ES = TAEM_guidance_TGTRAN.ES1 + TAEM_guidance_TGTRAN.EDRS * TAEM_guidance_TGCOMP.DRPRED;
	GRTLS.EST = GRTLS.ES - 10000;
	GRTLS.EMEP = TAEM_guidance_TGTRAN.EMEP_C1_IEL_1 + TAEM_guidance_TGTRAN.EMEP_C2_IEL_1 * TAEM_guidance_TGCOMP.DRPRED;
	GRTLS.EMAX = GRTLS.EN + 8000;
	GRTLS.EMIN = GRTLS.EN - 4000;
	}

#ECAL CONT (not used)
#else 
#	{
#	GRTLS.EN = GRTLS.EN_C1 + GRTLS.EN_C2 * TAEM_guidance_GTP.RPRED;
#	GRTLS.ES = GRTLS.ES_C1 + GRTLS.ES_C2 * TAEM_guidance_GTP.RPRED;
#	GRTLS.EST = GRTLS.ES - 10000;
#	GRTLS.EMEP = GRTLS.EMEP_C1 + GRTLS.EMEP_C2 * TAEM_guidance_GTP.RPRED;
#	GRTLS.EMAX = GRTLS.EN + 8000;
#	GRTLS.EMIN = GRTLS.EN - 4000;
#	}


#print("EOW is: ", TAEM_guidance_TGCOMP.EOW);
#print("ES is: ", GRTLS.ES, " EN is : ", GRTLS.EN, " EMEP is : ", GRTLS.EMEP);
#print("EOW/ES is: ", TAEM_guidance_TGCOMP.EOW / GRTLS.ES, " EOW/EN is : ", TAEM_guidance_TGCOMP.EOW / GRTLS.EN, " EOW / EMEP is : ", TAEM_guidance_TGCOMP.EOW / GRTLS.EMEP);

};


#Energy computation for Spec 54 EOW (dist in Nm)
var alternate_COMP = {

	EN_alternate: [0,0],
	ES_alternate: [0,0],
	EMEP_alternate: [0,0],
	DRPRED_alternate_site: [0,0],

	energy: func(dist, alternate_number) {
	
		#For Alternate site, HAC is based on 20° path and NEP for AL interface (5.8 Nmish)
		me.EN_alternate[alternate_number - 1] = TAEM_guidance_TGCOMP.EN_C1_IEL_1 + TAEM_guidance_TGCOMP.EN_C2_IEL_1 * me.DRPRED_alternate_site[alternate_number - 1];
		me.ES_alternate[alternate_number - 1] = TAEM_guidance_TGTRAN.ES1 + TAEM_guidance_TGTRAN.EDRS * me.DRPRED_alternate_site[alternate_number - 1];
		me.EMEP_alternate[alternate_number - 1] = TAEM_guidance_TGTRAN.EMEP_C1_IEL_1 + TAEM_guidance_TGTRAN.EMEP_C2_IEL_1 * me.DRPRED_alternate_site[alternate_number - 1];	

		#print ("EN alternate site 1 is :", me.EN_alternate[0], " ES alternate site 1 is : ", me.ES_alternate[0], " EMEP alternate site 1 is : ", me.EMEP_alternate[0]);
		#print ("EN alternate site 2 is :", me.EN_alternate[1], " ES alternate site 2 is : ", me.ES_alternate[1], " EMEP alternate site 2 is : ", me.EMEP_alternate[1]);
	},


	distance: func(alternate_number) {
		
		if (alternate_number == 1)
			{
			var pos = SpaceShuttle.state_vector_position();
			#Direct distance to landing site
			me.DRPRED_alternate_site[alternate_number - 1] = pos.distance_to(SpaceShuttle.landing_alt_site_one) * M2NM * 6076.12;
			#print("Alternate distance 1 in Nm is : ", me.DRPRED_alternate_site[alternate_number - 1] / 6076.12);

			#Acquisition turn
			var PHAVG = 20 + 10 * TAEM_jsbsim.mach;
			PHAVG = SpaceShuttle.MIDVAL(PHAVG, 50, 70);
			var RTAC = (TAEM_jsbsim.VH * TAEM_jsbsim.V_true) / (32.174 * math.tan(PHAVG * 0.0174533));
			var ARCAC = RTAC * math.abs(delta_azimuth_entry(SpaceShuttle.landing_alt_site_one)) * 0.0174533;
			
			me.DRPRED_alternate_site[alternate_number - 1] = me.DRPRED_alternate_site[alternate_number - 1] + ARCAC;
			#print("Alternate distance 1 with acq turn in Nm is : ", me.DRPRED_alternate_site[alternate_number - 1] / 6076.12);
			}

		else
			{
			var pos = SpaceShuttle.state_vector_position();
			#Direct distance to landing site
			me.DRPRED_alternate_site[alternate_number - 1] = pos.distance_to(SpaceShuttle.landing_alt_site_two) * M2NM * 6076.12;
			#print("Alternate distance 2 in Nm is : ", me.DRPRED_alternate_site[alternate_number - 1] / 6076.12);

			#Acquisition turn
			var PHAVG = 20 + 10 * TAEM_jsbsim.mach;
			PHAVG = SpaceShuttle.MIDVAL(PHAVG, 50, 70);
			var RTAC = (TAEM_jsbsim.VH * TAEM_jsbsim.V_true) / (32.174 * math.tan(PHAVG * 0.0174533));
			var ARCAC = RTAC * math.abs(delta_azimuth_entry(SpaceShuttle.landing_alt_site_two)) * 0.0174533;
			
			me.DRPRED_alternate_site[alternate_number - 1] = me.DRPRED_alternate_site[alternate_number - 1] + ARCAC;
			#print("Alternate distance 2 with acq turn in Nm is : ", me.DRPRED_alternate_site[alternate_number - 1] / 6076.12);
			}

	},


};

var GRTRN = func {

#Iphase 8 (PRTLS) / Iphase 6 (Alpha recovery) / Iphase 5 (Nz Hold) / Iphase 4 (Alpha transition) / Done in ap.xml for now

var alpha_recovery = getprop("/fdm/jsbsim/systems/ap/grtls/alpha-recovery-active");
var nz_hold = getprop("/fdm/jsbsim/systems/ap/grtls/Nz-hold-active");
var alpha_transition = getprop("/fdm/jsbsim/systems/ap/grtls/alpha-transition-active");
var GRALPR_RTLS = getprop("fdm/jsbsim/systems/ap/grtls/mach-alpha-schedule-RTLS");

#PRTLS to Alpha recovery
if ((alpha_recovery == 1) and (TAEM_guidance_TGINIT.IPHASE != 6)) 
	{
	TAEM_guidance_TGINIT.IPHASE = 6;
	#TAEM_Speedbrake_control();

	#Timestep lowered
	TAEM_guidance_TGINIT.DTG = 0.96;

	#First QBAR computation for GRNZC loop
	TAEM_guidance_TGINIT.QBARF = TAEM_jsbsim.QBAR;

	#Max dynamical pressure (Max EAS 471 kts for CA)
	if (GRTLS.CONT == "ON") {TAEM_guidance_TGNZC.QBMXNZ = 400;} #CONT or CONT after RTLS
	else if ((GRTLS.CONT == "OFF") and (GRTLS.PRTLS == "ON")) {TAEM_guidance_TGNZC.QBMXNZ = 300;} #RTLS

	#Roll target is zero for Alpha recovery (done in TGPHIC)
	#TAEM_guidance_TGPHIC.PHIC_AT = 0;
	#setprop("fdm/jsbsim/systems/ap/grtls/roll-tgt-deg", TAEM_guidance_TGPHIC.PHIC_AT);
	}

#Alpha recovery to Nz Hold transition
else if ((TAEM_guidance_TGINIT.IPHASE == 6) and (TAEM_jsbsim.NZ > GRTLS.NZSW))
	{
	TAEM_guidance_TGINIT.IPHASE = 5;
	setprop("/fdm/jsbsim/systems/ap/grtls/Nz-hold-active", 1);
	setprop("/fdm/jsbsim/systems/ap/grtls/alpha-recovery-active", 0);

	#Last Alpha is stored to limit initial AOA during Nz Hold
	setprop("fdm/jsbsim/systems/ap/grtls/alpha-cmd", TAEM_jsbsim.ALPHA);

	#Max ECAL bank
	if (GRTLS.ECAL == "ON") {TAEM_guidance_TGPHIC.PHILIMIT = 70;}
	}

#Nz hold to Alpha transition (4.1)
else if ((TAEM_guidance_TGINIT.IPHASE == 5) and (((TAEM_jsbsim.ALPHA > GRALPR_RTLS) and (GRTLS.CONT == "OFF") and (-TAEM_jsbsim.HDOT > GRTLS.HDTRN)) or ((GRTLS.CONT == "ON") and (-TAEM_jsbsim.HDOT > GRTLS.HDTRNA)) or ((GRTLS.ECAL == "ON") and (-TAEM_jsbsim.HDOT > GRTLS.HDTBNK) and (-TAEM_jsbsim.HDOT_DOT < 0)))) 
	{
	TAEM_guidance_TGINIT.IPHASE = 4;

	#ECAL roll logic activation
	if (GRTLS.ECAL == "ON")
		{
		#GRTLS.IRLPTF = "ON";
		#GRTLS.IBNK = "ON";
		}
	
	setprop("/fdm/jsbsim/systems/ap/grtls/alpha-transition-active", 1);
	setprop("/fdm/jsbsim/systems/ap/grtls/Nz-hold-active", 0);

	#Alpha at transition
	GRTLS.ALPTRN_INIT = TAEM_jsbsim.ALPHA;
	#print("alpha at transition is : ", GRTLS.ALPTRN_INIT);

	#Max Bank Angle for GRPHIC (45 for RTLS / 70 for ECAL)
	if ((GRTLS.PRTLS == "ON") and (GRTLS.CONT == "OFF")) {TAEM_guidance_TGPHIC.PHILIMIT = 45;}
	else if (GRTLS.CONT == "ON")  {TAEM_guidance_TGPHIC.PHILIMIT = 70;}
	}

else if ((TAEM_guidance_TGINIT.IPHASE == 4) and (TAEM_guidance_TGINIT.GRTLS_TAEM_INIT == 1)) 
	{
	#Initial NZC for Iphase 1 (avoid a transient with last NZC from Nz hold phase)
	TAEM_guidance_TGNZC.NZC = 0;
	#TAEM_guidance_TGNZC.NZC = TAEM_jsbsim.NZ - math.cos(TAEM_jsbsim.THETAR) / SpaceShuttle.MIDVAL(math.cos(TAEM_jsbsim.PHIR), 0.5, 1) * (TAEM_jsbsim.V_true / (TAEM_jsbsim.V_true + TAEM_guidance_TDAP.VCO));                       
	#TAEM_guidance_TGNZC.NZC = SpaceShuttle.MIDVAL(TAEM_guidance_TGNZC.NZC, -1, 1);
	TAEM_guidance_TGINIT.IPHASE = 1;

	#print("CORNZC  is: ", TAEM_guidance_TGNZC.NZC);
	}



#Reference Angle of Attack computation for Alpha transition
if (TAEM_guidance_TGINIT.IPHASE == 4)
	{
	if (GRTLS.ECAL == "ON")
		{
		GRTLS.EN_DELTA = TAEM_guidance_TGCOMP.EOW - GRTLS.EN;
		GRTLS.ENLIMHI = GRTLS.EMAX;
		GRTLS.ENLIMLO = GRTLS.EMIN;
		GRTLS.ALPHA_MIN = getprop("fdm/jsbsim/systems/ap/grtls/mach-alpha-schedule-min");
		GRTLS.ALPHA_MAX = getprop("fdm/jsbsim/systems/ap/grtls/mach-alpha-schedule-max");

		#ECAL Energy management (3.3.1 to 3.3.9)
		if (GRTLS.EN_DELTA < 0)
			{
			if ((math.abs(TAEM_guidance_GTP.DPSAC) < GRTLS.DPSAC2) and (GRTLS.EN_DELTA < GRTLS.ENLIMLO)) {GRTLS.EN_ALPHA_BIAS = GRTLS.ENALPL;}
			else {GRTLS.EN_ALPHA_BIAS = GRTLS.EN_BIAS1;}
			}

		else if ((GRTLS.EN_DELTA >= 0) and (GRTLS.EN_DELTA < GRTLS.ENLIMHI))
			{
			if (GRTLS.EN_DELTA < GRTLS.EN_DELTA_OLD) {GRTLS.EN_ALPHA_BIAS = GRTLS.EN_BIAS2;}	
			else {GRTLS.EN_ALPHA_BIAS = GRTLS.EN_BIAS3;}	
			}
		else if (GRTLS.EN_DELTA >= GRTLS.ENLIMHI)
			{
			if (TAEM_jsbsim.NZ > GRTLS.TLFMX1) {GRTLS.EN_ALPHA_BIAS = GRTLS.EN_BIAS4;}
			else {GRTLS.EN_ALPHA_BIAS = GRTLS.ENALPU;}
			}

		GRTLS.EN_DELTA_OLD = GRTLS.EN_DELTA;
		GRTLS.GRALPR = MIDVAL(GRTLS.GRALPR + GRTLS.EN_ALPHA_BIAS, GRTLS.ALPHA_MIN, GRTLS.ALPHA_MAX);
		print("Delta EN is ", GRTLS.EN_DELTA, " EN alpha Bias is : ", GRTLS.EN_ALPHA_BIAS, " Alpha commanded is ", GRTLS.GRALPR); 

		#3.3.10 IPPRTF ECAL logic pitch protection flag to be done
		}
	
	else if ((GRTLS.ECAL == "OFF") and (GRTLS.CONT == "ON")) {GRTLS.GRALPR = getprop("fdm/jsbsim/systems/ap/grtls/mach-alpha-schedule-CONT");}
	else if ((GRTLS.PRTLS == "ON") and (GRTLS.CONT == "OFF")) {GRTLS.GRALPR = GRALPR_RTLS;}
	}

#print("GRTLS Iphase is : ", TAEM_guidance_TGINIT.IPHASE, " GRALPR is : ", GRTLS.GRALPR);
};


var GRNZC = func {

#Test function for Alpha command converted into NZC with qbar and Nz filter (not used in ap.xml)
if ((TAEM_guidance_TGINIT.IPHASE == 6) or (TAEM_guidance_TGINIT.IPHASE == 4))
	{

	#Qbar filtered as a function of QBAR and QBAR dot in a computation cycle 
	TAEM_guidance_TGCOMP.QBARD = SpaceShuttle.MIDVAL(TAEM_guidance_TGCOMP.CQG * (TAEM_jsbsim.QBAR - TAEM_guidance_TGINIT.QBARF), -5, 5);
	TAEM_guidance_TGINIT.QBARF = TAEM_guidance_TGINIT.QBARF + TAEM_guidance_TGCOMP.QBARD * TAEM_guidance_TGINIT.DTG;
	#print("Q bar filtered is : ", TAEM_guidance_TGINIT.QBARF);


	#Factor to be used for Qbar limit for AP
	TAEM_guidance_TGINIT.QBD = 0.68113143 * TAEM_guidance_TGINIT.QBD + 0.31886857 * TAEM_guidance_TGCOMP.QBARD;	

	#Initial NZC based on Alpha required
	TAEM_guidance_TGNZC.DNZC = ((GRTLS.ALPCMD - TAEM_jsbsim.ALPHA) * GRTLS.ALPE1 - TAEM_jsbsim.Q * R2D) * GRTLS.ALPE2;
	#print("NZC alpha is: ", TAEM_guidance_TGNZC.DNZC);

	#Minimum q bar profile (Max is calculated in GRTRN)
	TAEM_guidance_TGNZC.MXQBWT = SpaceShuttle.MIDVAL(TAEM_guidance_TGNZC.QBWT2 + TAEM_guidance_TGNZC.QBMSL2 * (TAEM_jsbsim.mach - TAEM_guidance_TGNZC.QMACH2), TAEM_guidance_TGNZC.QBWT2, TAEM_guidance_TGNZC.QBWT3);
	TAEM_guidance_TGNZC.QBLL = TAEM_guidance_TGNZC.MXQBWT * TAEM_jsbsim.weight; #Slugs
	#TAEM_guidance_TGNZC.QBMNNZ = TAEM_guidance_TGNZC.QBLL / math.max(math.cos(TAEM_jsbsim.PHIR), TAEM_guidance_TDAP.CPMIN);
	TAEM_guidance_TGNZC.QBMNNZ = 100;
	#print("QBMNZ is ", TAEM_guidance_TGNZC.QBMNNZ, " QBMXNZ is ", TAEM_guidance_TGNZC.QBMXNZ);
	
	#Upper/Lower Nz limits based on Max/Min QBAR
	TAEM_guidance_TGNZC.QBNZUL = -TAEM_guidance_TGNZC.QBG2 * (TAEM_guidance_TGNZC.QBG1 * (TAEM_guidance_TGNZC.QBMNNZ - TAEM_guidance_TGINIT.QBARF) - TAEM_guidance_TGINIT.QBD);
	TAEM_guidance_TGNZC.QBNZLL = -TAEM_guidance_TGNZC.QBG2 * (TAEM_guidance_TGNZC.QBG1 * (TAEM_guidance_TGNZC.QBMXNZ - TAEM_guidance_TGINIT.QBARF) - TAEM_guidance_TGINIT.QBD);

	#QBAR NZ limitations
	TAEM_guidance_TGNZC.DNZCL = MIDVAL(TAEM_guidance_TGNZC.DNZC, TAEM_guidance_TGNZC.QBNZLL, TAEM_guidance_TGNZC.QBNZUL);
	
	#NZCdot parameter and final NZC
	TAEM_guidance_TGNZC.DNZCD = SpaceShuttle.MIDVAL((TAEM_guidance_TGNZC.DNZCL - TAEM_guidance_TGNZC.NZC) * TAEM_guidance_TGCOMP.CQG, -TAEM_guidance_TGNZC.DNZCDL, TAEM_guidance_TGNZC.DNZCDL);
	TAEM_guidance_TGNZC.NZC = TAEM_guidance_TGNZC.NZC + TAEM_guidance_TGNZC.DNZCD * TAEM_guidance_TGINIT.DTG;
	#print("DNZCD is: ", TAEM_guidance_TGNZC.DNZCD);

	#Final NZC filter for max g's allowed IPHASE dependant
	TAEM_guidance_TGNZC.NZC = SpaceShuttle.MIDVAL(TAEM_guidance_TGNZC.NZC, -2, 2);

	#setprop("/fdm/jsbsim/systems/ap/grtls/Nz-cmd", TAEM_guidance_TGNZC.NZC + 1);

	#print("NZC is: ", TAEM_guidance_TGNZC.NZC);

	}

#Nz commanded during Nz Hold
else if (TAEM_guidance_TGINIT.IPHASE == 5)
	{
	GRTLS.SMNZ1 = GRTLS.SMNZC3 * GRTLS.SMNZ1;
	GRTLS.SMNZ2 = GRTLS.SMNZ2 - GRTLS.SMNZC4;
	if (GRTLS.SMNZ2 < GRTLS.SMNZ2L) {GRTLS.SMNZ2 = GRTLS.SMNZ2L;}

	#Nz Commanded (GRNZC1 normaly is targeted for RTLS / replaced by Nz targeted for Contigency)
	if (GRTLS.CONT == "OFF") {TAEM_guidance_TGNZC.NZC = GRTLS.GRNZC1 - GRTLS.SMNZ1 - GRTLS.SMNZ2 + GRTLS.DGRNZ;}
	else {TAEM_guidance_TGNZC.NZC = (GRTLS.DGRNZT - 1) - GRTLS.SMNZ1 - GRTLS.SMNZ2;}

	#If bank activated, 1G addition has to be changed with cos theta / cos phi.
	#setprop("/fdm/jsbsim/systems/ap/grtls/Nz-cmd", math.min(TAEM_guidance_TGNZC.NZC + 1, GRTLS.DGRNZT));

	#Final NZC filter for max g's allowed IPHASE dependant
	TAEM_guidance_TGNZC.NZC = SpaceShuttle.MIDVAL(TAEM_guidance_TGNZC.NZC, -3, 3);

	#print("NZC is: ", TAEM_guidance_TGNZC.NZC, " Exp lead term is: ", GRTLS.SMNZ1, " Linear lead term is: ", GRTLS.SMNZ2);
	}

};


var GRALPC = func {

#Alpha recovery phase
if (TAEM_guidance_TGINIT.IPHASE == 6)
	{
	if (GRTLS.ALPCMD == 0)
		{
		if (GRTLS.CONT == "OFF") {GRTLS.ALPCMD = GRTLS.ALPREC;}
		else {GRTLS.ALPCMD = 58;}
		}

	if (-TAEM_jsbsim.HDOT < GRTLS.HDMAX) {GRTLS.HDMAX = -TAEM_jsbsim.HDOT;}
	else if ((getprop("/fdm/jsbsim/systems/ap/grtls/Nz-tgt") == 0) and (-TAEM_jsbsim.HDOT > GRTLS.HDMAX))
		{
		#RTLS
		if (GRTLS.CONT == "OFF")
			{
			#1.3
			GRTLS.DGRNZ = MIDVAL((GRTLS.HDNOM - GRTLS.HDMAX) * GRTLS.DHDNZ, GRTLS.DHDLL, GRTLS.DHDUL);
			GRTLS.DGRNZT = GRTLS.GRNZC1 + GRTLS.DGRNZ + 1;

			#Transition to Iphase 5
			GRTLS.NZSW = GRTLS.NZSW + GRTLS.DGRNZ;
			}


		#Contingency
		else
			{
			#1.4
			GRTLS.DGRNZT = MIDVAL(GRTLS.DNZB - GRTLS.HDMAX * GRTLS.DHDNZ, GRTLS.DNZMIN, GRTLS.DNZMAX);
			GRTLS.ITGTNZ = "OFF";

			#print("Nz ECAL target before prebank is: ", GRTLS.DGRNZT);
			
			#1.4.3 Thermal Nz constraint to be done
			
			#Nz Hold phase Pre Bank for ECAL (Deactivated)
			if ((GRTLS.ECAL == "ON") and (math.abs(TAEM_guidance_GTP.DPSAC) > GRTLS.DPSAC1) and (GRTLS.DGRNZT < GRTLS.DNZMX1)) 
				{
				#GRTLS.ITGTNZ = "ON";
				#GRTLS.DGRNZT = GRTLS.DGRNZT + GRTLS.DNZ1;
				#print("Prebank flag is : ", GRTLS.ITGTNZ ," Nz ECAL target with prebank is: ", GRTLS.DGRNZT);
				}

			#1.4.6 WIP
			#GRTLS.SMNZ1 = GRTLS.ZDT1 * GRTLS.DGRNZT;

			GRTLS.DGRNZ = GRTLS.DGRNZT - GRTLS.GRNZC1A - 1;
			GRTLS.NZSW = GRTLS.GRNZC1A - GRTLS.SMNZ1 - GRTLS.SMNZ2 + 1; 
			}

		#Transition to Iphase 5
		#GRTLS.NZSW = GRTLS.NZSW + GRTLS.DGRNZ;

		#Nz Target stored
		setprop("/fdm/jsbsim/systems/ap/grtls/Nz-tgt", GRTLS.DGRNZT);

		#print("Hdot max is : ", GRTLS.HDMAX, " Nz target is ", GRTLS.DGRNZT, " DGRNZ is : ", GRTLS.DGRNZ, " Transition Nz to phase 5 is : ", GRTLS.NZSW);
		}
	}

else if (TAEM_guidance_TGINIT.IPHASE == 4) 
	{
	#2.1 Init
	if (GRTLS.IGRA == 0) 
		{
		GRTLS.ALPCMD = GRTLS.ALPTRN_INIT;
		GRTLS.IGRA = 1;

		#print("Alpha commanded at alpha transition is : ", GRTLS.ALPCMD);
		}

	#3.1 Smoothed AOA command
	else if (GRTLS.IGRA == 1)
		{
		if ((GRTLS.ECAL == "ON ") and (TAEM_jsbsim.NZ < GRTLS.TLFMX1)) {GRTLS.DGRALP = MIDVAL(GRTLS.GRALPR - GRTLS.ALPTRN_INIT, GRTLS.GRALL, GRTLS.GRALU2);}
		else {GRTLS.DGRALP = MIDVAL(GRTLS.GRALPR - GRTLS.ALPTRN_INIT, GRTLS.GRALL, GRTLS.GRALU);}

		GRTLS.ALPCMD = GRTLS.ALPCMD + GRTLS.DGRALP;

		if (((GRTLS.DGRALP < 0) and (GRTLS.ALPCMD <= GRTLS.GRALPR)) or ((GRTLS.DGRALP > 0) and (GRTLS.ALPCMD >= GRTLS.GRALPR)))
			{
			GRTLS.ALPCMD = GRTLS.GRALPR;
			GRTLS.IGRA = 2;
			}

		#3.4 IPPRTF logic for max alpha G load depending to be done
		}
	
	#4.1 IPPRTF logic for max alpha G load depending to be done

	#4 AOA commanded after transition (can't exceed alpha at Alpha transition init)
	else if (GRTLS.IGRA == 2) {GRTLS.ALPCMD = GRTLS.GRALPR;}
		
	}

setprop("fdm/jsbsim/systems/ap/grtls/alpha-cmd", GRTLS.ALPCMD);

#print("Alpha commanded is : ", GRTLS.ALPCMD, " DGRALP is :", GRTLS.DGRALP, " IGRA is :", GRTLS.IGRA);
};




var GRSBC = func {

#Speebrakes open above qbar 20 (80.6) and ramps back down to 65 below mach 4
if (TAEM_guidance_TGINIT.IPHASE != 4)
	{
	if ((TAEM_guidance_TGINIT.IPHASE == 6) and (GRTLS.CONT == "ON")) {return;}

	if ((TAEM_jsbsim.QBAR > 20) and (GRTLS.SPDBRK_FLAG == 0)) 
		{
		TAEM_guidance_TGSBC.DSBC_AT = GRTLS.GRSBL1;
		GRTLS.SPDBRK_FLAG = 1;
		}
	}


else if (TAEM_guidance_TGINIT.IPHASE == 4) 
	{
		if ((TAEM_jsbsim.QBAR > 20) and (GRTLS.SPDBRK_FLAG == 0)) 
			{
			TAEM_guidance_TGSBC.DSBC_AT = GRTLS.GRSBL1;
			GRTLS.SPDBRK_FLAG = 1;
			}
		

		else if ((TAEM_jsbsim.mach < 4) and (GRTLS.SPDBRK_FLAG == 1))  
			{
			TAEM_guidance_TGSBC.DSBC_AT = MIDVAL(GRTLS.MACHSB * TAEM_jsbsim.mach + GRTLS.MACHSBI, GRTLS.GRSBL1, GRTLS.GRSBL2);
			#GRTLS.SPDBRK_FLAG = 2;
			}

	#S-turns (98.6°)
	#else if ((GRTLS.ISTP4 == 0) and (GRTLS.SPDBRK_FLAG > 0))  
	#	{
	#	TAEM_guidance_TGSBC.DSBC_AT = TAEM_guidance_TGSBC.DSBLIM;
	#	GRTLS.SPDBRK_FLAG = 0;
	#	}
	}

#Start of SB control loop 
if (TAEM_guidance_TGINIT.SB_LOOP == 0) {TAEM_Speedbrake_control();}

#print("Speedbrakes setting is : ", TAEM_guidance_TGSBC.DSBC_AT);
};


var GRPHIC = func {

#Alpha transition during intact abort (RTLS)
if ((GRTLS.CONT == "OFF") and (TAEM_guidance_TGINIT.IPHASE == 4))
	{
	#1.1 
	if ((GRTLS.CONT == "OFF") and (GRTLS.ISTP4 == 0))
		{
		#1.2 
		if (TAEM_guidance_TGCOMP.EOW < GRTLS.EST)	
			{
			GRTLS.ISTP4 = 1;
			GRTLS.DPSAC_INIT = "OFF";
			}
		}

	#1.3
	else 
		{
		if ((TAEM_guidance_TGCOMP.EOW > GRTLS.ES) and (TAEM_jsbsim.mach < GRTLS.MSW3) and (GRTLS.ISTP4 == 1))
			{
			#1.3.1
			#Alpha transition S-turns flag
			GRTLS.ISTP4 = 0;	
			SpaceShuttle.callout.make("Initiating S-turn to deplete energy!", "info");
			setprop("/fdm/jsbsim/systems/ap/taem/s-turn-init",1);

			#Turn away form the HAC
			TAEM_guidance_TGTRAN.S_sign = -TAEM_guidance_GTP.YSGN;

			#Conditionnal for less than 90° HAC depending on PSD (course)
			TAEM_guidance_TGTRAN.SPSI = TAEM_guidance_TGTRAN.S_sign * geo.normdeg180(rwy_coord.PSD);
			if ((TAEM_guidance_TGTRAN.SPSI < 0) and (TAEM_guidance_GTP.PSHA < 90)) {TAEM_guidance_TGTRAN.S_sign = -TAEM_guidance_TGTRAN.S_sign;}
			}
		}

	#1.4
	if (GRTLS.ISTP4 == 0) {TAEM_guidance_TGPHIC.PHIC = TAEM_guidance_TGTRAN.S_sign * TAEM_guidance_TGPHIC.PHILIMIT;}
	else {TAEM_guidance_TGPHIC.PHIC = 2.5 * TAEM_guidance_GTP.DPSAC;}

	#1.5
	TAEM_guidance_TGPHIC.PHIC_AT = SpaceShuttle.MIDVAL(TAEM_guidance_TGPHIC.PHIC, -TAEM_guidance_TGPHIC.PHILIMIT, TAEM_guidance_TGPHIC.PHILIMIT);
	}


#Intact abort before Alpha transition or Contingency abort without ECAL
#else if ((((GRTLS.PRTLS == "ON") and (TAEM_guidance_TGINIT.IPHASE != 4)) or ((GRTLS.CONT == "ON") and (GRTLS.ECAL == "OFF"))) and (TAEM_guidance_TGPHIC.PHIC_AT != 0))
#	{
	#2
#	TAEM_guidance_TGPHIC.PHIC = 0;
#	TAEM_guidance_TGPHIC.PHIC_AT = TAEM_guidance_TGPHIC.PHIC;
#	}

#ECAL high energy S-turns
else if ((GRTLS.ECAL == "ON") and (TAEM_guidance_TGINIT.IPHASE == 4) and ((TAEM_guidance_TGCOMP.EOW > GRTLS.ES) or (GRTLS.ISTP4 == 0)))
	{

	#1.2 
	if (TAEM_guidance_TGCOMP.EOW < GRTLS.EST)	
		{
		GRTLS.ISTP4 = 1;
		GRTLS.DPSAC_INIT = "OFF";
		}

	else if (TAEM_guidance_TGCOMP.EOW > GRTLS.ES)
		{

		if (GRTLS.ISTP4 == 1)
			{
			SpaceShuttle.callout.make("Initiating S-turn to deplete energy!", "info");
			setprop("/fdm/jsbsim/systems/ap/taem/s-turn-init",1);
			}

		#1.3.1
		#Alpha transition S-turns flag
		GRTLS.ISTP4 = 0;

		#Turn away form the HAC
		TAEM_guidance_TGTRAN.S_sign = -TAEM_guidance_GTP.YSGN;

		#Conditionnal for less than 90° HAC depending on PSD (course)
		TAEM_guidance_TGTRAN.SPSI = TAEM_guidance_TGTRAN.S_sign * geo.normdeg180(rwy_coord.PSD);
		if ((TAEM_guidance_TGTRAN.SPSI < 0) and (TAEM_guidance_GTP.PSHA < 90)) {TAEM_guidance_TGTRAN.S_sign = -TAEM_guidance_TGTRAN.S_sign;}

		#S-turn ECAL bank limitations 1.3.2
		if (GRTLS.DPSAC_INIT == "OFF")
			{
			GRTLS.DPSACI = TAEM_guidance_GTP.DPSAC;
			GRTLS.DPSAC_INIT = "ON";
			}

		TAEM_guidance_TGPHIC.PHIC_AT = GRTLS.PHISTN * math.sgn(GRTLS.DPSACI);

		GRTLS.DPSACLMT = math.min((GRTLS.DPHISLP * TAEM_jsbsim.mach) + GRTLS.DPHIINT, GRTLS.DPSACLMT_MAX);
		GRTLS.DPSACT = -GRTLS.DPSACLMT * math.sgn(GRTLS.DPSACI);
		if (GRTLS.DPSACT >= 0)
			{
			if (TAEM_guidance_GTP.DPSAC >= GRTLS.DPSACT) {GRTLS.DPSAC_INIT = "OFF";}
			}
		else
			{
			if (TAEM_guidance_GTP.DPSAC <= GRTLS.DPSACT) {GRTLS.DPSAC_INIT = "OFF";}
			}

		#3.3 Ecal Roll protection logic for
		if ((TAEM_guidance_TGINIT.IPHASE == 4) and (-TAEM_jsbsim.HDOT < GRTLS.HDTRN) and (-TAEM_jsbsim.HDOT_DOT < 0)) {(GRTLS.IRLPTF = "ON");}
		#else if ((TAEM_guidance_TGINIT.IPHASE == 4) and (-TAEM_jsbsim.HDOT > GRTLS.HDTRN)) {(GRTLS.IRLPTF = "OFF");}
		else {GRTLS.IRLPTF = "OFF";}
		#print("Roll protection flag is : ", GRTLS.IRLPTF);

		if (GRTLS.IRLPTF == "ON")
			{
			#3.4.1 to be done / missing values
			
			var TEMP1 = TAEM_jsbsim.THETAR;
			if (debug.isnan(TEMP1) == 1)  {TEMP1 = 0;}

			var TEMP2 = math.cos(TEMP1) / math.max(GRTLS.DNZMX2 - math.abs(TAEM_jsbsim.NZ), 0.1);
			#print("Temp value for phinz2 is : ", TEMP2);

			if (debug.isnan(TEMP2) == 0) {GRTLS.PHINZ2 = R2D * math.acos(MIDVAL(TEMP2, 0.8,-0.8));}
			else {GRTLS.PHINZ2 = TAEM_guidance_TGPHIC.PHILIMIT;}

			if (math.abs(TAEM_guidance_TGPHIC.PHIC_AT) > math.abs(GRTLS.PHINZ2)) {TAEM_guidance_TGPHIC.PHIC_AT = GRTLS.PHINZ2 * math.sgn(TAEM_guidance_TGPHIC.PHIC_AT);}

			#print("PHINZ 2 value is : ", GRTLS.PHINZ2);
			}


		#print("DPSACI is : ", GRTLS.DPSACI, " DPSACLMT is : ", GRTLS.DPSACLMT, " DPSACT is : ", GRTLS.DPSACT, " DPSAC init is ", GRTLS.DPSAC_INIT);
		}

	}

#ECAL bank logic
else if ((GRTLS.ECAL == "ON") and ((TAEM_guidance_TGINIT.IPHASE > 4) or ((TAEM_guidance_TGINIT.IPHASE == 4) and (TAEM_guidance_TGCOMP.EOW < GRTLS.ES) and (GRTLS.ISTP4 == 1))))
	{
	#Roll logic (I bank on in transition function)
	if ((-TAEM_jsbsim.HDOT > GRTLS.HDTBNK) and (TAEM_guidance_TGINIT.IPHASE != 6) and (-TAEM_jsbsim.HDOT_DOT > 0) and (GRTLS.IBNK == "OFF")) 
		{
		GRTLS.IBNK = "ON";
		setprop("fdm/jsbsim/systems/ap/grtls/ecal-ibnk", 1);
		#print("IBNK is : ", GRTLS.IBNK);
		}

	if (GRTLS.IBNK == "ON")
		{
		TAEM_guidance_TGPHIC.PHIC = 2.5 * TAEM_guidance_GTP.DPSAC;
		TAEM_guidance_TGPHIC.PHIC_AT = SpaceShuttle.MIDVAL(TAEM_guidance_TGPHIC.PHIC, -TAEM_guidance_TGPHIC.PHILIMIT, TAEM_guidance_TGPHIC.PHILIMIT);

		#Roll protection
		if ((TAEM_guidance_TGINIT.IPHASE == 4) and (-TAEM_jsbsim.HDOT < GRTLS.HDTRN) and (-TAEM_jsbsim.HDOT_DOT < 0)) {(GRTLS.IRLPTF = "ON");}
		#else if ((TAEM_guidance_TGINIT.IPHASE == 4) and (-TAEM_jsbsim.HDOT > GRTLS.HDTRN)) {(GRTLS.IRLPTF = "OFF");}
		else {GRTLS.IRLPTF = "OFF";}
		#print("Roll protection flag is : ", GRTLS.IRLPTF);

		if (GRTLS.IRLPTF == "ON")
			{
			#3.4.1 to be done / missing values
			
			var TEMP1 = TAEM_jsbsim.THETAR;
			if (debug.isnan(TEMP1) == 1)  {TEMP1 = 0;}

			var TEMP2 = math.cos(TEMP1) / math.max(GRTLS.DNZMX2 - math.abs(TAEM_jsbsim.NZ), 0.1);
			#print("Temp value for phinz2 is : ", TEMP2);

			if (debug.isnan(TEMP2) == 0) {GRTLS.PHINZ2 = R2D * math.acos(MIDVAL(TEMP2, 0.8,-0.8));}
			else {GRTLS.PHINZ2 = TAEM_guidance_TGPHIC.PHILIMIT;}

			if (math.abs(TAEM_guidance_TGPHIC.PHIC_AT) > math.abs(GRTLS.PHINZ2)) {TAEM_guidance_TGPHIC.PHIC_AT = GRTLS.PHINZ2 * math.sgn(TAEM_guidance_TGPHIC.PHIC_AT);}

			#print("PHINZ 2 value is : ", GRTLS.PHINZ2);
			}

		}

	
	#No Nz Hold Prebank
	else if ((GRTLS.IBNK == "OFF") and (GRTLS.ITGTNZ == "OFF")) {TAEM_guidance_TGPHIC.PHIC_AT = 0;}

	#Nz hold Prebank // Activated once hdot is less negative
	else if ((GRTLS.IBNK == "OFF") and (GRTLS.ITGTNZ == "ON")) 
		{
		if ((-TAEM_jsbsim.HDOT > -10000) and (GRTLS.ITGTNZ == "ON"))
			{
			TAEM_guidance_TGPHIC.PHIC_AT = 20 * math.sgn(TAEM_guidance_GTP.DPSAC);
			#SpaceShuttle.callout.make("Nz hold prebank", "help");
			#GRTLS.ITGTNZ = "OFF";
			}
		}
	}

#Contigency abort with ECAL deactivated (No Bank)
else if ((GRTLS.ECAL == "OFF") and (GRTLS.CONT == "ON")) {TAEM_guidance_TGPHIC.PHIC_AT = 0;}

setprop("fdm/jsbsim/systems/ap/grtls/roll-tgt-deg", TAEM_guidance_TGPHIC.PHIC_AT);
#print("Roll commanded is: ", TAEM_guidance_TGPHIC.PHIC_AT, " Phi limit is : ", TAEM_guidance_TGPHIC.PHILIMIT);

};


# TAEM range predictions - GTP - TAEM Ground Track Computations ###########################################################

var TAEM_GTP = func {

#HAC X center relative coordinates
TAEM_guidance_GTP.XCIR = TAEM_guidance_XHAC.XHAC - rwy_coord.X;

#Direct distance in Pre Final phase and A/L mode
if (((TAEM_guidance_TGINIT.IPHASE == 3) and (TAEM_guidance_GTP.XCIR < 2000)) or (TAEM_guidance_TGINIT.PMODE > 0)) {TAEM_guidance_GTP.RPRED = TAEM_guidance_GTP.RPREDF;}

#ACQ/HDG/S-Turns
else
	{
	
	#1.1 to 1.3  Distance to Tangency HAC Point (Wp1)
	TAEM_guidance_GTP.YCIR = TAEM_guidance_GTP.YSGN * TAEM_guidance_TGINIT.RF - rwy_coord.Y;
	TAEM_guidance_GTP.RCIRC = math.sqrt(math.pow(TAEM_guidance_GTP.XCIR, 2) + math.pow(TAEM_guidance_GTP.YCIR, 2));

	if (TAEM_guidance_GTP.RCIRC > TAEM_guidance_GTP.RTURN) {TAEM_guidance_GTP.RTAN = math.sqrt(math.pow(TAEM_guidance_GTP.RCIRC, 2) - math.pow(TAEM_guidance_GTP.RTURN, 2));}
	else {TAEM_guidance_GTP.RTAN = 0;}


	#2.1 to 2.9 Delta Azimuth and Hac Turn Angle
	TAEM_guidance_GTP.PSC = math.atan2(TAEM_guidance_GTP.YCIR, TAEM_guidance_GTP.XCIR);
	TAEM_guidance_GTP.PST = geo.normdeg180((TAEM_guidance_GTP.PSC - TAEM_guidance_GTP.YSGN * math.atan2(TAEM_guidance_GTP.RTURN, TAEM_guidance_GTP.RTAN)) * 57.29578);

	#Daz (Delta azimuth function for PRTLS)
	if (TAEM_guidance_TGINIT.IPHASE == 8) {TAEM_guidance_GTP.DPSAC = delta_azimuth_entry(TAEM_WP_1);}
	else {TAEM_guidance_GTP.DPSAC = geo.normdeg180(TAEM_guidance_GTP.PST - rwy_coord.PSD);}
	setprop("/fdm/jsbsim/systems/taem-guidance/delta-azimuth-deg", TAEM_guidance_GTP.DPSAC);
	#print("TAEM delta Azimuth is: ", TAEM_guidance_GTP.DPSAC);

	
	#HTA
	TAEM_guidance_GTP.PSHAN = -TAEM_guidance_GTP.PST * TAEM_guidance_GTP.YSGN;
	if (((TAEM_guidance_GTP.PSHA > 271) or (TAEM_guidance_GTP.PSHAN < -1) or (TAEM_guidance_GTP.YSGN != math.sgn(rwy_coord.Y))) and (TAEM_guidance_GTP.PSHA > 90)) {TAEM_guidance_GTP.PSHAN = TAEM_guidance_GTP.PSHAN + 360;}
	TAEM_guidance_GTP.PSHA = TAEM_guidance_GTP.PSHAN;
	#print("Hac turn angle is: ", TAEM_guidance_GTP.PSHA);

	#HAC radius and In HAC distance
	TAEM_guidance_GTP.RTURN = TAEM_guidance_TGINIT.RF + 0.093 * math.pow(TAEM_guidance_GTP.PSHA, 2);
	TAEM_guidance_GTP.RPRED2 = (TAEM_guidance_TGINIT.RF * TAEM_guidance_GTP.PSHA + 0.093 * math.pow(TAEM_guidance_GTP.PSHA,3.0) / 3) * 0.0174533 - TAEM_guidance_XHAC.XHAC;
	#print("RTURN is: ", TAEM_guidance_GTP.RTURN / 6076.12 , " RPRED 2 is: ", TAEM_guidance_GTP.RPRED2 / 6076.12);
	#print("RTAN is: ", TAEM_guidance_GTP.RTAN / 6076.12);
	
	#Acquisition and Tangency Point distances for ACQ / S-turns / GRTLS
	if ((TAEM_guidance_TGINIT.IPHASE < 2) or (TAEM_guidance_TGINIT.IPHASE == 4) or (TAEM_guidance_TGINIT.IPHASE == 5) or (TAEM_guidance_TGINIT.IPHASE == 6))
		{

		#3.1 to 3.4 Acq turn radius and Arc length ARCAC (to be adjusted for higher bank during ECAL)
		if (TAEM_guidance_TGINIT.IPHASE < 2)
			{
			var PHAVG = 63.33 - 13.33 * TAEM_jsbsim.mach;
			PHAVG = SpaceShuttle.MIDVAL(PHAVG, 30, 50);
			var RTAC = (TAEM_jsbsim.VH * TAEM_jsbsim.V_true) / (32.174 * math.tan(PHAVG * 0.0174533));
			var ARCAC = RTAC * math.abs(TAEM_guidance_GTP.DPSAC) * 0.0174533;
			#print("ARCAC TAEM is: ", ARCAC/ 6076.12);
			}


		#ECAL CONT
		else
			{
			var PHAVG = 20 + 10 * TAEM_jsbsim.mach;
			PHAVG = SpaceShuttle.MIDVAL(PHAVG, 50, 70);
			var RTAC = (TAEM_jsbsim.VH * TAEM_jsbsim.V_true) / (32.174 * math.tan(PHAVG * 0.0174533));
			var ARCAC = RTAC * math.abs(TAEM_guidance_GTP.DPSAC) * 0.0174533;
			#print("ARCAC during ECAL is: ", ARCAC/ 6076.12);
			}


		#3.5 to 3.6 
		#GRTLS additional term DELRNG to compensate high velocity during initial S-turn 
		#if ((TAEM_guidance_TGINIT.GRTLS_TAEM_INIT == 1) and (TAEM_guidance_TGINIT.IPHASE == 0))
		#	{
		#	var t = math.abs((TAEM_guidance_TGPHIC.PHIC_AT - TAEM_jsbsim.PHIR * 57.29578) / 5);
		#	var DELRANG = t * TAEM_jsbsim.V_true;

			#Total GRTLS
		#	ARCAC = TAEM_guidance_GTP.ARCAC + DELRANG;

		#	print("DELRANG is : ", DELRANG);
		#	}

		#4.1 to 4.4 Distance to Tangency point at the end of acq turn RC
		var RC_1 = RTAC * (1 - math.cos(TAEM_guidance_GTP.DPSAC * 0.0174533));
		var RC_2 = (TAEM_guidance_GTP.RTAN ) - RTAC * math.abs(math.sin(TAEM_guidance_GTP.DPSAC * 0.0174533));
		var RC = math.sqrt(math.pow(RC_1,2) + math.pow(RC_2,2));
		#print("RC is: ", RC/ 6076.12);

		#Total
		TAEM_guidance_GTP.RTAN = ARCAC + RC;
		#print("RTAN for iphase less than 2 is: ", TAEM_guidance_GTP.RTAN / 6076.12);


		

		}



	#Distance to Runway for IPHASE < 3
	TAEM_guidance_GTP.RPRED = TAEM_guidance_GTP.RPRED2 + TAEM_guidance_GTP.RTAN;
	
	}	

#Distance for jsbsim in Nm
TAEM_guidance_GTP.RPRED_nm = TAEM_guidance_GTP.RPRED / 6076.12;
setprop("/fdm/jsbsim/systems/taem-guidance/distance-to-runway-nm", TAEM_guidance_GTP.RPRED_nm);
#print("TAEM distance to runway in Nm is : ", TAEM_guidance_GTP.RPRED_nm);



#VREL and target azimuth computation for PRTLS and GRTLS loop
if (TAEM_guidance_TGINIT.IPHASE > 3)
	{
	TAEM_guidance_GTP.VREL_fps = ((TAEM_guidance_GTP.RPRED - TAEM_guidance_GTP.RPRED_last) / TAEM_guidance_TGINIT.DTG);

	setprop("/fdm/jsbsim/systems/entry_guidance/vrel-fps", TAEM_guidance_GTP.VREL_fps);
	setprop("/fdm/jsbsim/systems/entry_guidance/target-azimuth-deg", geo.aircraft_position().course_to(TAEM_threshold));
	setprop("/fdm/jsbsim/systems/entry_guidance/remaining-distance-nm", TAEM_guidance_GTP.RPRED_nm);

	#Vrel sign
	var vrel_sign = getprop("/fdm/jsbsim/systems/entry_guidance/vrel-sign");

	if ((TAEM_guidance_GTP.VREL_fps > 0.0) and (vrel_sign != 1))
		{
		setprop("/fdm/jsbsim/systems/entry_guidance/vrel-sign", 1);
		}
	else if ((TAEM_guidance_GTP.VREL_fps < 0.0) and (vrel_sign != -1))
		{
		setprop("/fdm/jsbsim/systems/entry_guidance/vrel-sign", -1);
		}

	TAEM_guidance_GTP.RPRED_last = TAEM_guidance_GTP.RPRED;
	#print("Vrel fps is : ", TAEM_guidance_GTP.VREL_fps);
	}
};

# TAEM General Computations - TGCOMP - TAEM Energy / Altitude / QBAR / Spiral Adjust Functions ###########################################################

var TAEM_TGCOMP = func {

#1.1 to 1.2
TAEM_guidance_TGCOMP.DRPRED = TAEM_guidance_GTP.RPRED + TAEM_guidance_XHAC.XALI;
TAEM_guidance_TGCOMP.EOW = TAEM_jsbsim.H_qfe + math.pow(TAEM_jsbsim.V_true,2) / 64.348;
#print("DRPRED is: ", TAEM_guidance_TGCOMP.DRPRED, " EOW is: ", TAEM_guidance_TGCOMP.EOW);

#2.1 Reference EN: two linear E/W segments and coeff computations (IEL = f(EOWSPT))
if (TAEM_guidance_TGCOMP.DRPRED < TAEM_guidance_TGCOMP.EOWSPT) {TAEM_guidance_TGCOMP.IEL = 2;}
else {TAEM_guidance_TGCOMP.IEL = 1;}


#Energy dump shift for large hac
TAEM_guidance_TGCOMP.EN_shift = SpaceShuttle.MIDVAL(TAEM_guidance_TGCOMP.EN_C2_IEL_1  * (TAEM_guidance_GTP.RPRED2 - TAEM_guidance_TGCOMP.R2MAX), 0, TAEM_guidance_TGCOMP.ESHFMX);

if (TAEM_guidance_TGCOMP.IEL == 1)
	{
	#Specific energy ref IEL 1 linear function
	TAEM_guidance_TGCOMP.EN = TAEM_guidance_TGCOMP.EN_C1_IEL_1 + TAEM_guidance_TGCOMP.EN_C2_IEL_1 * TAEM_guidance_TGCOMP.DRPRED - TAEM_guidance_TGCOMP.EN_shift;
	}
else
	{
	#Specific energy ref IEL 2 linear function
	TAEM_guidance_TGCOMP.EN = TAEM_guidance_TGCOMP.EN_C1_IEL_2 + TAEM_guidance_TGCOMP.EN_C2_IEL_2 * TAEM_guidance_TGCOMP.DRPRED - TAEM_guidance_TGCOMP.EN_shift;
	}

#print("IEL is: ", TAEM_guidance_TGCOMP.IEL, " EN_C1 is: ", TAEM_guidance_TGCOMP.EN_C1 , " EN_C2 is: ", TAEM_guidance_TGCOMP.EN_C2 , " EN is: ", TAEM_guidance_TGCOMP.EN, " Shift is: ", TAEM_guidance_TGCOMP.EN_shift);


#4.1 to 4.3 Altitude reference

#Altitude profile TAEM
if (TAEM_guidance_TGINIT.PMODE < 1)
	{
	#Cubic boundary (linear above PBRC)
	if (TAEM_guidance_TGCOMP.DRPRED > TAEM_guidance_TGCOMP.PBRC) {TAEM_guidance_TGCOMP.HREF = TAEM_guidance_TGCOMP.PBHC + 0.1125953 * (TAEM_guidance_TGCOMP.DRPRED - TAEM_guidance_TGCOMP.PBRC);}

	else
		{
		#A/L linear segment gamma depending 
		if (TAEM_guidance_TGCOMP.DRPRED < 0) {TAEM_guidance_TGCOMP.HREF = TAEM_guidance_XHAC.HALI - TAEM_guidance_XHAC.TGGS * TAEM_guidance_TGCOMP.DRPRED;}

		#Cubic segment between A/L and PBRC (TGGS fixed at 20 ° for a same cubic segment IGS independant)
		else {TAEM_guidance_TGCOMP.HREF = TAEM_guidance_XHAC.HALI + 0.36397023 * TAEM_guidance_TGCOMP.DRPRED + TAEM_guidance_TGCOMP.CUBIC_C3 * math.pow(TAEM_guidance_TGCOMP.DRPRED, 2) + TAEM_guidance_TGCOMP.CUBIC_C4 * math.pow(TAEM_guidance_TGCOMP.DRPRED, 3);}
		}
 	}

#Altitude profile on the OGS
else if ((TAEM_guidance_TGINIT.PMODE == 1) or (TAEM_guidance_TGINIT.PMODE == 2))
	{
	#Steep glideslope ((x - xzero) * tan(OGS))
	TAEM_guidance_TGCOMP.HREF = (rwy_coord.X - TAEM_guidance_XHAC.XA) * TAEM_guidance_XHAC.TGGS;
	}


#Alitude profile into the flare 
else if (TAEM_guidance_TGINIT.PMODE == 3)
	{
	#Constant load flare circle
	if (TAEM_guidance_TGINIT.FMODE < 3)
		{
		#Circular flare Href
		TAEM_guidance_TGCOMP.HREF = TAEM_guidance_TGCOMP.HK - math.sqrt(math.pow(TAEM_guidance_TGCOMP.R_flare, 2) - math.pow(rwy_coord.X - TAEM_guidance_XHAC.XK, 2));
		}

	#Exponential capture of IGS
	else 
		{
		#Href from exp. decay to shallow glideslope
		TAEM_guidance_TGNZC.HERREXP = TAEM_guidance_TGCOMP.HD * math.exp((TAEM_guidance_XHAC.XE - rwy_coord.X) / TAEM_guidance_TGCOMP.sigma_EXP);
		TAEM_guidance_TGCOMP.HREF = (rwy_coord.X - TAEM_guidance_TGCOMP.XA2) * TAEM_guidance_TGCOMP.TGGAMMAREF2 + TAEM_guidance_TGNZC.HERREXP;
		}
	}

#print("HREF is: ", TAEM_guidance_TGCOMP.HREF , " PBHC is: ", TAEM_guidance_TGCOMP.PBHC, " C3 is: ", TAEM_guidance_TGCOMP.CUBIC_C3, " C4 is: ", TAEM_guidance_TGCOMP.CUBIC_C4);

#5.1 to 5.2 Qbar ref

#Two linear segments
if (TAEM_guidance_TGCOMP.DRPRED < TAEM_guidance_TGCOMP.PBRCQ) {TAEM_guidance_TGCOMP.QBREF = SpaceShuttle.MIDVAL(TAEM_guidance_TGCOMP.QBRUL - 1.031695e-3 * TAEM_guidance_TGCOMP.DRPRED, TAEM_guidance_TGCOMP.QBRLL, TAEM_guidance_TGCOMP.QBRUL);}
else {TAEM_guidance_TGCOMP.QBREF = SpaceShuttle.MIDVAL(TAEM_guidance_TGCOMP.QBRLL + 5.869595e-4 * (TAEM_guidance_TGCOMP.DRPRED - TAEM_guidance_TGCOMP.PBRCQ), TAEM_guidance_TGCOMP.QBRLL, TAEM_guidance_TGCOMP.QBRML);} 

#print("QBREF is: ", TAEM_guidance_TGCOMP.QBREF);


#5.3 to 5.4 HAC Low Energy final radius Shrinking (need to run a specific function for new HAC center and radius // RTURN // RPRED2)

# In the HAC more than 90 degrees to go and under Auto guidance
if ((TAEM_guidance_TGINIT.IPHASE == 2) and (TAEM_guidance_GTP.PSHA > 90) and (TAEM_jsbsim.auto_pitch == 1) and (TAEM_jsbsim.auto_roll == 1)) 
	{
	TAEM_guidance_TGCOMP.HREFOH = TAEM_guidance_TGCOMP.HREF - SpaceShuttle.MIDVAL(0.11 * (TAEM_guidance_TGCOMP.DRPRED - 35705), 0, 6000);

	#Final Radius adjustement
	TAEM_guidance_TGCOMP.DRF = -3 * (TAEM_guidance_TGCOMP.HREFOH - TAEM_jsbsim.H_qfe) / (TAEM_guidance_GTP.PSHA * 0.0174533);

	#New final radius (Between 5000 and 14000 feet)
	TAEM_guidance_TGINIT.RF = SpaceShuttle.MIDVAL(TAEM_guidance_TGINIT.RF + TAEM_guidance_TGCOMP.DRF, TAEM_guidance_GTP.RFMN, TAEM_guidance_GTP.RFMX);

	#New Hac center computations if RF is shrinking
	if (TAEM_guidance_TGINIT.RF != TAEM_guidance_TGINIT.RF_last)  
		{
		TAEM_guidance_XHAC.construct_hac_center();
		if ((TAEM_guidance_XHAC.hac_shrink_flag == 0) and (TAEM_guidance_TGINIT.RF < 13500))
			{
			SpaceShuttle.callout.make("Low energy condition detected, HAC radius will shrink", "help");
			TAEM_guidance_XHAC.hac_shrink_flag = 1;
			}
		}

	TAEM_guidance_TGINIT.RF_last = TAEM_guidance_TGINIT.RF;

	#print("RF is: ", TAEM_guidance_TGINIT.RF, " RF last is : ", TAEM_guidance_TGINIT.RF_last);
	}


#5.5 to 5.8 Altitude and range error

#Altitude error 
TAEM_guidance_TGCOMP.HERROR = TAEM_guidance_TGCOMP.HREF - TAEM_jsbsim.H_qfe;

#Jsbsim GS deviation for PFD and ap
setprop("/fdm/jsbsim/systems/taem-guidance/glideslope-deviation-ft", TAEM_guidance_TGCOMP.HERROR);


#Tan(Gamma) targeted

#TAEM
if (TAEM_guidance_TGINIT.PMODE < 1)
	{
	if (TAEM_guidance_TGCOMP.DRPRED > TAEM_guidance_TGCOMP.PBRC) {TAEM_guidance_TGCOMP.DHDRRF = - 0.1125953;}
	else {TAEM_guidance_TGCOMP.DHDRRF = - SpaceShuttle.MIDVAL(-TAEM_guidance_XHAC.TGGS  + 2 * TAEM_guidance_TGCOMP.CUBIC_C3 * TAEM_guidance_TGCOMP.DRPRED + 3 * TAEM_guidance_TGCOMP.CUBIC_C4 * math.pow(TAEM_guidance_TGCOMP.DRPRED, 2), 0.1125953, -TAEM_guidance_XHAC.TGGS);}
	}

#AutoLand 
else 
	{
	#OGS
	if ((TAEM_guidance_TGINIT.PMODE == 1) or (TAEM_guidance_TGINIT.PMODE == 2)) {TAEM_guidance_TGCOMP.DHDRRF = -TAEM_guidance_TGCOMP.HREF / (TAEM_guidance_GTP.RPRED + TAEM_guidance_XHAC.XA);}
		
	#Flare
	else	
		{
		#Circular flare (from 2000 feet gamma steep to 200 feet gamma IGS) // clamped between IGS and OGS gammas
		if (TAEM_guidance_TGINIT.FMODE < 3) 
			{
			TAEM_guidance_TGCOMP.DHDRRF = SpaceShuttle.MIDVAL(math.tan((TAEM_guidance_XHAC.GAMMA - (2000 - TAEM_guidance_TGCOMP.HREF) * (TAEM_guidance_XHAC.GAMMA  + 1.5) / 1800) * 0.0174533), TAEM_guidance_XHAC.TGGS, TAEM_guidance_TGCOMP.TGGAMMAREF2);
			}

		#Exp decay IGS (clamped at Gamma IGS 1.5°)
		else {TAEM_guidance_TGCOMP.DHDRRF = math.min(-TAEM_guidance_TGCOMP.HREF / (TAEM_guidance_GTP.RPRED + TAEM_guidance_TGCOMP.XA2), TAEM_guidance_TGCOMP.TGGAMMAREF2);}
		}
		
	#print ("Tan Gamma ref is : ", math.atan2(TAEM_guidance_TGCOMP.DHDRRF,1) * 57.29578);
	} 


#Range error
TAEM_guidance_TGCOMP.DELRNG = TAEM_guidance_TGCOMP.HERROR / TAEM_guidance_TGCOMP.DHDRRF;


#print("HERROR is: ", TAEM_guidance_TGCOMP.HERROR, " DHDRRF is: ", TAEM_guidance_TGCOMP.DHDRRF, " DELRNG is: ", TAEM_guidance_TGCOMP.DELRNG);



#6.1 to 6.5 Filtered QBAR and QBAR rate / QBAR error and EAS commanded

#At first pass only, store the intial QBAR into QBARF
if (TAEM_guidance_TGCOMP.INIT == 0) {TAEM_guidance_TGINIT.QBARF = TAEM_jsbsim.QBAR;}

else
	{
	#Qbar filtered as a function of QBAR and QBAR dot in a computation cycle (0.48s)
	TAEM_guidance_TGCOMP.QBARD = SpaceShuttle.MIDVAL(TAEM_guidance_TGCOMP.CQG * (TAEM_jsbsim.QBAR - TAEM_guidance_TGINIT.QBARF), -5, 5);
	TAEM_guidance_TGINIT.QBARF = TAEM_guidance_TGINIT.QBARF + TAEM_guidance_TGCOMP.QBARD * TAEM_guidance_TGINIT.DTG;
	}

#Factor to be used for Qbar limit for AP
TAEM_guidance_TGINIT.QBD = 0.68113143 * TAEM_guidance_TGINIT.QBD + 0.31886857 * TAEM_guidance_TGCOMP.QBARD;

#Error and EAS
TAEM_guidance_TGCOMP.QBERR = TAEM_guidance_TGCOMP.QBREF - TAEM_guidance_TGINIT.QBARF;
TAEM_guidance_TGCOMP.EAS_CMD = 17.1865 * math.sqrt(TAEM_guidance_TGCOMP.QBREF);

#print("QBARD is: ", TAEM_guidance_TGCOMP.QBARD, " QBARF is: ", TAEM_guidance_TGINIT.QBARF, " QBERR is: ", TAEM_guidance_TGCOMP.QBERR, " EAS_ref is: ", TAEM_guidance_TGCOMP.EAS_CMD,);

if (TAEM_guidance_TGCOMP.INIT == 0) {TAEM_guidance_TGCOMP.INIT = 1;}
};


# TAEM transitions - TGTRAN - Transition / Limits / S-turns and MEP Energy functions ###########################################################

var TAEM_TGTRAN = func {

var major_mode = getprop("/fdm/jsbsim/systems/dps/major-mode");
var sturn_init = getprop("/fdm/jsbsim/systems/ap/taem/s-turn-init"); 
var stage_preset = getprop("/sim/presets/stage"); #Approach scenario

#ES and EMEP computations until 20000 feet for Vert Traj display (Upper and Lower energy boundaries during HAC)
if (TAEM_jsbsim.H_qfe >= 20000)
	{
	#3.1 S-turns Energy
	TAEM_guidance_TGTRAN.ES = TAEM_guidance_TGTRAN.ES1 + TAEM_guidance_TGTRAN.EDRS * TAEM_guidance_TGCOMP.DRPRED;

	#3.3 MEP / STIN

	#EMEP
	if (TAEM_guidance_TGCOMP.IEL == 1) {TAEM_guidance_TGTRAN.EMEP = TAEM_guidance_TGTRAN.EMEP_C1_IEL_1 + TAEM_guidance_TGTRAN.EMEP_C2_IEL_1 * TAEM_guidance_TGCOMP.DRPRED;}
	else {TAEM_guidance_TGTRAN.EMEP = TAEM_guidance_TGTRAN.EMEP_C1_IEL_2 + TAEM_guidance_TGTRAN.EMEP_C2_IEL_2 * TAEM_guidance_TGCOMP.DRPRED;}

	#Straight-In
	TAEM_guidance_TGTRAN.EMOH = TAEM_guidance_TGTRAN.EMOH_C1 + TAEM_guidance_TGTRAN.EMOH_C2 * TAEM_guidance_TGCOMP.DRPRED;


	#print("ES is: ", TAEM_guidance_TGTRAN.ES, " EMEP is: ", TAEM_guidance_TGTRAN.EMEP, " EMOH is: ", TAEM_guidance_TGTRAN.EMOH);
	}

#TAEM (IPHASE)
if (TAEM_guidance_TGINIT.TG_END == 0)
	{
	#TAEM ending
	if (TAEM_guidance_TGINIT.IPHASE == 3) 
		{

		#TAEM ending (Hand-over to Autoland logic // to be done) // Altitude/gamma/Xrange/Qbar conditions or A/L forced below H_ref2 (5000 ft raised to 12000 ft until autoland wip logic is done)

		if ((stage_preset == 4) or (TAEM_jsbsim.H_qfe < 5000) or ((math.abs(TAEM_guidance_TGCOMP.HERROR) < (TAEM_jsbsim.H_qfe * 0.19 - 900)) and (math.abs(rwy_coord.Y) < (TAEM_jsbsim.H_qfe * 0.18 - 800)) and (math.abs(TAEM_jsbsim.gamma - TAEM_guidance_XHAC.GAMMA) < (TAEM_jsbsim.H_qfe * 0.0007 - 3)) and (math.abs(TAEM_guidance_TGCOMP.QBERR) < 24) and (TAEM_jsbsim.H_qfe < 12000)))
			{
			
			#For Approach scenario correct initialisation
			TAEM_guidance_phase = 3;
			TAEM_guidance_TGINIT.IPHASE = 7;
			
			#Auto-Land Phase started and TAEM IPHASE terminated
			TAEM_guidance_TGINIT.TG_END = 1;

			#PMODE1: Acquire OGS
			TAEM_guidance_TGINIT.PMODE = 1; 
			TAEM_guidance_string = "CAPT";


			#print("TAEM guidance finished.", " PMODE is: ", TAEM_guidance_TGINIT.PMODE);

			

			SpaceShuttle.callout.make("Autoland is fully available until Touchdown, Good monitoring", "help");
			setprop("/fdm/jsbsim/systems/ap/taem/hac-turn-init", 0);
			setprop("/fdm/jsbsim/systems/ap/taem/s-turn-init", 0);
			setprop("/fdm/jsbsim/systems/ap/taem/al-init", 1);

			}
			
		}

	#Pre-Final Transition
	else if (((TAEM_guidance_GTP.RPRED < TAEM_guidance_XHAC.RPRED3) or (TAEM_jsbsim.H_qfe < 7000) or (stage_preset == 4)) and (TAEM_guidance_TGINIT.IPHASE < 3))
		{
		TAEM_guidance_phase = 3;
		TAEM_guidance_TGINIT.IPHASE = 3;
		TAEM_guidance_string = "PREFNL";
		#PHIO (done in roll loop)

		#Pre-final faster computations (10 Hz)
		TAEM_guidance_TGINIT.DTG = 0.10;

		#Max 30° of bank if HAC shrink logic not activated
		if (TAEM_guidance_XHAC.hac_shrink_flag == 0) {TAEM_guidance_TGINIT.PHILIM = 30;}
		else {TAEM_guidance_TGINIT.PHILIM = 50;}
		 
		#Min/Max Nz for Pitch command (wider deadband)
		TAEM_guidance_TGINIT.DNZUL = 1.0;
		TAEM_guidance_TGINIT.DNZLL = -0.5;
		}

	#ACQ/S-turns and HDG boundary
	else 
		{
		#GRTLS flag for additional functions to be done later on
		#if ((major_mode != 305) and (TAEM_guidance_TGINIT.GRTLS_TAEM_INIT == 0)) {TAEM_guidance_TGINIT.GRTLS_TAEM_INIT = 1;}
			
		#S-turns ending
		if ((TAEM_guidance_TGINIT.IPHASE + 1) == 1) 
			{
			if (TAEM_guidance_TGCOMP.EOW < (TAEM_guidance_TGCOMP.EN + 10000))
				{
				TAEM_guidance_TGINIT.IPHASE = 1;
				TAEM_guidance_TGINIT.PHILIM = 50;
				setprop("/fdm/jsbsim/systems/ap/taem/s-turn-init",0);
				}
			}

		#ACQ
		else if ((TAEM_guidance_TGINIT.IPHASE + 1) == 2) 
			{
			TAEM_guidance_string = "ACQ";

			#HAC transition (1.05 Rturn at Wp1)
			if (TAEM_guidance_GTP.RCIRC < (TAEM_guidance_GTP.HAC_transition * TAEM_guidance_GTP.RTURN))
				{
				SpaceShuttle.callout.make("Turn "~TAEM_WP_1.turn_direction~" into HAC!", "help");
				TAEM_guidance_string = "HDG";
				TAEM_guidance_phase = 2;

				TAEM_guidance_TGINIT.IPHASE = 2;
				TAEM_guidance_TGINIT.PHILIM = 60;	

				if (TAEM_guidance_GTP.PSHA < 180) TAEM_MCC_flags.one_eighty = 1;
				if (TAEM_guidance_GTP.PSHA < 90)  TAEM_MCC_flags.ninety = 1;

				SpaceShuttle.callout.make("Commander, you should take CSS by now and make John Young proud of you", "help");
				setprop("/fdm/jsbsim/systems/ap/taem/hac-turn-init", 1);
				#print("Waypoint 1 reached!");
				return; 	
				}
				


			#S-turns logic
			if ((TAEM_guidance_TGTRAN.sturn_threshold == 0) and (TAEM_guidance_TGCOMP.EOW > TAEM_guidance_TGTRAN.ES))
				{
				##Sturns illegal flag (1)

				#OPS 3 TAEM // S turns between 25 and 55 Nm and for HAC less than 200 °
				if ((major_mode == 305) and ((TAEM_guidance_GTP.RPRED < 151903) or (TAEM_guidance_GTP.RPRED > 334186) or (TAEM_guidance_GTP.PSHA > 200))) 
					{
					TAEM_guidance_TGTRAN.sturn_threshold = 1;
					return;
					}

				#OPS 6 GRTLS // S turns above 35 Nm and no HAC turn degrees restriction
				else if ((major_mode == 603) and (TAEM_guidance_GTP.RPRED < 212664)) 
					{
					TAEM_guidance_TGTRAN.sturn_threshold = 1;
					return;
					}


				##If legal, we proceed with s-turns limits
				TAEM_guidance_TGINIT.IPHASE = 0;
				TAEM_guidance_TGINIT.PHILIM = 50;

				#30° TAEM vs 45° for GRTLS in supersonic if Sturn triggered
				if (TAEM_guidance_TGINIT.GRTLS_TAEM_INIT == 1) {TAEM_guidance_TGPHIC.PHIMIN = 45;}
				else {TAEM_guidance_TGPHIC.PHIMIN = 30;}
				
				TAEM_guidance_string = "S-TURN";

				SpaceShuttle.callout.make("Initiating S-turn to deplete energy!", "info");
				setprop("/fdm/jsbsim/systems/ap/taem/s-turn-init",1);

				#Turn away form the HAC
				TAEM_guidance_TGTRAN.S_sign = -TAEM_guidance_GTP.YSGN;

				#Conditionnal for less than 90° HAC depending on PSD (course)
				TAEM_guidance_TGTRAN.SPSI = TAEM_guidance_TGTRAN.S_sign * rwy_coord.PSD;
				if ((TAEM_guidance_TGTRAN.SPSI < 0) and (TAEM_guidance_GTP.PSHA < 90)) {TAEM_guidance_TGTRAN.S_sign = -TAEM_guidance_TGTRAN.S_sign;}
				}

			}

		#HAC
		else if ((TAEM_guidance_TGINIT.IPHASE + 1) == 3) {return;} 

		}
	}


#Autoland (PMODE 1 to 4)
else
	{
	#­PMODE2 forced at 5000 feet 
	if ((TAEM_guidance_TGINIT.PMODE == 1) and ((math.abs(TAEM_jsbsim.gamma - TAEM_guidance_XHAC.GAMMA) < 2) or (TAEM_jsbsim.H_qfe < 5000)))
		{
		#OGS track transition / PMODE2: Track OGS until Pre-Flare
		TAEM_guidance_phase = 4;
		TAEM_guidance_TGINIT.PMODE = 2;
		TAEM_guidance_string = "OGS";

		#MLS
		HUD_data_set.MLS_acquired = 1;

		#Approach/Landing Logic  higher computation guidance cycle 
		TAEM_guidance_TGINIT.DTG = 0.10;

		#Roll Channel 
		TAEM_guidance_TGINIT.PHILIM = 20;
		
		SpaceShuttle.callout.make("Tracking the Outer Steep Glide Slope", "help");
		}

	
	else if ((TAEM_guidance_TGINIT.PMODE == 2) and (TAEM_jsbsim.H_qfe < 2000))
		{
		#Approach/Landing Logic  TDAP guidance cycle even faster (50ms)
		#TAEM_guidance_TGINIT.DTG = 0.05;

		#Flare transition
		TAEM_guidance_TGINIT.PMODE = 3;
		TAEM_guidance_phase = 5;
		TAEM_guidance_string = "FLARE";

		#Flare submode 1 (first flare)
		TAEM_guidance_TGINIT.FMODE = 1;

		#Store last OGS pass Gamma
		TAEM_guidance_TGNZC.GAMMASYNC = TAEM_jsbsim.gamma;

		SpaceShuttle.callout.make("Pull up flare open loop", "help");

		#print("last OGS gamma is : ", TAEM_guidance_TGNZC.GAMMASYNC);

		#Roll Channel no limitation changes
		}

	else if (TAEM_guidance_TGINIT.PMODE == 3)
		{
		#Flare submode 2 (Constant load flare circle)
		if ((TAEM_guidance_TGINIT.FMODE == 1) and (TAEM_jsbsim.H_qfe < 1700)) 
			{
			TAEM_guidance_TGINIT.FMODE = 2;
			SpaceShuttle.callout.make("Pull up flare closed loop", "help");
			}
			
		#Flare submode 3 (Exponential capture of IGS)
		else if ((TAEM_guidance_TGINIT.FMODE == 2) and (rwy_coord.X > TAEM_guidance_XHAC.XE)) 
			{
			TAEM_guidance_TGINIT.FMODE = 3;
			SpaceShuttle.callout.make("Transition to Inner Shallow glideslope", "help");
			}

		#Transition to final flare (forced at 30 RA/agl)
		else if ((TAEM_guidance_TGINIT.FMODE == 3) and (((TAEM_jsbsim.H_qfe < 80) and (TAEM_jsbsim.HDOT < 16)) or (TAEM_jsbsim.H_agl < 30)))
			{
			#Final flare transition
			TAEM_guidance_TGINIT.PMODE = 4;
			TAEM_guidance_phase = 7;
			TAEM_guidance_string = "FNLFL";

			#Store last HDOT value for Final Flare Hdot estimation (open loop)
			TAEM_guidance_TGNZC.HDEST = -TAEM_jsbsim.HDOT;

			SpaceShuttle.callout.make("Final Flare, buckle up", "help");
			}

		#Automatic Landing gear down if AutoLand
		#if ((TAEM_jsbsim.H_qfe < 300) and (getprop("/fdm/jsbsim/systems/ap/automatic-pitch-control") == 1) and (getprop("/fdm/jsbsim/systems/ap/automatic-roll-control") == 1) and (getprop("/controls/gear/gear-down") == 0))
		#	{
		#	setprop("/fdm/jsbsim/systems/landing/landing-gear-arm-cmd", 1);		
		#	setprop("/controls/gear/gear-down-cmd",1);	
		#	setprop("/controls/gear/gear-down",1);

		#	SpaceShuttle.callout.make("Automatic gear extension", "help");	
		#	}
		}
	#Basic Transition for WOW (need to be expanded for further on ground HUD mode) // either Main gear is on the ground (X-wind landing)
	else if ((TAEM_guidance_TGINIT.PMODE == 4) and (getprop("/fdm/jsbsim/gear/unit[1]/WOW") or getprop("/fdm/jsbsim/gear/unit[2]/WOW")))
		{
		#WOW transition
		TAEM_guidance_TGINIT.PMODE = 5;
		TAEM_guidance_phase = 8;
		TAEM_guidance_string = "WOW";

		#Lateral guidance transition
		TAEM_guidance_TGPHIC.FLATTURN = 1;


		#End the AutoLand at Touchdown if at least one AP is engaged
		if ((TAEM_jsbsim.auto_pitch == 1) or (TAEM_jsbsim.auto_roll == 1))
			{
			settimer( func {
				
				setprop("/fdm/jsbsim/systems/ap/automatic-pitch-control", 0);
				setprop("/fdm/jsbsim/systems/ap/css-pitch-control", 1);
				setprop("/fdm/jsbsim/systems/ap/automatic-roll-control", 0);
				setprop("/fdm/jsbsim/systems/ap/css-roll-control", 1);

				SpaceShuttle.callout.make("Autoland AP disconnected, CSS and manual rollout", "help");

				}, 2.0); 
			}
		}
	}
	
#print("IPHASE is: ", TAEM_guidance_TGINIT.IPHASE , " TG end is: ", TAEM_guidance_TGINIT.TG_END, " PMODE is: ", TAEM_guidance_TGINIT.PMODE, " FMODE is :", TAEM_guidance_TGINIT.FMODE, " DTG is: ", TAEM_guidance_TGINIT.DTG);
};



# TAEM Pitch Nz Commanded - TGNZC - Normal Acceleration Command Function ###########################################################

var TAEM_TGNZC = func {

#1.1 to 1.4 DNZC calculated based on Altitude error

#TAEM
if (TAEM_guidance_TGINIT.TG_END == 0)
	{
	TAEM_guidance_TGNZC.GDH = SpaceShuttle.MIDVAL(TAEM_guidance_TGNZC.GDHC - TAEM_guidance_TGNZC.GDHS * TAEM_jsbsim.H_qfe, 0.3, 1.0);
	TAEM_guidance_TGNZC.HDREF = TAEM_jsbsim.VH * TAEM_guidance_TGCOMP.DHDRRF;
	TAEM_guidance_TGNZC.HDERR = TAEM_guidance_TGNZC.HDREF + TAEM_jsbsim.HDOT; 
	TAEM_guidance_TGNZC.DNZC = TAEM_guidance_TGNZC.DNZCG * TAEM_guidance_TGNZC.GDH * (TAEM_guidance_TGNZC.HDERR + TAEM_guidance_TGNZC.HDREQG * TAEM_guidance_TGNZC.GDH * TAEM_guidance_TGCOMP.HERROR);
	}

#Autoland
else
	{
	#OGS 
	if ((TAEM_guidance_TGINIT.PMODE == 1) or (TAEM_guidance_TGINIT.PMODE == 2))
		{
		#H Dot error
		TAEM_guidance_TGNZC.HDREF = TAEM_jsbsim.VH * TAEM_guidance_TGCOMP.DHDRRF;
		TAEM_guidance_TGNZC.HDERR = TAEM_guidance_TGNZC.HDREF + TAEM_jsbsim.HDOT; 
		
		#Integration
		TAEM_guidance_TGNZC.H_int = TAEM_guidance_TGNZC.KHINT * SpaceShuttle.MIDVAL(TAEM_guidance_TGCOMP.HERROR, -50, 50) * TAEM_guidance_TGINIT.DTG;

		#DNZC Autoland
		TAEM_guidance_TGNZC.DNZC = TAEM_guidance_TGNZC.KH * SpaceShuttle.MIDVAL(TAEM_guidance_TGCOMP.HERROR, -300, 300) + TAEM_guidance_TGNZC.H_int * TAEM_guidance_TGNZC.KHINT + TAEM_guidance_TGNZC.KHDOT * TAEM_guidance_TGNZC.HDERR; 

		#HUD OGS cue (fixed at steep gamma ref)
		HUD_data_set.vangle_guidance = -TAEM_guidance_XHAC.GAMMA;
		}

	#Flare and IGS
	else if (TAEM_guidance_TGINIT.PMODE == 3)
		{
		#Open loop NZC common to the 3 submodes

			#V_true needs to be filter with a lag filter NF = 1 /C1 = A13
			TAEM_guidance_TGNZC.V_true_filtered = TAEM_lag_filter(1, TAEM_guidance_TGNZC.A13, TAEM_jsbsim.V_true);
			
			#Actual Gamma clamped for first V_trued filtered pass
			TAEM_guidance_TGNZC.GAMMASYNC = SpaceShuttle.clamp(-TAEM_jsbsim.HDOT / TAEM_guidance_TGNZC.V_true_filtered, -1, 1);
			TAEM_guidance_TGNZC.GAMMASYNC = math.asin(TAEM_guidance_TGNZC.GAMMASYNC) * 57.29578;

			#Max overload nz during open loop as a function of speed (ref is 300 Kts / 0.26g) (V_ground_fps available below 3000 feet into the jsbsim hash)
			TAEM_guidance_TGNZC.NZ_max_openloop = math.pow(TAEM_jsbsim.V_ground, 2) / TAEM_guidance_TGCOMP.R_flare;

			#Theta dot max
			TAEM_guidance_TGNZC.theta_dot_max = (TAEM_guidance_TGNZC.NZ_max_openloop * 57.3) / TAEM_guidance_TGNZC.V_true_filtered;

			#Gamma error between last computed gamma in OGS and targeted IGS gamma (1.5°)
			TAEM_guidance_TGNZC.GAMMAERR_filtered = SpaceShuttle.MIDVAL((TAEM_guidance_TGNZC.GAMMAREF2 - TAEM_guidance_TGNZC.GAMMASYNC - TAEM_guidance_TGNZC.GAMMAERR_filtered * TAEM_guidance_TGINIT.DTG) * (1 / TAEM_guidance_TGNZC.TPRED), -TAEM_guidance_TGNZC.theta_dot_max, TAEM_guidance_TGNZC.theta_dot_max) ;

			#Nz commanded open loop 
			TAEM_guidance_TGNZC.NZCOM5 = TAEM_guidance_TGNZC.GAMMAERR_filtered * TAEM_guidance_TGNZC.V_true_filtered / (57.3 * 32.2);

			#Nz commanded open loop filtered
			TAEM_guidance_TGNZC.NZCOM5 = TAEM_lag_filter(2, TAEM_guidance_TGNZC.A40, TAEM_guidance_TGNZC.NZCOM5);

			#print("V_true filtered is : ", TAEM_guidance_TGNZC.V_true_filtered);
			#print("Nz open loop is : ", TAEM_guidance_TGNZC.NZCOM5);
			
		#Initial Pullup (open loop only)
		if (TAEM_guidance_TGINIT.FMODE == 1) 
			{
			TAEM_guidance_TGNZC.HDREF = -TAEM_jsbsim.V_ground * (rwy_coord.X - TAEM_guidance_XHAC.XK) / (TAEM_guidance_TGCOMP.HREF - TAEM_guidance_TGCOMP.HK);
			TAEM_guidance_TGNZC.DNZC = TAEM_guidance_TGNZC.NZCOM5;
			}

		#Constant load flare circle and exponential capture of IGS
		else
			{
			
			#Hdot ref and error 
			if  (TAEM_guidance_TGINIT.FMODE == 2) #Flare closed loop
				{
				TAEM_guidance_TGNZC.HDREF = -TAEM_jsbsim.V_ground * (rwy_coord.X - TAEM_guidance_XHAC.XK) / (TAEM_guidance_TGCOMP.HREF - TAEM_guidance_TGCOMP.HK); #AIAA autoland Tsikalas paper
				#TAEM_guidance_TGNZC.HDREF = TAEM_jsbsim.V_ground * TAEM_guidance_TGCOMP.DHDRRF;
				}

			else  #Exp decay
				{
				TAEM_guidance_TGNZC.HDREF = TAEM_jsbsim.V_ground * TAEM_guidance_TGCOMP.TGGAMMAREF2 - (TAEM_guidance_TGNZC.HERREXP * TAEM_jsbsim.V_ground) / TAEM_guidance_TGCOMP.sigma_EXP;
				}

			TAEM_guidance_TGNZC.HDERR = TAEM_guidance_TGNZC.HDREF + TAEM_jsbsim.HDOT; 
			TAEM_guidance_TGNZC.NZCOM3 = SpaceShuttle.MIDVAL(TAEM_guidance_TGNZC.HDERR, -200, 200) * TAEM_guidance_TGNZC.KHDOTERR;

			#H error (it contains integration term)
			TAEM_guidance_TGNZC.NZCOM4 = SpaceShuttle.MIDVAL(TAEM_guidance_TGCOMP.HERROR, -200, 200) * TAEM_guidance_TGNZC.KH_pullup * (1 + TAEM_guidance_TGNZC.KI_pullup * TAEM_guidance_TGINIT.DTG); 

			#Final Nz commanded 
			TAEM_guidance_TGNZC.DNZC = TAEM_guidance_TGNZC.NZCOM5 + TAEM_guidance_TGNZC.NZCOM3 + TAEM_guidance_TGNZC.NZCOM4;
			
			#HUD flare cue (Dynamic flare cue based on gamma ref between steep and shallow glideslope)
			HUD_data_set.vangle_guidance = SpaceShuttle.MIDVAL(-math.asin(math.clamp(TAEM_guidance_TGNZC.HDREF / TAEM_jsbsim.V_ground,-1.0,1.0)) * 57.29578, 1.5, -TAEM_guidance_XHAC.GAMMA);

			#print("Nz Herror flare is  : ", TAEM_guidance_TGNZC.NZCOM4, " Nz Hdot err flare is : ", TAEM_guidance_TGNZC.NZCOM3);
			}

		
		}

	#Final flare
	else if (TAEM_guidance_TGINIT.PMODE == 4)
		{
		#Open loop 

		#Integral loop (Integral initial condition is Hdot during flare) // integral removed from real guidance for stability purpose
			#TAEM_guidance_TGNZC.HDOT_openloop = TAEM_guidance_TGNZC.HDOT_TD1 - ((TAEM_guidance_TGNZC.HDOT_openloop * TAEM_guidance_TGINIT.DTG - TAEM_guidance_TGNZC.HDEST) / TAEM_guidance_TGNZC.TFLR );
		TAEM_guidance_TGNZC.HDOT_openloop = (TAEM_guidance_TGNZC.HDOT_TD1 - TAEM_guidance_TGNZC.HDEST) / TAEM_guidance_TGNZC.TFLR;

		#DNZC (with correction factor for sloped runway)
		TAEM_guidance_TGNZC.K_slope = TAEM_guidance_TGNZC.slope_factor * TAEM_threshold.slope + 1;
		TAEM_guidance_TGNZC.NZCOM6 = TAEM_guidance_TGNZC.K_slope * (TAEM_guidance_TGNZC.HDOT_openloop * TAEM_guidance_TGNZC.KFLR);

 
		#Closed loop (Based on Hdot error only) agl vs qfe (?)

		#Hdot ref  (math max vs abs depenging of papers)
		TAEM_guidance_TGNZC.HDREF = -math.max(TAEM_jsbsim.H_qfe - TAEM_guidance_TGNZC.H_NOACC, 0) / TAEM_guidance_TGNZC.TAU_TD2 + TAEM_guidance_TGNZC.HDOT_TD2;

		#Hdot ref filtered
		TAEM_guidance_TGNZC.HDREF = TAEM_lag_filter(3, TAEM_guidance_TGNZC.A3, TAEM_guidance_TGNZC.HDREF);

		#Hdot error
		TAEM_guidance_TGNZC.HDERR = (TAEM_guidance_TGNZC.HDREF + TAEM_jsbsim.HDOT) * TAEM_guidance_TGNZC.KHDOT;

		#DNZC
		TAEM_guidance_TGNZC.NZCOM7 = TAEM_guidance_TGNZC.HDERR + TAEM_guidance_TGNZC.HDERR * TAEM_guidance_TGNZC.KIFLR * TAEM_guidance_TGINIT.DTG;

		#Final Nz commanded
		TAEM_guidance_TGNZC.DNZC = TAEM_guidance_TGNZC.NZCOM6 + TAEM_guidance_TGNZC.NZCOM7;

		#HUD cue (normally not shown for final flare) / -3fps targeted
		HUD_data_set.vangle_guidance = -math.atan2(-3, TAEM_jsbsim.VH) * 57.29578;

		#print("NZ open loop FF is: ", TAEM_guidance_TGNZC.NZCOM6, " HDREF is: ", TAEM_guidance_TGNZC.HDREF, " HDERR is : ", TAEM_guidance_TGNZC.HDERR / TAEM_guidance_TGNZC.KHDOT);
		#print("Nz Hdot closed loop is ", TAEM_guidance_TGNZC.NZCOM7, " Nz Hdot is : ", TAEM_guidance_TGNZC.HDERR, " Nz Hdot integral is : ", TAEM_guidance_TGNZC.HDERR * TAEM_guidance_TGNZC.KIFLR * TAEM_guidance_TGINIT.DTG);
		}

	#print("HUD commanded gamma is: ", HUD_data_set.vangle_guidance);
	}

#print("H dot ref is : ", TAEM_guidance_TGNZC.HDREF);

#1.5 to 1.7 Minimum dynamic pressure profile (function of weight and mach)

if (TAEM_jsbsim.mach < TAEM_guidance_TGNZC.QMACH2) {TAEM_guidance_TGNZC.MXQBWT = SpaceShuttle.MIDVAL(TAEM_guidance_TGNZC.QBWT1 + TAEM_guidance_TGNZC.QBMSL1 * (TAEM_jsbsim.mach - TAEM_guidance_TGNZC.QMACH1), TAEM_guidance_TGNZC.QBWT2, TAEM_guidance_TGNZC.QBWT1);}
else {TAEM_guidance_TGNZC.MXQBWT = SpaceShuttle.MIDVAL(TAEM_guidance_TGNZC.QBWT2 + TAEM_guidance_TGNZC.QBMSL2 * (TAEM_jsbsim.mach - TAEM_guidance_TGNZC.QMACH2), TAEM_guidance_TGNZC.QBWT2, TAEM_guidance_TGNZC.QBWT3);}

TAEM_guidance_TGNZC.QBLL = TAEM_guidance_TGNZC.MXQBWT * TAEM_jsbsim.weight; #Slugs
TAEM_guidance_TGNZC.QBMNNZ = TAEM_guidance_TGNZC.QBLL / math.max(math.cos(TAEM_jsbsim.PHIR), TAEM_guidance_TDAP.CPMIN);


#1.8 Maximum dynamic pressure profile (340 below mach 1, 300 above)

if (TAEM_jsbsim.mach > TAEM_guidance_TGNZC.QBM1) {TAEM_guidance_TGNZC.QBMXNZ = 300;}
else {TAEM_guidance_TGNZC.QBMXNZ = SpaceShuttle.MIDVAL(TAEM_guidance_TGNZC.QBMX2 + TAEM_guidance_TGNZC.QBMXS1 * (TAEM_jsbsim.mach - TAEM_guidance_TGNZC.QBM1), 300, 340);}


#1.9 Max dynamic pressure lowered for an OVH HAC (Energy dump / pull up logic to be subsonic into the HAC)

if ((TAEM_guidance_TGNZC.EQLOWL < TAEM_guidance_TGCOMP.EOW) and (TAEM_guidance_TGCOMP.EOW < TAEM_guidance_TGNZC.EQLOWU) and (TAEM_guidance_GTP.PSHA > 0))
	{
	TAEM_guidance_TGNZC.QBMXNZ = SpaceShuttle.MIDVAL(TAEM_guidance_TGNZC.QBREF2 - TAEM_guidance_TGNZC.PQBWRR * (TAEM_guidance_GTP.RPRED2 - TAEM_guidance_TGCOMP.R2MAX) + (TAEM_guidance_TGCOMP.EOW - TAEM_guidance_TGCOMP.EN) / TAEM_guidance_TGNZC.PEWRR, TAEM_guidance_TGNZC.QBMNNZ, TAEM_guidance_TGNZC.QBMXNZ); 
	}

#print("HDREF is : ", TAEM_guidance_TGNZC.HDREF, " DNZC is : ", TAEM_guidance_TGNZC.DNZC);
#print("QBMNZ is ", TAEM_guidance_TGNZC.QBMNNZ, " QBMXNZ is ", TAEM_guidance_TGNZC.QBMXNZ);

#Bailout Qbar limitation (180) // Full logic to be finished (TAEM management page 35 // Alpha limits)
if (getprop("/fdm/jsbsim/systems/ap/auto-bailout-active") == 1)
	{
	TAEM_guidance_TGNZC.QBMNNZ = 180;
	TAEM_guidance_TGNZC.QBMXNZ = 180;
	}

#Final flare lowered Qbar min to avoid Pitch Down input (200 kt)
if (TAEM_guidance_TGINIT.PMODE > 3) {TAEM_guidance_TGNZC.QBMNNZ = 130;}

#2.1 to 2.2 Upper/Lower Nz limits based on Max/Min QBAR

TAEM_guidance_TGNZC.QBNZUL = -TAEM_guidance_TGNZC.QBG2 * (TAEM_guidance_TGNZC.QBG1 * (TAEM_guidance_TGNZC.QBMNNZ - TAEM_guidance_TGINIT.QBARF) - TAEM_guidance_TGINIT.QBD);
TAEM_guidance_TGNZC.QBNZLL = -TAEM_guidance_TGNZC.QBG2 * (TAEM_guidance_TGNZC.QBG1 * (TAEM_guidance_TGNZC.QBMXNZ - TAEM_guidance_TGINIT.QBARF) - TAEM_guidance_TGINIT.QBD);

#print("QBNZUL is ", TAEM_guidance_TGNZC.QBNZUL, " QBNZLL is ", TAEM_guidance_TGNZC.QBNZLL);


#Pre-Final: No Energy limitations for NZC (Only qbar)
if (TAEM_guidance_TGINIT.IPHASE > 2) {TAEM_guidance_TGNZC.NZC = SpaceShuttle.MIDVAL(TAEM_guidance_TGNZC.DNZC, TAEM_guidance_TGNZC.QBNZLL, TAEM_guidance_TGNZC.QBNZUL);}

else
	{

	#3.1 to 3.4 Energy Upper/Lower Nz limits
	TAEM_guidance_TGNZC.EMAX = TAEM_guidance_TGCOMP.EN + TAEM_guidance_TGNZC.EDELNZU;
	TAEM_guidance_TGNZC.EMIN = TAEM_guidance_TGCOMP.EN - TAEM_guidance_TGNZC.EDELNZL;
	TAEM_guidance_TGNZC.EOWNZUL = (TAEM_guidance_TGNZC.GEUL * TAEM_guidance_TGNZC.GDH * (TAEM_guidance_TGNZC.EMAX - TAEM_guidance_TGCOMP.EOW) + TAEM_guidance_TGNZC.HDERR) * TAEM_guidance_TGNZC.GEHDUL * TAEM_guidance_TGNZC.GDH;
	TAEM_guidance_TGNZC.EOWNZLL = (TAEM_guidance_TGNZC.GELL * TAEM_guidance_TGNZC.GDH * (TAEM_guidance_TGNZC.EMIN - TAEM_guidance_TGCOMP.EOW) + TAEM_guidance_TGNZC.HDERR) * TAEM_guidance_TGNZC.GEHDLL * TAEM_guidance_TGNZC.GDH;

	#print("EOWNZLL is ", TAEM_guidance_TGNZC.EOWNZLL, " EOWNZUL is ", TAEM_guidance_TGNZC.EOWNZUL);


	#3.5 to 3.6 Energy and QBAR NZ limitations
	TAEM_guidance_TGNZC.DNZCL = SpaceShuttle.MIDVAL(TAEM_guidance_TGNZC.DNZC, TAEM_guidance_TGNZC.EOWNZLL, TAEM_guidance_TGNZC.EOWNZUL);
	TAEM_guidance_TGNZC.DNZCL = SpaceShuttle.MIDVAL(TAEM_guidance_TGNZC.DNZCL, TAEM_guidance_TGNZC.QBNZLL, TAEM_guidance_TGNZC.QBNZUL);

	#print("DNZCL is: ", TAEM_guidance_TGNZC.DNZCL);

	#3.7 to 3.8 NZCdot parameter and final NZC
	TAEM_guidance_TGNZC.DNZCD = SpaceShuttle.MIDVAL((TAEM_guidance_TGNZC.DNZCL - TAEM_guidance_TGNZC.NZC) * TAEM_guidance_TGCOMP.CQG, -TAEM_guidance_TGNZC.DNZCDL, TAEM_guidance_TGNZC.DNZCDL);
	TAEM_guidance_TGNZC.NZC = TAEM_guidance_TGNZC.NZC + TAEM_guidance_TGNZC.DNZCD * TAEM_guidance_TGINIT.DTG;

	#print("DNZCD is: ", TAEM_guidance_TGNZC.DNZCD);
	}

#Final NZC filter for max g's allowed IPHASE dependant
TAEM_guidance_TGNZC.NZC = SpaceShuttle.MIDVAL(TAEM_guidance_TGNZC.NZC, TAEM_guidance_TGINIT.DNZLL, TAEM_guidance_TGINIT.DNZUL);

#print("NZC is: ", TAEM_guidance_TGNZC.NZC);

};


# TAEM Speedbrake - TGSBC - TAEM Speedbrake modulation ###########################################################

var TAEM_TGSBC = func {

#OI-22 logic with blended Energy/Qbar between 15000 feet and A/L to be added later


#Fixed Supersonic value (65°) except for S-turns
if (TAEM_jsbsim.mach > TAEM_guidance_TGSBC.DSBCM) 
	{
	if (TAEM_guidance_TGINIT.IPHASE == 0) {TAEM_guidance_TGSBC.DSBC_AT = TAEM_guidance_TGSBC.DSBLIM;}
	else {TAEM_guidance_TGSBC.DSBC_AT = TAEM_guidance_TGSBC.DSBSUP;}
	}

#Subsonic
else
	{
	
	#Limits
	TAEM_guidance_TGSBC.DSBCLL = SpaceShuttle.MIDVAL(TAEM_guidance_TGSBC.DSBSUP + 650 * (TAEM_jsbsim.mach - TAEM_guidance_TGSBC.DSBCM), 15, TAEM_guidance_TGSBC.DSBSUP);
	TAEM_guidance_TGSBC.DSBCUL = SpaceShuttle.MIDVAL(TAEM_guidance_TGSBC.DSBSUP - 336 * (TAEM_jsbsim.mach - TAEM_guidance_TGSBC.DSBCM), TAEM_guidance_TGSBC.DSBSUP, TAEM_guidance_TGSBC.DSBLIM);

	#Upper SB limit is updated with Low Energy limit considerations up to Pre-Final: ((EOW + 7000) - EN) * 0.015
	if (TAEM_guidance_TGINIT.IPHASE < 3)
		{
		TAEM_guidance_TGSBC.SB_energy_UL = (TAEM_guidance_TGCOMP.EOW + TAEM_guidance_TGSBC.DSBCE2 - TAEM_guidance_TGCOMP.EN) * TAEM_guidance_TGSBC.DSBCE1;
		TAEM_guidance_TGSBC.DSBCUL = SpaceShuttle.MIDVAL(TAEM_guidance_TGSBC.SB_energy_UL, TAEM_guidance_TGSBC.DSBCLL, TAEM_guidance_TGSBC.DSBCUL);
		}


	#Bailout 
	if (getprop("/fdm/jsbsim/systems/ap/auto-bailout-active") == 1) {TAEM_guidance_TGSBC.DSBC = 0;}

	else
		{
		#S-turns (98.6°)
		if (TAEM_guidance_TGINIT.IPHASE == 0) {TAEM_guidance_TGSBC.DSBC = TAEM_guidance_TGSBC.DSBLIM;}


		#ACQ/HAC (Qbar ref error)
		else if ((TAEM_guidance_TGINIT.IPHASE == 1) or (TAEM_guidance_TGINIT.IPHASE == 2))
			{
			#Qbar error
			TAEM_guidance_TGSBC.DSBE = TAEM_guidance_TGSBC.GSBE * TAEM_guidance_TGCOMP.QBERR;

			#Integral loop based on QBERR * dT if DSBC within limits (first pass dampening)
			if ((TAEM_guidance_TGSBC.DSBC > TAEM_guidance_TGSBC.DSBCLL) and (TAEM_guidance_TGSBC.DSBC < TAEM_guidance_TGSBC.DSBCUL))
				{
				TAEM_guidance_TGINIT.DSBI = SpaceShuttle.MIDVAL(TAEM_guidance_TGINIT.DSBI + TAEM_guidance_TGSBC.GSBI * TAEM_guidance_TGCOMP.QBERR * TAEM_guidance_TGINIT.DTG, -20, 20);
				}

			#Unlimited SB command
			TAEM_guidance_TGSBC.DSBC = TAEM_guidance_TGSBC.DSBNOM - TAEM_guidance_TGSBC.DSBE - TAEM_guidance_TGINIT.DSBI;
			}


		#Pre-Final and Auto-Land(EAS ref error)
		else 
			{
			#EAS error (only used for Speed control in final) // ft.s for gains // lag filter needed
			TAEM_guidance_TGSBC.V_equivalent_filtered = TAEM_lag_filter(4, TAEM_guidance_TGSBC.A14, TAEM_jsbsim.V_equivalent);
			TAEM_guidance_TGSBC.EASERR = TAEM_guidance_TGSBC.V_equivalent_filtered - 507;

			#print ("EAS sb filtered is : ", TAEM_guidance_TGSBC.V_equivalent_filtered );

			#Pre-Final
			if (TAEM_guidance_TGINIT.IPHASE == 3) {TAEM_guidance_TGSBC.DSBC = TAEM_guidance_TGSBC.DSBNOM + TAEM_guidance_TGSBC.EASERR * TAEM_guidance_TGSBC.GSB;}

			#AutoLand
			else
				{
				#OGS capture
				if (TAEM_guidance_TGINIT.PMODE == 1) {TAEM_guidance_TGSBC.DSBC = TAEM_guidance_TGSBC.DSBNOM + TAEM_guidance_TGSBC.EASERR * TAEM_guidance_TGSBC.GSB;}

				#OGS 
				else if (TAEM_guidance_TGINIT.PMODE > 1) 
					{
					#Before 3000 feet EAS modulation
					if (TAEM_jsbsim.H_qfe > 3000)
						{
						#Integral additionnal term EASerror * dT
						TAEM_guidance_TGINIT.DSBI = SpaceShuttle.MIDVAL(TAEM_guidance_TGSBC.EASERR, -10, 10) * TAEM_guidance_TGSBC.GSBI * TAEM_guidance_TGINIT.DTG;

						TAEM_guidance_TGSBC.DSBC = TAEM_guidance_TGSBC.DSBNOM + TAEM_guidance_TGSBC.EASERR * TAEM_guidance_TGSBC.GSB + TAEM_guidance_TGINIT.DSBI;
						#print("Speedbrake final test is : ", "OK");
						}

					#Fixed setting below 3000 feet
					else if (TAEM_jsbsim.H_qfe < 3000) 
						{
						
						#Variables used only below 3000 feet QFE for Speedbrake retracts 
						#Effective windspeed 
						
						var windspeed_kts = (TAEM_jsbsim.VH - TAEM_jsbsim.V_ground) * 0.592484;

						#Density factor (ISA deviation)
						var Z_density = getprop("/fdm/jsbsim/atmosphere/density-altitude");

						#Sim-Time for windspeed derivative
						var sim_time = getprop("/fdm/jsbsim/sim-time-sec");

						#Effective windspeed stored between 3000 and 500 feet 
							#if (TAEM_jsbsim.H_qfe > 500)
							#	{
							#	TAEM_guidance_TGSBC.windspeed_int = windspeed_kts + TAEM_guidance_TGSBC.windspeed_int_last;
							#	TAEM_guidance_TGSBC.windspeed_int_last = TAEM_guidance_TGSBC.windspeed_int;
							#
							#	#print("Windspeed int is: ", TAEM_guidance_TGSBC.windspeed_int);
							#	}

						#First retract at 3000 feet
						if (TAEM_guidance_TGSBC.first_retract_flag == 0)
							{
							var sb_mode = getprop("/fdm/jsbsim/systems/approach-guidance/speedbrake-mode-string"); #Lower TD speed targeted for Short // Shorter touchdown zone and lower speed targeted for ELS
							var weight_final = getprop("/fdm/jsbsim/inertia/weight-lbs"); #Light/ Heavy/ Super heavy weight boundaries ( 222 klbs // 245 klbs)
							var aim_point_string = getprop("/fdm/jsbsim/systems/approach-guidance/aim-point-string"); #Close In Vs Normal aimpoint ( More energy to be dissipated for close in)

							#Nominal: Touchdown zone at 2500 feet / Lightweight speed 195kts / Heavy 205kts
							#Short: Touchdown zone at 1500 feet / Lightweight speed 195kts / Heavy 205 kts ( plus 10% Speedbrakes to land shorter)
							#ELS: Touchdown zone at 1000 feet / Heavyweight only 195kts (plus 25% speedbrakes to land slower and shorter) // Mainly for Abort 


							#Effective wind at 3000ft stored
							TAEM_guidance_TGSBC.windspeed_3000 = windspeed_kts;
							TAEM_guidance_TGSBC.timestamp_3000 = sim_time;

							#Windspeed Donwrange sensitivity
							var windspeed_sensitivity = 84.2 * TAEM_guidance_TGSBC.windspeed_3000;

							#Density Altitude Downrange Sensitivity
							var density_sensitivity = 0.25 * Z_density - 500;

							#EAS error Downrange Sensitivity (knots)
							var eas_error_sensitivity = 75 * TAEM_guidance_TGSBC.EASERR * 0.592484;

							#Weight Donwrange Sensitivity (IGS dependant)
							var weight_sensitivity = 0;
							if (TAEM_guidance_XHAC.IGS == 1) {weight_sensitivity = 0.04205 * weight_final - 8069;}
							else {weight_sensitivity = 0.0335 * weight_final - 7337;}

							#Sb mode factor
							var sb_option_sensitivity = 0;
							if (sb_mode == "SHORT") {sb_option_sensitivity = 1000;}
							else if (sb_mode == "ELS") {sb_option_sensitivity = 2500;}

							#Aim point factor
							var sb_aim_point_sensitivity = 0;
							if (aim_point_string == "CLSE") {sb_aim_point_sensitivity = 1000;}
							

							#SB retract angle as a function of Downrange sensitivity / Final setting (Max 80% of SB) Windspeed sensitivity removed for now ( too strong correction )
							var total_downrange = density_sensitivity + eas_error_sensitivity + weight_sensitivity + sb_option_sensitivity + sb_aim_point_sensitivity;
							TAEM_guidance_TGSBC.DSBC = math.min(0.01307 * total_downrange + 15, 80);

							#One time loop and setting
							TAEM_guidance_TGSBC.first_retract_flag  = 1;

							#print("winspeed sensitivity is : ", windspeed_sensitivity, " density_sensitivity is : ", density_sensitivity, " eas_error_sensitivity is: ", eas_error_sensitivity, " weight_sensitivity: ", weight_sensitivity);
							#print("sb_option_sensitivity is : ", sb_option_sensitivity, " sb_aim_point_sensitivity is : ", sb_aim_point_sensitivity);
							#print("total downrange is : ", total_downrange);
							#print("3000 time stamp : ", TAEM_guidance_TGSBC.timestamp_3000);
							}
						
						#Second retract recomputation below 500 feet based on delta Wind between 3000 and 500 feet
						else if ((TAEM_jsbsim.H_qfe < 500 ) and (TAEM_guidance_TGSBC.second_retract_flag == 0))
							{
							#Second retract based on windshifts only (max 50° more)
							
							#Effective wind at 500ft stored
							TAEM_guidance_TGSBC.timestamp_500 = sim_time;
							TAEM_guidance_TGSBC.windspeed_500 = windspeed_kts;
							#print("500 time stamp : ", TAEM_guidance_TGSBC.timestamp_500);

							#Mean effective wind since 3000 ft
							#TAEM_guidance_TGSBC.windspeed_int = (TAEM_guidance_TGSBC.windspeed_int * TAEM_guidance_TGINIT.DTG) / (TAEM_guidance_TGSBC.timestamp_500 - TAEM_guidance_TGSBC.timestamp_3000);
							
							#Wind changes since 3000 feet
							TAEM_guidance_TGSBC.windshift_500 = TAEM_guidance_TGSBC.windspeed_500 - TAEM_guidance_TGSBC.windspeed_3000;

							#Sb previous value adjusted
							TAEM_guidance_TGSBC.DSBC = math.min(TAEM_guidance_TGSBC.DSBC + 0.01307 * 84.2 * TAEM_guidance_TGSBC.windshift_500, 80);

							#One time loop and setting
							TAEM_guidance_TGSBC.second_retract_flag = 1;

							#print("windshift at 500 feet is: ", TAEM_guidance_TGSBC.windshift_500);
							}
						
						#WOW: full SB
						else if (SpaceShuttle.slowdown_loop_flag == 1) {TAEM_guidance_TGSBC.DSBC = 100;}
						}

					}
				}
			}
		}

	#Final limited SB command
	TAEM_guidance_TGSBC.DSBC_AT = SpaceShuttle.MIDVAL(TAEM_guidance_TGSBC.DSBC, TAEM_guidance_TGSBC.DSBCLL, TAEM_guidance_TGSBC.DSBCUL);

	}

#Start of SB control loop if not started during GRTLS 
if (TAEM_guidance_TGINIT.SB_LOOP == 0) {TAEM_Speedbrake_control();}

#print("SB lower limit is: ", TAEM_guidance_TGSBC.DSBCLL, " SB upper limit is: ", TAEM_guidance_TGSBC.DSBCUL, " SB Energy limit is: ", TAEM_guidance_TGSBC.SB_energy_UL);
#print("SB commanded is: ", TAEM_guidance_TGSBC.DSBC_AT);
};



# TAEM Speedbrake control #

var TAEM_Speedbrake_control = func {

#Control of the SB out of the TGSBC function to work at the correct rate ( 10.9 deg/s and halved with one APU remaining  ---> PRL)
if (TAEM_guidance_TGINIT.SB_LOOP == 0) {TAEM_guidance_TGINIT.SB_LOOP = 1;}

if (getprop("/fdm/jsbsim/systems/ap/automatic-sb-control") == 1)	
		{
		var sb_state = getprop("/controls/shuttle/speedbrake");
		var sb_max = TAEM_guidance_TGSBC.DSBC_AT / 100;

		if (sb_state > (sb_max + 0.01)) {SpaceShuttle.decrease_speedbrake();}
		else if (sb_state < (sb_max - 0.01)) {SpaceShuttle.increase_speedbrake();}
		
		}

#End of SB control after WOW or if no TAEM guidance available anymore
if ((TAEM_guidance_TGINIT.AL_END == 1) or ((TAEM_guidance_available == 0) and (GRTLS.INIT_PASS == 0))) {return;}

#dT = 0.1 / it gives 10°/s for closing and 5°/s for opening
settimer( func {TAEM_Speedbrake_control(); }, 0.1);

};




# TAEM Roll commanded - TGPHIC  ###########################################################

var TAEM_TGPHIC = func {

#Roll Command Limits (Mach and IPHASE dependant)
TAEM_guidance_TGPHIC.PHILIMIT = SpaceShuttle.MIDVAL(TAEM_guidance_TGPHIC.PHIMIN - 300 * (TAEM_jsbsim.mach - 0.95), TAEM_guidance_TGPHIC.PHIMIN, TAEM_guidance_TGINIT.PHILIM);

#TAEM
if (TAEM_guidance_TGINIT.TG_END == 0)
	{
	#1 S-turns 
	if (TAEM_guidance_TGINIT.IPHASE == 0) 
		{
		TAEM_guidance_TGPHIC.PHIC = TAEM_guidance_TGTRAN.S_sign * TAEM_guidance_TGPHIC.PHILIMIT;
		}

	#2 ACQ (2.5 * Delta Az)
	else if (TAEM_guidance_TGINIT.IPHASE == 1) 
		{
		TAEM_guidance_TGPHIC.TTH = time_to_hac();
		TAEM_guidance_TGPHIC.PHIC = 2.5 * TAEM_guidance_GTP.DPSAC;
		}

	#3 HAC
	else if (TAEM_guidance_TGINIT.IPHASE == 2)
		{
		#Radial error
		TAEM_guidance_TGPHIC.RERRC = TAEM_guidance_GTP.RCIRC - TAEM_guidance_GTP.RTURN;

		#Too far from HAC (1.15 Nm / 7000 feet in the docs and 1500 feet in sim), guidance to tangent point 
		if (TAEM_guidance_TGPHIC.RERRC > 1500) 
			{
			#Max roll back to 50°
			if (TAEM_guidance_TGPHIC.PHILIMIT > 50) {TAEM_guidance_TGPHIC.PHILIMIT = 50;}
			TAEM_guidance_TGPHIC.PHIC = 2.5 * TAEM_guidance_GTP.DPSAC;
			}

		#Radius error guidance
		else
			{
			#Actual radial rate
			TAEM_guidance_TGPHIC.RDOT = -(TAEM_guidance_GTP.XCIR * rwy_coord.X_dot + TAEM_guidance_GTP.YCIR * rwy_coord.Y_dot) / TAEM_guidance_GTP.RCIRC;

			#Centrifugal force acceleration counter variable
			TAEM_guidance_TGPHIC.PHIP2C = 57.29578 * (math.pow(TAEM_jsbsim.VH, 2) - math.pow(TAEM_guidance_TGPHIC.RDOT, 2)) / (32.174 * TAEM_guidance_GTP.RTURN);

			#Reference radial rate
			TAEM_guidance_TGPHIC.RDOTRF = -57.29578 * TAEM_jsbsim.VH * (0.186 * TAEM_guidance_GTP.PSHA) / TAEM_guidance_GTP.RTURN;

			#Roll commanded clamp to a max to avoid to turn away from the HAC
			TAEM_guidance_TGPHIC.PHIC = TAEM_guidance_GTP.YSGN * math.max(0, TAEM_guidance_TGPHIC.PHIP2C + 0.02 * TAEM_guidance_TGPHIC.RERRC + 0.2 * (TAEM_guidance_TGPHIC.RDOT - TAEM_guidance_TGPHIC.RDOTRF));
			}
		}

	#4 Pre-final 
	else if (TAEM_guidance_TGINIT.IPHASE == 3) 
		{
		#Roll based on final axis lateral error
		TAEM_guidance_TGPHIC.YERRC = SpaceShuttle.MIDVAL(-TAEM_guidance_TGPHIC.GY * rwy_coord.Y, -TAEM_guidance_TGPHIC.YERRLM, TAEM_guidance_TGPHIC.YERRLM);
		TAEM_guidance_TGPHIC.PHIC = TAEM_guidance_TGPHIC.YERRC - TAEM_guidance_TGPHIC.GYDOT * rwy_coord.Y_dot;

		#Unstable
			#In case of large bank is needed, Roll Limit goes from 30 to 60°
			#if (math.abs(TAEM_guidance_TGPHIC.PHIC) > 100) {TAEM_guidance_TGPHIC.PHILIMIT = 60;}

		#Roll fader for first 5 iterations to avoid oscillations (ISR Roll fader constant) // More stable without
		#if (TAEM_guidance_TGINIT.ISR > 0)
		#	{
		#	TAEM_guidance_TGPHIC.DPHI = (TAEM_guidance_TGPHIC.PHIC - TAEM_guidance_TGPHIC.PHIO) / TAEM_guidance_TGINIT.ISR;
		#	TAEM_guidance_TGINIT.ISR = TAEM_guidance_TGINIT.ISR - 1;
		#	TAEM_guidance_TGPHIC.PHIC = TAEM_guidance_TGPHIC.PHIO + TAEM_guidance_TGPHIC.DPHI;
		#	TAEM_guidance_TGPHIC.PHIO = TAEM_guidance_TGPHIC.PHIC;
		#	}

		}
	}


#Autoland (Tsikalas paper)
else
	{
	#Y error clamped
	TAEM_guidance_TGPHIC.YERR = SpaceShuttle.MIDVAL(rwy_coord.Y, -TAEM_guidance_TGPHIC.YLIM, TAEM_guidance_TGPHIC.YLIM);

	#Y error integration
	TAEM_guidance_TGPHIC.YINTERR = SpaceShuttle.MIDVAL(TAEM_guidance_TGPHIC.YERR , -TAEM_guidance_TGPHIC.YINTLIM, TAEM_guidance_TGPHIC.YINTLIM) * TAEM_guidance_TGPHIC.AINT * TAEM_guidance_TGINIT.DTG;

	#Phi unlimited f(Y, Ydot, Yint error)
	TAEM_guidance_TGPHIC.PHIC = -rwy_coord.Y_dot * TAEM_guidance_TGPHIC.KYDOT - TAEM_guidance_TGPHIC.YERR * TAEM_guidance_TGPHIC.KY1 - TAEM_guidance_TGPHIC.YINTERR;

	#If WOW, phi and phi commanded = 0 // no roll rate error
	if (TAEM_guidance_TGPHIC.FLATTURN == 1) 
		{
		TAEM_guidance_TGPHIC.PHIC = 0;
		TAEM_jsbsim.PHIR = 0;
		}
	}
	

#5 Unlimited Roll commanded is then limited with TGTRAN Phi Limits
if (getprop("/fdm/jsbsim/systems/ap/auto-bailout-active") == 1) {TAEM_guidance_TGPHIC.PHIC_AT = 0;}
else 
	{
	TAEM_guidance_TGPHIC.PHIC_AT = SpaceShuttle.MIDVAL(TAEM_guidance_TGPHIC.PHIC, -TAEM_guidance_TGPHIC.PHILIMIT, TAEM_guidance_TGPHIC.PHILIMIT);
	}

#6 To be done when TGNZC will be completed (we need Pitch NZ commanded): OI-22 final limitations update for High G due to negative Pitch (Strong Windshears)


#print("Phi limit is : ", TAEM_guidance_TGPHIC.PHILIMIT, " ISR is : ", TAEM_guidance_TGINIT.ISR, " Roll commanded is: ", TAEM_guidance_TGPHIC.PHIC_AT, " Y int is: ", TAEM_guidance_TGPHIC.YINTERRC);
#print ("RERRC is : ", TAEM_guidance_TGPHIC.RERRC);
};



# TAEM FCS function - TDAP - Pitch/Roll/Yaw Input from TAEM functions ---> Body Axis rate output for Jsbsim FCS  ###########################################################

#Main TDAP loop
var TAEM_TDAP = func {

### Pitch ###

#Nz hold pitch guidance
if (TAEM_guidance_TGINIT.IPHASE == 5) 
	{
	#Nz error
	TAEM_guidance_TDAP.DNZCMP = -math.cos(TAEM_jsbsim.THETAR) / SpaceShuttle.MIDVAL(math.cos(TAEM_jsbsim.PHIR), 0.5, 1);
	setprop("/fdm/jsbsim/systems/ap/grtls/Nz-cmd", math.min(TAEM_guidance_TGNZC.NZC - TAEM_guidance_TDAP.DNZCMP, GRTLS.DGRNZT));
	}

#TAEM pitch guidance
else
	{
	#Filtered coordinated turn pitch rate (RTANP)
	TAEM_guidance_TDAP.XIN = TAEM_jsbsim.R * SpaceShuttle.MIDVAL(math.tan(TAEM_jsbsim.PHIR), -1, 1);
	TAEM_guidance_TDAP.RTANP = TDAP_filter(1, TAEM_guidance_TDAP.XIN);

	#Nz error
	TAEM_guidance_TDAP.DNZCMP = -math.cos(TAEM_jsbsim.THETAR) / SpaceShuttle.MIDVAL(math.cos(TAEM_jsbsim.PHIR), 0.5, 1);
	TAEM_guidance_TDAP.NZERR = TAEM_jsbsim.NZ - (1 + TAEM_guidance_TDAP.VCO / TAEM_jsbsim.V_true) * TAEM_guidance_TGNZC.NZC + TAEM_guidance_TDAP.DNZCMP;
	#print("NZ err is : ", TAEM_guidance_TDAP.NZERR);

	#Nzerr conversion to negative Pitch rate (QC)
	TAEM_guidance_TDAP.XIN = TAEM_guidance_TDAP.NZERR * TAEM_guidance_TDAP.GQN;
	TAEM_guidance_TDAP.QC = TDAP_filter(2, TAEM_guidance_TDAP.XIN);

	#print("NZ err pitch rate is is : ", TAEM_guidance_TDAP.NZERR * TAEM_guidance_TDAP.GQN);

	#Unlimited negative Pitch rate (BCSL)
	TAEM_guidance_TDAP.XIN = TAEM_guidance_TDAP.QC - TAEM_guidance_TDAP.RTANP;
	TAEM_guidance_TDAP.BCSL = TDAP_filter(3, TAEM_guidance_TDAP.XIN);

	#Angle of Attack limitations (to be finished)
	#TAEM_guidance_TDAP.ALPMIN = SpaceShuttle.MIDVAL(TAEM_guidance_TDAP.AMNS * TAEM_jsbsim.mach + TAEM_guidance_TDAP.AMNI, -100, 100);

	#Negative Pitch rate commanded into jsbsim (Rad)
	setprop("/fdm/jsbsim/systems/ap/taem/pitch_rate_target", TAEM_guidance_TDAP.BCSL * 0.0174533);

	#print("Pitch rate target is : ", TAEM_guidance_TDAP.BCSL);
	}


### Roll ###

#Roll rate limitations
var GPBANK = SpaceShuttle.MIDVAL(TAEM_guidance_TDAP.GPS * TAEM_jsbsim.mach + TAEM_guidance_TDAP.GPI, TAEM_guidance_TDAP.GPLL, TAEM_guidance_TDAP.GPUL);
var PCLIM = SpaceShuttle.MIDVAL(TAEM_guidance_TDAP.PCS * TAEM_jsbsim.mach + TAEM_guidance_TDAP.PCI, TAEM_guidance_TDAP.PCLL, TAEM_guidance_TDAP.PCUL);

#Bank error history for HUD diamond guidance
var i = (size(TAEM_guidance_TDAP.BANKER_prev) - 1);
while (i > 0 )
	{
	TAEM_guidance_TDAP.BANKER_prev[i] = TAEM_guidance_TDAP.BANKER_prev[i - 1];
	i = i - 1;
	}
TAEM_guidance_TDAP.BANKER_prev[0] = TAEM_guidance_TDAP.BANKER;

TAEM_guidance_TDAP.BANKER = TAEM_guidance_TGPHIC.PHIC_AT - TAEM_jsbsim.PHIR / 0.0174533;
#print ("Roll error is : ", TAEM_guidance_TDAP.BANKER);

#Zero Bank error sum at loop start
TAEM_guidance_TDAP.BANKER_sum = 0;

#Bank error for HUD flight director
for (var i = 0; i < size(TAEM_guidance_TDAP.BANKER_prev); i = i + 1)
	{
	TAEM_guidance_TDAP.BANKER_sum = TAEM_guidance_TDAP.BANKER_sum + TAEM_guidance_TDAP.BANKER_prev[i];
	}

TAEM_guidance_TDAP.BANKER_sum = TAEM_guidance_TDAP.BANKER_sum / size(TAEM_guidance_TDAP.BANKER_prev);
#print ("roll error mean is : ", TAEM_guidance_TDAP.BANKER_sum);

#Bank error conversion into roll rate error
TAEM_guidance_TDAP.BRATE = SpaceShuttle.MIDVAL(TAEM_guidance_TDAP.BANKER * GPBANK, -PCLIM, PCLIM) * 0.0174533; 

#Roll rate tgt and bank error into jsbsim Roll channel GRTLS.IBNK
if (((GRTLS.IBNK == "ON") and (TAEM_guidance_TGINIT.IPHASE == 5)) or (TAEM_guidance_TGINIT.IPHASE == 4))
	{
	setprop("/fdm/jsbsim/systems/ap/grtls/roll-cmd-alpha-transition", SpaceShuttle.MIDVAL(TAEM_guidance_TDAP.BANKER * TAEM_guidance_TDAP.GPLL, -10, 10) * 0.0174533);
	#print ("Roll rate target in degrees is : ", SpaceShuttle.MIDVAL(TAEM_guidance_TDAP.BANKER * 0.5, -10, 10));
	} 
#else if ((TAEM_guidance_TGINIT.IPHASE == 5) or (TAEM_guidance_TGINIT.IPHASE == 6))
#	{
#	setprop("/fdm/jsbsim/systems/ap/grtls/roll-cmd-alpha-transition", SpaceShuttle.MIDVAL(TAEM_guidance_TDAP.BANKER * TAEM_guidance_TDAP.GPLL, -2, 2) * 0.0174533);
#	print ("Roll rate target in degrees is : ", SpaceShuttle.MIDVAL(TAEM_guidance_TDAP.BANKER * 0.5, -2, 2));
#	}
else if ((TAEM_guidance_TGINIT.IPHASE < 4) or (TAEM_guidance_TGINIT.IPHASE == 7)) 
	{
	setprop("/fdm/jsbsim/systems/ap/taem/roll_rate_target", TAEM_guidance_TDAP.BRATE);
	#print ("Roll rate target in degrees is : ", TAEM_guidance_TDAP.BRATE / 0.0174533);
	} 


#setprop("/fdm/jsbsim/systems/ap/taem/bank-error", TAEM_guidance_TDAP.BANKER * 0.05); #deg with Thorsten gain (0.05)
#setprop("/fdm/jsbsim/systems/ap/taem/phic_at", TAEM_guidance_TGPHIC.PHIC_AT);



};



### Filters ###

#Generalized first order filter used in TDAP // NF is the filter number (same filter but different variables) // XIN is the variable to be filtered
var TDAP_filter = func (NF, XIN) {

#Variables dependant of the update speed (DTG) and NF
var GX2 = 0;
var GX3 = 0;

#Filtered coordinated turn pitch rate (R*TANPHI)
if (NF == 1)
	{
	#Filter coefficients
	GX2 = (1 - math.exp(-TAEM_guidance_TGINIT.DTG));
	GX3 = -math.exp(-TAEM_guidance_TGINIT.DTG);

	#First pass
	if (TAEM_guidance_TGINIT.LOOP == 0)
		{
		TAEM_guidance_TDAP.XI_1 = XIN;
		TAEM_guidance_TDAP.XO_1 = XIN * GX2 / (1 + GX3);
		}

	else
		{
		TAEM_guidance_TDAP.XO_1 = GX2 * TAEM_guidance_TDAP.XI_1 - GX3 * TAEM_guidance_TDAP.XO_1;
		TAEM_guidance_TDAP.XI_1 = XIN;
		}

	#print ("RTANP is: ", TAEM_guidance_TDAP.XO_1);

	#XIN filtered
	return TAEM_guidance_TDAP.XO_1;
	}


#Filtered pitch rate error (NZerr * GQN)
else if (NF == 2)
	{
	#Filter coefficients
	GX2 = ((1 - math.exp(-2 * TAEM_guidance_TGINIT.DTG)) / 2) - 0.2;
	GX3 = -math.exp(-2 * TAEM_guidance_TGINIT.DTG);

	#First pass
	if (TAEM_guidance_TGINIT.LOOP == 0)
		{
		TAEM_guidance_TDAP.XI_2 = XIN;
		TAEM_guidance_TDAP.XO_2 = XIN * (0.2 + GX2) / (1 + GX3);
		}

	else
		{
		TAEM_guidance_TDAP.XO_2 = XIN * 0.2 + GX2 * TAEM_guidance_TDAP.XI_2 - GX3 * TAEM_guidance_TDAP.XO_2;
		TAEM_guidance_TDAP.XI_2 = XIN;
		}

	#print ("QC is : ", TAEM_guidance_TDAP.XO_2);

	#XIN filtered
	return TAEM_guidance_TDAP.XO_2;
	}


#Unlimited pitch error rate (QC - RTANP)
else if (NF == 3)
	{
	#Filter coefficients
	GX2 = ((1 - math.exp(-2 * TAEM_guidance_TGINIT.DTG)) / 2) - 2;
	GX3 = -math.exp(-2 * TAEM_guidance_TGINIT.DTG);

	#First pass
	if (TAEM_guidance_TGINIT.LOOP == 0)
		{
		TAEM_guidance_TDAP.XI_3 = XIN;
		TAEM_guidance_TDAP.XO_3 = XIN * (2 + GX2) / (1 + GX3);
		}

	else
		{
		TAEM_guidance_TDAP.XO_3 = XIN * 2 + GX2 * TAEM_guidance_TDAP.XI_3 - GX3 * TAEM_guidance_TDAP.XO_3;
		TAEM_guidance_TDAP.XI_3 = XIN;
		}

	#print ("BCSL is : ", TAEM_guidance_TDAP.XO_3);

	#XIN filtered
	return TAEM_guidance_TDAP.XO_3;
	}					

};


#Lag filter C1/(C1 + s) to avoid input/output with the jsbsim FGfilter function / Used for several filtered Autoland parameters
#NF (Number of filter) allows for specific Xin and Xout previous stored variables in the hash.

var TAEM_lag_filter = func (NF, C1, XIN) {

var denom = 2.0 + TAEM_guidance_TGINIT.DTG * C1;
var ca = (TAEM_guidance_TGINIT.DTG * C1) / denom;
var cb = (2 - TAEM_guidance_TGINIT.DTG * C1) / denom;

#V_true filter for open loop flare A13
if (NF == 1)
	{
	TAEM_lag_filter_variables.XO_1 = (XIN + TAEM_lag_filter_variables.XI_1) * ca + TAEM_lag_filter_variables.XO_1 * cb;
	TAEM_lag_filter_variables.XI_1 = XIN;
	return TAEM_lag_filter_variables.XO_1;

	#print ("Lag filter Vt filtered is : ", TAEM_lag_filter_variables.XO_1);
	}

#Flare open loop NZC A40
else if (NF == 2)
	{
	TAEM_lag_filter_variables.XO_2 = (XIN + TAEM_lag_filter_variables.XI_2) * ca + TAEM_lag_filter_variables.XO_2 * cb;
	TAEM_lag_filter_variables.XI_2 = XIN;
	return TAEM_lag_filter_variables.XO_2;

	#print ("Lag filter open loop flare Nz is : ", TAEM_lag_filter_variables.XO_2);
	}

#Final flare Hdot ref A13
else if (NF == 3)
	{
	TAEM_lag_filter_variables.XO_3 = (XIN + TAEM_lag_filter_variables.XI_3) * ca + TAEM_lag_filter_variables.XO_3 * cb;
	TAEM_lag_filter_variables.XI_3 = XIN;
	return TAEM_lag_filter_variables.XO_3;

	#print ("Final flare hdot ref filtered is : ", TAEM_lag_filter_variables.XO_3);
	}

#EAS filtered for SB EAS error A14
else if (NF == 4)
	{
	TAEM_lag_filter_variables.XO_4 = (XIN + TAEM_lag_filter_variables.XI_4) * ca + TAEM_lag_filter_variables.XO_4 * cb;
	TAEM_lag_filter_variables.XI_4 = XIN;
	return TAEM_lag_filter_variables.XO_4;

	#print ("EAS sb filtered is : ", TAEM_lag_filter_variables.XO_4);
	}

#Filter for TAEM bank error 
else if (NF == 5)
	{
	TAEM_lag_filter_variables.XO_5 = (XIN + TAEM_lag_filter_variables.XI_5) * ca + TAEM_lag_filter_variables.XO_5 * cb;
	TAEM_lag_filter_variables.XI_5 = XIN;
	return TAEM_lag_filter_variables.XO_5;

	#print ("Bank error filtered is : ", TAEM_lag_filter_variables.XO_5);
	}

}

# smart flare guidance ##################################################################

var smart_flare  = func (alt_agl, airspeed, vspeed) {

var alt_m = alt_agl * 0.3048;
var vspeed_m = vspeed * 0.3048;
var hspeed_m = math.sqrt(math.pow(airspeed * 0.3048, 2.0) - math.pow(vspeed_m, 2.0));

var alt_tgt_m = 190.0 * 0.3048;
var angle = -TAEM_jsbsim.gamma;
var angle_tgt = 1.5;
var angle_rad = angle * math.pi/180.0;
var angle_tgt_rad = angle_tgt * math.pi/180.0; 


var t = (alt_m - alt_tgt_m) / (0.5 *  hspeed_m * (angle_rad + angle_tgt_rad ));
var a = (angle - angle_tgt) / t;

return a;

}


# virtual rwy symbology for the HUD #######################################################

var update_HUD_symbology = func (pos) {


var vdist = pos.alt() - TAEM_threshold.alt();
var heading = getprop("/orientation/heading-deg");


# aim point and touchdown point

var dist = pos.distance_to(TAEM_threshold);
var course = pos.course_to(TAEM_threshold);
var vAngle_rad = math.atan2(vdist, dist);

var aim_dist = pos.distance_to(TAEM_AP);
var aim_course= pos.course_to(TAEM_AP);
var vAimAngle_rad = math.atan2(vdist, aim_dist);

HUD_data_set.vangle_aim = vAimAngle_rad * 180.0/math.pi;
HUD_data_set.hangle_aim = (aim_course - heading);

#print("vangle aim is: ", HUD_data_set.vangle_aim);

HUD_data_set.vangle_threshold = math.atan2(vdist, dist) * 180.0/math.pi;
HUD_data_set.hangle_threshold = (course - heading);

# virtual runway edges

var nr_dist = pos.distance_to(TAEM_rwy_nr);
var nr_course = pos.course_to(TAEM_rwy_nr);

HUD_data_set.vangle_nr = math.atan2(vdist, nr_dist) * 180.0/math.pi;
HUD_data_set.hangle_nr = (nr_course - heading);


var nl_dist = pos.distance_to(TAEM_rwy_nl);
var nl_course = pos.course_to(TAEM_rwy_nl);

HUD_data_set.vangle_nl = math.atan2(vdist, nl_dist) * 180.0/math.pi;
HUD_data_set.hangle_nl = (nl_course - heading);

var fr_dist = pos.distance_to(TAEM_rwy_fr);
var fr_course = pos.course_to(TAEM_rwy_fr);

HUD_data_set.vangle_fr = math.atan2(vdist, fr_dist) * 180.0/math.pi;
HUD_data_set.hangle_fr = (fr_course - heading);


var fl_dist = pos.distance_to(TAEM_rwy_fl);
var fl_course = pos.course_to(TAEM_rwy_fl);

HUD_data_set.vangle_fl = math.atan2(vdist, fl_dist) * 180.0/math.pi;
HUD_data_set.hangle_fl = (fl_course - heading);

}



# set threshold for TAEM guidance #########################################################

var set_TAEM_threshold = func (site_string, runway_string) {


var data = SpaceShuttle.landing_site_data.entry_by_name(site_string);

if (data.name != site_string)
	{
	SpaceShuttle.callout.make("No TAEM guidance data to site available.", "help");
	TAEM_threshold.set_lat(0.0);
	TAEM_threshold.set_lon(0.0);
	return;
	}

if (data.rwy_pri == runway_string)
	{
	TAEM_threshold.set_latlon(data.TAEM_pri_lat, data.TAEM_pri_lon);
	TAEM_threshold.heading = data.TAEM_pri_heading;	
	TAEM_threshold.elevation = data.TAEM_pri_elevation;
	TAEM_threshold.slope = data.TAEM_pri_slope;
	TAEM_threshold.MLS_channel = data.TAEM_pri_MLS_channel;
	TAEM_threshold.MLS_available = data.TAEM_pri_MLS_flag;
	TAEM_threshold.set_alt(TAEM_threshold.elevation * 0.3048);
	TAEM_threshold.rwy_length = data.TAEM_rwy_length;
	SpaceShuttle.EGRT_data.RWID = 1; #runway ID for entry guidance
	}
else
	{
	TAEM_threshold.set_latlon(data.TAEM_sec_lat, data.TAEM_sec_lon);
	TAEM_threshold.heading = data.TAEM_sec_heading;	
	TAEM_threshold.elevation = data.TAEM_sec_elevation;
	TAEM_threshold.slope = data.TAEM_sec_slope;
	TAEM_threshold.MLS_channel = data.TAEM_sec_MLS_channel;
	TAEM_threshold.MLS_available = data.TAEM_sec_MLS_flag;
	TAEM_threshold.set_alt(TAEM_threshold.elevation * 0.3048);
	TAEM_threshold.rwy_length = data.TAEM_rwy_length;
	SpaceShuttle.EGRT_data.RWID = 2;
	}



}

# set threshold for Alt Sites Entry And Contigency #########################################################

#Always the primary runway selected 
var set_alt_site_threshold = func (site_index) {

var data = SpaceShuttle.landing_site_data.entry_by_index(site_index);
var array = [];

if (data.index != site_index)
	{
	SpaceShuttle.callout.make("No Alternate site valid", "help");
	append(array, 0, 0, "", "", "");
	}

else
	{
	append(array,data.TAEM_pri_lat, data.TAEM_pri_lon, data.index, data.rwy_pri_name, data.shortname);
	}

return array;

}


#Dist in Nm with distance to runway nm in hsit.nas

var get_hsit_scale = func (dist) {

var hac_init = getprop("/fdm/jsbsim/systems/ap/taem/hac-turn-init");
var al_init = getprop("/fdm/jsbsim/systems/ap/taem/al-init");

#Scale almost constant up to Hac intercept
if ((hac_init == 0) and (al_init == 0))
	{
	if (dist > 70) {return 0.2;} #70Nm (Vert traj 1 boundary)
	else if (dist < 25) {return 1.0;} #28 Nm (360 ° HAC)
	else {return 1.0 - 0.8 * ((dist - 25)/45);}
	}

#Then it really starts to get bigger up to A/L
else if (hac_init == 1)
	{
	if (dist > 25) {return 1.0;} #26 Nm 360° HAC
	else if (dist < 7) {return 2.5;} #7 Nm A/L
	else {return 2.5 - 1.5 * ((dist - 7)/18);}
	}

#Runway in A/L even bigger 
else if (al_init == 1)
	{
	return 3.0;
	}


}


var get_hsit_x = func (dist, rel_angle, scale) {

var dist_x = math.sin(rel_angle) * dist;
return x = 265 + dist_x / 240.0 * scale;


}

var get_hsit_y = func (dist, rel_angle, scale) {

var dist_y = math.cos (rel_angle) * dist;
return 265 - dist_y / 240.0 * scale;

}


#Time to HAC function used for Spec 50 and TAEM pfd mode
var time_to_hac = func {

var gs_kts = getprop("/velocities/groundspeed-kt");
var wp_1_distance = SpaceShuttle.TAEM_guidance_GTP.RTAN / 6076.12;
var d_transition = SpaceShuttle.TAEM_guidance_GTP.RTURN * math.sqrt(math.pow(TAEM_guidance_GTP.HAC_transition, 2) - 1) / 6076.12;

var wp_1_time = ((wp_1_distance - d_transition) / gs_kts) * 3600; 

#print("time before hac is: ", wp_1_time);
return wp_1_time;
			
}


######################################################################################################################################################################

##### Older TAEM functions (used before April 2021 and TAEM original logic simulation) #####


# A/L Approach guidance Loop (HUD guidance mainly) (Done directly in TGNZC for Hud cue commanded 06/2021)  ###########################################################

var TAEM_ALEXEC = func {


#OGS acquire or track
if ((TAEM_guidance_TGINIT.PMODE == 1) or (TAEM_guidance_TGINIT.PMODE == 2))
	{
	#Final OGS path 20° (IGS 1) or 18° (IGS 2)
	#var OGS_gamma = 20.0;
	#if (TAEM_guidance_XHAC.IGS == 2) {OGS_gamma = 18.0;}
	
	var OGS_gamma = math.atan2(TAEM_guidance_TGCOMP.HREF, TAEM_guidance_GTP.RPRED + TAEM_guidance_XHAC.XA) * 57.29578;
	var delta_gamma = math.atan2(-TAEM_guidance_TGCOMP.HERROR, TAEM_guidance_GTP.RPRED + TAEM_guidance_XHAC.XA) * 57.29578;
	var correction_factor = 1 + math.abs(TAEM_guidance_TGCOMP.HERROR / TAEM_guidance_TGCOMP.HREF);

	#Gamma plus Delta_gamma
	HUD_data_set.vangle_guidance = OGS_gamma + correction_factor * delta_gamma;  
	#print("correction_factor is: ", correction_factor);

	#var GLSD = (HUD_data_set.vangle_aim - OGS_gamma);
	#HUD_data_set.vangle_guidance = OGS_gamma + 1.0 * GLSD;
	}

#Pre-Flare
else if (TAEM_guidance_TGINIT.PMODE == 3)
	{
	# use smart pull-up guidance
		var pull_up_speed = smart_flare (TAEM_jsbsim.H_qfe, TAEM_jsbsim.V_ground, TAEM_jsbsim.HDOT);
		#IGS intercept altitude

		if (TAEM_jsbsim.H_qfe > 195)
			{
			HUD_data_set.vangle_guidance = HUD_data_set.vangle_guidance - pull_up_speed * TAEM_guidance_TGINIT.DTG;
			if (HUD_data_set.vangle_guidance < 1.5) {HUD_data_set.vangle_guidance = 1.5;}
			}
		else {HUD_data_set.vangle_guidance = 1.5;}

	#Circular flare Gamma targeted from TAEM/Autoland guidance (clamped between 20 and 1.5°)
	#HUD_data_set.vangle_guidance = -math.atan2(TAEM_guidance_TGCOMP.DHDRRF, 1) * 57.29578;
	}


else if (TAEM_guidance_TGINIT.PMODE == 4)
	{
	#Final flare Hdot aim is 2fps
	HUD_data_set.vangle_guidance = math.atan2(2, TAEM_jsbsim.VH) * 57.29578;
	}

#No AL END for approach trainer (stage 4)
else if ((TAEM_jsbsim.V_true < 280.0) and (TAEM_guidance_TGINIT.PMODE == 5) and (getprop("/sim/presets/stage") != 4))
	{
	TAEM_guidance_TGINIT.AL_END = 1;
	#print ("Approach guidance signing off!");
	return;
	}

#print("HUD commanded gamma is: ", HUD_data_set.vangle_guidance);
}



# the central TAEM guidance loop #########################################################

var TAEM_guidance_loop = func (stage, radius_error_last) {

#Order for TAEM functions: XHAC / GTP / TGCOMP / TGTRAN / TGNZC / TGSBC / TGPHIC then Pitch/Roll into jsbsim

TAEM_loop_running = 1;

#Computations for coordinates in Rwy frame
#TAEM_rwy_frame();

#GTP / Handles distance computations
#TAEM_GTP();

#var pos = geo.aircraft_position();
var pos = state_vector_position();

#Velocity variables
var mach = getprop("/fdm/jsbsim/velocities/mach");
var V_true = getprop("/fdm/jsbsim/velocities/vtrue-fps");
var VH = getprop("/fdm/jsbsim/velocities/u-fps");

#Altitude above Runway Threshold for Path calculations
var alt_qfe = getprop("/position/altitude-ft") - SpaceShuttle.TAEM_threshold.elevation;

#Wp1 distance (RTAN)
var dist_wp1 = pos.distance_to(TAEM_WP_1) / 1853.0;

TAEM_predictor_set.update();

if (TAEM_guidance_available == 0)
	{
	TAEM_loop_running = 0;
	return;
	} 

update_HUD_symbology(pos);
#area_nav_set.update_signals();


### IPHASE 1 (ACQ) ###

if (stage == 1) # glide to WP 1
	{
	var course = pos.course_to(TAEM_WP_1);
		
	setprop("/fdm/jsbsim/systems/taem-guidance/course", course);

	var heading = getprop("/orientation/heading-deg");

	var glideslope_deviation = SpaceShuttle.get_glideslope_deviation(alt_qfe, TAEM_guidance_GTP.RPRED / 6076.12);
	### TGCOMP ###
	TAEM_TGCOMP();

	### TGTRAN ###
	TAEM_TGTRAN();

	#Clamp here GS deviation to avoid huge value in vspeed target (taem ap)
	if ((mach > 1.0) and (dist_wp1 < 10.0)) #Pull up 10.0 Nm before entering the HAC to be subsonic once into it 
		{
		#10000 feet low Bias ---> Hdot of + 400 feetish commanded // 2Gish Nz max
		setprop("/fdm/jsbsim/systems/taem-guidance/glideslope-deviation-ft", 7500);
		}
	else
		{
		#glideslope_deviation = SpaceShuttle.MIDVAL(glideslope_deviation, -10000, 5000);
		setprop("/fdm/jsbsim/systems/taem-guidance/glideslope-deviation-ft", TAEM_guidance_TGCOMP.HERROR);
		}

	#TAEM_energy_management();
	
	#TGTRAN
	#Normaly Transition is when pos.distance_to(HAC center) < 1.1 * RTURN (almost the same than 1.5Nm before wp1)
	if (dist_wp1 < 1.5) {
			SpaceShuttle.callout.make("Turn "~TAEM_WP_1.turn_direction~" into HAC!", "help");
			TAEM_guidance_string = "HDG";
			TAEM_guidance_phase = 2;

			TAEM_guidance_TGINIT.IPHASE = 2;
			TAEM_guidance_TGINIT.PHILIM = 60;	

			SpaceShuttle.callout.make("Commander, you should take CSS by now and make John Young proud of you", "help");
			setprop("/fdm/jsbsim/systems/ap/taem/hac-turn-init", 1);
			#print("Waypoint 1 reached!");
			
			setprop("/fdm/jsbsim/systems/ap/taem/set-bank-target", TAEM_guidance_GTP.YSGN * 30.0);
			stage = 2;
			}

	}

### IPHASE 2 (HDG) ###

else if (stage == 2) # turn around HAC
	{

	TAEM_guidance_string = "HDG";
	

	#Factor difference between MEP and NEP ( 3Nm difference for distance remaining)
	
	var dist_al = 7.5;
	var alti_transition = 14000;
	var entry_point_string = getprop("/fdm/jsbsim/systems/taem-guidance/entry-point-string");
	
	if (entry_point_string == "MEP") 
		{
		dist_al = 4.5;
		alti_transition = 9000;
		#hac_radius_factor = hac_radius_factor_mep;
		}
	else 
		{
		dist_al = 7.5;
		alti_transition = 15000;
		#hac_radius_factor = hac_radius_factor_nep;
		}


	### TGPHIC ###
	var radius_error = (TAEM_guidance_GTP.RTURN * 0.3048) - pos.distance_to(TAEM_HAC_center);
	var rdot = (radius_error - radius_error_last)/0.48;
	if (radius_error_last == -1.0) {rdot = 0.0;}
	radius_error_last  = radius_error;
	setprop("/fdm/jsbsim/systems/taem-guidance/radial-error-nm", radius_error/ 1853.);
	setprop("/fdm/jsbsim/systems/taem-guidance/rdot-fps", rdot / 0.3048);
	
	#print("Radial error is: ", radius_error/ 1853);

	### TGCOMP ###
	TAEM_TGCOMP();

	### TGTRAN ###
	TAEM_TGTRAN();

	
	#Clamp here GS deviation to avoid huge value in vspeed target (taem ap)
	#Max 5000 ft low gives (-0.04 * 5000 = 200 ft/s of correction ie. zero gamma regarding hdot ref taem)
	#Pull up logic to be continued in the HAC if mach still above 1

	var glideslope_deviation = SpaceShuttle.get_glideslope_deviation(alt_qfe, TAEM_guidance_GTP.RPRED / 6076.12);
	
	if (mach > 1.0)
		{
		#Level off if Supersonic (pull up also in ap.xml once in hac / line 1912)
		setprop("/fdm/jsbsim/systems/taem-guidance/glideslope-deviation-ft", 0);
		}
	else
		{
		glideslope_deviation = SpaceShuttle.MIDVAL(glideslope_deviation, -10000, 5000);
		setprop("/fdm/jsbsim/systems/taem-guidance/glideslope-deviation-ft", TAEM_guidance_TGCOMP.HERROR);
		}

	
	
	
	TAEM_energy_management();

	var heading = getprop("/orientation/heading-deg");
	var delta_az = TAEM_threshold.heading - heading;
	

	if (delta_az > 180.0) {delta_az = delta_az - 360.0;}
	if (delta_az < -180.0) {delta_az = delta_az + 360.0;}


	#TO DO THERE: Roll logic radius error based on X final track (CDI) until A/L conditions (AUTOLAND logic then)
	#Then Pre-final transition (between 12k and 6kft // 7 Nm and 4 Nm // NEP and MEP)

	if (((TAEM_guidance_GTP.RPRED / 6076.12) < dist_al) or ((math.abs(delta_az) < 10.0) and (alt_qfe < alti_transition)))
		{
		TAEM_guidance_phase = 3;
		TAEM_guidance_TGINIT.IPHASE = 3;
		#print("TAEM guidance finished.");
		#setprop("/sim/messages/copilot", "Take CSS and turn into final!");
		SpaceShuttle.callout.make("Take CSS and turn into final!", "help");
		setprop("/fdm/jsbsim/systems/ap/taem/set-bank-target", 0.0);
		setprop("/fdm/jsbsim/systems/ap/taem/hac-turn-init", 0);
		setprop("/fdm/jsbsim/systems/ap/taem/s-turn-init", 0);
		setprop("/fdm/jsbsim/systems/ap/taem/al-init", 1);

		TAEM_guidance_string = "PREFNL";
		approach_guidance_loop();

		settimer( func {
		
		setprop("/fdm/jsbsim/systems/ap/automatic-pitch-control", 0);
		setprop("/fdm/jsbsim/systems/ap/css-pitch-control", 1);
		setprop("/fdm/jsbsim/systems/ap/automatic-roll-control", 0);
		setprop("/fdm/jsbsim/systems/ap/css-roll-control", 1);

		}, 5.0);


		stage = 3;
		}
	}
else if (stage == 3)
	{



	### TGCOMP ###
	TAEM_TGCOMP();

	### TGTRAN ###
	TAEM_TGTRAN();

	setprop("/fdm/jsbsim/systems/taem-guidance/glideslope-deviation-ft", TAEM_guidance_TGCOMP.HERROR);

	if (TAEM_guidance_GTP.RPRED < 3000) {return;}

	}

#print("stage bug is: ", stage);
#TDAP guidance cycle 2.08 Hz (T = 0.48 s)
settimer( func {TAEM_guidance_loop(stage, radius_error_last); }, 0.48);

}

# TAEM energy management - Speedbrake and S-Turns #################################################

var TAEM_energy_management = func {

var major_mode = getprop("/fdm/jsbsim/systems/dps/major-mode");

# no TAEM energy management during powered RTLS
if (major_mode == 601) {return;}
	

SpaceShuttle.body_flap_management();


#First Energy Logic (Thorsten)
	var distance_to_runway = getprop("/fdm/jsbsim/systems/taem-guidance/distance-to-runway-nm");
	var alt = getprop("/position/altitude-ft") * 0.3048;
	var gsld = getprop("/fdm/jsbsim/systems/taem-guidance/glideslope-deviation-ft") * 0.3048;
	var nominal_alt = gsld + alt;

	var speed_kts = getprop("/fdm/jsbsim/velocities/ve-kts");

	var speed = speed_kts * 1853.0 / 3600.0;
	var nominal_speed = 270.0 * 1853.0 / 3600.0;
	

	var g = 9.81;

	var E_act = g * alt + speed * speed;
	var E_nom = g * nominal_alt + nominal_speed * nominal_speed;

	var E_ratio = E_act/E_nom;

	var dH_equiv_ft = (E_act - E_nom)/g / 0.3048;

	setprop("/fdm/jsbsim/systems/taem-guidance/energy-ratio", E_ratio);
	setprop("/fdm/jsbsim/systems/taem-guidance/dH-equiv-ft", dH_equiv_ft);


#Post Flight Datas Interpolated Energy Logic (Jan 2021)
	var EW_actual = getprop("/fdm/jsbsim/systems/entry_guidance/taem-EW-actual-ratio-ft");
	var EW_sturn = getprop("/fdm/jsbsim/systems/entry_guidance/taem-EW-sturn-ratio-ft");
	var EW_nominal = getprop("/fdm/jsbsim/systems/entry_guidance/taem-EW-nominal-ratio-ft");
	var EW_mep = getprop("/fdm/jsbsim/systems/entry_guidance/taem-EW-mep-ratio-ft");
	var eas_nominal = getprop("/fdm/jsbsim/systems/entry_guidance/taem-nominal-eas-kts"); 
	var entry_point_string = getprop("/fdm/jsbsim/systems/taem-guidance/entry-point-string");
	#var hac_turn_degrees = getprop("/fdm/jsbsim/systems/taem-guidance/turn-hac-degrees");
	var sd = eas_nominal - speed_kts;




#Flag to end Sturn when energy is back 10000 ft above Nominal EW 
var sturn_init = getprop("/fdm/jsbsim/systems/ap/taem/s-turn-init"); 
var sturn_treshold = getprop("/fdm/jsbsim/systems/ap/taem/s-turn-treshold"); 
var EW_sturn_end = EW_nominal + 10000;


# S-turns if lots of energy is to be depleted  // Actual EW above Sturn EW

if ((EW_actual > EW_sturn) and (TAEM_guidance_phase == 1) and (sturn_treshold == 0))
	{
	
	##Sturns illegal (1)
	#OPS 3 TAEM // S turns between 25 and 55 Nm and for HAC less than 200 °
	if (major_mode == 305) #and ((distance_to_runway < 25) or (distance_to_runway > 55) or (hac_turn_degrees > 200))) 
		{
		setprop("/fdm/jsbsim/systems/ap/taem/s-turn-treshold",1);
		return;
		}

	#OPS 6 GRTLS // S turns above 35 Nm and no HAC turn degrees restriction
	else if ((major_mode == 603) and (distance_to_runway < 35))
		{
		setprop("/fdm/jsbsim/systems/ap/taem/s-turn-treshold",1);
		return;
		}

	##If Sturns legal (2)
	else {setprop("/fdm/jsbsim/systems/ap/taem/s-turn-treshold",2);}
	}


else if (sturn_treshold == 2)
	{
	TAEM_guidance_string = "S-TURN";
		

		#Bank to 30° for OPS 3 and 45° for OPS 6 S-turn
		var bank_value = 30.0;
		if (major_mode == 603) {bank_value = 45.0;}

		if (sturn_init == 1) # we are in a turn
			{
			var delta_az = getprop("/fdm/jsbsim/systems/taem-guidance/delta-azimuth-deg");
			if (delta_az > 30.0)		
				{
				if (getprop("/fdm/jsbsim/systems/ap/taem/set-bank-target") == -bank_value)
					{
					#setprop("/sim/messages/copilot", "S-turn reversal!");
					SpaceShuttle.callout.make("S-turn reversal!", "info");
					}

				setprop("/fdm/jsbsim/systems/ap/taem/set-bank-target", bank_value);

				}
			else if (delta_az < -30.0)
				{
				if (getprop("/fdm/jsbsim/systems/ap/taem/set-bank-target") == bank_value)
					{
					#setprop("/sim/messages/copilot", "S-turn reversal!");
					SpaceShuttle.callout.make("S-turn reversal!", "info");
					}
				setprop("/fdm/jsbsim/systems/ap/taem/set-bank-target", -bank_value);
				}

			}
		else
			{
			setprop("/fdm/jsbsim/systems/ap/taem/set-bank-target", bank_value);
			#setprop("/sim/messages/copilot", "Initiating S-turn to deplete energy!");
			SpaceShuttle.callout.make("Initiating S-turn to deplete energy!", "info");
			setprop("/fdm/jsbsim/systems/ap/taem/s-turn-init",1);
		
			}
	
	#Ending condition// TAEM recomputation as we moved from initial WP1 trajectory
	if (EW_actual < EW_sturn_end) 
		{
		SpaceShuttle.compute_TAEM_guidance_targets();
		setprop("/fdm/jsbsim/systems/ap/taem/s-turn-treshold",0);
		}
	}
	


#No Sturn and Sturn trigger reset de EW sturn
else if (((EW_actual < EW_sturn) and (sturn_treshold == 0)) or (sturn_treshold == 1))
	{
	if (TAEM_guidance_phase == 1)
		{TAEM_guidance_string = "ACQ";}
	else
		{TAEM_guidance_string = "HDG";}
	setprop("/fdm/jsbsim/systems/ap/taem/s-turn-init",0);
	}


# auto-SB (TAEM Guidance book/ Entry Handbook)

var mach = getprop("/fdm/jsbsim/velocities/mach");
var sb_max = 0.65;
#var hac_init = getprop("/fdm/jsbsim/systems/ap/taem/hac-turn-init");
var altitude_ft_qfe = getprop("/position/altitude-ft") - SpaceShuttle.TAEM_threshold.elevation;
#Wp1 distance
var pos = state_vector_position();
var dist_wp1 = pos.distance_to(TAEM_WP_1) / 1853.0;

#Between mach 2.5 and 0.95, SB at 65 % unless S turn engaged or approaching hac in supersonic // then below mach 0.95 SB modulation to maintain E/W nominal up to 15kfeet

if ((mach > 1.0) and (sturn_init == 0)) #Fix SB setting
	{
	if (dist_wp1 < 10) {sb_max = 1.0;}
	else {sb_max = 0.65;}
	}
else if (sturn_init == 1) #S-turn // Full SB
	{
	sb_max = 1.0;
	}
else #E/W SB management
	{
	#Normal/neutral position for SB is 0.25 ( 25 ° ie. no Cd/Cl/Cm with new FDM as per wind data tunnel)
	var sb_max = 0.25;
	
	if (altitude_ft_qfe > 22000) #E/W management
		{
		sb_max = getprop("/fdm/jsbsim/systems/entry_guidance/taem-SB-EW-management");
		}
	else  #Transition to A/L SB management (qbar/EAS management)
		{
		sb_max = getprop("/fdm/jsbsim/systems/entry_guidance/taem-SB-eas-management");
		}
		

	# we're short on energy and never use SB below Mach 0.95 (neutral at 0.25 % ie. 25 ° with latest Cl/d SB // Cl and Cd = 0 at 25°)
	#MEP boundary before HAC
	#Handled by the low energy SB management directly
		#if ((EW_actual < EW_mep) and (SpaceShuttle.TAEM_guidance_phase < 2))
			#{
			#sb_max = 0.25;
			#}

	#Initial Thorsten SB logic
		#sd = eas_nominal - speed_kts;
		#if (sd < -5.0)
		#	{sb_max = 1.0;}
		#else if ((sd > -5.0) and (sd < 5.0))
		#	{sb_max = 0.8;}
		#else if ((sd > 5.0) and (sd < 15.0))
		#	{sb_max = 0.6;}
		#else if ((sd > 15.0) and (sd < 25.0))
		#	{sb_max = 0.4;}
		#else if (sd > 25)
		#	{sb_max = 0.25;}
	}




if (getprop("/fdm/jsbsim/systems/ap/automatic-sb-control") == 1)	
	{
	var sb_state = getprop("/controls/shuttle/speedbrake");

	if (sb_state > sb_max) {SpaceShuttle.decrease_speedbrake();}
	else if (sb_state < sb_max) {SpaceShuttle.increase_speedbrake();}
	}

}

# the  approach guidance loop ###########################################################


var approach_guidance_loop = func {

SpaceShuttle.body_flap_management();

#var pos = geo.aircraft_position();

var pos = state_vector_position();

var dist = pos.distance_to(TAEM_threshold);
var course = pos.course_to(TAEM_threshold);
var heading = getprop("/orientation/heading-deg");

var alt_agl = getprop("/position/altitude-agl-ft");
var vspeed = getprop("/fdm/jsbsim/velocities/v-down-fps");
var airspeed = getprop("/fdm/jsbsim/velocities/ve-kts");

update_HUD_symbology(pos);

#Computations for coordinates in Rwy frame
TAEM_rwy_frame();

#XGAC done in initial TAEM computations

#GTP / Handles distance computations
TAEM_GTP();

#TGCOMP for reference functions
TAEM_TGCOMP();

#TGTRAN for phase boundaries and limits
TAEM_TGTRAN();

#TGNZC to be done (Pitch still based on vertical speed error)

#TGSBC to be finished (No automatic Speedbrake for now)

#TGPHIC for Roll commanded
TAEM_TGPHIC();

if (TAEM_guidance_phase == 3) 
	{
	if ((math.abs (course-heading) < 10.0) and (math.abs(HUD_data_set.vangle_aim - 17.0) < 10.0))
		{
		# we acquired glideslope
		HUD_data_set.MLS_acquired = 1;
		TAEM_guidance_phase = 4;
		TAEM_guidance_string = "OGS";
		}
	}

if (TAEM_guidance_phase == 4)
	{
	
	#Final OGS path 20° (IGS 1) or 18° (IGS 2)
	var OGS_gamma = 20.0;
	if (TAEM_guidance_XHAC.IGS == 2) {OGS_gamma = 18.0;}
	
	var GLSD = (HUD_data_set.vangle_aim - OGS_gamma);
	HUD_data_set.vangle_guidance = OGS_gamma + 1.2 * GLSD;

	#print("HUD commanded gamma is: ", HUD_data_set.vangle_guidance);

	

	var sb_max = 0.25;
	var final_sb_flag = 0;
	


	#Approach/Landing Handbook logic and values

	# auto-SB control  (Max 300 kts with SB full extended) above 3000 ft QFE
	if (alt_agl > 3000.0) {sb_max = getprop("/fdm/jsbsim/systems/entry_guidance/taem-SB-eas-management");}

	# Fix SB setting below 3000 ft QFE
	else if ((alt_agl < 3000.0) and (final_sb_flag == 0))
		{
		var sb_eas_factor = getprop("/fdm/jsbsim/systems/entry_guidance/taem-SB-eas-flare-management"); #Wind and Eas factor
		var sb_mode = getprop("/fdm/jsbsim/systems/approach-guidance/speedbrake-mode-string"); #Lower TD speed targeted for Short // Shorter touchdown zone and lower speed targeted for ELS
		var weight_final = getprop("/fdm/jsbsim/inertia/weight-lbs"); #Light/ Heavy/ Super heavy weight boundaries ( 222 klbs // 245 klbs)
		var aim_point_string = getprop("/fdm/jsbsim/systems/approach-guidance/aim-point-string"); #Close In Vs Normal aimpoint ( More energy to be dissipated for close in)

		#SB factor variables for standard configuration (Nominal SB and aim point / Lightweight)
		var sb_option = 0;
		var sb_aim_point = 0;
		var sb_weight = 0;

		#Nominal: Touchdown zone at 2500 feet / Lightweight speed 195kts / Heavy 205kts
		#Short: Touchdown zone at 1500 feet / Lightweight speed 195kts / Heavy 205 kts ( plus 10% Speedbrakes to land shorter)
		#ELS: Touchdown zone at 1000 feet / Heavyweight only 195kts (plus 25% speedbrakes to land slower and shorter) // Mainly for Abort 

		#Sb mode factor
		if (sb_mode == "SHORT") {sb_option = 0.10;}
		else if (sb_mode == "ELS") {sb_option = 0.25;}

		#Aim point factor
		if (aim_point_string == "CLSE") {sb_aim_point = 0.1;}
		

		#Weight factor
		if (weight_final > 222000) {sb_weight = 0.1;}

		
		#Final setting (Max 80% of SB)

		sb_max = math.min(sb_eas_factor + sb_option + sb_aim_point + sb_weight, 0.8);
		
		#One time loop and setting
		final_sb_flag = 1;
		}

	if (getprop("/fdm/jsbsim/systems/ap/automatic-sb-control") == 1)	
		{
		var sb_state = getprop("/controls/shuttle/speedbrake");

		if (sb_state > sb_max) {SpaceShuttle.decrease_speedbrake();}
		else if (sb_state < sb_max) {SpaceShuttle.increase_speedbrake();}
		
		}
	

	if (alt_agl < 2000.0)
		{
		# we initiate pre-flare
		TAEM_guidance_phase = 5;
		TAEM_guidance_string = "FLARE";
		}
	}

if (TAEM_guidance_phase == 5)
	{

	# use smart pull-up guidance

	var pull_up_speed = smart_flare (alt_agl, airspeed, vspeed);
	var dt = getprop("/sim/time/delta-sec");


	HUD_data_set.vangle_guidance = HUD_data_set.vangle_guidance - pull_up_speed * dt;
	if (HUD_data_set.vangle_guidance < 1.5)
		{HUD_data_set.vangle_guidance = 1.5;}

	if (vspeed < 15.0)
		{
		# transit to inner glideslope
		TAEM_guidance_phase = 6;
		TAEM_guidance_string = "IGS";
		}

	if (alt_agl < 50.0)
		{
		TAEM_guidance_phase = 7;
		TAEM_guidance_string = "FNLFL";
		HUD_data_set.vangle_guidance = 0.1;
		}

	}


if (TAEM_guidance_phase == 6) 
	{
	var dt = getprop("/sim/time/delta-sec");
	HUD_data_set.vangle_guidance = HUD_data_set.vangle_guidance - 1.5 * dt;
	if (HUD_data_set.vangle_guidance < 1.5)
		{HUD_data_set.vangle_guidance = 1.5;}
	

	if (alt_agl < 50.0)
		{
		TAEM_guidance_phase = 7;
		TAEM_guidance_string = "FNLFL";
		HUD_data_set.vangle_guidance = 0.1;
		}


	}

if (airspeed < 170.0) 
	{
	#print ("Approach guidance signing off!");
	return;
	}

#settimer( approach_guidance_loop, 0.0);

}
