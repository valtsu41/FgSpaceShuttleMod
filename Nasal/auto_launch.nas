# auto launch guidance for the Space Shuttle
# Thorsten Renk 2016
# GinGin 2020


var auto_launch_stage = 0;
var auto_launch_timer = 0.0;
var aux_flag = 0;

#TAL PEG target after a droop flag
var droop_flag = 0;

#Droop inh flag for 1EO droop stuck procedure
var droop_inh = 0;

var auto_launch_traj_loft = 0.0;
var auto_launch_mps_climbout_bias = 0.0;
var auto_launch_srb_climbout_bias = 0.0;
var gamma_meco = 0.0;

var auto_launch_TAL_dist_difference_v = 0.0;

var xtrack_refloc = geo.Coord.new();
xtrack_refloc.last_xtrack = 0.0;
xtrack_refloc.correction = 0.0;


xtrack_refloc.approach_speed = 0.0;
xtrack_refloc.last_app_speed = 0.0;
xtrack_refloc.buffer = 0.0;

var auto_launch_loop = func {

var shuttle_pos = geo.aircraft_position();


#var actual_course = SpaceShuttle.peg4_refloc.course_to(shuttle_pos);
#var dist = SpaceShuttle.peg4_refloc.distance_to(shuttle_pos);

var actual_course = SpaceShuttle.xtrack_refloc.course_to(shuttle_pos);
var dist = SpaceShuttle.xtrack_refloc.distance_to(shuttle_pos);

var launch_azimuth = getprop("/fdm/jsbsim/systems/ap/launch/launch-azimuth");

var xtrack = SpaceShuttle.sgeo_crosstrack(actual_course, launch_azimuth, dist)  * 0.0005399568;

# compute speed by a running average

xtrack_refloc.approach_speed = (xtrack_refloc.last_xtrack - xtrack)/ 0.1; # using loop time constant
xtrack_refloc.buffer = (xtrack_refloc.approach_speed + xtrack_refloc.last_app_speed)/2.0;
xtrack_refloc.last_app_speed = xtrack_refloc.approach_speed;
xtrack_refloc.approach_speed = xtrack_refloc.buffer;



xtrack_refloc.last_xtrack = xtrack;


setprop("/fdm/jsbsim/systems/ap/launch/cross-track", xtrack);
setprop("/fdm/jsbsim/systems/ap/launch/cross-track-approach-speed", xtrack_refloc.approach_speed);

if (auto_launch_stage == 0)
	{
	# check for clear gantry, then initiate rotation to launch course
	
	if (getprop("/position/altitude-agl-ft") > 200.0)
		{
		auto_launch_stage = 1;
		setprop("/fdm/jsbsim/systems/ap/launch/stage", 1);
		aux_flag = 0;
		}
	}
else if (auto_launch_stage == 1)
	{
	
	# enable throttling already during rotation to azimuth
	#print ("Auto launch timer: ", auto_launch_timer);

	var throttle_commanded_stage1 = getprop("/fdm/jsbsim/systems/ap/launch/throttle-command");

	if (getprop("/fdm/jsbsim/systems/ap/automatic-sb-control") == 1)
		{
		#Throttle commanded done in a jsbsim table for a 2 stage bucket
		if (getprop("/fdm/jsbsim/systems/mps/number-engines-operational") == 3.0) 
			{
			SpaceShuttle.ssme_array[0].throttle_to(throttle_commanded_stage1);
			SpaceShuttle.ssme_array[1].throttle_to(throttle_commanded_stage1);
			SpaceShuttle.ssme_array[2].throttle_to(throttle_commanded_stage1);
			}

		#Throttle up if we lose an engine
		else
			{
			SpaceShuttle.ssme_array[0].throttle_to(1.0);
			SpaceShuttle.ssme_array[1].throttle_to(1.0);
			SpaceShuttle.ssme_array[2].throttle_to(1.0);
			}
		}
			
	# check for launch course reached, then initiate pitch down assuming we're high enough
	# either a course error or a minimum altitude to have the same start of pitchdown if 180 ° roll or not at lift off 

	if ((math.abs(getprop("/fdm/jsbsim/systems/ap/launch/stage1-course-error")) < 0.3) and (getprop("/position/altitude-agl-ft") > 1000))
		{
		auto_launch_stage = 2;
		setprop("/fdm/jsbsim/systems/ap/launch/stage", 2);
		setprop("/fdm/jsbsim/systems/ap/launch/pitch-target", 70.0 + auto_launch_srb_climbout_bias);
		setprop("/fdm/jsbsim/systems/ap/launch/pitch-max-rate-norm", 0.25);
		aux_flag = 0;
		}

	

	}
else if (auto_launch_stage == 2)
	{
	
	#Up to 2 stage Bucket logic based on Vtrue boundaries
	var throttle_commanded_stage1 = getprop("/fdm/jsbsim/systems/ap/launch/throttle-command");
	
	if (getprop("/fdm/jsbsim/systems/ap/automatic-sb-control") == 1)
		{
		#Throttle commanded done in a jsbsim table for a 2 stage bucket
		if (getprop("/fdm/jsbsim/systems/mps/number-engines-operational") == 3.0) 
			{
			SpaceShuttle.ssme_array[0].throttle_to(throttle_commanded_stage1);
			SpaceShuttle.ssme_array[1].throttle_to(throttle_commanded_stage1);
			SpaceShuttle.ssme_array[2].throttle_to(throttle_commanded_stage1);
			}

		#Throttle up if we lose an engine
		else
			{
			SpaceShuttle.ssme_array[0].throttle_to(1.0);
			SpaceShuttle.ssme_array[1].throttle_to(1.0);
			SpaceShuttle.ssme_array[2].throttle_to(1.0);
			}
		}

	var vt = getprop("/fdm/jsbsim/velocities/vtrue-fps");
	var engop = getprop("/fdm/jsbsim/systems/mps/number-engines-operational");
	var eng_out_pitch = getprop("/fdm/jsbsim/systems/ap/launch/stage2-eo-pitch-schedule");

	###JSBsim Open Loop guidance functions for better Linear Interpolation
	
	#Pitch schedule based on Engines remaining ( RTLS preparation) // additionnal 10° of pitch bias for 2EO for higher flight pat angle at SRB sep
	if (engop == 3.0) {setprop("/fdm/jsbsim/systems/ap/launch/pitch-target", getprop("/fdm/jsbsim/systems/ap/launch/stage2-nominal-pitch-schedule") + auto_launch_srb_climbout_bias);}
	else if (engop == 2.0) {setprop("/fdm/jsbsim/systems/ap/launch/pitch-target", eng_out_pitch);}
	else {setprop("/fdm/jsbsim/systems/ap/launch/pitch-target", eng_out_pitch + 10);}



	#Pitch rate based on margin with Max Q bar and CBW with Nz Load Relief protection
	setprop("/fdm/jsbsim/systems/ap/launch/pitch-max-rate-norm", getprop("/fdm/jsbsim/systems/ap/launch/stage2-pitch-rate-schedule"));


	#Transition towards SRB sep conditionnal
	if ((getprop("/fdm/jsbsim/propulsion/srb-pc50-discrete") == 1) and (aux_flag == 0)) {aux_flag = 1;}
		

	else if ((getprop("/controls/shuttle/SRB-static-model") == 0) and (getprop("/fdm/jsbsim/systems/failures/shuttle-destroyed") == 0) and (aux_flag == 1))
		{
		auto_launch_stage = 3;

		setprop("/fdm/jsbsim/systems/ap/launch/pitch-max-rate-norm", 0.1);
	
		meco_time.init(26000.0);
		droop_guidance.init();

		setprop("/fdm/jsbsim/systems/ap/launch/stage", 3);	


		#Either less than 3 engines ===> preparing RTLS at 2mn30 or 3 engines else nominal ascent

		if (engop < 3.0)
			{
			setprop("/fdm/jsbsim/systems/ap/launch/pitch-target", 40.0);
			aux_flag = 0;
			}

		else
			{	
				

			var payload_factor = getprop("/fdm/jsbsim/inertia/pointmass-weight-lbs[5]") /53700.00;
			payload_factor =  payload_factor +  (getprop("/fdm/jsbsim/inertia/pointmass-weight-lbs[1]") - 29250.0) / 26850.00;
		
			# OMS kit
			payload_factor = payload_factor + getprop("/fdm/jsbsim/propulsion/tank[27]/contents-lbs")/53700.0;
			payload_factor = payload_factor + getprop("/fdm/jsbsim/propulsion/tank[26]/contents-lbs")/53700.0;

			var throttle_factor = 10.0 * (getprop("/fdm/jsbsim/systems/throttle/throttle-factor") - 1.0);

			var lat = getprop("/position/latitude-deg") * math.pi/180.0;
			var heading = getprop("/orientation/heading-deg") * math.pi/180.0;
			var geo_factor = math.sin(heading) * math.cos(lat);
		


			var v_up_tgt = 920.0 + auto_launch_srb_climbout_bias * 8.0;

			var delta_vspeed = (v_up_tgt - 42.0 * payload_factor) + 0.3048 * getprop("/fdm/jsbsim/velocities/v-down-fps");

			var delta_alt = delta_vspeed * 895.0;
			var loft_factor = delta_alt / 7000.0;

			if (loft_factor > 5.0) {loft_factor = 5.0;}
			if (loft_factor < -2.0) {loft_factor = -2.0;}

			var start_pitch_target = 13 + 5.0 * payload_factor + 2.7 - 4.5 * geo_factor - 8.0 * throttle_factor + auto_launch_mps_climbout_bias + loft_factor;

			#Pitch before PEG initial convergence
			setprop("/fdm/jsbsim/systems/ap/launch/pitch-target", start_pitch_target);
			setprop("/fdm/jsbsim/systems/ap/launch/initial-stage3-pitch", start_pitch_target);
			SpaceShuttle.PEG.PEG_theta = 20;
			

			#PEG loop init
			if (SpaceShuttle.PEG.peg_loop_init == 0) 
				{
				settimer(func {SpaceShuttle.UPFG_main_routine()}, 5);
				}

			# fire an OMS assist burn if requested
			if (getprop("/fdm/jsbsim/systems/ap/launch/oms-assist-burn") == 1)
				{

				var oms_assist_time = getprop("/fdm/jsbsim/systems/ap/launch/oms-assist-duration-s");

				settimer ( func {

					setprop("/controls/engines/engine[5]/throttle", 1.0);
					setprop("/controls/engines/engine[6]/throttle", 1.0);

					}, 10.0 );
			

				settimer ( func {

 					if (getprop("/fdm/jsbsim/systems/oms/oms-dump-cmd") == 1) {return;}

					setprop("/controls/engines/engine[5]/throttle", 0.0);
					setprop("/controls/engines/engine[6]/throttle", 0.0);

					}, 10.0 + oms_assist_time );
				}


			aux_flag = 0;
			}
		}
	
	}
else if (auto_launch_stage == 3)
	{
	
	#PEG loop init
	#if (SpaceShuttle.PEG.peg_loop_init == 0) {SpaceShuttle.UPFG_main_routine();}

	var guidance = getprop("/fdm/jsbsim/systems/entry_guidance/guidance-mode");

	if (guidance == 2) # TAL abort requires different guidance after SRB sep
		{
		#PEG loop not stopped 
		print ("TAL abort declared, switching to TAL guidance...");
		auto_TAL_init();
		return;
		}
	else if (guidance == 3) # RTLS abort, prtls_loop should be running at this point
		{
		#PEG loop stopped
		SpaceShuttle.PEG.peg_meco = 1;
		print ("RTLS abort declared, switching to RTLS guidance...");
		return;
		}
	
	var alt = getprop("/position/altitude-ft");
	var mach = getprop("/fdm/jsbsim/velocities/mach");
	var vi = getprop("/fdm/jsbsim/velocities/eci-velocity-mag-fps");
	var pitch_init = getprop("/fdm/jsbsim/systems/ap/launch/initial-stage3-pitch");
	var v_down = getprop("/fdm/jsbsim/velocities/v-down-fps");

	#Tb init computed there for PEG initial guess Tgo
	meco_time.run();

	#Droop altitude update
	droop_guidance.run (alt);

	#Pitch target coming from PEG guidance
	setprop("/fdm/jsbsim/systems/ap/launch/pitch-target", SpaceShuttle.PEG.PEG_theta);
	
	#AP stage 4 transition
	#Transition slightly before Single Engine Ops 3 capability to init auto droop logic function 
	if (vi > (0.95 * SpaceShuttle.ascent_abort_boundary.single_ops3)) 
		{
		auto_launch_stage = 4;
		setprop("/fdm/jsbsim/systems/ap/launch/stage", 4);
		setprop("/fdm/jsbsim/systems/ap/launch/pitch-max-rate-norm", 0.15);
		setprop("/fdm/jsbsim/systems/ap/launch/hdot-target", 50.0);
		droop_guidance.init();
		aux_flag = 0;
		}	
	
	}
else if (auto_launch_stage == 4)
	{
	
	#PEG loop init
	#if (SpaceShuttle.PEG.peg_loop_init == 0) {SpaceShuttle.UPFG_main_routine();}

	var alt = getprop("/position/altitude-ft");
	var mach = getprop("/fdm/jsbsim/velocities/mach");
	var vi = getprop("/fdm/jsbsim/velocities/eci-velocity-mag-fps");
	var vt = getprop("/fdm/jsbsim/velocities/vtrue-fps");
	var x_norm = getprop("/fdm/jsbsim/accelerations/Nx");
	var rthu_enable = getprop("/fdm/jsbsim/systems/ap/launch/rthu-enable");
	var guidance = getprop("/fdm/jsbsim/systems/entry_guidance/guidance-mode");
	
	#No droop if single engine TAL is available
	if (vi < SpaceShuttle.ascent_abort_boundary.single_engine_tal) {droop_guidance.run (alt);}
	meco_time.run();

	#Pitch target coming from PEG guidance
	setprop("/fdm/jsbsim/systems/ap/launch/pitch-target", SpaceShuttle.PEG.PEG_theta);


	#TAL after a droop (SE ops 3 109% boundary) or 3 engines TAL (during stage 4/ past 12800 kfts ish Vi)
	if (guidance == 2) # TAL abort requires different guidance after SRB sep
		{
		print ("TAL abort declared, switching to TAL guidance...");
		auto_TAL_init();
		return;
		}

	else if (guidance == 3) # Contigency abort
		{
		#PEG loop stopped
		SpaceShuttle.PEG.peg_meco = 1;
		print ("Contigency declared, switching to CA guidance...");
		return;
		}
	

	# dynamically throttle back when we reach acceleration limit
	#if ((SpaceShuttle.PEG.aT > SpaceShuttle.PEG.aL) and (vi > 18000))
	if ((x_norm > 3.0) and (vi > 18000))
		{
		
		#PEG flag for phase changes
		if (SpaceShuttle.PEG.peg_constant_accel == 0) {SpaceShuttle.PEG.peg_constant_accel = 1;}
		
		SpaceShuttle.meco_time.set_mode(1);

		# an engine can be in electric lockup, so we may need to look at more engines
		# to determine current throttle value

		var current_throttle = 1; 
		var engop = getprop("/fdm/jsbsim/systems/mps/number-engines-operational");

		if (SpaceShuttle.failure_cmd.ssme1 == 1)
			{current_throttle = getprop("/controls/engines/engine[0]/ap-throttle-cmd");}
		else if (SpaceShuttle.failure_cmd.ssme2 == 1)
			{current_throttle = getprop("/controls/engines/engine[1]/ap-throttle-cmd");}
		else
			{current_throttle = getprop("/controls/engines/engine[2]/ap-throttle-cmd");}

		var new_throttle = current_throttle - 0.01;
		# In case 3G's limiting reaches 68% (91% for less than 3 Engines op) before fine count
		var throttle_mini = 0;
		if (engop < 3) 
			{
			throttle_mini = (91 - 67) / (getprop("/mission/launch/throttle-nominal-percent") - 67);
			}
		if (new_throttle < throttle_mini) {new_throttle = throttle_mini;}

		#Minimum thrust 10sish before MECO (fine count)
		if (SpaceShuttle.PEG.peg_fine_count == 1) {new_throttle = throttle_mini;}    

		if (getprop("/fdm/jsbsim/systems/ap/automatic-sb-control") == 1)
			{
			SpaceShuttle.ssme_array[0].throttle_to(new_throttle);
			SpaceShuttle.ssme_array[1].throttle_to(new_throttle);
			SpaceShuttle.ssme_array[2].throttle_to(new_throttle);
			}

		aux_flag = 2;  #In case of no RTHU enabled and no aux flag 2 before vi = 21800, it forces the transition towards Hdot meco target

		}

	#Inc Error correction adjustment if track is less than 90° (inc-acquire-sign = -1) or more than 90 ° (inc-acquire-sign = 1) without RTHU //reversed for RTHU
	#(problem arised for eastwards launch, oscillating between North EAST/WEST azimuth and course wrong correction close to MECO, increasing the Dinc)
	#Stage 4 only
	


	var inc_acquire_sign = getprop("/fdm/jsbsim/systems/ap/launch/inc-acquire-sign");
	var course_deg = getprop("/fdm/jsbsim/velocities/course-deg");

	
	if ((course_deg < 90.0) or (course_deg > 270.0))
		{
		if ((rthu_enable == 1) and (vt > 13100)) {setprop("/fdm/jsbsim/systems/ap/launch/inc-acquire-sign", 1);}
		else {setprop("/fdm/jsbsim/systems/ap/launch/inc-acquire-sign", -1);}
		}

	else if ((course_deg > 90.0) and (course_deg < 270.0))
		{
		if ((rthu_enable == 1) and (vt > 13100)) {setprop("/fdm/jsbsim/systems/ap/launch/inc-acquire-sign", -1);}
		else {setprop("/fdm/jsbsim/systems/ap/launch/inc-acquire-sign", 1);}
		}




	# automatic roll to heads up attitude
	#Based now on Vrel (Vt = 12200) Ascent Handbook (12700 in SCOM)

	if ((aux_flag == 0) and (vt > 12200))
		{
		if ((rthu_enable == 1) and (getprop("/fdm/jsbsim/systems/fcs/control-mode") != 13))
			{
			#setprop("/fdm/jsbsim/systems/ap/launch/inc-acquire-sign", -inc_acquire_sign); Replaced Inc acquire sign  management above
			setprop("/fdm/jsbsim/systems/ap/launch/pitch-max-rate-norm", 0.25);
			setprop("/fdm/jsbsim/systems/ap/launch/rthu-cmd", 1);
			SpaceShuttle.PEG.rthu_flag = 1;
			}
		aux_flag = 1;
		}
	else if ((aux_flag == 1) and (getprop("/fdm/jsbsim/systems/ap/launch/stage4-rthu-in-progress") == 0))  
		{
		setprop("/fdm/jsbsim/systems/ap/launch/pitch-max-rate-norm", 0.15);
		aux_flag = 2;
		}
	
	#RTHU gain for UFPG steering  (+10° at 12200 fps / 0.0° at 20000 fps)
	if (SpaceShuttle.PEG.rthu_flag == 1) {SpaceShuttle.PEG.rthu_gain = SpaceShuttle.MIDVAL(0, 10, -0.001282 * vt + 25.64);}


	# change hdot target to Gamma meco target prior to MECO
	#var hdotmeco = math.tan(SpaceShuttle.PEG.gamma_d / 57.29578) * vi;
	var hdotmeco = math.tan(SpaceShuttle.PEG.gamma_d / 57.29578) * SpaceShuttle.PEG.vd_norm;

	if (((vi > 21800) and (vi < 22200)) and (aux_flag == 2)) 
		{
		setprop("/fdm/jsbsim/systems/ap/launch/hdot-target", -hdotmeco / 2);
		setprop("/fdm/jsbsim/systems/ap/launch/pitch-max-rate-norm", 0.1);
		aux_flag = 3;
		} 
	
	#Final gamma targeting
	else if ((vi > 22300) and (vi < 22800))                      
		{
		setprop("/fdm/jsbsim/systems/ap/launch/hdot-target", -hdotmeco);
		aux_flag = 4;
		} 


	# increase pitch maneuverability as centrifugal force builds up

	if ((aux_flag == 4) and (getprop("/fdm/jsbsim/systems/orbital/periapsis-km") > -500.0))
		{
		setprop("/fdm/jsbsim/systems/ap/launch/pitch-max-rate-norm", 0.15);
		}




	# MECO if apoapsis target is met

	#MECO when TGO is less than half guidance cycle or Ap target constraint met (Vi = Vd norm)
	if (
		(
			(
				(
					SpaceShuttle.PEG.tgo_total <= 0.5
				) or (
					vi > SpaceShuttle.PEG.vd_norm
				)
			) and (
				SpaceShuttle.PEG.advanced_peg_flag == 1
			)
		) or (
			(
				getprop("/fdm/jsbsim/systems/orbital/apoapsis-km") >= getprop("/fdm/jsbsim/systems/ap/launch/apoapsis-target")
			) and (
				SpaceShuttle.PEG.advanced_peg_flag == 0
			)
		) or (
			getprop("/consumables/fuel/tank/level-norm") < 0.005
		)
	)
		{
		
		#Meco flag for PEG
		SpaceShuttle.PEG.peg_meco = 1;

		SpaceShuttle.meco_time.set_mode(2);

		if (getprop("/fdm/jsbsim/systems/ap/automatic-sb-control") == 1)
			{	
			

			SpaceShuttle.ssme_array[0].throttle_to(0.0);
			SpaceShuttle.ssme_array[1].throttle_to(0.0);
			SpaceShuttle.ssme_array[2].throttle_to(0.0);

			ssme_array[0].cutoff();
			ssme_array[1].cutoff();
			ssme_array[2].cutoff();


			}

		#if throttles are in manual, engines may not be
		#cut-off yet even though apoapsis target has been
		#reached. We check for thrust to be null to go on with MECO
		#signing off and handover to orbital loop.
		
		var thrust_ssme1 = getprop("/engines/engine[0]/thrust_lb");
		var thrust_ssme2 = getprop("/engines/engine[1]/thrust_lb");
		var thrust_ssme3 = getprop("/engines/engine[2]/thrust_lb");

		var thrust_all_ssme = thrust_ssme1 + thrust_ssme2 + thrust_ssme3;
		# print("thrust_all= ",thrust_all_ssme);

		if (thrust_all_ssme == 0)
		   {
		
		setprop("/fdm/jsbsim/systems/ap/launch/regular-meco-condition",1);





		print ("MECO - auto-launch guidance signing off!");
		print ("Thank you for flying with us!");

		# use RCS to null residual rates, disengage launch AP

		settimer( func {
			setprop("/fdm/jsbsim/systems/fcs/control-mode",20);
			controls.centerFlightControls();

			setprop("/fdm/jsbsim/systems/ap/css-pitch-control", 1);
			setprop("/fdm/jsbsim/systems/ap/automatic-pitch-control", 0);    
			setprop("/fdm/jsbsim/systems/ap/css-roll-control", 1);
			setprop("/fdm/jsbsim/systems/ap/automatic-roll-control", 0);


			}, 2.0);
		
		# prepare ETsep and MM 104 transition

		settimer( func {
			setprop("/fdm/jsbsim/systems/dps/major-mode", 104);
			SpaceShuttle.dk_listen_major_mode_transition(104);
			SpaceShuttle.ops_transition_auto("p_dps_mnvr");
			SpaceShuttle.mission_auto_OMS1();
			}, 23.0);

		settimer( func {


			if ((SpaceShuttle.failure_cmd.etsep_mode != 1) and (getprop("/controls/shuttle/et-separation-mode") == 0))
				{
				if (cws_msg_hash.et_sep_man == 0)
					{
					create_fault_message("    ET SEP    "," MAN", "1234 ", 3, "PASS");	#G1, Class3 SM alert
					cws_msg_hash.et_sep_man = 1;
					auto_launch_stage = 5;
					setprop("/fdm/jsbsim/systems/ap/launch/stage", 5);      #Stage 5 placeholder to stop calculations from stage 4 when switching to mm 104 ( Not the case before)
					}
				}
			else if (getprop("/controls/shuttle/et-separation-mode") == 0)
				{
				external_tank_separate();
				auto_launch_stage = 5;
				setprop("/fdm/jsbsim/systems/ap/launch/stage", 5);
				}
			

			}, 20.0);

		# start main orbital loop
		orbital_loop_init();

		SpaceShuttle.mission_post_meco();	

		return;
		   }
		}
	}


auto_launch_timer = auto_launch_timer + 0.1;


settimer(auto_launch_loop, 0.1);

}


var auto_TAL_init = func {

if (auto_launch_stage > 2)
	{
	# we need to pitch up more on the ballistic climb to get into a good trajectory

	var lat = getprop("/position/latitude-deg") * math.pi/180.0;
	var heading = getprop("/orientation/heading-deg") * math.pi/180.0;

	var payload_factor = getprop("/fdm/jsbsim/inertia/pointmass-weight-lbs[5]") /53700.00;
	payload_factor =  payload_factor +  (getprop("/fdm/jsbsim/inertia/pointmass-weight-lbs[1]") - 29250.0) / 26850.00;
	# OMS kit
	payload_factor = payload_factor + getprop("/fdm/jsbsim/propulsion/tank[27]/contents-lbs")/53700.0;
	payload_factor = payload_factor + getprop("/fdm/jsbsim/propulsion/tank[26]/contents-lbs")/53700.0;

	var geo_factor = math.sin(heading) * math.cos(lat);

	# during the ballistic ascent, pitch target depends on number of good engines
	# to allow 3 engine TAL

	var pitch_tgt = 35.0 + 12.0 * payload_factor + 12.0 - 15.0 * geo_factor;

	var num_engines = getprop("/fdm/jsbsim/systems/mps/number-engines-operational");
	if (num_engines == 3) 
		{pitch_tgt = 11.0 + 5.0 * payload_factor + 4.0 - 6.0 * geo_factor;}

	setprop("/fdm/jsbsim/systems/ap/launch/pitch-target",pitch_tgt) ;

	#PEG MECO parameters recomputation
	#if (droop_flag == 1) {SpaceShuttle.UFPG_reset("reset");}
	SpaceShuttle.UFPG_reset("TAL");

	}

auto_TAL_loop();

}



var auto_TAL_loop = func {


var abort_mode = getprop("/fdm/jsbsim/systems/abort/abort-mode");

if (abort_mode > 4) # a contingency abort has been called
	{
	#PEG loop stopped
	SpaceShuttle.PEG.peg_meco = 1;
	return;
	}

if (auto_launch_stage == 3)
	{

	
	var shuttle_pos = geo.aircraft_position();
	
	var course_tgt = shuttle_pos.course_to (SpaceShuttle.landing_site);
	setprop("/fdm/jsbsim/systems/ap/launch/course-target", course_tgt);
	setprop("/fdm/jsbsim/systems/ap/launch/rthu-enable", 1);


	# compute MECO target
	# ballistic impact range approx. range to site

	var a = getprop("/fdm/jsbsim/systems/orbital/semimajor-axis-length-ft");
	var epsilon = getprop("/fdm/jsbsim/systems/orbital/epsilon");
	var R = getprop("/position/sea-level-radius-ft");
	
	var A = -a*a * epsilon * epsilon;
	var B = -2.0 * a * a * a * epsilon;
	var C = a*a * (R*R - a*a );

	var x = 1.0/(2.0 * A) * (-B - math.sqrt(B*B - 4.0 * A * C)) + a * epsilon;
	
	var arg = x/R;

	var dist_ballistic = math.acos(arg) * R * 0.3048;
	var dist = shuttle_pos.distance_to (SpaceShuttle.landing_site);

	setprop("/fdm/jsbsim/systems/ap/launch/distance-ballistic-km", dist_ballistic/1000.0);
	setprop("/fdm/jsbsim/systems/ap/launch/distance-site-km", dist/1000.0);

	# during the ballistic ascent, pitch target depends on number of good engines
	# to allow 3 engine TAL

	var vi = getprop("/fdm/jsbsim/velocities/eci-velocity-mag-fps");
	var v_down = getprop("/fdm/jsbsim/velocities/v-down-fps");
	var alt = getprop("/position/altitude-ft");
	var num_engines = getprop("/fdm/jsbsim/systems/mps/number-engines-operational");
	var pitch_tgt = 30.0;
	
	meco_time.run();
	droop_guidance.run (alt);

	#Pitch target coming from PEG guidance
	setprop("/fdm/jsbsim/systems/ap/launch/pitch-target", SpaceShuttle.PEG.PEG_theta);
	
	#MECO CO TAL
	auto_launch_TAL_dist_difference_v = ((dist - 1400000.0) - dist_ballistic)/1500.0;

	if (vi > 12000)
		{
		auto_launch_stage = 4;
		setprop("/fdm/jsbsim/systems/ap/launch/stage", 4);
		setprop("/fdm/jsbsim/systems/ap/launch/pitch-max-rate-norm", 0.1);
		setprop("/fdm/jsbsim/systems/ap/launch/hdot-target", -50.0);
		droop_guidance.init();
		aux_flag = 0;
		}	
	
	}
else if (auto_launch_stage == 4)
	{

	var shuttle_pos = geo.aircraft_position();
	
	var course_tgt = shuttle_pos.course_to (SpaceShuttle.landing_site);
	setprop("/fdm/jsbsim/systems/ap/launch/course-target", course_tgt);

	var alt = getprop("/position/altitude-ft");
	var mach = getprop("/fdm/jsbsim/velocities/mach");
	var vi = getprop("/fdm/jsbsim/velocities/eci-velocity-mag-fps");
	var vt = getprop("/fdm/jsbsim/velocities/vtrue-fps");
	var redes_vi = getprop("/fdm/jsbsim/systems/abort/boundary-one-engine-tal-redes");

	#No droop if SE TAL is available 
	if (vi < redes_vi) {droop_guidance.run (alt);}
	meco_time.run();

	#Pitch target coming from PEG guidance
	setprop("/fdm/jsbsim/systems/ap/launch/pitch-target", SpaceShuttle.PEG.PEG_theta);

	# dynamically throttle back when we reach acceleration limit
	
	if (getprop("/fdm/jsbsim/accelerations/Nx") > 3.0)
		{
		#PEG flag for phase changes
		if (SpaceShuttle.PEG.peg_constant_accel == 0) {SpaceShuttle.PEG.peg_constant_accel = 1;}

		# an engine can be in electric lockup, so we may need to look at more engines
		# to determine current throttle value
		var engop = getprop("/fdm/jsbsim/systems/mps/number-engines-operational");
		var current_throttle = 1; 

		if (SpaceShuttle.failure_cmd.ssme1 == 1)
			{current_throttle = getprop("/controls/engines/engine[0]/ap-throttle-cmd");}
		else if (SpaceShuttle.failure_cmd.ssme2 == 1)
			{current_throttle = getprop("/controls/engines/engine[1]/ap-throttle-cmd");}
		else
			{current_throttle = getprop("/controls/engines/engine[2]/ap-throttle-cmd");}

		var new_throttle = current_throttle - 0.01;

		# In case 3G's limiting reaches 68% (91% for less than 3 Engines op) before fine count
		var throttle_mini = 0;
		if (engop < 3) 
			{
			throttle_mini = (91 - 67) / (getprop("/mission/launch/throttle-nominal-percent") - 67);
			#throttle_mini = (91 - 67) / (100 * getprop("/fdm/jsbsim/systems/throttle/throttle-nominal") - 67);
			}
		if (new_throttle < throttle_mini) {new_throttle = throttle_mini;}

		if (SpaceShuttle.PEG.peg_fine_count == 1) {new_throttle = throttle_mini;}

		if (getprop("/fdm/jsbsim/systems/ap/automatic-sb-control") == 1)
			{
			SpaceShuttle.ssme_array[0].throttle_to(new_throttle);
			SpaceShuttle.ssme_array[1].throttle_to(new_throttle);
			SpaceShuttle.ssme_array[2].throttle_to(new_throttle);
			}

		}

	# we need to pitch up briskly, but then reduce sensitivity to avoid oscillations
	# insert a Mach cut in case the TAL init happens late and we never pitch up so much

	if ((aux_flag == 0) and ((vi > 12500) or (getprop("/orientation/pitch-deg") > 40.0)))
		{
		setprop("/fdm/jsbsim/systems/ap/launch/pitch-max-rate-norm", 0.1);
		aux_flag = 1;
		}

	# automatic roll to heads up attitude

	if ((aux_flag == 1) and (vt > 14000))
		{
		#if (getprop("/fdm/jsbsim/systems/ap/launch/rthu-enable") == 1)
		if ((getprop("/fdm/jsbsim/systems/ap/launch/rthu-enable") == 1) and (getprop("/fdm/jsbsim/systems/fcs/control-mode") != 13))
			{
			setprop("/fdm/jsbsim/systems/ap/launch/pitch-max-rate-norm", 0.15);
			setprop("/fdm/jsbsim/systems/ap/launch/rthu-cmd", 1);
			}
		aux_flag = 2;
		}
	else if ((aux_flag == 2) and (getprop("/fdm/jsbsim/systems/ap/launch/stage4-TAL-rthu-in-progress") == 0))
		{
		setprop("/fdm/jsbsim/systems/ap/launch/pitch-max-rate-norm", 0.1);
		aux_flag = 3;
		}


	# change hdot target to shallow flight path prior to MECO

	if ((vi > 21000) and (aux_flag == 3))
		{
		setprop("/fdm/jsbsim/systems/ap/launch/hdot-target", -55);
		setprop("/fdm/jsbsim/systems/ap/launch/pitch-max-rate-norm", 0.1);
		aux_flag = 4;
		} 

	# compute MECO target
	# ballistic impact range approx. range to site

	var a = getprop("/fdm/jsbsim/systems/orbital/semimajor-axis-length-ft");
	var epsilon = getprop("/fdm/jsbsim/systems/orbital/epsilon");
	var R = getprop("/position/sea-level-radius-ft");
	
	var A = -a*a * epsilon * epsilon;
	var B = -2.0 * a * a * a * epsilon;
	var C = a*a * (R*R - a*a );

	var x = 1.0/(2.0 * A) * (-B - math.sqrt(B*B - 4.0 * A * C)) + a * epsilon;
	
	var arg = SpaceShuttle.clamp(x/R, 0.0, 1.0);

	var dist_ballistic = math.acos(arg) * R * 0.3048;
	var dist = shuttle_pos.distance_to (SpaceShuttle.landing_site);


	# ballistic impact point targeting doesn't work well for low velocity, it goes short
	# so we need to delay MECO somewhat
	
	var dist_corr_low_v = 0.0; 
	
	if (vi < 22000.0)
		{
		dist_corr_low_v = (22000.0 - vi) * 250.0;
		}

	setprop("/fdm/jsbsim/systems/ap/launch/ballistic-distance-km", dist_ballistic/1000.0);

	#print (dist_ballistic, " ", (dist - 700000.0), " ", getprop("/fdm/jsbsim/velocities/eci-velocity-mag-fps"));
	#print ("Distance to TAL site in Nm is: ", dist * M2NM); 

	auto_launch_TAL_dist_difference_v = ((dist - 2500000.0) - dist_ballistic)/1500.0;

	# MECO if apoapsis target is met
	#Lower dist ballistic for a lower Vi at cutoff ( more in the middle of TAL range 24.3 ish, compared to before at the limit of High energy one at Vi = 25 kftish)

	#if ((dist_ballistic >  (dist - 2500000.0 + dist_corr_low_v)) or (manual_meco == 0) or (vi > SpaceShuttle.PEG.vd_norm) or (getprop("/consumables/fuel/tank/level-norm") < 0.005))
	#if ((dist * M2NM < 3000) or (manual_meco == 0) or (vi > SpaceShuttle.PEG.vd_norm) or (getprop("/consumables/fuel/tank/level-norm") < 0.005))
	if ((vi > SpaceShuttle.PEG.vd_norm) or (getprop("/consumables/fuel/tank/level-norm") < 0.005))
		{
		
		#PEG exited
		SpaceShuttle.PEG.peg_meco = 1;

		SpaceShuttle.meco_time.set_mode(2);

		if (getprop("/fdm/jsbsim/systems/ap/automatic-sb-control") == 1)
			{	
			SpaceShuttle.ssme_array[0].throttle_to(0.0);
			SpaceShuttle.ssme_array[1].throttle_to(0.0);
			SpaceShuttle.ssme_array[2].throttle_to(0.0);

			ssme_array[0].cutoff();
			ssme_array[1].cutoff();
			ssme_array[2].cutoff();


			}

		setprop("/fdm/jsbsim/systems/ap/launch/regular-meco-condition",1);
		print ("MECO - auto-TAL guidance signing off!");
		print ("Have a good entry and remember to close umbilical doors!");

		# use RCS to null residual rates and disengage launch AP

		settimer( func {
			setprop("/fdm/jsbsim/systems/fcs/control-mode",20);
			controls.centerFlightControls();

			setprop("/fdm/jsbsim/systems/ap/css-pitch-control", 1);
			setprop("/fdm/jsbsim/systems/ap/automatic-pitch-control", 0);    
			setprop("/fdm/jsbsim/systems/ap/css-roll-control", 1);
			setprop("/fdm/jsbsim/systems/ap/automatic-roll-control", 0);

			}, 2.0);

		# prepare ETsep and MM 104 transition

		settimer( func {


				if ((SpaceShuttle.failure_cmd.etsep_mode != 1) and (getprop("/controls/shuttle/et-separation-mode") == 0))
					if (cws_msg_hash.et_sep_auto == 0)
						{
						create_fault_message("    ET SEP    "," MAN", "1234 ", 3, "PASS");	
						cws_msg_hash.et_sep_auto = 1;
						auto_launch_stage = 5;
						setprop("/fdm/jsbsim/systems/ap/launch/stage", 5);
						}


				if (getprop("/controls/shuttle/et-separation-mode") == 0)
					{
					external_tank_separate();
					auto_launch_stage = 5;
					setprop("/fdm/jsbsim/systems/ap/launch/stage", 5);
					}


				}, 20.0);

		settimer( func {
			setprop("/fdm/jsbsim/systems/dps/major-mode", 104);
			SpaceShuttle.dk_listen_major_mode_transition(104);
			SpaceShuttle.ops_transition_auto("p_dps_mnvr");
			}, 23.0);

		# start main orbital loop
		orbital_loop_init();

		SpaceShuttle.mission_post_meco_TAL();	


		return;
		}
	}



auto_launch_timer = auto_launch_timer + 0.1;

settimer(auto_TAL_loop, 0.1);


}



var droop_guidance = {

	droop_alt: 0.0,
	R_sea: 6700000.0,
	pitch_tgt: 74.0,
	pitch_tgt_flown: 74.0,
	pitch_tgt_set: 0,
	active: 0,
	running_engine: -1,

	init: func {

		me.R_sea = getprop("/position/sea-level-radius-ft") * 0.3048;

		me.nd_ref_v_down =  props.globals.getNode("/fdm/jsbsim/velocities/v-down-fps", 1);
		me.nd_ref_vert_acc = props.globals.getNode("/fdm/jsbsim/accelerations/hdotdot-ft_s2", 1);
		me.nd_ref_v_inrtl = props.globals.getNode("/fdm/jsbsim/velocities/eci-velocity-mag-fps", 1);
		me.nd_ref_thrust1 = props.globals.getNode("/fdm/jsbsim/propulsion/engine[0]/thrust-lbs", 1);
		me.nd_ref_thrust2 = props.globals.getNode("/fdm/jsbsim/propulsion/engine[1]/thrust-lbs", 1);
		me.nd_ref_thrust3 = props.globals.getNode("/fdm/jsbsim/propulsion/engine[2]/thrust-lbs", 1);
		me.nd_ref_mass = props.globals.getNode("/fdm/jsbsim/inertia/weight-lbs", 1);

	},

	engage: func {
		me.active = 1;
		setprop("/fdm/jsbsim/systems/ap/launch/droop-guidance-active", 1);
		setprop("/fdm/jsbsim/systems/ap/launch/pitch-max-rate-norm", 0.15);
		droop_flag = 1;
		droop_inh = 1;
		
	},

	disengage: func {
		me.active = 0;
		setprop("/fdm/jsbsim/systems/ap/launch/droop-guidance-active", 0);
		setprop("/fdm/jsbsim/systems/ap/launch/pitch-max-rate-norm", 0.1);
		SpaceShuttle.PEG.v_bias = [0,0,0]; #Reset of Vmiss for better reconvergence
		SpaceShuttle.UFPG_reset("TAL"); #TAL/Droop target for PEG
		SpaceShuttle.callout.make("Droop disengaged, TAL abort might be needed for Auto Steering after a sucessful droop", "help");
	},


	droop_trajectory : func (t, phi, y0, vy0, vx0, T, R, Delta) {

		var phi_rad = phi * math.pi/180.0;

		var cphi = math.cos(phi_rad);
		var sphi = math.sin(phi_rad);

		var A = 0.5 * (T * sphi - 9.41 + vx0*vx0/R);
		var B = 0.333 * (T/R * vx0 * cphi + 0.5 * Delta * sphi);
		var C = 0.08333 * (T*T/R * cphi * cphi + vx0/R * Delta * cphi);
		var D = 0.05 * (T * Delta / R * math.pow(cphi, 2.0));
		var E = 0.008333 * (Delta * Delta/R * cphi * cphi);

		return y0 + vy0 * t + A* math.pow(t, 2.0) + B * math.pow(t, 3.0) + C * math.pow(t, 4.0) + D * math.pow(t, 5.0) + E * math.pow(t, 6.0);
		},

	run : func (altitude) {

		if (me.pitch_tgt < 35.0) 
			{
			#if (me.active == 1) {me.disengage();}
			if (me.active == 0) 
				{
				#print ("Droop guidance signing off."); 
				return;
				}
			}

		#print ("Pitch tgt: ", me.pitch_tgt);

		if (me.droop_alt > 110000)
			{
			me.pitch_tgt = me.pitch_tgt - 1.0;
			if (me.pitch_tgt_set == 0)
				{
				me.pitch_tgt_flown = me.pitch_tgt;
				}
			}




		var y0 = altitude * 0.3048;
		var R = y0 + me.R_sea;


		var vy0 = -me.nd_ref_v_down.getValue() * 0.3048;
		var vx0 = me.nd_ref_v_inrtl.getValue() * 0.3048;

		var n_engines = getprop("/fdm/jsbsim/systems/mps/number-engines-operational");
		var abort_mode = getprop("/fdm/jsbsim/systems/abort/abort-mode");

		if (me.active == 1)
			{
			if (me.pitch_tgt_set == 0)
				{
				setprop("/fdm/jsbsim/systems/ap/launch/droop-pitch-target", me.pitch_tgt);
				me.pitch_tgt_flown = me.pitch_tgt;
				me.pitch_tgt_set = 1;
				}

			if (vy0 > 0.0)
				{
				if (me.nd_ref_vert_acc.getValue() < 0.0) # make sure we don't disengage early while the original trajectory is rising
					{
					me.disengage();
					me.pitch_tgt = 0.0;
					}
				}
			#Forced to disengage if an intact abort or a contigency one is called after droop engage (Might happen at OPS6/3 boundary)
			if (abort_mode > 4)
				{
				me.disengage();
				me.pitch_tgt = 0.0;
				}
			}

		else if (me.active == 0) 
			{
			#Hyptothetical Mode parameter (Null vertical speed) / early WIP 
			vy0 = 0;

			#Logic for 1EO droop to be done there
			if ((n_engines == 1) or ((n_engines == 2) and (droop_inh == 1)))
				{			
				me.engage();
				}
			}

		var T = me.nd_ref_thrust1.getValue();
		T = T + me.nd_ref_thrust2.getValue();
		T = T + me.nd_ref_thrust3.getValue();

		var throttle_setting = T/(n_engines * 470717.0);

		var M = me.nd_ref_mass.getValue();

		T = T/M;
		T = T/n_engines;


		T = T * 9.81;
		T = T * 0.97;

		var Delta = M/(M - (1042.1 * throttle_setting)) - 1.0;
		Delta = Delta * 9.81;

		var alt_last = 1000000.0;
		var alt = 0.0;


		
		for (var i=0; i< 35; i=i+1)
			{
			alt = me.droop_trajectory (i * 15.0 + 15.0, me.pitch_tgt_flown, y0, vy0, vx0, T, R, Delta);
		
			if (alt > alt_last) 
				{

				me.droop_alt =  alt_last;
				return;
				}
			else if (alt < 0.0)
				{

				me.droop_alt =  0.0;
				return;
				}


			alt_last = alt;
			}
		me.droop_alt = alt;
		return;
		},


};

var meco_time = {

	
	time_to_meco: 0.0,
	inertial_target: 0.0,
	counter: 0,
	mode: 0,
	delta_met: 0,


	init: func (inertial_target) {

		me.nd_ref_v_inrtl = props.globals.getNode("/fdm/jsbsim/velocities/eci-velocity-mag-fps", 1);
		me.nd_ref_thrust1 = props.globals.getNode("/fdm/jsbsim/propulsion/engine[0]/thrust-lbs", 1);
		me.nd_ref_thrust2 = props.globals.getNode("/fdm/jsbsim/propulsion/engine[1]/thrust-lbs", 1);
		me.nd_ref_thrust3 = props.globals.getNode("/fdm/jsbsim/propulsion/engine[2]/thrust-lbs", 1);
		me.nd_ref_mass = props.globals.getNode("/fdm/jsbsim/inertia/weight-lbs", 1);

		me.inertial_target = inertial_target * 0.3048;

		me.delta_met = getprop("/fdm/jsbsim/systems/timer/delta-MET");

	},
	
	run: func {

		if (me.mode == 2) {return;}


		# don't need to execute in every guidance cycle
		if (me.counter == 5) 
			{me.counter = 0;}
		else
			{
			me.counter = me.counter + 1;
			return;
			}		
			
		var n_engines = getprop("/fdm/jsbsim/systems/mps/number-engines-operational");

		var T = me.nd_ref_thrust1.getValue();
		T = T + me.nd_ref_thrust2.getValue();
		T = T + me.nd_ref_thrust3.getValue();

		var throttle_setting = T/(n_engines * 470717.0);

		var M = me.nd_ref_mass.getValue();

		var a0 = T/M;
		a0 = a0 * 9.81;

		# to SI units
		M = M * 0.45359237;
		T = T * 4.44822161526;

		#print ("Current acceleration: ", a0/9.81, " g");
		#print ("Current mass:" , M, " kg");
		#print ("Current thrust:", T, " N");

		var mdot = 14.5939 * SpaceShuttle.PEG_jsbsim.m_dot; #Slugs to kg
		#var mdot = 1417.9 * throttle_setting; #Lbs to kg

		var v0 = me.nd_ref_v_inrtl.getValue() * 0.3048;

		var t_3g = 0.0;
		var v_3g = v0;
		

		var ops =  getprop("/fdm/jsbsim/systems/dps/ops");
		var ppd = getprop("/fdm/jsbsim/systems/ap/rtls/powered-pitchdown-active");
		

		#3G target and 2 mn of throttling down for a nominal MECO vs 2.9gish for RTLS one ( plus minimum thrust 5 seconds before MECO)
		
		if (me.mode == 0)  
			{
		#	if (ops != 6) #Nominal MECO calculation
		#		{
		#		t_3g = (29.43 * M - T) / (29.43 * mdot );   #3G
		#		v_3g = v0 + 4434.12 * math.ln(M/(M - mdot * t_3g)); #Time between 3G and MECO
		#		}

		#	else 	 #Rough RTLS MECO calculation that works OK, to be fine tunned with PEG algorithm book (It converges quite well after PPA)
			if (ops == 6)
				{

				#Time to 3.0G (end of RTLS roughly)
				if (ppd == 0) {t_3g = (29.43 * M - T) / (29.43 * mdot );}

				#If ppd initiated, Tmeco nulled to avoid NaN a few seconds before real MECO
				else {t_3g = 0.0;}
				

				#No 3g throttling ( time to meco = t3g only for RTLS)
				v_3g = me.inertial_target;
				}
			}
		#print ("Time to throttling: ", t_3g, " s");
		#print ("Inertial velocity at throttling: ", v_3g, " m/s");
		#print ("Time to MECO: ", me.time_to_meco, " s");

	#Tweak there // To be redone with RTLS tgo based on PEG guidance	
	if (ops == 6) {me.time_to_meco = t_3g + (me.inertial_target - v_3g) / 29.43;}

	#PEG tgo_total
	else {me.time_to_meco = SpaceShuttle.PEG.tgo_total;}
	
	},


	set_mode: func (mode) {

		me.mode = mode;

	},

	get_mode: func () {

		return me.mode;

	},

	get: func () {

		return me.time_to_meco + me.delta_met;
	},


};

