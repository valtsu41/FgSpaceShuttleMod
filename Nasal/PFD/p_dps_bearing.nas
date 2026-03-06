#---------------------------------------
# SpaceShuttle PFD Page include:
#        Page: p_dps_bearing
# Description: Spec 54 bearing displays page / OI-33 PASS new dps SPEC function (SCOM)
#      Author: GinGin, 2025
#---------------------------------------

var PFD_addpage_p_dps_bearing = func(device)
{
    var p_dps_bearing = device.addPage("CRTBearing", "p_dps_bearing");

	#Elements from svg file
    p_dps_bearing.group = device.svg.getElementById("p_dps_bearing");
    p_dps_bearing.group.setColor(dps_r, dps_g, dps_b);

	p_dps_bearing.main_landing_site_number = device.svg.getElementById("p_dps_bearing_main_landing_site");  
    p_dps_bearing.main_landing_site_iata = device.svg.getElementById("p_dps_bearing_main_landing_site_iata");  
	p_dps_bearing.alternate_one_landing_site_number = device.svg.getElementById("p_dps_bearing_alternate_one_landing_site");  
    p_dps_bearing.alternate_one_landing_site_iata = device.svg.getElementById("p_dps_bearing_alternate_one_landing_site_iata"); 
	p_dps_bearing.alternate_two_landing_site_number = device.svg.getElementById("p_dps_bearing_alternate_two_landing_site");  
    p_dps_bearing.alternate_two_landing_site_iata = device.svg.getElementById("p_dps_bearing_alternate_two_landing_site_iata");

	p_dps_bearing.main_landing_site_number.enableUpdate();
	p_dps_bearing.main_landing_site_iata.enableUpdate();
	p_dps_bearing.alternate_one_landing_site_number.enableUpdate();
	p_dps_bearing.alternate_one_landing_site_iata.enableUpdate();
	p_dps_bearing.alternate_two_landing_site_number.enableUpdate();
	p_dps_bearing.alternate_two_landing_site_iata.enableUpdate();

	p_dps_bearing.site_main_iata = device.svg.getElementById("p_dps_bearing_site_main_iata");
	p_dps_bearing.site_alt_one_iata = device.svg.getElementById("p_dps_bearing_site_alt_one_iata");
	p_dps_bearing.site_alt_two_iata = device.svg.getElementById("p_dps_bearing_site_alt_two_iata");

	p_dps_bearing.site_main_iata.enableUpdate();
	p_dps_bearing.site_alt_one_iata.enableUpdate();
	p_dps_bearing.site_alt_two_iata.enableUpdate();

	#Delta Azimuth
	p_dps_bearing.delta_az_main = device.svg.getElementById("p_dps_bearing_delta_az_main");
	p_dps_bearing.delta_az_alt_one = device.svg.getElementById("p_dps_bearing_delta_az_alt_one");
	p_dps_bearing.delta_az_alt_two = device.svg.getElementById("p_dps_bearing_delta_az_alt_two");

	#p_dps_bearing.delta_az_alt_one.enableUpdate();
	#p_dps_bearing.delta_az_alt_two.enableUpdate();

	#EW group 
	p_dps_bearing.entry_EW_main_group = device.svg.getElementById("p_dps_bearing_entry_EW_main_group");
	p_dps_bearing.entry_EW_alt_one_group = device.svg.getElementById("p_dps_bearing_entry_EW_alt_one_group");
	p_dps_bearing.entry_EW_alt_two_group = device.svg.getElementById("p_dps_bearing_entry_EW_alt_two_group");

	#Colors
	p_dps_bearing.entry_EW_main_group.setColor(0.8, 0.8, 0.4);
	p_dps_bearing.site_main_iata.setColor(0.8, 0.8, 0.4);
	p_dps_bearing.delta_az_main.setColor(0.8, 0.8, 0.4);
	p_dps_bearing.entry_EW_alt_one_group.setColor(1.0, 1.0, 1.0);
	p_dps_bearing.site_alt_one_iata.setColor(1.0, 1.0, 1.0);
	p_dps_bearing.delta_az_alt_one.setColor(1.0, 1.0, 1.0);
	p_dps_bearing.entry_EW_alt_two_group.setColor(0.6, 1.0, 1.0);
	p_dps_bearing.site_alt_two_iata.setColor(0.6, 1.0, 1.0);
	p_dps_bearing.delta_az_alt_two.setColor(0.6, 1.0, 1.0);

	#Ascent / Entry Group
	p_dps_bearing.entry_group = device.svg.getElementById("p_dps_bearing_entry_group");
	p_dps_bearing.entry_EW_group = device.svg.getElementById("p_dps_bearing_entry_EW_group");


    


	#Elements that are called once whenever this page goes on display (fixed elements)
    p_dps_bearing.ondisplay = func {

		var major_mode = getprop("/fdm/jsbsim/systems/dps/major-mode");
		var ops = getprop("/fdm/jsbsim/systems/dps/ops");
		
		#Either Ascent or Entry Bearing function (Only entry supported for now)
		if (ops == 1) {device.DPS_menu_title.setText("                ASCENT BEARING");}
		else {device.DPS_menu_title.setText("                  ENTRY BEARING");}
        device.MEDS_menu_title.setText("      DPS MENU");

		#Spec 54 in PASS only
		var ops_string = major_mode~"1/054/";
        device.DPS_menu_ops.setText(ops_string);


		#Group for shuttle fixed symbols
		p_dps_bearing.shuttle_group = device.symbols.createChild("group")
			.setTranslation(320,350);

		#Fixed shuttle symbol for OPS 6/3
		var data = SpaceShuttle.draw_shuttle_back();
		p_dps_bearing.shuttle_box = p_dps_bearing.shuttle_group.createChild("path")
			.setStrokeLineWidth(1.5)
			.setColor(dps_r, dps_g, dps_b)
			.setTranslation(0,0)
			.setScale(0.6)
			.setCenter(0, 10) #To have a rotation around the true center
			.moveTo(data[0][0], data[0][1]);

		for (var i = 0; (i< size(data)-1); i=i+1)
        	{
			var set = data[i+1]; 
			p_dps_bearing.shuttle_box.lineTo(set[0], set[1]);
			}
		setsize(data, 0);

		#Instantaneous impact point 
		data = SpaceShuttle.draw_cross();
		p_dps_bearing.iip = p_dps_bearing.shuttle_group.createChild("path")
			.setStrokeLineWidth(1.2)
			.setColor(1, 1, 1)
			.setScale(1.2)
			.moveTo(data[0][0], data[0][1]);

		for (var i = 0; (i< size(data)-1); i=i+1)
        	{
			var set = data[i+1]; 
			p_dps_bearing.iip.lineTo(set[0], set[1]);
			}
		setsize(data, 0);

		#Roll reversal lines for MM304 (dashed for 10° RR lines)// length fixed
		p_dps_bearing.roll_reversal_lines_group = p_dps_bearing.shuttle_group.createChild("group");

		data = [[0, 0], [0, -70]];
		p_dps_bearing.rr_line_left = p_dps_bearing.roll_reversal_lines_group.createChild("path")
			.setStrokeLineWidth(1.0)
			.setColor(dps_r, dps_g, dps_b)
			.setScale(1.0, 1.0)
			.setRotation(-0.1832596)
			.moveTo(data[0][0], data[0][1]);

		for (var i = 0; (i< size(data)-1); i=i+1)
        	{
			var set = data[i+1]; 
			p_dps_bearing.rr_line_left.lineTo(set[0], set[1]);
			}
		setsize(data, 0);

		data = [[0, 0], [0, -70]];
		p_dps_bearing.rr_line_right = p_dps_bearing.roll_reversal_lines_group.createChild("path")
			.setStrokeLineWidth(1.0)
			.setColor(dps_r, dps_g, dps_b)
			.setScale(1.0, 1.0)
			.setRotation(0.1832596)
			.moveTo(data[0][0], data[0][1]);

		for (var i = 0; (i< size(data)-1); i=i+1)
        	{
			var set = data[i+1]; 
			p_dps_bearing.rr_line_right.lineTo(set[0], set[1]);
			}
		setsize(data, 0);

		data = [[0, 0], [0, -100]];
		p_dps_bearing.rr_dashed_left = p_dps_bearing.roll_reversal_lines_group.createChild("path")
			.setStrokeDashArray([5,5])
			.setStrokeLineWidth(1.0)
			.setColor(dps_r, dps_g, dps_b)
			.setScale(1.0, 1.0)
			.setRotation(-0.1832596)
			.moveTo(data[0][0], data[0][1]);

		for (var i = 0; (i< size(data)-1); i=i+1)
        	{
			var set = data[i+1]; 
			p_dps_bearing.rr_dashed_left.lineTo(set[0], set[1]);
			}
		setsize(data, 0);

		data = [[0, 0], [0, -100]];
		p_dps_bearing.rr_dashed_right = p_dps_bearing.roll_reversal_lines_group.createChild("path")
			.setStrokeDashArray([5,5])
			.setStrokeLineWidth(1.0)
			.setColor(dps_r, dps_g, dps_b)
			.setScale(1.0, 1.0)
			.setRotation(0.1832596)
			.moveTo(data[0][0], data[0][1]);

		for (var i = 0; (i< size(data)-1); i=i+1)
        	{
			var set = data[i+1]; 
			p_dps_bearing.rr_dashed_right.lineTo(set[0], set[1]);
			}
		setsize(data, 0);


		#Range circle based on main landing site / common clipping for the group
		#Rings are not displayed on all the page
		device.tapes.set("clip", "rect(0px, 592px, 385px, 170px)");

		#Main site Azimuth Line // lenght variable (update)
		data = [[0, 0], [0, -150]];
		p_dps_bearing.main_azimuth_line = device.tapes.createChild("path")
			.setStrokeLineWidth(1)
			.setTranslation(320,350)
			.setColor(0.8, 0.8, 0.4)
			.setScale(1.0, 1.0)
			.moveTo(data[0][0], data[0][1]);

		for (var i = 0; (i< size(data)-1); i=i+1)
        	{
			var set = data[i+1]; 
			p_dps_bearing.main_azimuth_line.lineTo(set[0], set[1]);
			}
		setsize(data, 0);


		p_dps_bearing.distance_ring = device.tapes.createChild("group")
			.setTranslation(320,350);

		#Main landing site
		p_dps_bearing.main_site_label = p_dps_bearing.distance_ring.createChild("text")
			.setText("")
			.setColor(0.8, 0.8, 0.4)
			.setFontSize(12)
			.setFont(SpaceShuttle.p_pfd_font_1)
			.setAlignment("center-bottom")
			.setTranslation(0.0,-5.0)
			.setRotation(0.0);

		
		data = SpaceShuttle.draw_circle(1, 5);
		p_dps_bearing.main_site_symbol = p_dps_bearing.distance_ring.createChild("path")
			.setStrokeLineWidth(1)
			.setColor(0.8, 0.8, 0.4)
			.setScale(0.7, 1.20)
			.moveTo(data[0][0], data[0][1]);

		for (var i = 0; (i< size(data)-1); i=i+1)
        	{
			var set = data[i+1]; 
			p_dps_bearing.main_site_symbol.lineTo(set[0], set[1]);
			}
		setsize(data, 0);

		#Dashed range rings
		data = SpaceShuttle.draw_circle(190, 190);
		p_dps_bearing.plot_int_ring = p_dps_bearing.distance_ring.createChild("path")
			.setStrokeLineWidth(1.0)
			.setColor(dps_r, dps_g, dps_b)
			.setScale(1.0)
			.moveTo(data[0][0], data[0][1]);

		var i = 0;
		while (i < (size(data)-1))
        	{
			var set = data[i+1]; 
			if (math.fmod(i, 2) == 0) {p_dps_bearing.plot_int_ring.moveTo(set[0], set[1]);}
			else {p_dps_bearing.plot_int_ring.lineTo(set[0], set[1]);}
			i = i + 1;
			}
		setsize(data, 0);

		data = SpaceShuttle.draw_circle(95, 95);
		p_dps_bearing.plot_ext_ring = p_dps_bearing.distance_ring.createChild("path")
			.setStrokeLineWidth(1.0)
			.setColor(dps_r, dps_g, dps_b)
			.setScale(1.0)
			.moveTo(data[0][0], data[0][1]);

		var i = 0;
		while (i < (size(data)-1))
        	{
			var set = data[i+1]; 
			if (math.fmod(i, 2) == 0) {p_dps_bearing.plot_ext_ring.moveTo(set[0], set[1]);}
			else {p_dps_bearing.plot_ext_ring.lineTo(set[0], set[1]);}
			i = i + 1;
			}
		setsize(data, 0);
		

		p_dps_bearing.int_ring_range = p_dps_bearing.distance_ring.createChild("text")
			.setText("1000")
			.setColor(dps_r, dps_g, dps_b)
			.setFontSize(12)
			.setFont(SpaceShuttle.p_pfd_font_1)
			.setAlignment("center-bottom")
			.setTranslation(67.0,67.0)
			.setRotation(0.0);

		p_dps_bearing.ext_ring_range = p_dps_bearing.distance_ring.createChild("text")
			.setText("2000")
			.setColor(dps_r, dps_g, dps_b)
			.setFontSize(12)
			.setFont(SpaceShuttle.p_pfd_font_1)
			.setAlignment("center-bottom")
			.setTranslation(134.0,134.0)
			.setRotation(0.0);

		p_dps_bearing.main_site_label.enableUpdate();
		p_dps_bearing.int_ring_range.enableUpdate();
		p_dps_bearing.ext_ring_range.enableUpdate();


		#Alternate Landing Site One
		p_dps_bearing.alt_one_site = device.tapes.createChild("group")
			.setTranslation(320,350);

		p_dps_bearing.alt_one_site_label = p_dps_bearing.alt_one_site.createChild("text")
			.setText("")
			.setColor(1, 1, 1)
			.setFontSize(12)
			.setFont(SpaceShuttle.p_pfd_font_1)
			.setAlignment("center-bottom")
			.setTranslation(0.0,-5.0)
			.setRotation(0.0);

		p_dps_bearing.alt_one_site_symbol = p_dps_bearing.alt_one_site.createChild("group");
		canvas.draw.circle(p_dps_bearing.alt_one_site_symbol, 2)
			.setColor(1, 1, 1)
			.setColorFill(1, 1, 1)
			.setScale(1.0, 1.0)
			.setTranslation(0.0, 0.0);


		#Alternate Landing Site Two
		p_dps_bearing.alt_two_site = device.tapes.createChild("group")
			.setTranslation(320,350);

		p_dps_bearing.alt_two_site_label = p_dps_bearing.alt_two_site.createChild("text")
			.setText("")
			.setColor(0.6, 1.0, 1.0)
			.setFontSize(12)
			.setFont(SpaceShuttle.p_pfd_font_1)
			.setAlignment("center-bottom")
			.setTranslation(0.0,-5.0)
			.setRotation(0.0);

		p_dps_bearing.alt_two_site_symbol = p_dps_bearing.alt_two_site.createChild("group");
		canvas.draw.diamond(p_dps_bearing.alt_two_site_symbol, 4, 6, center_x = 0, center_y = 0) 
			.setColor(0.6, 1.0, 1.0)
			.setColorFill(0.6, 1.0, 1.0)
			.setScale(1.0, 1.0)
			.setTranslation(0.0, 0.0);

		
		p_dps_bearing.alt_one_site_label.enableUpdate();
		p_dps_bearing.alt_two_site_label.enableUpdate();
			

		#Energy cues for ECAL Alpha transition 
		p_dps_bearing.EW_group = device.symbols.createChild("group");
		
		#Main site 
		p_dps_bearing.EW_group_main = p_dps_bearing.EW_group.createChild("group");
		data = SpaceShuttle.draw_tmarker_right();
		p_dps_bearing.actual_energy_main_site = p_dps_bearing.EW_group_main.createChild("path")
        .setStrokeLineWidth(1.0)
        .setColor(0.8, 0.8, 0.4) 
		.setScale(0.9, 0.7) 
		.moveTo(data[0][0], data[0][1]);

		for (var i = 0; (i< size(data)-1); i=i+1)
       	 	{
			var set = data[i+1]; 
			p_dps_bearing.actual_energy_main_site.lineTo(set[0], set[1]);
			}

		#Nominal E/W
		data = SpaceShuttle.draw_rect(10, 0);
		p_dps_bearing.energy_nom_main = p_dps_bearing.EW_group_main.createChild("path")
			.setStrokeLineWidth(1.0)
			.setColor(0.8, 0.8, 0.4)  
		.moveTo(data[0][0], data[0][1]);

		for (var i = 0; (i< size(data)-1); i=i+1)
				{
			var set = data[i+1]; 
			p_dps_bearing.energy_nom_main.lineTo(set[0], set[1]);
			}

		#Upper Nominal E/W (+8000 ft)
		data = SpaceShuttle.draw_rect(12, 0);
		p_dps_bearing.energy_nom_upper_main = p_dps_bearing.EW_group_main.createChild("path")
			.setStrokeLineWidth(1.0)
			.setColor(0.8, 0.8, 0.4)  # Yellow color for ladder markers
		.moveTo(data[0][0], data[0][1]);

		for (var i = 0; (i< size(data)-1); i=i+1)
				{
			var set = data[i+1]; 
			p_dps_bearing.energy_nom_upper_main.lineTo(set[0], set[1]);
			}

		#Lower Nominal E/W (-4000 ft)
		data = SpaceShuttle.draw_rect(12, 0);
		p_dps_bearing.energy_nom_lower_main = p_dps_bearing.EW_group_main.createChild("path")
			.setStrokeLineWidth(1.0)
			.setColor(0.8, 0.8, 0.4)  # Yellow color for ladder markers 
		.moveTo(data[0][0], data[0][1]);

		for (var i = 0; (i< size(data)-1); i=i+1)
				{
			var set = data[i+1]; 
			p_dps_bearing.energy_nom_lower_main.lineTo(set[0], set[1]);
			}
		

		#Alt one site 
		p_dps_bearing.EW_group_alt_one = p_dps_bearing.EW_group.createChild("group");
		data = SpaceShuttle.draw_tmarker_right();
		p_dps_bearing.actual_energy_alt_one_site = p_dps_bearing.EW_group_alt_one.createChild("path")
        .setStrokeLineWidth(1.0)
        .setColor(1, 1, 1)
		.setScale(0.9, 0.7) 
		.moveTo(data[0][0], data[0][1]);

		for (var i = 0; (i< size(data)-1); i=i+1)
       	 	{
			var set = data[i+1]; 
			p_dps_bearing.actual_energy_alt_one_site.lineTo(set[0], set[1]);
			}
		
		#Nominal E/W
		data = SpaceShuttle.draw_rect(10, 0);
		p_dps_bearing.energy_nom_alt_one = p_dps_bearing.EW_group_alt_one.createChild("path")
			.setStrokeLineWidth(1.0)
			.setColor(1, 1, 1)  
		.moveTo(data[0][0], data[0][1]);

		for (var i = 0; (i< size(data)-1); i=i+1)
				{
			var set = data[i+1]; 
			p_dps_bearing.energy_nom_alt_one.lineTo(set[0], set[1]);
			}

		#Upper Nominal E/W (+8000 ft)
		data = SpaceShuttle.draw_rect(12, 0);
		p_dps_bearing.energy_nom_upper_alt_one = p_dps_bearing.EW_group_alt_one.createChild("path")
			.setStrokeLineWidth(1.0)
			.setColor(1, 1, 1)  
		.moveTo(data[0][0], data[0][1]);

		for (var i = 0; (i< size(data)-1); i=i+1)
				{
			var set = data[i+1]; 
			p_dps_bearing.energy_nom_upper_alt_one.lineTo(set[0], set[1]);
			}

		
		#Lower Nominal E/W (-4000 ft)
		data = SpaceShuttle.draw_rect(12, 0);
		p_dps_bearing.energy_nom_lower_alt_one = p_dps_bearing.EW_group_alt_one.createChild("path")
			.setStrokeLineWidth(1.0)
			.setColor(1, 1, 1)  	 
		.moveTo(data[0][0], data[0][1]);

		for (var i = 0; (i< size(data)-1); i=i+1)
				{
			var set = data[i+1]; 
			p_dps_bearing.energy_nom_lower_alt_one.lineTo(set[0], set[1]);
			}


		#Alt two site 
		p_dps_bearing.EW_group_alt_two = p_dps_bearing.EW_group.createChild("group");
		data = SpaceShuttle.draw_tmarker_right();
		p_dps_bearing.actual_energy_alt_two_site = p_dps_bearing.EW_group_alt_two.createChild("path")
        .setStrokeLineWidth(1.0)
        .setColor(0.6, 1.0, 1.0)  
		.setScale(0.9, 0.7) 
		.moveTo(data[0][0], data[0][1]);

		for (var i = 0; (i< size(data)-1); i=i+1)
       	 	{
			var set = data[i+1]; 
			p_dps_bearing.actual_energy_alt_two_site.lineTo(set[0], set[1]);
			}

		#Nominal E/W
		data = SpaceShuttle.draw_rect(10, 0);
		p_dps_bearing.energy_nom_alt_two = p_dps_bearing.EW_group_alt_two.createChild("path")
			.setStrokeLineWidth(1.0)
			.setColor(0.6, 1.0, 1.0)  
		.moveTo(data[0][0], data[0][1]);

		for (var i = 0; (i< size(data)-1); i=i+1)
				{
			var set = data[i+1]; 
			p_dps_bearing.energy_nom_alt_two.lineTo(set[0], set[1]);
			}

		#Upper Nominal E/W (+8000 ft)
		data = SpaceShuttle.draw_rect(12, 0);
		p_dps_bearing.energy_nom_upper_alt_two = p_dps_bearing.EW_group_alt_two.createChild("path")
			.setStrokeLineWidth(1.0)
			.setColor(0.6, 1.0, 1.0)   
		.moveTo(data[0][0], data[0][1]);

		for (var i = 0; (i< size(data)-1); i=i+1)
				{
			var set = data[i+1]; 
			p_dps_bearing.energy_nom_upper_alt_two.lineTo(set[0], set[1]);
			}

		
		#Lower Nominal E/W (-4000 ft)
		data = SpaceShuttle.draw_rect(12, 0);
		p_dps_bearing.energy_nom_lower_alt_two = p_dps_bearing.EW_group_alt_two.createChild("path")
			.setStrokeLineWidth(1.0)
			.setColor(0.6, 1.0, 1.0)  	 
		.moveTo(data[0][0], data[0][1]);

		for (var i = 0; (i< size(data)-1); i=i+1)
				{
			var set = data[i+1]; 
			p_dps_bearing.energy_nom_lower_alt_two.lineTo(set[0], set[1]);
			}


		#No rings / rr lines displayed if no guidance 
		p_dps_bearing.distance_ring.setVisible(0);
		p_dps_bearing.alt_one_site.setVisible(0);
		p_dps_bearing.alt_two_site.setVisible(0);
		p_dps_bearing.roll_reversal_lines_group.setVisible(0);
		p_dps_bearing.main_azimuth_line.setVisible(0);
		p_dps_bearing.iip.setVisible(0);
		p_dps_bearing.EW_group.setVisible(0);

		


		#Switch elements between Ascent and Entry Bearing mode
		if (ops == 1) 
			{
			p_dps_bearing.distance_ring.setVisible(0);
			p_dps_bearing.shuttle_group.setVisible(0);
			p_dps_bearing.entry_group.setVisible(0);
			p_dps_bearing.alt_one_site.setVisible(0);
			p_dps_bearing.alt_two_site.setVisible(0);
			}

		else if (ops == 3)
			{
			#No E/W ladders for Entry
			p_dps_bearing.entry_EW_group.setVisible(0);
			}

		else if (ops == 6)
			{
			#No RR lines for ECAL
			p_dps_bearing.roll_reversal_lines_group.setVisible(0);
			}
	
	
    }



	#Fixed elements cleaned when canvas is exited
    p_dps_bearing.offdisplay = func {
    
    device.symbols.removeAllChildren();
	device.tapes.removeAllChildren();
    device.nom_traj_plot.removeAllChildren();
    }



	#Elements redrawn at each update loop
    p_dps_bearing.update = func {

	var major_mode = getprop("/fdm/jsbsim/systems/dps/major-mode");
	var ops = getprop("/fdm/jsbsim/systems/dps/ops");
	var pos = SpaceShuttle.state_vector_position();

	#Entry Bearing OPS 3
	if (ops == 3)
		{
		
		#Main site
		p_dps_bearing.main_landing_site_number.updateText(sprintf("%2d",SpaceShuttle.landing_site.index));
		p_dps_bearing.main_landing_site_iata.updateText(SpaceShuttle.landing_site.rwy_pri);

		if (SpaceShuttle.EGD.START_LOOP == 1)
			{
			#Elements visible once guidance is running
			p_dps_bearing.distance_ring.setVisible(1);
			p_dps_bearing.roll_reversal_lines_group.setVisible(1);
			p_dps_bearing.main_azimuth_line.setVisible(1);
			p_dps_bearing.iip.setVisible(1);

			#Main site items displayed with guidance only
			p_dps_bearing.site_main_iata.updateText(left(SpaceShuttle.landing_site.rwy_pri, 3));
			p_dps_bearing.main_site_label.updateText(left(SpaceShuttle.landing_site.rwy_pri, 3));
			p_dps_bearing.delta_az_main.setText(sprintf("%4d",SpaceShuttle.EGRT_data.DELAZ * 57.29578));

			#Shuttle box bank
			var shuttle_bank = getprop("/fdm/jsbsim/systems/navigation/state-vector/roll-deg");
			shuttle_bank = SpaceShuttle.MIDVAL(-90, 90, shuttle_bank);
			p_dps_bearing.shuttle_box.setRotation(shuttle_bank * D2R);

			#RR lines logic
			p_dps_bearing.rr_line_left.setRotation(-SpaceShuttle.EGD.Y[1]);
			p_dps_bearing.rr_line_right.setRotation(SpaceShuttle.EGD.Y[1]);

			#Main site and range rings (Nm)
			var distance_factor = 4000;

			if (SpaceShuttle.EGRT_data.TRANGE > 2000) 
				{
				distance_factor = 4000;
				p_dps_bearing.int_ring_range.updateText("2000");
				p_dps_bearing.ext_ring_range.updateText("4000");
				}		
			else if ((SpaceShuttle.EGRT_data.TRANGE < 2000) and (SpaceShuttle.EGRT_data.TRANGE > 1000))
				{
				distance_factor = 2000;
				p_dps_bearing.int_ring_range.updateText("1000");
				p_dps_bearing.ext_ring_range.updateText("2000");
				}
			else if ((SpaceShuttle.EGRT_data.TRANGE < 1000) and (SpaceShuttle.EGRT_data.TRANGE > 500))
				{
				distance_factor = 1000;
				p_dps_bearing.int_ring_range.updateText("500");
				p_dps_bearing.ext_ring_range.updateText("1000");
				}
			else if ((SpaceShuttle.EGRT_data.TRANGE < 500) and (SpaceShuttle.EGRT_data.TRANGE > 250))
				{
				distance_factor = 500;
				p_dps_bearing.int_ring_range.updateText("250");
				p_dps_bearing.ext_ring_range.updateText("500");
				}
			else if (SpaceShuttle.EGRT_data.TRANGE < 250)
				{
				distance_factor = 250;
				p_dps_bearing.int_ring_range.updateText("125");
				p_dps_bearing.ext_ring_range.updateText("250");
				}

			var scale = 190 / distance_factor;
			var converted_dist = scale * SpaceShuttle.EGRT_data.TRANGE;
			var main_site_x = math.sin(-SpaceShuttle.EGRT_data.DELAZ) * converted_dist;
			var main_site_y = -math.cos(-SpaceShuttle.EGRT_data.DELAZ) * converted_dist;

			#p_dps_bearing.distance_ring.setTranslation(320, 350 -scale * SpaceShuttle.EGRT_data.TRANGE);
			p_dps_bearing.distance_ring.setTranslation(320 + main_site_x, 350 + main_site_y);
			
			#Main site Azimuth Line // lenght variable (update)
			p_dps_bearing.main_azimuth_line.pop_back();
			p_dps_bearing.main_azimuth_line.addSegment(canvas.Path.VG_LINE_TO_REL, [main_site_x, main_site_y]);
			#p_dps_bearing.main_azimuth_line.setStrokeDashArray([converted_dist - 10,10]);

			#Instantaneous impact point
			var ballistic_range = SpaceShuttle.instantaneous_impact_point();
			ballistic_range = scale * ballistic_range;
			p_dps_bearing.iip.setTranslation(0, -ballistic_range);

			#print("ballistic range y is : ", 350 - ballistic_range);

			#Distance Label
			#p_dps_bearing.ext_ring_range.setTranslation(320 + main_site_x, 350 + main_site_y);	
			#print("Center X is : ", p_dps_bearing.ext_ring_range.getCenter()[0], "Center Y is : ", p_dps_bearing.ext_ring_range.getCenter()[1]);


			#Alternate sites 
			if (SpaceShuttle.alt_sites.entry_flag == 1)
				{
				#Delta azimuth to alternate
				var delta_az = 0;

				if (SpaceShuttle.alt_sites.one_index != 0)
					{
					
					p_dps_bearing.alternate_one_landing_site_number.updateText(sprintf("%2d",SpaceShuttle.alt_sites.one_index));
					p_dps_bearing.alternate_one_landing_site_iata.updateText(SpaceShuttle.alt_sites.one_iata);
					p_dps_bearing.site_alt_one_iata.updateText(SpaceShuttle.alt_sites.one_shortname);

					#Delta Az
					delta_az = SpaceShuttle.delta_azimuth_entry(SpaceShuttle.landing_alt_site_one);
					p_dps_bearing.delta_az_alt_one.setText(sprintf("%4d",delta_az));

					#Site shortname
					p_dps_bearing.alt_one_site.setVisible(1);
					p_dps_bearing.alt_one_site_label.updateText(SpaceShuttle.alt_sites.one_shortname);

					#Azimuth and distance (same factor than for the main site)
					converted_dist = scale * pos.distance_to(SpaceShuttle.landing_alt_site_one) * M2NM;
					main_site_x = math.sin(-delta_az * D2R) * converted_dist;
					main_site_y = -math.cos(-delta_az * D2R) * converted_dist;
					p_dps_bearing.alt_one_site.setTranslation(320 + main_site_x, 350 + main_site_y);

					#print("distance to alt one in Nm is :", pos.distance_to(SpaceShuttle.landing_alt_site_one) * M2NM);

					}
				else 
					{
					p_dps_bearing.delta_az_alt_one.setText("");
					p_dps_bearing.alt_one_site.setVisible(0);
					}

				if (SpaceShuttle.alt_sites.two_index != 0)
					{
					
					p_dps_bearing.alternate_two_landing_site_number.updateText(sprintf("%2d",SpaceShuttle.alt_sites.two_index));
					p_dps_bearing.alternate_two_landing_site_iata.updateText(SpaceShuttle.alt_sites.two_iata);
					p_dps_bearing.site_alt_two_iata.updateText(SpaceShuttle.alt_sites.two_shortname);

					#Delta Az
					delta_az = SpaceShuttle.delta_azimuth_entry(SpaceShuttle.landing_alt_site_two);
					p_dps_bearing.delta_az_alt_two.setText(sprintf("%4d",delta_az));

					#Site shortname
					p_dps_bearing.alt_two_site.setVisible(1);
					p_dps_bearing.alt_two_site_label.updateText(SpaceShuttle.alt_sites.two_shortname);

					#Azimuth and distance (same factor than for the main site)
					converted_dist = scale * pos.distance_to(SpaceShuttle.landing_alt_site_two) * M2NM;
					main_site_x = math.sin(-delta_az * D2R) * converted_dist;
					main_site_y = -math.cos(-delta_az * D2R) * converted_dist;
					p_dps_bearing.alt_two_site.setTranslation(320 + main_site_x, 350 + main_site_y);

					}
				else 
					{
					p_dps_bearing.delta_az_alt_two.setText("");
					p_dps_bearing.alt_two_site.setVisible(0);
					}

				}

			else
				{
				p_dps_bearing.alternate_one_landing_site_number.updateText("");
				p_dps_bearing.alternate_two_landing_site_number.updateText("");
				p_dps_bearing.alternate_one_landing_site_iata.updateText("");
				p_dps_bearing.alternate_two_landing_site_iata.updateText("");
				p_dps_bearing.site_alt_one_iata.updateText("");
				p_dps_bearing.site_alt_two_iata.updateText("");
				p_dps_bearing.delta_az_alt_one.setText("");
				p_dps_bearing.delta_az_alt_two.setText("");
				p_dps_bearing.alt_one_site.setVisible(0);
				p_dps_bearing.alt_two_site.setVisible(0);
				}


			}

		}

		#Contigency abort logic
		else if (((major_mode == 602) or (major_mode == 603)) and (GRTLS.CONT == "ON"))
			{
			
			if (GRTLS.INIT_PASS == 1)
				{
				#Main site
				p_dps_bearing.main_landing_site_number.updateText(sprintf("%2d",SpaceShuttle.landing_site.index));
				p_dps_bearing.main_landing_site_iata.updateText(SpaceShuttle.landing_site.rwy_pri);

				#Elements visible once GRTLS is activated
				p_dps_bearing.distance_ring.setVisible(1);
				p_dps_bearing.main_azimuth_line.setVisible(1);
				p_dps_bearing.iip.setVisible(1);
				p_dps_bearing.EW_group.setVisible(1);

				#Main site items displayed with guidance only
				p_dps_bearing.site_main_iata.updateText(left(SpaceShuttle.landing_site.rwy_pri, 3));
				p_dps_bearing.main_site_label.updateText(left(SpaceShuttle.landing_site.rwy_pri, 3));
				p_dps_bearing.delta_az_main.setText(sprintf("%4d",-TAEM_guidance_GTP.DPSAC));

				#Shuttle box bank
				var shuttle_bank = getprop("/fdm/jsbsim/systems/navigation/state-vector/roll-deg");
				shuttle_bank = SpaceShuttle.MIDVAL(-90, 90, shuttle_bank);
				p_dps_bearing.shuttle_box.setRotation(shuttle_bank * D2R);

				#Main site and range rings (Nm)
				var distance_factor = 4000;

				if (TAEM_guidance_GTP.RPRED_nm > 2000) 
					{
					distance_factor = 4000;
					p_dps_bearing.int_ring_range.updateText("2000");
					p_dps_bearing.ext_ring_range.updateText("4000");
					}		
				else if ((TAEM_guidance_GTP.RPRED_nm < 2000) and (TAEM_guidance_GTP.RPRED_nm > 1000))
					{
					distance_factor = 2000;
					p_dps_bearing.int_ring_range.updateText("1000");
					p_dps_bearing.ext_ring_range.updateText("2000");
					}
				else if ((TAEM_guidance_GTP.RPRED_nm < 1000) and (TAEM_guidance_GTP.RPRED_nm > 500))
					{
					distance_factor = 1000;
					p_dps_bearing.int_ring_range.updateText("500");
					p_dps_bearing.ext_ring_range.updateText("1000");
					}
				else if ((TAEM_guidance_GTP.RPRED_nm < 500) and (TAEM_guidance_GTP.RPRED_nm > 250))
					{
					distance_factor = 500;
					p_dps_bearing.int_ring_range.updateText("250");
					p_dps_bearing.ext_ring_range.updateText("500");
					}
				else if (TAEM_guidance_GTP.RPRED_nm < 250)
					{
					distance_factor = 250;
					p_dps_bearing.int_ring_range.updateText("125");
					p_dps_bearing.ext_ring_range.updateText("250");
					}

				var scale = 190 / distance_factor;
				var converted_dist = scale * TAEM_guidance_GTP.RPRED_nm;
				var main_site_x = math.sin(TAEM_guidance_GTP.DPSAC * D2R) * converted_dist;
				var main_site_y = -math.cos(TAEM_guidance_GTP.DPSAC * D2R) * converted_dist;

				p_dps_bearing.distance_ring.setTranslation(320 + main_site_x, 350 + main_site_y);
				
				#Main site Azimuth Line // lenght variable (update)
				p_dps_bearing.main_azimuth_line.pop_back();
				p_dps_bearing.main_azimuth_line.addSegment(canvas.Path.VG_LINE_TO_REL, [main_site_x, main_site_y]);
				
				#Instantaneous impact point
				var ballistic_range = SpaceShuttle.instantaneous_impact_point();
				ballistic_range = scale * ballistic_range;
				p_dps_bearing.iip.setTranslation(0, -ballistic_range);

				#E/W ECAL Alpha transition Main site logic
				if ((TAEM_guidance_TGINIT.IPHASE == 4) and (GRTLS.ECAL == "ON"))
					{
					
					#Energy Scale 
					var EW_actual = TAEM_guidance_TGCOMP.EOW;
					var EW_nominal = GRTLS.EN;
					var EW_mep = GRTLS.EMEP;
					var EW_sturn = GRTLS.ES;
					var dEW = EW_sturn - EW_mep;

					if (dEW != 0)
						{
						#Actual EW scale
						var frac_EW_ac = (EW_actual - EW_mep) / dEW;
						frac_EW_ac = SpaceShuttle.clamp(frac_EW_ac, -0.5, 1.5);
						p_dps_bearing.actual_energy_main_site.setTranslation(73.15,335 - frac_EW_ac * 112);

						#Nominal EW scale
						var frac_EW_nom = (EW_nominal - EW_mep) / dEW;
						frac_EW_nom = SpaceShuttle.clamp(frac_EW_nom, -0.5, 1.5);
						p_dps_bearing.energy_nom_main.setTranslation(64.15,335 - frac_EW_nom * 112);

						#Nominal EW scale high (+8000ft)
						var frac_EW_nom_upper = ((EW_nominal + 8000) - EW_mep) / dEW;
						frac_EW_nom_upper = SpaceShuttle.clamp(frac_EW_nom_upper, -0.5, 1.5);
						p_dps_bearing.energy_nom_upper_main.setTranslation(66.15,335 - frac_EW_nom_upper * 112);

						#Nominal EW scale low (-4000ft)
						var frac_EW_nom_lower = ((EW_nominal - 4000) - EW_mep) / dEW;
						frac_EW_nom_lower = SpaceShuttle.clamp(frac_EW_nom_lower, -0.5, 1.5);
						p_dps_bearing.energy_nom_lower_main.setTranslation(66.15,335 - frac_EW_nom_lower * 112);
						}

					}


				#Alternate sites 
				#Delta azimuth to alternate
				var delta_az = 0;
					if (SpaceShuttle.alt_sites.one_index != 0)
						{

						#Energy and distance computations
						p_dps_bearing.EW_group_alt_one.setVisible(1);
						alternate_COMP.distance(1);
						
						#Alternate one site name
						p_dps_bearing.alternate_one_landing_site_number.updateText(sprintf("%2d",SpaceShuttle.alt_sites.one_index));
						p_dps_bearing.alternate_one_landing_site_iata.updateText(SpaceShuttle.alt_sites.one_iata);
						p_dps_bearing.site_alt_one_iata.updateText(SpaceShuttle.alt_sites.one_shortname);

						#Delta Az
						delta_az = SpaceShuttle.delta_azimuth_entry(SpaceShuttle.landing_alt_site_one);
						p_dps_bearing.delta_az_alt_one.setText(sprintf("%4d",-delta_az));

						#Site shortname
						p_dps_bearing.alt_one_site.setVisible(1);
						p_dps_bearing.alt_one_site_label.updateText(SpaceShuttle.alt_sites.one_shortname);

						#Azimuth and distance (same factor than for the main site)
						converted_dist = scale * alternate_COMP.DRPRED_alternate_site[0] / 6076.12;
						main_site_x = math.sin(delta_az * D2R) * converted_dist;
						main_site_y = -math.cos(delta_az * D2R) * converted_dist;
						p_dps_bearing.alt_one_site.setTranslation(320 + main_site_x, 350 + main_site_y);
						

						if ((TAEM_guidance_TGINIT.IPHASE == 4) and (GRTLS.ECAL == "ON"))
							{
							
							#Energy Scale 
							alternate_COMP.energy(alternate_COMP.DRPRED_alternate_site[0], 1);
							var EW_actual = TAEM_guidance_TGCOMP.EOW;
							var EW_nominal = alternate_COMP.EN_alternate[0];
							var EW_mep = alternate_COMP.EMEP_alternate[0];
							var EW_sturn = alternate_COMP.ES_alternate[0];
							var dEW = EW_sturn - EW_mep;

							if (dEW != 0)
								{
								#Actual EW scale
								var frac_EW_ac = (EW_actual - EW_mep) / dEW;
								frac_EW_ac = SpaceShuttle.clamp(frac_EW_ac, -0.5, 1.5);
								p_dps_bearing.actual_energy_alt_one_site.setTranslation(111.15,335 - frac_EW_ac * 112);
								
								#Nominal EW scale
								var frac_EW_nom = (EW_nominal - EW_mep) / dEW;
								frac_EW_nom = SpaceShuttle.clamp(frac_EW_nom, -0.5, 1.5);
								p_dps_bearing.energy_nom_alt_one.setTranslation(102.15,335 - frac_EW_nom * 112);

								#Nominal EW scale high (+8000ft)
								var frac_EW_nom_upper = ((EW_nominal + 8000) - EW_mep) / dEW;
								frac_EW_nom_upper = SpaceShuttle.clamp(frac_EW_nom_upper, -0.5, 1.5);
								p_dps_bearing.energy_nom_upper_alt_one.setTranslation(104.15,335 - frac_EW_nom_upper * 112);

								#Nominal EW scale low (-4000ft)
								var frac_EW_nom_lower = ((EW_nominal - 4000) - EW_mep) / dEW;
								frac_EW_nom_lower = SpaceShuttle.clamp(frac_EW_nom_lower, -0.5, 1.5);
								p_dps_bearing.energy_nom_lower_alt_one.setTranslation(104.15,335 - frac_EW_nom_lower * 112);
								}
							}

						}
					else 
						{
						p_dps_bearing.delta_az_alt_one.setText("");
						p_dps_bearing.alt_one_site.setVisible(0);
						p_dps_bearing.EW_group_alt_one.setVisible(0);
						}

					if (SpaceShuttle.alt_sites.two_index != 0)
						{
					
						#Energy and distance computations
						p_dps_bearing.EW_group_alt_two.setVisible(1);
						alternate_COMP.distance(2);
						
						#Alternate one site name
						p_dps_bearing.alternate_two_landing_site_number.updateText(sprintf("%2d",SpaceShuttle.alt_sites.two_index));
						p_dps_bearing.alternate_two_landing_site_iata.updateText(SpaceShuttle.alt_sites.two_iata);
						p_dps_bearing.site_alt_two_iata.updateText(SpaceShuttle.alt_sites.two_shortname);

						#Delta Az
						delta_az = SpaceShuttle.delta_azimuth_entry(SpaceShuttle.landing_alt_site_two);
						p_dps_bearing.delta_az_alt_two.setText(sprintf("%4d",-delta_az));

						#Site shortname
						p_dps_bearing.alt_two_site.setVisible(1);
						p_dps_bearing.alt_two_site_label.updateText(SpaceShuttle.alt_sites.two_shortname);

						#Azimuth and distance (same factor than for the main site)
						converted_dist = scale * alternate_COMP.DRPRED_alternate_site[1] / 6076.12;
						main_site_x = math.sin(delta_az * D2R) * converted_dist;
						main_site_y = -math.cos(delta_az * D2R) * converted_dist;
						p_dps_bearing.alt_two_site.setTranslation(320 + main_site_x, 350 + main_site_y);

						if ((TAEM_guidance_TGINIT.IPHASE == 4) and (GRTLS.ECAL == "ON"))
							{

							#Energy Scale 
							alternate_COMP.energy(alternate_COMP.DRPRED_alternate_site[1], 2);
							var EW_actual = TAEM_guidance_TGCOMP.EOW;
							var EW_nominal = alternate_COMP.EN_alternate[1];
							var EW_mep = alternate_COMP.EMEP_alternate[1];
							var EW_sturn = alternate_COMP.ES_alternate[1];
							var dEW = EW_sturn - EW_mep;

							if (dEW != 0)
								{
								#Actual EW scale
								var frac_EW_ac = (EW_actual - EW_mep) / dEW;
								frac_EW_ac = SpaceShuttle.clamp(frac_EW_ac, -0.5, 1.5);
								p_dps_bearing.actual_energy_alt_two_site.setTranslation(150.15,335 - frac_EW_ac * 112);
								
								#Nominal EW scale
								var frac_EW_nom = (EW_nominal - EW_mep) / dEW;
								frac_EW_nom = SpaceShuttle.clamp(frac_EW_nom, -0.5, 1.5);
								p_dps_bearing.energy_nom_alt_two.setTranslation(141.15,335 - frac_EW_nom * 112);

								#Nominal EW scale high (+8000ft)
								var frac_EW_nom_upper = ((EW_nominal + 8000) - EW_mep) / dEW;
								frac_EW_nom_upper = SpaceShuttle.clamp(frac_EW_nom_upper, -0.5, 1.5);
								p_dps_bearing.energy_nom_upper_alt_two.setTranslation(143.15,335 - frac_EW_nom_upper * 112);

								#Nominal EW scale low (-4000ft)
								var frac_EW_nom_lower = ((EW_nominal - 4000) - EW_mep) / dEW;
								frac_EW_nom_lower = SpaceShuttle.clamp(frac_EW_nom_lower, -0.5, 1.5);
								p_dps_bearing.energy_nom_lower_alt_two.setTranslation(143.15,335 - frac_EW_nom_lower * 112);
								}
							}
						}
					else 
						{
						p_dps_bearing.delta_az_alt_two.setText("");
						p_dps_bearing.alt_two_site.setVisible(0);
						p_dps_bearing.EW_group_alt_two.setVisible(0);
						}

				}
			}
		

    	device.update_common_DPS();
    }
    
    
    
	return p_dps_bearing;
}
