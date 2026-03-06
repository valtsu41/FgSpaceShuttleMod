##########################################
# CCTV overlay for the SRMS views
##########################################

var cctv_overlay = {

        update_loop_flag: 0,
        rmspowered: 0,
        color: [0.22, 0.86, 0.48, 0.5],
        vscale: 0.9,
	    
	init: func (path=nil) {

	    me.rmspowered = getprop("/fdm/jsbsim/systems/rms/rms-powered");

	    if (me.rmspowered == 0)
	    {
		return;
	    }
	    
#	    print("creating CCTV desktop canvas..");

	    me.desktop= canvas.getDesktop();
	    me.rootgroup = me.desktop.createChild("group");

	    if (path != nil)
	    {
		me.parsesvg(path);
	    }
	    else
	    {
		me.draw_overlay();
	    }
	    
	    
	    me.visible = 0;
	    me.rootgroup.hide();
	    
	    me.screen_xsize = 0.0;
	    me.screen_ysize = 0.0;

	    me.start_update();	    
	}, 

	draw_overlay: func {

	    var linewidth = 2;
	    me.overlay_xsize = 1300;
	    me.overlay_ysize = 1000;
	    
	    canvas.draw.rectangle(me.rootgroup,1300,1000,0,0,rounded=74)
		.setColor(me.color).setStrokeLineWidth(linewidth);
	    canvas.draw.circle(me.rootgroup,22,650,500)
		.setColor(me.color[0],me.color[1],me.color[2],me.color[3])
		.setStrokeLineWidth(linewidth);

	    me.rootgroup.createChild("path","crosshair")
		.moveTo(650,394).lineTo(650,394+74)
	    	.moveTo(650,532).lineTo(650,532+74)
		.moveTo(618,500).lineTo(618-74,500)
		.moveTo(682,500).lineTo(682+74,500)
		.setColor(me.color[0],me.color[1],me.color[2],me.color[3])
		.setStrokeLineWidth(linewidth)
		.setStrokeLineCap("round");

	    me.rootgroup.createChild("path","crossin")
		.moveTo(650,0).lineTo(650,74)
		.moveTo(650,1000).lineTo(650,1000-74)
		.moveTo(0,500).lineTo(74,500)
		.moveTo(1300-74,500).lineTo(1300,500)
		.setColor(me.color[0],me.color[1],me.color[2],me.color[3])
		.setStrokeLineWidth(linewidth)
		.setStrokeLineCap("round");

	    me.rootgroup.createChild("path","vlonglines")
		.moveTo(222,0).lineTo(222,1000)
		.moveTo(1300-222,0).lineTo(1300-222,1000)
		.setColor(me.color[0],me.color[1],me.color[2],me.color[3])
		.setStrokeLineWidth(linewidth)
		.setStrokeLineCap("round");

	    me.rootgroup.createChild("path","vshortlines")
		.moveTo(268,500-66).lineTo(268,500+66)
		.moveTo(1300-268,500-66).lineTo(1300-268,500+66)
		.setColor(me.color[0],me.color[1],me.color[2],me.color[3])
		.setStrokeLineWidth(0.5*linewidth)
		.setStrokeLineCap("round");

	    me.rootgroup.createChild("path","vthicklines")
		.moveTo(160,500-92).lineTo(160,500+92)
		.moveTo(1300-160,500-92).lineTo(1300-160,500+92)
		.setColor(me.color[0],me.color[1],me.color[2],me.color[3])
		.setStrokeLineWidth(2*linewidth)
		.setStrokeLineCap("round");
	},
	    
	parsesvg: func (path) {

	    canvas.parsesvg(me.rootgroup,path);
	    me.overlay_xsize = me.rootgroup.getSize()[0];
	    me.overlay_ysize = me.rootgroup.getSize()[1];

	},
	  
	    
	rescale_and_center: func (scale) {

	    var xsize = props.globals.getNode("/sim/startup/xsize").getValue();
	    var ysize = props.globals.getNode("/sim/startup/ysize").getValue();

#	    print ("x y=",xsize,ysize);
	    
	    if ((me.screen_xsize != xsize) or (me.screen_ysize != ysize))
	    {
		me.screen_xsize = xsize;
		me.screen_ysize = ysize;

		#scale over vertical axis
		var scalingfactor = scale * me.screen_ysize / me.overlay_ysize;
		me.rootgroup.setScale(scalingfactor);
	    
		#center
		me.rootgroup.setTranslation(0.5*(me.screen_xsize - scalingfactor * me.overlay_xsize),
					    0.5*(1.0-scale)*me.screen_ysize);
	    }
	    
	},
	    
	set_brightness: func (bright) {
	    
	    if ((me.rmspowered == 0) or (bright == me.color[3])) {
		return;
	    }
	    
	    me.color[3] = SpaceShuttle.clamp(bright, 0.0, 1.0);
	    
	    me.rootgroup.setColor(me.color[0],me.color[1],me.color[2],me.color[3]);
	    
	},  

	start_update: func {

	    if (me.update_loop_flag == 1)
	    {
		return;
	    }
	    me.update_loop_flag = 1;
	    me.update();

	},


	update: func {

	    if (me.update_loop_flag == 0)
	    {
		return;
	    }

	    me.rmspowered = props.globals.getNode("/fdm/jsbsim/systems/rms/rms-powered").getValue();

	    #print("rms power",me.rmspowered);
	    
	    if (me.rmspowered == 0)
	    {
		me.update_loop_flag = 0;
		me.destroy();
		return;
	    }
	    
	    var viewname = props.globals.getNode("/sim/current-view/name").getValue();

	    #print("view",viewname);
	    
	    if (viewname == 'SRMS')
	    {
		if (me.visible == 0)
		{
		    me.visible = 1;
		    me.rootgroup.show();
		}
		me.rescale_and_center(me.vscale);
		var brightness = props.globals.getNode("/fdm/jsbsim/systems/rms/cctv/overlay-brightness").getValue();
		me.set_brightness(brightness);
	    }
	    else
	    {
		if (me.visible == 1)
		{
		    me.visible = 0;
		    me.rootgroup.hide();
		}
	    }

	    settimer(func {me.update();}, 0.0);
	    
	},


	destroy: func {

	    me.screen_xsize = 0.0;
	    me.screen_ysize = 0.0;
	    me.visible = 0;
	    me.rootgroup.removeAllChildren();
	    me.desktop.del();

	},



};

