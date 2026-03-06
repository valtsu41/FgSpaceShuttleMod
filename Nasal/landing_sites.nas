
# define all Shuttle landing sites in one central location
# Thorsten Renk 2018 / GinGin 2021


#June 2021 Autoland "approved" landing sites: KSC, VBG, EDW (03 for concrete/90 for lakebed runway), ZZA, FMI, IPC
#July 2021 Autoland: KEF, YQX(Gander), PAR (Paro), INN (Innsbruck)
#September 2023 Autoland: HAO (Franch Polynesia), JDG
#December 2024 Autoland ECAL/BDA/TAL: PSM(Pease), NTU(Oceana), MYR(Myrtle Beach), ACY(Atlantic City), ILM(Wilmington), BER(Bermuda), Halifax(YHZ), Wallops(WAL), Cherry Point(NKT), Dover AFB(DOV), Gabreski(FOK)
#December 2024: Ivalo(IVL)									  
#More accurate threshold elevation is computed at the start of TAEM loop // SpaceShuttle.rwy_coord.RTE1 (used in Entry and TAEM guidance)
#Added an I-loaded value for sloped runways (zero by default) // Better Autoland final flare // slope in °
#TRUE Runway Heading / Runway elevation exact computation done in TGEXEC loop (TAEM.nas)

# This file has been modified as part of the Utility mod

var landing_site_data = {

	array: [],

	entry_by_name: func (name) {

		foreach (l; me.array)
			{
			if (l.name == name)
				{
				return l;
				}
			}
		print ("No matching landing site found - using KSC.");
		return me.array[0];

	},

	entry_by_index: func (index) {

		foreach (l; me.array)
			{
			if (l.index == index)
				{
				return l;
				}
			}
		print ("No matching landing site found - using KSC.");
		return me.array[0];
	},



};

var landing_site_entry = {
	new: func (coord, name,  shortname, rwy_pri, rwy_sec, function, index) {
	 	var l = { parents: [landing_site_entry] };
		l.coord = coord;
		l.name = name;
		l.shortname = shortname;
		l.rwy_pri = rwy_pri;
		l.rwy_sec = rwy_sec;
		l.rwy_pri_name = l.shortname~l.rwy_pri;
		l.rwy_sec_name = l.shortname~l.rwy_sec;
		l.tacan = "";
		l.function = function;
		l.index = index;
		l.text_vertical_offset = 0.0;
		l.TAEM_pri_lat = 0.0;
		l.TAEM_pri_lon = 0.0;
		l.TAEM_pri_heading = 0.0;
		l.TAEM_pri_elevation = 0.0;
		l.TAEM_pri_slope = 0.0;
		l.TAEM_pri_MLS_flag = 0;
		l.TAEM_pri_MLS_channel = 0.0;
		l.TAEM_sec_lat = 0.0;
		l.TAEM_sec_lon = 0.0;
		l.TAEM_sec_heading = 0.0;
		l.TAEM_sec_elevation = 0.0;
		l.TAEM_sec_slope = 0.0;
		l.TAEM_sec_MLS_flag = 0;
		l.TAEM_sec_MLS_channel = 0.0;
		l.TAEM_rwy_length = 0.0;
		return l;
		},



};

# Kennedy Space Center

var coord1 = geo.Coord.new();
coord1.set_latlon(28.615, -80.695, 0.0);
var ls_entry1 = landing_site_entry.new(coord1, "Kennedy Space Center", "KSC", "15", "33", "regular", 1);
ls_entry1.tacan = "059";
ls_entry1.TAEM_pri_lat = 28.632714;
ls_entry1.TAEM_pri_lon = -80.706036;
ls_entry1.TAEM_pri_heading = 150.11;
ls_entry1.TAEM_pri_elevation = 30.0;
ls_entry1.TAEM_pri_MLS_flag = 1;
ls_entry1.TAEM_pri_MLS_channel = 8;
ls_entry1.TAEM_sec_lat = 28.597067;
ls_entry1.TAEM_sec_lon = -80.682706;
ls_entry1.TAEM_sec_heading = 330.13;
ls_entry1.TAEM_sec_elevation = 30.0;
ls_entry1.TAEM_sec_MLS_flag = 1;
ls_entry1.TAEM_sec_MLS_channel = 6;
ls_entry1.TAEM_rwy_length = 4572.0;
append(landing_site_data.array, ls_entry1);

# Vandenberg Air Force Base

var coord2 = geo.Coord.new();
coord2.set_latlon(34.722, -120.567, 0.0);
var ls_entry2 = landing_site_entry.new(coord2, "Vandenberg Air Force Base", "VBG", "12", "30", "regular", 2);
ls_entry2.text_vertical_offset = 6.0;
ls_entry2.tacan = "059";
ls_entry2.TAEM_pri_lat = 34.75225874;
ls_entry2.TAEM_pri_lon = -120.6014361;
ls_entry2.TAEM_pri_heading = 136.64;
ls_entry2.TAEM_pri_elevation = 268.0;
ls_entry2.TAEM_pri_slope = 0.52;
ls_entry2.TAEM_pri_MLS_flag = 1;
ls_entry2.TAEM_pri_MLS_channel = 8;
ls_entry2.TAEM_sec_lat = 34.722397;
ls_entry2.TAEM_sec_lon = -120.567150;
ls_entry2.TAEM_sec_heading = 316.68;
ls_entry2.TAEM_sec_elevation = 240.0;
ls_entry2.TAEM_sec_slope = -0.52;
ls_entry2.TAEM_sec_MLS_flag = 1;
ls_entry2.TAEM_sec_MLS_channel = 6;
ls_entry2.TAEM_rwy_length = 4572.0;
append(landing_site_data.array, ls_entry2);


# Edwards Air Force Base (Concrete Runway)

var coord3 = geo.Coord.new();
coord3.set_latlon(34.949259, -117.866050, 0.0);
var ls_entry3 = landing_site_entry.new(coord3, "Edwards Air Force Base", "EDW", "04", "22", "regular", 3);
ls_entry3.text_vertical_offset = -6.0;
ls_entry3.tacan = "111";
ls_entry3.TAEM_pri_lat = 34.894592;
ls_entry3.TAEM_pri_lon = -117.904948;
ls_entry3.TAEM_pri_heading = 58.09;
ls_entry3.TAEM_pri_elevation = 2280.0;
ls_entry3.TAEM_pri_MLS_flag = 1;
ls_entry3.TAEM_pri_MLS_channel = 6;
ls_entry3.TAEM_sec_lat = 34.916262;
ls_entry3.TAEM_sec_lon = -117.862471;
ls_entry3.TAEM_sec_heading = 238.14;
ls_entry3.TAEM_sec_elevation = 2280.0;
ls_entry3.TAEM_sec_MLS_flag = 1;
ls_entry3.TAEM_sec_MLS_channel = 8;
ls_entry3.TAEM_rwy_length = 5572.0;
append(landing_site_data.array, ls_entry3);

# White Sands Space Harbor

var coord4 = geo.Coord.new();
coord4.set_latlon(32.936, -106.416, 0.0);
var ls_entry4 = landing_site_entry.new(coord4, "White Sands Space Harbor", "NOR", "14", "32", "regular", 4);
ls_entry4.tacan = "100";
ls_entry4.TAEM_pri_lat = 32.9754;
ls_entry4.TAEM_pri_lon = -106.3313;
ls_entry4.TAEM_pri_heading = 142.0;
ls_entry4.TAEM_pri_elevation = 4450.0;
ls_entry4.TAEM_pri_MLS_flag = 1;
ls_entry4.TAEM_pri_MLS_channel = 6;
ls_entry4.TAEM_sec_lat = 32.8815;
ls_entry4.TAEM_sec_lon = -106.2477;
ls_entry4.TAEM_sec_heading = 322.0;
ls_entry4.TAEM_sec_elevation = 4450.0;
ls_entry4.TAEM_sec_MLS_flag = 1;
ls_entry4.TAEM_sec_MLS_channel = 8;
ls_entry4.TAEM_rwy_length = 4572.0;
append(landing_site_data.array, ls_entry4);


# Zaragoza Airport (30R / 12L Military Side)

var coord5 = geo.Coord.new();
coord5.set_latlon(41.666, -1.042, 0.0);
var ls_entry5 = landing_site_entry.new(coord5, "Zaragoza Airport", "ZZA", "12L", "30R", "TAL", 5);
ls_entry5.tacan = "064"; 
ls_entry5.TAEM_pri_lat = 41.669035;
ls_entry5.TAEM_pri_lon = -1.039781;
ls_entry5.TAEM_pri_heading = 120.09;
ls_entry5.TAEM_pri_elevation = 850.0;
ls_entry5.TAEM_pri_MLS_flag = 0;
ls_entry5.TAEM_pri_MLS_channel = -1;
ls_entry5.TAEM_sec_lat = 41.655417;
ls_entry5.TAEM_sec_lon = -1.008368;
ls_entry5.TAEM_sec_heading = 300.16;
ls_entry5.TAEM_sec_elevation = 864.0;
ls_entry5.TAEM_sec_MLS_flag = 1;
ls_entry5.TAEM_sec_MLS_channel = 6;
ls_entry5.TAEM_rwy_length = 3718.0;
append(landing_site_data.array, ls_entry5);

# RAF Fairford

var coord6 = geo.Coord.new();
coord6.set_latlon(51.682, -1.79, 0.0);
var ls_entry6 = landing_site_entry.new(coord6, "RAF Fairford", "FFA", "09", "27", "TAL", 6);
ls_entry6.tacan = "081"; 
ls_entry6.TAEM_pri_lat = 51.6831;
ls_entry6.TAEM_pri_lon = -1.8049;
ls_entry6.TAEM_pri_heading = 87.0;
ls_entry6.TAEM_pri_elevation = 316.0;
ls_entry6.TAEM_pri_MLS_flag = 0;
ls_entry6.TAEM_pri_MLS_channel = -1;
ls_entry6.TAEM_sec_lat = 51.6838;
ls_entry6.TAEM_sec_lon = -1.7723;
ls_entry6.TAEM_sec_heading = 267.0;
ls_entry6.TAEM_sec_elevation = 256.0;
ls_entry6.TAEM_sec_MLS_flag = 0;
ls_entry6.TAEM_sec_MLS_channel = -1;
ls_entry6.TAEM_rwy_length = 3045.0;
append(landing_site_data.array, ls_entry6);

# Banjul International Airport

var coord7 = geo.Coord.new();
coord7.set_latlon(13.337, -16.652, 0.0);
var ls_entry7 = landing_site_entry.new(coord7, "Banjul International Airport", "BYD", "14", "32", "TAL", 7);
ls_entry7.tacan = "076";
ls_entry7.TAEM_pri_lat = 13.3451;
ls_entry7.TAEM_pri_lon = -16.6608;
ls_entry7.TAEM_pri_heading = 131.0;
ls_entry7.TAEM_pri_elevation = 102.0;
ls_entry7.TAEM_pri_MLS_flag = 0;
ls_entry7.TAEM_pri_MLS_channel = -1;
ls_entry7.TAEM_sec_lat = 13.3301;
ls_entry7.TAEM_sec_lon = -16.6428;
ls_entry7.TAEM_sec_heading = 311.0;
ls_entry7.TAEM_sec_elevation = 102.0;
ls_entry7.TAEM_sec_MLS_flag = 0;
ls_entry7.TAEM_sec_MLS_channel = -1;
ls_entry7.TAEM_rwy_length = 3600.0;
append(landing_site_data.array, ls_entry7);

var coord8 = geo.Coord.new();
coord8.set_latlon(37.178, -5.614, 0.0);
var ls_entry8 = landing_site_entry.new(coord8, "Moron Air Base", "MRN", "02", "20", "TAL", 8);
ls_entry8.tacan = "100";
ls_entry8.TAEM_pri_lat = 37.1633;
ls_entry8.TAEM_pri_lon = -5.6212;
ls_entry8.TAEM_pri_heading = 20.0;
ls_entry8.TAEM_pri_elevation = 300.0;
ls_entry8.TAEM_pri_MLS_flag = 0;
ls_entry8.TAEM_pri_MLS_channel = -1;
ls_entry8.TAEM_sec_lat = 37.1863;
ls_entry8.TAEM_sec_lon = -5.6106;
ls_entry8.TAEM_sec_heading = 200.0;
ls_entry8.TAEM_sec_elevation = 280.0;
ls_entry8.TAEM_sec_MLS_flag = 1;
ls_entry8.TAEM_sec_MLS_channel = 6;
ls_entry8.TAEM_rwy_length = 3597.0;
append(landing_site_data.array, ls_entry8);

# Istres Le Tube

var coord9 = geo.Coord.new();
coord9.set_latlon(43.52, 4.92, 0.0);
var ls_entry9 = landing_site_entry.new(coord9, "Le Tube", "FMI", "15", "33", "TAL", 9);
ls_entry9.text_vertical_offset = -6.0;
ls_entry9.tacan = "104";
ls_entry9.TAEM_pri_lat = 43.537731;
ls_entry9.TAEM_pri_lon = 4.913329;
ls_entry9.TAEM_pri_heading = 152.70;
ls_entry9.TAEM_pri_elevation = 90.0;
ls_entry9.TAEM_pri_MLS_flag = 0;
ls_entry9.TAEM_pri_MLS_channel = -1;
ls_entry9.TAEM_sec_lat = 43.507829;
ls_entry9.TAEM_sec_lon = 4.934598;
ls_entry9.TAEM_sec_heading = 332.73;
ls_entry9.TAEM_sec_elevation = 90.0;
ls_entry9.TAEM_sec_MLS_flag = 1;
ls_entry9.TAEM_sec_MLS_channel = 6;
ls_entry9.TAEM_rwy_length = 5000.0;
append(landing_site_data.array, ls_entry9);

# Madrid Barajas (WIP)

var coord10 = geo.Coord.new();
coord10.set_latlon(40.52284483, -3.574798331, 0.0);
var ls_entry10 = landing_site_entry.new(coord10, "Madrid", "MAD", "18R", "32L", "TAL", 10);
#ls_entry10.text_vertical_offset = -6.0;
ls_entry10.tacan = "064";
ls_entry10.TAEM_pri_lat = 40.52284483;
ls_entry10.TAEM_pri_lon = -3.574798331;
ls_entry10.TAEM_pri_heading = 181.2172304;
ls_entry10.TAEM_pri_elevation = 1964.0;
ls_entry10.TAEM_pri_MLS_flag = 1;
ls_entry10.TAEM_pri_MLS_channel = 8;
ls_entry10.TAEM_sec_lat = 40.46311852;
ls_entry10.TAEM_sec_lon = -3.553928362;
ls_entry10.TAEM_sec_heading = 323.2857053;
ls_entry10.TAEM_sec_elevation = 1926.0;
ls_entry10.TAEM_sec_MLS_flag = 1;
ls_entry10.TAEM_sec_MLS_channel = 6;
ls_entry10.TAEM_rwy_length = 4000.0;
append(landing_site_data.array, ls_entry10);

# Bermuda

var coord11 = geo.Coord.new();
coord11.set_latlon(32.363, -64.67, 0.0);
var ls_entry11 = landing_site_entry.new(coord11, "Bermuda", "BER", "12", "30", "ECAL", 11);
ls_entry11.tacan = "086";
ls_entry11.TAEM_pri_lat = 32.36668185157089;
ls_entry11.TAEM_pri_lon = -64.6940520491342;
ls_entry11.TAEM_pri_heading = 101.4835178011938;
ls_entry11.TAEM_pri_elevation = 25.0;
ls_entry11.TAEM_pri_MLS_flag = 0;
ls_entry11.TAEM_pri_MLS_channel = -1;
ls_entry11.TAEM_sec_lat = 32.36139514388834;
ls_entry11.TAEM_sec_lon = -64.6633149488380;
ls_entry11.TAEM_sec_heading = 281.5233422958061;
ls_entry11.TAEM_sec_elevation = 25.0;
ls_entry11.TAEM_sec_MLS_flag = 0;
ls_entry11.TAEM_sec_MLS_channel = -1;
ls_entry11.TAEM_rwy_length = 2947.0;
append(landing_site_data.array, ls_entry11);

# Halifax

var coord12 = geo.Coord.new();
coord12.set_latlon(44.875, -63.51, 0.0);
var ls_entry12 = landing_site_entry.new(coord12, "Halifax", "YHZ", "05", "23", "ECAL", 12);
ls_entry12.tacan = "110";
ls_entry12.TAEM_pri_lat = 44.86904809573748;
ls_entry12.TAEM_pri_lon = -63.5245699600776;
ls_entry12.TAEM_pri_heading = 34.95624941471183;
ls_entry12.TAEM_pri_elevation = 460.0;
ls_entry12.TAEM_pri_MLS_flag = 0;
ls_entry12.TAEM_pri_MLS_channel = -1;
ls_entry12.TAEM_sec_lat = 44.88830894199171;
ls_entry12.TAEM_sec_lon = -63.50556593455561;
ls_entry12.TAEM_sec_heading = 214.965814010649;
ls_entry12.TAEM_sec_elevation = 460.0;
ls_entry12.TAEM_sec_MLS_flag = 0;
ls_entry12.TAEM_sec_MLS_channel = -1;
ls_entry12.TAEM_rwy_length = 3200.0;
append(landing_site_data.array, ls_entry12);


# Wilmington

var coord13 = geo.Coord.new();
coord13.set_latlon(34.272, -77.896, 0.0);
var ls_entry13 = landing_site_entry.new(coord13, "Wilmington", "ILM", "06", "24", "ECAL", 13);
ls_entry13.tacan = "117";
ls_entry13.TAEM_pri_lat = 34.26191518319436;
ls_entry13.TAEM_pri_lon = -77.9102680450756;
ls_entry13.TAEM_pri_heading = 48.20485177890522;
ls_entry13.TAEM_pri_elevation = 25.0;
ls_entry13.TAEM_pri_MLS_flag = 0;
ls_entry13.TAEM_pri_MLS_channel = -1;
ls_entry13.TAEM_sec_lat = 34.27649281547363;
ls_entry13.TAEM_sec_lon = -77.8905209671320;
ls_entry13.TAEM_sec_heading = 228.2342761326738;
ls_entry13.TAEM_sec_elevation = 25.0;
ls_entry13.TAEM_sec_MLS_flag = 0;
ls_entry13.TAEM_sec_MLS_channel = -1;
ls_entry13.TAEM_rwy_length = 2440.0;
append(landing_site_data.array, ls_entry13);

# Atlantic City

var coord14 = geo.Coord.new();
coord14.set_latlon(39.454, -74.568, 0.0);
var ls_entry14 = landing_site_entry.new(coord14, "Atlantic City", "ACY", "13", "31", "ECAL", 14);
ls_entry14.tacan = "081";
ls_entry14.TAEM_pri_lat = 39.46423991643224;
ls_entry14.TAEM_pri_lon = -74.5909639582762;
ls_entry14.TAEM_pri_heading = 118.0722681713015;
ls_entry14.TAEM_pri_elevation = 60.0;
ls_entry14.TAEM_pri_MLS_flag = 0;
ls_entry14.TAEM_pri_MLS_channel = -1;
ls_entry14.TAEM_sec_lat = 39.45139309700305;
ls_entry14.TAEM_sec_lon = -74.5597970164412;
ls_entry14.TAEM_sec_heading = 298.1181277727185;
ls_entry14.TAEM_sec_elevation = 60.0;
ls_entry14.TAEM_sec_MLS_flag = 0;
ls_entry14.TAEM_sec_MLS_channel = -1;
ls_entry14.TAEM_rwy_length = 3048.0;
append(landing_site_data.array, ls_entry14);

# Myrtle Beach

var coord15 = geo.Coord.new();
coord15.set_latlon(33.675, -78.926, 0.0);
var ls_entry15 = landing_site_entry.new(coord15, "Myrtle Beach", "MYR", "18", "36", "ECAL", 15);
ls_entry15.tacan = "117";
ls_entry15.TAEM_pri_lat = 33.69247867817739;
ls_entry15.TAEM_pri_lon = -78.93150363281559;
ls_entry15.TAEM_pri_heading = 168.2475159895132;
ls_entry15.TAEM_pri_elevation = 30.0;
ls_entry15.TAEM_pri_MLS_flag = 0;
ls_entry15.TAEM_pri_MLS_channel = -1;
ls_entry15.TAEM_sec_lat = 33.66700133113504;
ls_entry15.TAEM_sec_lon = -78.9251383553846;
ls_entry15.TAEM_sec_heading = 348.2586609181608;
ls_entry15.TAEM_sec_elevation = 30.0;
ls_entry15.TAEM_sec_MLS_flag = 0;
ls_entry15.TAEM_sec_MLS_channel = -1;
ls_entry15.TAEM_rwy_length = 2897.0;
append(landing_site_data.array, ls_entry15);

# Gander

var coord16 = geo.Coord.new();
coord16.set_latlon(48.947, -54.560, 0.0);
var ls_entry16 = landing_site_entry.new(coord16, "Gander", "YQX", "03", "21", "ECAL", 16);
ls_entry16.tacan = "074";
ls_entry16.TAEM_pri_lat = 48.92087669;
ls_entry16.TAEM_pri_lon = -54.56831971;
ls_entry16.TAEM_pri_heading = 10.9249525992067;
ls_entry16.TAEM_pri_elevation = 420.0;
ls_entry16.TAEM_pri_MLS_flag = 0;
ls_entry16.TAEM_pri_MLS_channel = -1;
ls_entry16.TAEM_sec_lat = 48.94831428;
ls_entry16.TAEM_sec_lon = -54.56025227;
ls_entry16.TAEM_sec_heading = 190.930;
ls_entry16.TAEM_sec_elevation = 450.0;
ls_entry16.TAEM_sec_MLS_flag = 0;
ls_entry16.TAEM_sec_MLS_channel = -1;
ls_entry16.TAEM_rwy_length = 3109.0;
append(landing_site_data.array, ls_entry16);

# Pease

var coord17 = geo.Coord.new();
coord17.set_latlon(43.0742, -70.820, 0.0);
var ls_entry17 = landing_site_entry.new(coord17, "Pease", "PSM", "16", "34", "ECAL", 17);
ls_entry17.tacan = "118";
ls_entry17.TAEM_pri_lat = 43.09128184058126;
ls_entry17.TAEM_pri_lon = -70.8340700183717;
ls_entry17.TAEM_pri_heading = 149.2700669090505;
ls_entry17.TAEM_pri_elevation = 100.0;
ls_entry17.TAEM_pri_MLS_flag = 0;
ls_entry17.TAEM_pri_MLS_channel = -1;
ls_entry17.TAEM_sec_lat = 43.06466016893578;
ls_entry17.TAEM_sec_lon = -70.81241772125407;
ls_entry17.TAEM_sec_heading = 329.302303732998;
ls_entry17.TAEM_sec_elevation = 70.0;
ls_entry17.TAEM_sec_MLS_flag = 0;
ls_entry17.TAEM_sec_MLS_channel = -1;
ls_entry17.TAEM_rwy_length = 3451.0;
append(landing_site_data.array, ls_entry17);

# Oceana Naval Air Station

var coord18 = geo.Coord.new();
coord18.set_latlon(36.81, -76.012, 0.0);
var ls_entry18 = landing_site_entry.new(coord18, "Oceana NAS", "NTU", "05R", "23L", "ECAL", 18);
ls_entry18.tacan = "086";
ls_entry18.TAEM_pri_lat = 36.80548311976737;
ls_entry18.TAEM_pri_lon = -76.0499071614114;
ls_entry18.TAEM_pri_heading = 41.99755434138674;
ls_entry18.TAEM_pri_elevation = 23.0;
ls_entry18.TAEM_pri_MLS_flag = 0;
ls_entry18.TAEM_pri_MLS_channel = -1;
ls_entry18.TAEM_sec_lat = 36.82985286602816;
ls_entry18.TAEM_sec_lon = -76.0224888098813;
ls_entry18.TAEM_sec_heading = 222.0129135752685;
ls_entry18.TAEM_sec_elevation = 23.0;
ls_entry18.TAEM_sec_MLS_flag = 0;
ls_entry18.TAEM_sec_MLS_channel = -1;
ls_entry18.TAEM_rwy_length = 3651.0;
append(landing_site_data.array, ls_entry18);

# Cherry Point

var coord19 = geo.Coord.new();
coord19.set_latlon(34.88, -76.862, 0.0);
var ls_entry19 = landing_site_entry.new(coord19, "Cherry Point", "NKT", "32L", "14R", "ECAL", 19);
ls_entry19.tacan = "083";
ls_entry19.TAEM_pri_lat = 34.88661281778081;
ls_entry19.TAEM_pri_lon = -76.8628931979563;
ls_entry19.TAEM_pri_heading = 315.643112108429;
ls_entry19.TAEM_pri_elevation = 23.0;
ls_entry19.TAEM_pri_MLS_flag = 0;
ls_entry19.TAEM_pri_MLS_channel = -1;
ls_entry19.TAEM_sec_lat = 34.90300718369521;
ls_entry19.TAEM_sec_lon = -76.8824458038010;
ls_entry19.TAEM_sec_heading = 135.617281155124;
ls_entry19.TAEM_sec_elevation = 23.0;
ls_entry19.TAEM_sec_MLS_flag = 0;
ls_entry19.TAEM_sec_MLS_channel = -1;
ls_entry19.TAEM_rwy_length = 2100.0;
append(landing_site_data.array, ls_entry19);

# Wallops

var coord20 = geo.Coord.new();
coord20.set_latlon(37.94, -75.485, 0.0);
var ls_entry20 = landing_site_entry.new(coord20, "Wallops", "WAL", "28", "10", "ECAL", 20);
ls_entry20.tacan = "049";
ls_entry20.TAEM_pri_lat = 37.94279760084654;
ls_entry20.TAEM_pri_lon = -75.4573727494132;
ls_entry20.TAEM_pri_heading = 270.3669752574108;
ls_entry20.TAEM_pri_elevation = 23.0;
ls_entry20.TAEM_pri_MLS_flag = 0;
ls_entry20.TAEM_pri_MLS_channel = -1;
ls_entry20.TAEM_sec_lat = 37.94293537126607;
ls_entry20.TAEM_sec_lon = -75.4850192609505;
ls_entry20.TAEM_sec_heading = 90.34601000790989;
ls_entry20.TAEM_sec_elevation = 23.0;
ls_entry20.TAEM_sec_MLS_flag = 0;
ls_entry20.TAEM_sec_MLS_channel = -1;
ls_entry20.TAEM_rwy_length = 2100.0;
append(landing_site_data.array, ls_entry20);


# Dover AFB

var coord21 = geo.Coord.new();
coord21.set_latlon(39.13, -75.449, 0.0);
var ls_entry21 = landing_site_entry.new(coord21, "Dover", "DOV", "14", "32", "ECAL", 21);
ls_entry21.tacan = "049";
ls_entry21.TAEM_pri_lat = 39.13927346674114;
ls_entry21.TAEM_pri_lon = -75.4860685337123;
ls_entry21.TAEM_pri_heading = 125.9029594227582;
ls_entry21.TAEM_pri_elevation = 23.0;
ls_entry21.TAEM_pri_MLS_flag = 0;
ls_entry21.TAEM_pri_MLS_channel = -1;
ls_entry21.TAEM_sec_lat = 39.11859454505414;
ls_entry21.TAEM_sec_lon = -75.4492814381516;
ls_entry21.TAEM_sec_heading = 305.9475653032566;
ls_entry21.TAEM_sec_elevation = 23.0;
ls_entry21.TAEM_sec_MLS_flag = 0;
ls_entry21.TAEM_sec_MLS_channel = -1;
ls_entry21.TAEM_rwy_length = 2100.0;
append(landing_site_data.array, ls_entry21);


# Gabreski

var coord22 = geo.Coord.new();
coord22.set_latlon(40.85, -72.617, 0.0);
var ls_entry22 = landing_site_entry.new(coord22, "Gabreski", "FOK", "06", "24", "ECAL", 22);
ls_entry22.tacan = "083";
ls_entry22.TAEM_pri_lat = 40.8362889646892;
ls_entry22.TAEM_pri_lon = -72.6389684661206;
ls_entry22.TAEM_pri_heading = 42.37961723473122;
ls_entry22.TAEM_pri_elevation = 23.0;
ls_entry22.TAEM_pri_MLS_flag = 0;
ls_entry22.TAEM_pri_MLS_channel = -1;
ls_entry22.TAEM_sec_lat = 40.85439981176684;
ls_entry22.TAEM_sec_lon = -72.6170933680019;
ls_entry22.TAEM_sec_heading = 222.4003822943389;
ls_entry22.TAEM_sec_elevation = 23.0;
ls_entry22.TAEM_sec_MLS_flag = 0;
ls_entry22.TAEM_sec_MLS_channel = -1;
ls_entry22.TAEM_rwy_length = 2100.0;
append(landing_site_data.array, ls_entry22);


# Cap Cod CGAS (Otis)

var coord23 = geo.Coord.new();
coord23.set_latlon(41.65, -70.511, 0.0);
var ls_entry23 = landing_site_entry.new(coord23, "Cap Cod", "FMH", "32", "14", "ECAL", 23);
ls_entry23.tacan = "103";
ls_entry23.TAEM_pri_lat = 41.65151964556709;
ls_entry23.TAEM_pri_lon = -70.5112497799679;
ls_entry23.TAEM_pri_heading = 307.5419448242335;
ls_entry23.TAEM_pri_elevation = 100.0;
ls_entry23.TAEM_pri_MLS_flag = 0;
ls_entry23.TAEM_pri_MLS_channel = -1;
ls_entry23.TAEM_sec_lat = 41.66730634713067;
ls_entry23.TAEM_sec_lon = -70.5387566099145;
ls_entry23.TAEM_sec_heading = 127.5088548971991;
ls_entry23.TAEM_sec_elevation = 100.0;
ls_entry23.TAEM_sec_MLS_flag = 0;
ls_entry23.TAEM_sec_MLS_channel = -1;
ls_entry23.TAEM_rwy_length = 2100.0;
append(landing_site_data.array, ls_entry23);


# Mataveri Airport


var coord30 = geo.Coord.new();
coord30.set_latlon(-27.165, -109.415, 0.0);
var ls_entry30 = landing_site_entry.new(coord30, "Easter Island", "IPC", "10", "28", "TAL", 30);
ls_entry30.tacan = "118";
ls_entry30.TAEM_pri_lat = -27.15806124;
ls_entry30.TAEM_pri_lon = -109.4366002;
ls_entry30.TAEM_pri_heading = 117.2;
ls_entry30.TAEM_pri_elevation = 163.0;
ls_entry30.TAEM_pri_slope = 0.53;
ls_entry30.TAEM_pri_MLS_flag = 0;
ls_entry30.TAEM_pri_MLS_channel = -1;
ls_entry30.TAEM_sec_lat = -27.171427;
ls_entry30.TAEM_sec_lon = -109.4072639;
ls_entry30.TAEM_sec_heading = 297.2;
ls_entry30.TAEM_sec_elevation = 224.0;
ls_entry30.TAEM_sec_slope = 0.53;
ls_entry30.TAEM_sec_MLS_flag = 0;
ls_entry30.TAEM_sec_MLS_channel = -1;
ls_entry30.TAEM_rwy_length = 3318.0;
append(landing_site_data.array, ls_entry30);


# Diego Garcia

var coord32 = geo.Coord.new();
coord32.set_latlon(-7.313, 72.411, 0.0);
var ls_entry32 = landing_site_entry.new(coord32, "Diego Garcia", "JDG", "13", "31", "emergency", 32);
ls_entry32.tacan = "057";
ls_entry32.TAEM_pri_lat = -7.304595705;
ls_entry32.TAEM_pri_lon = 72.3970345;
ls_entry32.TAEM_pri_heading = 121.894613;
ls_entry32.TAEM_pri_elevation = 9.5887;
ls_entry32.TAEM_pri_MLS_flag = 0;
ls_entry32.TAEM_pri_MLS_channel = -1;
ls_entry32.TAEM_sec_lat = -7.321940312;
ls_entry32.TAEM_sec_lon = 72.42514548;
ls_entry32.TAEM_sec_heading = 301.8699;
ls_entry32.TAEM_sec_elevation = 11.71;
ls_entry32.TAEM_sec_MLS_flag = 0;
ls_entry32.TAEM_sec_MLS_channel = -1;
ls_entry32.TAEM_rwy_length = 3659.0;
append(landing_site_data.array, ls_entry32);


# Honolulu Intl.

var coord33 = geo.Coord.new();
coord33.set_latlon(21.307, -157.929, 0.0);
var ls_entry33 = landing_site_entry.new(coord33, "Honolulu", "HNL", "08", "26", "emergency", 33);
ls_entry33.tacan = "095";
ls_entry33.TAEM_pri_lat = 21.3068015;
ls_entry33.TAEM_pri_lon = -157.942542;
ls_entry33.TAEM_pri_heading = 90.0;
ls_entry33.TAEM_pri_elevation = 0.0;
ls_entry33.TAEM_pri_MLS_flag = 0;
ls_entry33.TAEM_pri_MLS_channel = -1;
ls_entry33.TAEM_sec_lat = 21.3067945;
ls_entry33.TAEM_sec_lon = -157.914083;
ls_entry33.TAEM_sec_heading = 270.0;
ls_entry33.TAEM_sec_elevation = 0.0;
ls_entry33.TAEM_sec_MLS_flag = 0;
ls_entry33.TAEM_sec_MLS_channel = -1;
ls_entry33.TAEM_rwy_length = 3753.0;
append(landing_site_data.array, ls_entry33);

# Keflavik

var coord34 = geo.Coord.new();
coord34.set_latlon(63.985, -22.618, 0.0);
var ls_entry34 = landing_site_entry.new(coord34, "Keflavik", "IKF", "10", "28", "emergency", 34);
ls_entry34.tacan = "057";
ls_entry34.TAEM_pri_lat = 63.98504066;
ls_entry34.TAEM_pri_lon = -22.65489706;
ls_entry34.TAEM_pri_heading = 89.947;
ls_entry34.TAEM_pri_elevation = 110.0;
ls_entry34.TAEM_pri_MLS_flag = 0;
ls_entry34.TAEM_pri_MLS_channel = -1;
ls_entry34.TAEM_sec_lat = 63.98504451;
ls_entry34.TAEM_sec_lon = -22.59248277;
ls_entry34.TAEM_sec_heading = 270.036;
ls_entry34.TAEM_sec_elevation = 175.0;
ls_entry34.TAEM_sec_MLS_flag = 0;
ls_entry34.TAEM_sec_MLS_channel = -1;
ls_entry34.TAEM_rwy_length = 3753.0;
append(landing_site_data.array, ls_entry34);

# Anderson Airforce Base Guam

var coord35 = geo.Coord.new();
coord35.set_latlon(13.584, 144.934, 0.0);
var ls_entry35 = landing_site_entry.new(coord35, "Andersen Air Force Base", "UAM", "06", "24", "emergency", 35);
ls_entry35.tacan = "078";
ls_entry35.TAEM_pri_lat = 13.5765189;
ls_entry35.TAEM_pri_lon = 144.91921413;
ls_entry35.TAEM_pri_heading = 66.0;
ls_entry35.TAEM_pri_elevation = 540.0;
ls_entry35.TAEM_pri_MLS_flag = 0;
ls_entry35.TAEM_pri_MLS_channel = -1;
ls_entry35.TAEM_sec_lat = 13.58715568;
ls_entry35.TAEM_sec_lon = 144.9434987;
ls_entry35.TAEM_sec_heading = 246.0;
ls_entry35.TAEM_sec_elevation = 590.0;
ls_entry35.TAEM_sec_MLS_flag = 0;
ls_entry35.TAEM_sec_MLS_channel = -1;
ls_entry35.TAEM_rwy_length = 3409.0;
append(landing_site_data.array, ls_entry35);

# Amilcar Cabral Capo Verde

var coord36 = geo.Coord.new();
coord36.set_latlon(16.73, -22.94, 0.0);
var ls_entry36 = landing_site_entry.new(coord36, "Amilcar Cabral", "CVS", "01", "19", "emergency", 36);
ls_entry36.tacan = "078";
ls_entry36.TAEM_pri_lat = 16.72694;
ls_entry36.TAEM_pri_lon = -22.94886;
ls_entry36.TAEM_pri_heading = 0.0;
ls_entry36.TAEM_pri_elevation = 185.0;
ls_entry36.TAEM_pri_MLS_flag = 0;
ls_entry36.TAEM_pri_MLS_channel = -1;
ls_entry36.TAEM_sec_lat = 16.745990;
ls_entry36.TAEM_sec_lon = -22.9489702;
ls_entry36.TAEM_sec_heading = 180.0;
ls_entry36.TAEM_sec_elevation = 178.0;
ls_entry36.TAEM_sec_MLS_flag = 0;
ls_entry36.TAEM_sec_MLS_channel = -1;
ls_entry36.TAEM_rwy_length = 3272.0;
append(landing_site_data.array, ls_entry36);

# Ascension Island

var coord37 = geo.Coord.new();
coord37.set_latlon(-7.97, -14.39, 0.0);
var ls_entry37 = landing_site_entry.new(coord37, "Ascension", "HAW", "13", "31", "emergency", 37);
ls_entry37.tacan = "059";
ls_entry37.TAEM_pri_lat = -7.96408717;
ls_entry37.TAEM_pri_lon = -14.4046994;
ls_entry37.TAEM_pri_heading = 117.0;
ls_entry37.TAEM_pri_elevation = 277.0;
ls_entry37.TAEM_pri_MLS_flag = 0;
ls_entry37.TAEM_pri_MLS_channel = -1;
ls_entry37.TAEM_sec_lat = -7.9751031;
ls_entry37.TAEM_sec_lon = -14.382679;
ls_entry37.TAEM_sec_heading = 297.0;
ls_entry37.TAEM_sec_elevation = 243.0;
ls_entry37.TAEM_sec_MLS_flag = 0;
ls_entry37.TAEM_sec_MLS_channel = -1;
ls_entry37.TAEM_rwy_length = 3054.0;
append(landing_site_data.array, ls_entry37);

# Wake Island

var coord38 = geo.Coord.new();
coord38.set_latlon(19.282, 166.63, 0.0);
var ls_entry38 = landing_site_entry.new(coord38, "Wake Island", "WAK", "10", "28", "emergency", 38);
ls_entry38.tacan = "082";
ls_entry38.TAEM_pri_lat = 19.2844870;
ls_entry38.TAEM_pri_lon = 166.6243612;
ls_entry38.TAEM_pri_heading = 102.0;
ls_entry38.TAEM_pri_elevation = 20.0;
ls_entry38.TAEM_pri_MLS_flag = 0;
ls_entry38.TAEM_pri_MLS_channel = -1;
ls_entry38.TAEM_sec_lat = 19.27952525;
ls_entry38.TAEM_sec_lon = 166.649126;
ls_entry38.TAEM_sec_heading = 282.0;
ls_entry38.TAEM_sec_elevation = 35.0;
ls_entry38.TAEM_sec_MLS_flag = 0;
ls_entry38.TAEM_sec_MLS_channel = -1;
ls_entry38.TAEM_rwy_length = 3000.0;
append(landing_site_data.array, ls_entry38);

# Lajes Air Base

var coord39 = geo.Coord.new();
coord39.set_latlon(38.761, -27.09, 0.0);
var ls_entry39 = landing_site_entry.new(coord39, "Lajes Air Base", "LAJ", "15", "33", "emergency", 39);
ls_entry39.tacan = "109";
ls_entry39.TAEM_pri_lat = 38.77373704;
ls_entry39.TAEM_pri_lon = -27.103272055;
ls_entry39.TAEM_pri_heading = 141.0;
ls_entry39.TAEM_pri_elevation = 170.0;
ls_entry39.TAEM_pri_MLS_flag = 0;
ls_entry39.TAEM_pri_MLS_channel = -1;
ls_entry39.TAEM_sec_lat = 38.75501808;
ls_entry39.TAEM_sec_lon = -27.08362772;
ls_entry39.TAEM_sec_heading = 321.0;
ls_entry39.TAEM_sec_elevation = 187.0;
ls_entry39.TAEM_sec_MLS_flag = 0;
ls_entry39.TAEM_sec_MLS_channel = -1;
ls_entry39.TAEM_rwy_length = 3314.0;
append(landing_site_data.array, ls_entry39);

# Hao French Polynesia (Vandenberg TAL site / ELS site)

var coord40 = geo.Coord.new();
coord40.set_latlon(-18.085, -140.934, 0.0);
var ls_entry40 = landing_site_entry.new(coord40, "Hao", "HOI", "12L", "30R", "emergency", 40);
ls_entry40.tacan = "109";
ls_entry40.TAEM_pri_lat = -18.06408753;
ls_entry40.TAEM_pri_lon = -140.9571815;
ls_entry40.TAEM_pri_heading = 134.97;
ls_entry40.TAEM_pri_elevation = 11.74;
ls_entry40.TAEM_pri_MLS_flag = 1;
ls_entry40.TAEM_pri_MLS_channel = 8;
ls_entry40.TAEM_sec_lat = -18.08554046;
ls_entry40.TAEM_sec_lon = -140.9345905;
ls_entry40.TAEM_sec_heading = 314.96;
ls_entry40.TAEM_sec_elevation = 13.24;
ls_entry40.TAEM_sec_MLS_flag = 0;
ls_entry40.TAEM_sec_MLS_channel = -1;
ls_entry40.TAEM_rwy_length = 10000.0;
append(landing_site_data.array, ls_entry40);

var lsFromApt = func(name, icao, pri, sec, index, elev_pri, elev_sec) {
	var apt = airportinfo(icao);
	var coord = geo.Coord.new(); coord.set_latlon(apt.lat, apt.lon, 0.0);
	var ls_entry = landing_site_entry.new(coord, name, substr(icao, 1), pri, sec, "emergency", index);
	ls_entry.TAEM_pri_lat = apt.runways[pri].lat;
	ls_entry.TAEM_pri_lon = apt.runways[pri].lon;
	ls_entry.TAEM_pri_heading = apt.runways[pri].heading;
	ls_entry.TAEM_pri_elevation = elev_pri;
	ls_entry.TAEM_pri_MLS_flag = 0;
	ls_entry.TAEM_pri_MLS_channel = -1;
	ls_entry.TAEM_sec_lat = apt.runways[sec].lat;
	ls_entry.TAEM_sec_lon = apt.runways[sec].lon;
	ls_entry.TAEM_sec_heading = apt.runways[sec].heading;
	ls_entry.TAEM_sec_elevation = elev_sec;
	ls_entry.TAEM_sec_MLS_flag = 0;
	ls_entry.TAEM_sec_MLS_channel = -1;
	ls_entry.TAEM_rwy_length = apt.runways[pri].length;
	return ls_entry
}


# SBFN


var ls_entry87 = lsFromApt("SBFN", "SBFN", "30", "12", 87, 177.49, 187.83);
append(landing_site_data.array, ls_entry87);


# Ajaccio

var ls_entry88 = lsFromApt("Ajaciio", "LFKJ", "02", "20", 88, 15, 17);
append(landing_site_data.array, ls_entry88);


# Edwards Air Force Base Lakebed STS 1 Rwy 23L/05R (11° East declinaison)

var coord90 = geo.Coord.new();
coord90.set_latlon(34.949259, -117.866050, 0.0);
var ls_entry90 = landing_site_entry.new(coord90, "Edwards Air Force Base Lakebed", "EDW", "23", "05", "regular", 90);
#ls_entry90.text_vertical_offset = -6.0;
ls_entry90.tacan = "111";
ls_entry90.TAEM_pri_lat = 34.966288;
ls_entry90.TAEM_pri_lon = -117.817081;
ls_entry90.TAEM_pri_heading = 244.62;
ls_entry90.TAEM_pri_elevation = 2270.0;
ls_entry90.TAEM_pri_MLS_flag = 1;
ls_entry90.TAEM_pri_MLS_channel = 8;
ls_entry90.TAEM_sec_lat = 34.948680;
ls_entry90.TAEM_sec_lon = -117.862327;
ls_entry90.TAEM_sec_heading = 064.57;
ls_entry90.TAEM_sec_elevation = 2270.0;
ls_entry90.TAEM_sec_MLS_flag = 1;
ls_entry90.TAEM_sec_MLS_channel = 6;
ls_entry90.TAEM_rwy_length = 5572.0;
append(landing_site_data.array, ls_entry90);



# Paro Airport

var coord91 = geo.Coord.new();
coord91.set_latlon(27.39551094, 89.42946992, 0.0);
var ls_entry91 = landing_site_entry.new(coord91, "Paro", "PBH", "33", "15", "emergency", 91);
#ls_entry91.text_vertical_offset = -6.0;
ls_entry91.tacan = "057";
ls_entry91.TAEM_pri_lat = 27.39551094;
ls_entry91.TAEM_pri_lon = 89.42946992;
ls_entry91.TAEM_pri_heading = 330.666;
ls_entry91.TAEM_pri_elevation = 7312.0;
ls_entry91.TAEM_pri_slope = 0.31;
ls_entry91.TAEM_pri_MLS_flag = 0;
ls_entry91.TAEM_pri_MLS_channel = -1;
ls_entry91.TAEM_sec_lat = 27.41086933;
ls_entry91.TAEM_sec_lon = 89.41974097;
ls_entry91.TAEM_sec_heading = 150.6458;
ls_entry91.TAEM_sec_elevation = 7345.0;
ls_entry91.TAEM_sec_slope = -0.31;
ls_entry91.TAEM_sec_MLS_flag = 0;
ls_entry91.TAEM_sec_MLS_channel = -1;
ls_entry91.TAEM_rwy_length = 2000.0;
append(landing_site_data.array, ls_entry91);



# Innsbruck Airport (WIP)

var coord92 = geo.Coord.new();
coord92.set_latlon(47.25883434, 11.33097804, 0.0);
var ls_entry92 = landing_site_entry.new(coord92, "Innsbruck", "PBH", "08", "26", "emergency", 92);
#ls_entry92.text_vertical_offset = -6.0;
ls_entry92.tacan = "111";
ls_entry92.TAEM_pri_lat = 47.25883434;
ls_entry92.TAEM_pri_lon = 11.33097804;
ls_entry92.TAEM_pri_heading = 081.008;
ls_entry92.TAEM_pri_elevation = 1917.0;
ls_entry92.TAEM_pri_slope =-0.22; 
ls_entry92.TAEM_pri_MLS_flag = 0;
ls_entry92.TAEM_pri_MLS_channel = -1;
ls_entry92.TAEM_sec_lat = 47.26161726;
ls_entry92.TAEM_sec_lon = 11.35695596;
ls_entry92.TAEM_sec_heading = 261.0484437;
ls_entry92.TAEM_sec_elevation = 1893;
ls_entry92.TAEM_sec_slope = 0.22;
ls_entry92.TAEM_sec_MLS_flag = 0;
ls_entry92.TAEM_sec_MLS_channel = -1;
ls_entry92.TAEM_rwy_length = 2000.0;
append(landing_site_data.array, ls_entry92);

# Ivalo(WIP)

var coord93 = geo.Coord.new();
coord93.set_latlon(68.61, 27.42, 0.0);
var ls_entry93 = landing_site_entry.new(coord93, "Ivalo", "IVL", "04", "22", "emergency", 93);
#ls_entry92.text_vertical_offset = -6.0;
ls_entry93.tacan = "111";
ls_entry93.TAEM_pri_lat = 68.59965774547529;
ls_entry93.TAEM_pri_lon = 27.38297755676018;
ls_entry93.TAEM_pri_heading = 46.93243951979029;
ls_entry93.TAEM_pri_elevation = 469;
ls_entry93.TAEM_pri_slope =0.0; 
ls_entry93.TAEM_pri_MLS_flag = 0;
ls_entry93.TAEM_pri_MLS_channel = -1;
ls_entry93.TAEM_sec_lat = 68.61487629659914;
ls_entry93.TAEM_sec_lon = 27.42769436131244;
ls_entry93.TAEM_sec_heading = 227.0296116459793;
ls_entry93.TAEM_sec_elevation = 469;
ls_entry93.TAEM_sec_slope = 0.0;
ls_entry93.TAEM_sec_MLS_flag = 0;
ls_entry93.TAEM_sec_MLS_channel = -1;
ls_entry93.TAEM_rwy_length = 2000.0;
append(landing_site_data.array, ls_entry93);



