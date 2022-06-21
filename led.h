#ifndef LED_H            // Guard against multiple inclusion
#define LED_H



const struct rgbcolor ledturnon[] = 
{
	{0,0,9},
	{0,0,0},
	{0,0,0},
	{0,0,9},
	{0,0,0},
	{0,0,0},	
	{0,0,9},
	{0,0,0},
	{0,0,0},
	{0,0,9},
	{0,0,9},
	{0,0,9},
	{0,0,9},
	{0,0,10},
	{0,0,10},
	{0,0,11},
	{0,0,11},
	{0,0,12},
	{0,0,12},
	{0,0,13},
	{0,0,13},
	{0,0,14},
	{0,0,14},
	{0,0,15},
	{0,0,15},
	{0,0,16},
	{0,0,16},
	{0,0,17},
	{0,0,17},
	{0,0,18},
	{0,0,18},
	{0,0,18},
	{0,0,18},
	{0,0,17},
	{0,0,17},
	{0,0,16},
	{0,0,16},
	{0,0,15},
	{0,0,15},
	{0,0,14},
	{0,0,14},
	{0,0,13},
	{0,0,13},
	{0,0,12},
	{0,0,12},
	{0,0,11},
	{0,0,11},
	{0,0,10},
	{0,0,10},
	{0,0,9},
	{0,0,9},
	{0,0,9},
	{0,0,9},
	{0,0,0}
};

const struct rgbcolor ledyelf[] = //  Flashing yellow
{
	{10,10,0},
	{0,0,0},
	{0,0,0},
	{0,0,0},
	{0,0,0},
	{0,0,0},
	{0,0,0},
	{10,10,0},	
	{0,0,0},
	{0,0,0},
	{0,0,0},
	{0,0,0},
	{0,0,0},
	{0,0,0},
	{10,10,0},
	{0,0,0},
	{0,0,0},
	{0,0,0},
	{0,0,0},
	{0,0,0},
	{0,0,0},
	{16,16,0},
	{0,0,0},
	{0,0,0}
};


const struct rgbcolor ledyelfs[] = //  SLOW Flashing yellow
{
	{0,12,12},
	{0,0,0},
	{0,0,0},
	{0,0,0},
	{0,0,0},
	{0,0,0},
	{0,0,0},
	{0,0,0},
	{0,0,0},
	{0,0,0},
	{0,0,0},
	{0,0,0},
	{0,0,0},
	{0,0,0}
};

const struct rgbcolor ledblecon[] = // flashing BLUE
{
	{0,9,0},
	{0,10,0},
	{0,11,0},
	{0,12,0},
	{0,13,0},
	{0,14,0},
	{0,15,0},
	{0,16,0},
	{0,15,0},
	{0,14,0},
	{0,13,0},	
	{0,12,0},	
	{0,11,0},	
	{0,10,0},
	{0,9,0},	
	{0,0,0},
	{0,0,0},
	{0,0,0},
	{0,0,0},
	{0,0,0},
	{0,0,0},
	{0,0,0},
	{0,0,0},
	{0,0,0},
	{0,0,0},
	{0,0,0},
	{0,0,0},
	{0,0,0},
	{0,0,0},
	{0,0,0}
};


const struct rgbcolor ledorgf[] = // flashing ORANGE
{
	{13,0,13},
	{14,0,14},
	{15,0,15},
	{16,0,16},
	{17,0,17},
	{18,0,18},
	{19,0,19},
	{20,0,20},
	{14,0,20},
	{14,0,20},
	{0,0,0},
	{0,0,0},
	{0,0,0},
	{0,0,0},
	{0,0,0},
	{0,0,0},
	{0,0,0},
	{0,0,0},
	{0,0,0},
	{0,0,0},
	{0,0,0}
};

const struct rgbcolor ledturnoff[] = // power off sequence
{
	{20,0,0},
	{0,0,0},
	{0,0,0},	
	{20,0,0},
	{0,0,0},
	{0,0,0},	
	{20,0,0},
	{19,0,0},
	{18,0,0},
	{17,0,0},
	{16,0,0},
	{15,0,0},
	{14,0,0},
	{13,0,0},
	{12,0,0},
	{11,0,0},
	{10,0,0},
	{9,0,0},
	{9,0,0},
	{0,0,0},
	{0,0,0},
	{0,0,0},
	{0,0,0},
	{0,0,0},
	{0,0,0},
	{20,0,0},
	{20,0,0},
	{20,0,0},
	{20,0,0}
};


const struct rgbcolor led_00[] = //  Charge indication, 0-24% RED
{
	{9,0,0},
	{10,0,0},
	{11,0,0},
	{12,0,0},
	{0,0,0},
	{0,0,0},
	{0,0,0},
	{0,0,0},
	{0,0,0},
	{0,0,0},
	{0,0,0},
	{0,0,0},
	{0,0,0},
	{0,0,0},
	{0,0,0},
	{0,0,0},
	{0,0,0},
	{0,0,0},
	{0,0,0},
	{0,0,0}
};

const struct rgbcolor led_25[] = //  Charge indication, 25%-49% YEL
{
	{9,9,0},
	{10,10,0},
	{11,11,0},
	{12,12,0},
	{0,0,0},
	{0,0,0},
	{0,0,0},
	{0,0,0},
	{0,0,0},
	{0,0,0},
	{0,0,0},
	{0,0,0},
	{0,0,0},
	{0,0,0},
	{0,0,0},
	{0,0,0},
	{0,0,0},
	{0,0,0},
	{0,0,0},
	{0,0,0}
};


const struct rgbcolor led_50[] = //  Charge indication, 50%-95% GRN
{
	{0,9,0},
	{0,10,0},
	{0,11,0},
	{0,12,0},
	{0,0,0},
	{0,0,0},
	{0,0,0},
	{0,0,0},
	{0,0,0},
	{0,0,0},
	{0,0,0},
	{0,0,0},
	{0,0,0},
	{0,0,0},
	{0,0,0},
	{0,0,0},
	{0,0,0},
	{0,0,0},
	{0,0,0},
	{0,0,0}
};


const struct rgbcolor led_95[] = //  Charge indication, 95+%
{
	{0,14,0}
};


#define HAP_NOM 15 //nominal haptic time in tenmS 

// Haptic and beep FeedBack player constants - 100mS intervals
// entries - {frequency in HZ, Duration in tenmS, Haptic Duration in tenmS}

#define NOTE_C7 2093
#define NOTE_E7 2637
#define NOTE_G7 3136

const struct feedback fbturnon[]=
{
   {0,0,0},
   {NOTE_C7, 20,0},
   {0,0,HAP_NOM},
   {NOTE_E7, 20,0},
   {0,0,0},
   {NOTE_G7, 20,0},
   {0,0,HAP_NOM}

};


const struct feedback fbturnoff[]=
{
   {NOTE_G7, 20,0},
   {0,0,HAP_NOM},
   {NOTE_E7, 20,0},
   {0,0,0},   
   {NOTE_C7, 20,0},
   {0,0,0},
   {0,0,HAP_NOM}   
};


const struct feedback fbshortup[]=
{
   {NOTE_C7, 5,HAP_NOM},
   {0,0,0},
   {NOTE_E7, 5,0}

};

const struct feedback fbshortdn[]=
{
   {NOTE_E7, 5,HAP_NOM},
   {0,0,0},
   {NOTE_C7, 5,0},
   {0,0,0},
   {0,0,0}
};

#endif                      // Avoid multiple inclusion

/*************************** End of file ****************************/