----------------------------------------------------------------------------------------------------
-- FILE NAME: HT_Main_CXP_P040_Rev0_Top.vhd
----------------------------------------------------------------------------------------------------
-- COPYRIGHT Huron Digital Pathology
----------------------------------------------------------------------------------------------------
-- Project:              HT   
-- Board:                P034
-- FPGA:                 Lattice LCMXO2-2000HC-XTG100C
-- Designer:             Alfonso Ribes
-- Date of Origin:       May 11, 2021

----------------------------------------------------------------------------------------------------
--  General Description:
--  The FPGA on this board does several things:
--  1) Controls power to entire board (FPGA and Arduino/micro are always on)
--  2) Controls power to the autoloader board	
--  3) Takes in the trigger from the Galil encoder and outputs a trigger to the DALSA frame grabber
-- 	4) 
--




library ieee;
use ieee.std_logic_1164.all;
use ieee.std_logic_unsigned.all;
use IEEE.NUMERIC_STD.ALL;

library MACHXO2;
use MACHXO2.components.all;


entity HT_Main_CXP_P040_Rev0_Top is
Port 
	(
    iOSC_50MHz_P     			: in STD_LOGIC;       -- PIN 88 - 50MHz LVDS oscillator
    iOSC_50MHz_N     			: in STD_LOGIC;       -- PIN 87 - 50MHz oscillator not used at the moment
	
	oOSC_Enable					: out STD_LOGIC;  	  -- PIN 1 	- HI = enable 50MHz LVDS oscillator
	oReset_N_ETH				: out STD_LOGIC;  	  -- PIN 2 	- Reset_N signal to ethernet board, LO = Reset
	iReset_N_Button				: in  STD_LOGIC;   	  -- PIN 3 	- Connected to button. LO = Reset
	iPin23_IN1_Door				: in  STD_LOGIC;   	  -- PIN 4 	- Signal from Arduino which controls door motor driver
	oRed_Ind_LED   				: out STD_LOGIC;      -- PIN 7  - Red indicator LED on board (HI = OFF) 
	oSystem_ON  				: out STD_LOGIC;      -- PIN 8  - Main system signal that turns on 24V_SW (HI = ON) 
	oRed_LED_Pulse  			: out STD_LOGIC;      -- PIN 9  - 
	iFront_Panel_SW_ON  		: in STD_LOGIC;       -- PIN 10 - Input from front panel switch (HI = ON) 
    oReset_N     				: out STD_LOGIC;      -- PIN 12 - Active low reset for Arduino (LO = Reset)
    oErase_N    				: out STD_LOGIC;      -- PIN 13 - Active low erase signal used to program Arduino
    oWhite_LED_Pulse    		: out STD_LOGIC;      -- PIN 14 - White LED control (HI = ON)
    oFrame_Grab_Trig    		: out STD_LOGIC;      -- PIN 15 - Output trigger to frame grabber 
    iEncoder_Trig       		: in STD_LOGIC;       -- PIN 16 - Input trigger from Galil encoder
	iPin11_Prev_Cam_Trig     	: in STD_LOGIC;       -- PIN 17 - 
	iPin29_Top_Illumination     : in STD_LOGIC;       -- PIN 18 - 
--	oOut_To_Galil	    		: out STD_LOGIC;      -- PIN 19 - Spare signal out to DMC 
	iSlideRel_Home_Sens			: in STD_LOGIC;       -- PIN 20 - Slide release home sensor
    oSlideRel_Step				: out STD_LOGIC;      -- PIN 21 - Step output to TI slide release motor driver
--    iRST_Ard_FromGalil_N		: in STD_LOGIC;       -- PIN 24 - Active low reset signal for Arduino from Galil DMC
--    iIn_From_Galil			: in STD_LOGIC;       -- PIN 25 - Spare signal in from Galil
    iERASE_CMD   				: in STD_LOGIC;       -- PIN 28 - Input from Arduino micro helper
    oSlideRel_DIR				: out STD_LOGIC;      -- PIN 29 - DIR output to TI slide release motor driver
    oSlideRel_ENA      			: out STD_LOGIC;      -- PIN 30 - Enable signal out to TI slide release motor driver
 --   iPin18_ENA_Red_LED			: in STD_LOGIC;       -- PIN 34 - 
 --   iPin13_ENA_Green_LED		: in STD_LOGIC;       -- PIN 35 - 
    oGreen_LED_Pulse			: out STD_LOGIC;      -- PIN 36 - 
 --   iPin19_ENA_Blue_LED			: in STD_LOGIC;       -- PIN 37 - 
    oPreview_Cam_Trig			: out STD_LOGIC;      -- PIN 38 - Trigger to preview camera
    oPreview_Top_Illumination	: out STD_LOGIC;      -- PIN 39 - Top preview illumination (HI = ON)
    oPreview_Bot_Illumination	: out STD_LOGIC;      -- PIN 40 - Bottom preview illumination (HI = ON) 
    iPin24_IN2_Door				: in STD_LOGIC;       -- PIN 42 - Signal from Arduino which controls door motor driver
	oDoor_IN1					: out STD_LOGIC;      -- PIN 43 - Signal connected to door motor driver
	oDoor_IN2					: out STD_LOGIC;      -- PIN 45 - Signal connected to door motor driver
    iRFID_Data_IN				: in STD_LOGIC;       -- PIN 47 - Serial data from RFID board
    oRFID_CLK					: out STD_LOGIC;      -- PIN 51 - Clock for serial data to RFID board
    oRFID_Data_OUT				: out STD_LOGIC;      -- PIN 52 - Serial data to RFID board
    iPin36_RFID_Reset_N			: in STD_LOGIC;       -- PIN 53 - Active low reset for RFID board
    iPin44_RFID_S0				: in STD_LOGIC;       -- PIN 54 - 
    iPin45_RFID_S1				: in STD_LOGIC;       -- PIN 57 - 
    iPin46_RFID_S2				: in STD_LOGIC;       -- PIN 58 - 
    iPin47_RFID_EN_N			: in STD_LOGIC;       -- PIN 59 - 
    oPin48_CartridgeSW1			: out STD_LOGIC;      -- PIN 60 - 
    oPin49_CartridgeSW2			: out STD_LOGIC;      -- PIN 61 - 
    oPin50_CartridgeSW3			: out STD_LOGIC;      -- PIN 62 - 
    oPin51_CartridgeSW4			: out STD_LOGIC;      -- PIN 63 - 
    oPin43_CartridgeSW5			: out STD_LOGIC;      -- PIN 64 - 
    iPin42_WhiteLED_ENA_N		: in STD_LOGIC;       -- PIN 65 - 
    iPin41_Home					: in STD_LOGIC;       -- PIN 66 - Home bit used for door and slide release motors
    iPin40_Bot_Illumination		: in STD_LOGIC;   	  -- PIN 67 - 
    iPin39_GO					: in STD_LOGIC;       -- PIN 68 - 'GO' rising edge initiates slide release mechanism move
    iPin38_Pulsing_Mode			: in STD_LOGIC;       -- PIN 69 - Enables long pulsing on white LED (Enable pulsing = HI)
    iPin37_Micro_Online			: in STD_LOGIC;       -- PIN 70 - 
    oBlue_LED_Pulse				: out STD_LOGIC;       -- PIN 71 - 
    oPin34_ACK					: out STD_LOGIC;      -- PIN 74 - 
    oPin32_DONE					: out STD_LOGIC;      -- PIN 75 - 
	iPin12_Select_SlideRel		: in STD_LOGIC;       -- PIN 78 - 
    iSlideRel_ENCA 				: in STD_LOGIC;       -- PIN 83 - Step signal to Trinamic motor driver
    iSlideRel_ENCB 				: in STD_LOGIC;       -- PIN 84 - Step signal to Trinamic motor driver
	ioSCL						: inout STD_LOGIC;    -- PIN 86 - I2C clock signal
	ioSDA						: inout STD_LOGIC;    -- PIN 85 - I2C data signal
    iSlideRel_ENCX 				: in STD_LOGIC;        -- PIN 96 - Enable signal to Trinamic motor driver
	oSystem_ON_N				: out STD_LOGIC        -- PIN 99 - Encoder Index signal input
    );
end HT_Main_CXP_P040_Rev0_Top;





architecture Behavioral of HT_Main_CXP_P040_Rev0_Top is

constant cSimulation 						: boolean := FALSE; 
constant cRGB_Emissions_Test 				: boolean := TRUE; 

-- Currently running 26.6MHz internal crystal
constant cOneSecond 						: natural := 26600000; 
constant cOne_MilliSec						: natural := 26600; 
constant cOne_MicroSec						: natural := 27; 
constant cNum_MilliSec_InOneSec				: natural := 1000; 

constant cInitiate_Soft_Power_RST_In_mS 	: natural := (2*cNum_MilliSec_InOneSec)-1;   
-- The point in miliseconds (2 sec) at which soft power reset is initiated by the FPGA
-- Only 3.3V and 5V stay on during a soft reset

constant cSoft_Power_RST_In_mSec 			: natural := cInitiate_Soft_Power_RST_In_mS +(4*cNum_MilliSec_InOneSec)-1;   
-- The point in miliseconds (6 seconds) at which point a soft power reset ends.
-- Actual off time is 4 seconds

constant cStability_Time_In_mS 				: natural := 100; 
-- Time, in mS, at which point Copley and Arduino IOs are assumed to be stable

constant cExt_Reset_Time_In_mS				: natural := 100; 
constant cFront_SW_Threshold_In_mS			: natural := 100; 
constant cArduino_ResetTime_In_mS 			: natural := 600; 
constant cEthernet_ResetTime_In_mS 			: natural := cArduino_ResetTime_In_mS - 100; 

constant cSPI_Period_Counts					: natural := 1330-4; -- 20kHz clock
constant cvSyncIn_Code 						:  STD_LOGIC_VECTOR (7 downto 0) := "10000011"; 

constant cStart_Trig 						: natural := cOne_MicroSec*11; 
constant cEnd_Trig 							: natural := cStart_Trig + (cOne_MicroSec*20); 
constant cStart_LED 						: natural := cOne_MicroSec*1;
constant cEnd_LED 							: natural := cStart_LED + (cOne_MicroSec*120); 
constant cReset_Trig_Cntr 					: natural := cEnd_LED + 3; 

-- Slide Release 
-- I need to convert encoder distance to microsteps
-- Encoder says 2000 counts per revolution so 8000 total counts due to quadrature
-- 1.8 degrees per step, 16 microsteps per steps so 3200 microsteps per revolution
-- 8000/3200 = 2.5 encoder counts per microstep = 5/2
constant cMinimum_uSteps					: natural := 9000; -- Nominally 9000 to achieve crusing speed at half-way point
constant cEncoder_Counts_Per_Rev			: natural := 2000*4; -- Encoder spec is 2000 counts per rev times 4 edges for A and B phases
constant cuSteps_Per_Rev 					: natural := 200*16; -- 1.8 deg per step, so 200 steps per rev, 16 microsteps per step
constant cMinimum_Position_Distance			: natural := cMinimum_uSteps * cEncoder_Counts_Per_Rev / cuSteps_Per_Rev;
constant cZero_Position 					: natural := 20000; -- nominally larger maximum distance back to hard stop

constant cStartAccel_StepCount				: natural := 2;
constant cAccelDistance_NumSteps			: natural := 4500;
constant cStopAccel_StepCount				: natural := cStartAccel_StepCount + cAccelDistance_NumSteps;

constant cStep_HI_NumCounts		 			: natural := 10*cOne_MicroSec;
constant cUpdate_CLK_Cntr_NumCounts		 	: natural := 20*cOne_MicroSec;
constant cTimeout_NumSteps		 			: natural := 100000;

constant cStart_Period_NumCounts 			: natural := 5320; -- Corresponds to a start speed of 5,000pps
constant cA 								: natural := 53200000; -- Constant A from spreadsheet, corresponds to an acceleration of 30,000ppsps
constant cB									: natural := 10000; -- Constant B from spreadsheet
constant cC									: natural := 30000; -- Constant C from spreadsheet, corresponds to cruise speed of 15,000pps





-- Startup
signal sFront_SW_ON							:  STD_LOGIC := '0';
signal sFront_SW_ON_1D						:  STD_LOGIC := '0';

signal svOneMilliSec_Cntr					:  STD_LOGIC_VECTOR (15 downto 0) := (others => '0'); 
signal sOneMilliSec							:  STD_LOGIC := '0';

signal sSoft_Power_Reset_Request			:  STD_LOGIC := '0';
signal sSoft_Power_Reset_InProgress			:  STD_LOGIC := '0';
signal svSoftPower_RST_Cntr					:  STD_LOGIC_VECTOR (15 downto 0) := (others => '0'); 

signal sStable_System    					:  STD_LOGIC := '0';  
signal sStable_System_1D    				:  STD_LOGIC := '0';  
signal sStable_System_RE    				:  STD_LOGIC := '0';  

signal sReset_Arduino						:  STD_LOGIC := '0';   
signal sReset_Arduino_N						:  STD_LOGIC := '0';   
signal sResetStability_Cntr_Enable			:  STD_LOGIC := '0';   
signal svResetStability_Cntr 				:  STD_LOGIC_VECTOR (15 downto 0) := (others => '0'); 
signal sRST_Ard_FromGalil_N					:  STD_LOGIC := '0';   
signal sRST_Ard_FromGalil_N_1D				:  STD_LOGIC := '0';   
signal sExt_Reset							:  STD_LOGIC := '0';   
signal sEnable_Ext_RST_Cntr					:  STD_LOGIC := '0';   
signal svExt_RST_Cntr 						:  STD_LOGIC_VECTOR (15 downto 0) := (others => '0'); 
signal sPin37_Micro_Online					:  STD_LOGIC := '0';   
signal sPin37_Micro_Online_1D				:  STD_LOGIC := '0';   
signal sSystem_ON							:  STD_LOGIC := '0';   
signal sSystem_ON_N							:  STD_LOGIC := '0';   

-- Pre-registers for outputs
signal sRed_Ind_LED							:  STD_LOGIC := '0';
signal sRed_Ind_LED_1D						:  STD_LOGIC := '0';
signal sFrame_Grab_Trig						:  STD_LOGIC := '0';

-- Red indicator LED
signal svRedIndLED_Cntr						:  STD_LOGIC_VECTOR (31 downto 0) := (others => '0'); 
signal sRedIndLED_Cntr_RST					:  STD_LOGIC := '0';

-- Frame grabber trigger
signal sEncoder	    						:  STD_LOGIC := '0';   
signal sEncoder_1D    						:  STD_LOGIC := '0';   
signal svEncoder_Del 						:  STD_LOGIC_VECTOR (1 downto 0) := (others => '0'); 
signal sEncoder_PD   						:  STD_LOGIC := '0';   
signal sEncoder_PD_1D  						:  STD_LOGIC := '0';   
signal sEncoder_PD_FE  						:  STD_LOGIC := '0';   
signal sFE_Trig_NoGlitch					:  STD_LOGIC := '0';   
signal sEnable_Trig_Cntr					:  STD_LOGIC := '0';   
signal svTrig_Cntr 							:  STD_LOGIC_VECTOR (31 downto 0) := (others => '0'); 
signal sTrigger								:  STD_LOGIC := '0';   

-- LED
signal sEnable_LED							:  STD_LOGIC := '0';   
signal sWhite_LED_ENA_N    					:  STD_LOGIC := '0';   
signal sWhite_LED_ENA_N_1D    				:  STD_LOGIC := '0';   
signal sPulsing_Mode    					:  STD_LOGIC := '0';   
signal sPulsing_Mode_1D    					:  STD_LOGIC := '0';   
signal sWhite_LED_Pulse    					:  STD_LOGIC := '0';   
signal svPulse_Mode_SEL						:  STD_LOGIC_VECTOR (2 downto 0) := (others => '0'); 
signal sRed_LED_Enable						:  STD_LOGIC := '0';   
signal sGreen_LED_Enable					:  STD_LOGIC := '0';   
signal sBlue_LED_Enable						:  STD_LOGIC := '0';   
signal svRGB_Pulse_Cntr						:  STD_LOGIC_VECTOR (1 downto 0) := (others => '0'); 
signal sRed_LED_ENA_Arduino						:  STD_LOGIC := '0';
signal sGreen_LED_ENA_Arduino					:  STD_LOGIC := '0';
signal sBlue_LED_ENA_Arduino					:  STD_LOGIC := '0';
signal sRed_LED_ENA_Arduino_1D					:  STD_LOGIC := '0';
signal sGreen_LED_ENA_Arduino_1D				:  STD_LOGIC := '0';
signal sBlue_LED_ENA_Arduino_1D					:  STD_LOGIC := '0';
signal sRed_LED_Pulse						:  STD_LOGIC := '0';
signal sGreen_LED_Pulse						:  STD_LOGIC := '0';
signal sBlue_LED_Pulse						:  STD_LOGIC := '0';

-- Slide
signal sSlideRel_ENCA    					:  STD_LOGIC := '0';   
signal sSlideRel_ENCA_1D    				:  STD_LOGIC := '0';   
signal sSlideRel_ENCA_2D    				:  STD_LOGIC := '0';   
signal sSlideRel_ENCA_RE    				:  STD_LOGIC := '0';   
signal sSlideRel_ENCA_FE    				:  STD_LOGIC := '0';   
signal sSlideRel_ENCB    					:  STD_LOGIC := '0';   
signal sSlideRel_ENCB_1D    				:  STD_LOGIC := '0';   
signal sSlideRel_ENCB_2D    				:  STD_LOGIC := '0';   
signal sSlideRel_ENCB_RE    				:  STD_LOGIC := '0';   
signal sSlideRel_ENCB_FE    				:  STD_LOGIC := '0';   
signal sSlideRel_ENCX    					:  STD_LOGIC := '0';   
signal sSlideRel_ENCX_1D    				:  STD_LOGIC := '0';   
signal sSlideRel_ENCX_2D    				:  STD_LOGIC := '0';   
signal sENCX_RE    							:  STD_LOGIC := '0';   
signal svRequested_Position					:  STD_LOGIC_VECTOR (23 downto 0) := (others => '0'); 
signal svPre_Requested_Position				:  STD_LOGIC_VECTOR (23 downto 0) := (others => '0'); 
signal svSlideRel_LivePosition				:  STD_LOGIC_VECTOR (23 downto 0) := (others => '0'); 
signal svPresent_Position					:  STD_LOGIC_VECTOR (23 downto 0) := (others => '0'); 
signal sReached_Requested_Position			:  STD_LOGIC := '0';   
signal sSlideRel_Home_Sensor				:  STD_LOGIC := '0';   
signal sSlideRel_IsAtHomeSensor				:  STD_LOGIC := '0';   
signal sPin39_GO							:  STD_LOGIC := '0';   
signal sGO									:  STD_LOGIC := '0';   
signal sGO_1D								:  STD_LOGIC := '0';   
signal sGO_StartPulse						:  STD_LOGIC := '0';   
signal sPin41_Home							:  STD_LOGIC := '0';   
signal sHome_Cntrl_Bit						:  STD_LOGIC := '0';   
signal sStart_Homing						:  STD_LOGIC := '0';   
signal sStart_Moving						:  STD_LOGIC := '0';   
signal sHoming								:  STD_LOGIC := '0';   
signal sMoving								:  STD_LOGIC := '0';   
signal sSD_DIR								:  STD_LOGIC := '0';   
signal sSlideRel_ENA						:  STD_LOGIC := '0';   
signal sStep								:  STD_LOGIC := '0';   
signal svMove_Distance						:  STD_LOGIC_VECTOR (23 downto 0) := (others => '0');   
signal sMove_Below_Minimum					:  STD_LOGIC := '0';   
signal sAccelerating						:  STD_LOGIC := '0';   
signal sDecelerating						:  STD_LOGIC := '0';   
signal sSet_To_MaxSpeed						:  STD_LOGIC := '0';   
signal svPre_Den_Division_Accel				:  STD_LOGIC_VECTOR (31 downto 0) := (others => '0');   
signal svPre_Den_Division_Decel				:  STD_LOGIC_VECTOR (31 downto 0) := (others => '0');   
signal sUpdate_Cntr_RST						:  STD_LOGIC := '0';   
signal svUpdate_CLK_Cntr					:  STD_LOGIC_VECTOR (15 downto 0) := (others => '0');   
signal svCLK_Cntr_End						:  STD_LOGIC_VECTOR (31 downto 0) := (others => '0');   
signal sCLK_Cntr_RST						:  STD_LOGIC := '0';   
signal sCLK_Cntr_RST_1D						:  STD_LOGIC := '0';   
signal sCLK_Cntr_RST_RE						:  STD_LOGIC := '0';   
signal svCLK_Cntr							:  STD_LOGIC_VECTOR (15 downto 0) := (others => '0');   
signal cvAcceleration_Cntr					:  STD_LOGIC_VECTOR (15 downto 0) := (others => '0');   
signal svStep_Cntr							:  STD_LOGIC_VECTOR (23 downto 0) := (others => '0'); 
signal svStartDecel_StepCount				:  STD_LOGIC_VECTOR (23 downto 0) := (others => '0'); 
signal svPre_StartDecel_StepCount			:  STD_LOGIC_VECTOR (23 downto 0) := (others => '0'); 
signal svStopDecel_StepCount				:  STD_LOGIC_VECTOR (23 downto 0) := (others => '0'); 
signal sMoveDone							:  STD_LOGIC := '0';   
signal sSL_Encoder_Phase_Changed			:  STD_LOGIC := '0';   
signal sSL_Encoder_Phase_Changed_1D			:  STD_LOGIC := '0';   
signal svSL_Current_Encoder_Phase			:  STD_LOGIC_VECTOR (1 downto 0) := (others => '0'); 
signal svSL_Previous_Encoder_Phase			:  STD_LOGIC_VECTOR (1 downto 0) := (others => '0'); 
signal svRequested_Position_LSByte			:  STD_LOGIC_VECTOR (7 downto 0) := (others => '0'); 
signal svRequested_Position_MByte			:  STD_LOGIC_VECTOR (7 downto 0) := (others => '0'); 
signal svRequested_Position_MSByte			:  STD_LOGIC_VECTOR (7 downto 0) := (others => '0'); 
signal svSD_Step_Count_LSByte				:  STD_LOGIC_VECTOR (7 downto 0) := (others => '0'); 
signal svSD_Step_Count_MByte				:  STD_LOGIC_VECTOR (7 downto 0) := (others => '0'); 
signal svSD_Step_Count_MSByte				:  STD_LOGIC_VECTOR (7 downto 0) := (others => '0'); 
signal svEnable_RGB_LEDs_Byte				:  STD_LOGIC_VECTOR (7 downto 0) := (others => '0'); 
signal sPin12_Select_SlideRel				:  STD_LOGIC := '0';   
signal sPin12_Select_SlideRel_1D			:  STD_LOGIC := '0';   
signal svDoor_LivePosition					:  STD_LOGIC_VECTOR (23 downto 0) := (others => '0'); 
signal sDoor_ENCA							:  STD_LOGIC := '0';   
signal sDoor_ENCA_1D						:  STD_LOGIC := '0';   
signal sDoor_ENCA_2D						:  STD_LOGIC := '0';   
signal sDoor_ENCA_RE						:  STD_LOGIC := '0';   
signal sDoor_ENCA_FE						:  STD_LOGIC := '0';   
signal sDoor_ENCB							:  STD_LOGIC := '0';   
signal sDoor_ENCB_1D						:  STD_LOGIC := '0';   
signal sDoor_ENCB_2D						:  STD_LOGIC := '0';   
signal sDoor_ENCB_RE						:  STD_LOGIC := '0';   
signal sDoor_ENCB_FE						:  STD_LOGIC := '0';   
signal sDoor_Encoder_Phase_Changed			:  STD_LOGIC := '0';   
signal sDoor_Encoder_Phase_Changed_1D		:  STD_LOGIC := '0';   
signal svDoor_Current_Encoder_Phase			:  STD_LOGIC_VECTOR (1 downto 0) := (others => '0');   
signal svDoor_Previous_Encoder_Phase		:  STD_LOGIC_VECTOR (1 downto 0) := (others => '0');   
signal sDoor_Reached_Requested_Position		:  STD_LOGIC := '0';   
signal sDoor_Home_Sensor					:  STD_LOGIC := '0';   
signal sDoor_IsAtHomeSensor					:  STD_LOGIC := '0';   
signal sAtHome_Sensor						:  STD_LOGIC := '0';   
signal sMovingTo_HomeSensor					:  STD_LOGIC := '0';   
signal sDoor_ENA							:  STD_LOGIC := '0';   
signal sPin34_ACK							:  STD_LOGIC := '0';   
signal sPin32_DONE							:  STD_LOGIC := '0';   



-- RFID Serializer
signal sReset_SPI_Cntr						:  STD_LOGIC := '0';   
signal svSPI_Cntr							:  STD_LOGIC_VECTOR (15 downto 0) := (others => '0'); 
signal sCartridge1							:  STD_LOGIC := '0';
signal sCartridge2							:  STD_LOGIC := '0';
signal sCartridge3							:  STD_LOGIC := '0';
signal sCartridge4							:  STD_LOGIC := '0';
signal sCartridge5							:  STD_LOGIC := '0';
signal sCartridge1_1D						:  STD_LOGIC := '0';
signal sCartridge2_1D						:  STD_LOGIC := '0';
signal sCartridge3_1D						:  STD_LOGIC := '0';
signal sCartridge4_1D						:  STD_LOGIC := '0';
signal sCartridge5_1D						:  STD_LOGIC := '0';
signal svCartridge 							:  STD_LOGIC_VECTOR (4 downto 0) := (others => '0'); 

signal sSPI_clk								:  STD_LOGIC := '0';
signal sSPI_clk_1D							:  STD_LOGIC := '0';
signal sSPI_clk_2D							:  STD_LOGIC := '0';
signal sSPI_clk_RE							:  STD_LOGIC := '0';
signal sSPI_clk_FE							:  STD_LOGIC := '0';
signal svSPI_clk_Period_Cntr 				:  STD_LOGIC_VECTOR (9 downto 0) := (others => '0'); 
signal svGood_SPI_clk_Cntr 					:  STD_LOGIC_VECTOR (4 downto 0) := (others => '0'); 
signal sGood_SPI_clk						:  STD_LOGIC := '0';
signal svSyncIn_Code 						:  STD_LOGIC_VECTOR (7 downto 0) := (others => '0'); 
signal sWait_For_Incoming_Data				:  STD_LOGIC := '0';
signal svIncoming_Data_Cntr 				:  STD_LOGIC_VECTOR (3 downto 0) := (others => '0'); 
signal svIncoming_Data_Vector 				:  STD_LOGIC_VECTOR (4 downto 0) := (others => '0'); 
signal svCartridge_Data 					:  STD_LOGIC_VECTOR (4 downto 0) := (others => '0'); 

signal sSPI_Data_IN							:  STD_LOGIC := '0';
signal sSPI_Data_IN_1D						:  STD_LOGIC := '0';

signal sAna_Mux_Sel2						:  STD_LOGIC := '0';
signal sAna_Mux_Sel1						:  STD_LOGIC := '0';
signal sAna_Mux_Sel0						:  STD_LOGIC := '0';
signal sMux_ENA_N							:  STD_LOGIC := '0';
signal sRFID_RST_N							:  STD_LOGIC := '0';
signal sAna_Mux_Sel2_1D						:  STD_LOGIC := '0';
signal sAna_Mux_Sel1_1D						:  STD_LOGIC := '0';
signal sAna_Mux_Sel0_1D						:  STD_LOGIC := '0';
signal sMux_ENA_N_1D						:  STD_LOGIC := '0';
signal sRFID_RST_N_1D						:  STD_LOGIC := '0';

signal sOutput_SPI_Data						:  STD_LOGIC := '0';
signal sOutput_SPI_Data_1D					:  STD_LOGIC := '0';
signal sOutput_SPI_Data_FE					:  STD_LOGIC := '0';
signal sOutput_SPI_Data_RE					:  STD_LOGIC := '0';
signal sCapture_Incoming_Data				:  STD_LOGIC := '0';  
signal sStart_Outputting_SPI_Data			:  STD_LOGIC := '0';
signal svOutput_SPI_Data_Vector 			:  STD_LOGIC_VECTOR (12 downto 0) := (others => '0'); 
signal sRFID_Data_Out						:  STD_LOGIC := '0';
signal svOutput_SPI_Data_Counter 			:  STD_LOGIC_VECTOR (3 downto 0) := (others => '0'); 
signal sEnd_SPI_Data_Output					:  STD_LOGIC := '0';
signal sEnd_SPI_Data_Output_1D				:  STD_LOGIC := '0';

-- Division
signal sStart_Pulse_Division				:  STD_LOGIC := '0';   
signal svNum_Division						:  STD_LOGIC_VECTOR (31 downto 0) := (others => '0'); 
signal svDen_Division						:  STD_LOGIC_VECTOR (31 downto 0) := (others => '0'); 
signal svResult_Division					:  STD_LOGIC_VECTOR (31 downto 0) := (others => '0'); 
signal sFinish_Pulse_Division				:  STD_LOGIC := '0';   

signal svGPO_0, svGPO_1						:  STD_LOGIC_VECTOR (7 downto 0) := (others => '0'); 
signal sI2C_Enable							:  STD_LOGIC := '0';   
signal sInternal_OSC_26M60					:  STD_LOGIC := '0'; 	 
signal svCLK_Cntr_End_Div2					:  STD_LOGIC_VECTOR (31 downto 0) := (others => '0'); 

signal sOSC_50MHz							:  STD_LOGIC := '0'; 

signal svToggle_OSC_ENA_Cntr				:  STD_LOGIC_VECTOR (15 downto 0) := (others => '0'); 
signal sreset_n_button_1d					:  STD_LOGIC := '0'; 
signal sosc_enable							:  STD_LOGIC := '0'; 
signal sstart_reset_arduino					:  STD_LOGIC := '0'; 
signal sreset_n_button						:  STD_LOGIC := '0'; 
signal spostcopley_cntr_enable_fe			:  STD_LOGIC := '0'; 
signal sstart_reset_arduino_1d				:  STD_LOGIC := '0'; 
signal sstart_reset_arduino_re				:  STD_LOGIC := '0'; 
signal senable_rstarduino_cntr				:  STD_LOGIC := '0'; 
signal svrstarduino_cntr					:  STD_LOGIC_VECTOR (15 downto 0) := (others => '0'); 
signal sreset_ethernet						:  STD_LOGIC := '0'; 
signal spostcopley_cntr_enable_1d			:  STD_LOGIC := '0'; 
signal sRequest_Ext_Reset					:  STD_LOGIC := '0'; 
signal sMain_24V_SW_ON						:  STD_LOGIC := '0'; 
signal sMain_24V_SW_ON_1D					:  STD_LOGIC := '0'; 
signal sMain_24V_SW_ON_RE					:  STD_LOGIC := '0'; 
signal svFront_SW_Cntr						:  STD_LOGIC_VECTOR (15 downto 0) := (others => '0'); 
signal sTurn_System_ON						:  STD_LOGIC := '0'; 





attribute NOM_FREQ : string;
attribute NOM_FREQ of Instance_Int_OSC : label is "26.60";

COMPONENT ILVDS
PORT (A: IN std_logic;
AN: IN std_logic;
Z: OUT std_logic);
END COMPONENT;


COMPONENT OSCH
-- synthesis translate_off
GENERIC (NOM_FREQ: string := "26.60");
-- synthesis translate_on
PORT (STDBY: IN std_logic;
OSC: OUT std_logic;
SEDSTDBY: OUT std_logic);
END COMPONENT;


component i2c_gpio_P034_Rev6 is

  generic (
             GPI_PORT_NUM       : integer    := 1;       -- GPI port number
			 GPI_DATA_WIDTH     : integer    := 8;       -- GPI data width
			 GPO_PORT_NUM       : integer    := 7;       -- GPO port number
			 GPO_DATA_WIDTH     : integer    := 8;       -- GPO data width
			 MEM_ADDR_WIDTH     : integer    := 8;       -- Memory addrss width
			 IRQ_NUM            : integer    := 4;       -- Interrupt request number
			 MAX_MEM_BURST_NUM  : std_logic_vector (7 downto 0)    := "00001000";       -- Maximum memory burst number
		     INTQ_OPENDRAIN     : bit        := '1'      -- INTQ opendrain setting (S_ON/S_OFF)
		   );

port(

SCL      : inout std_logic;
SDA      : inout std_logic;
GPO_0    : out std_logic_vector(GPO_DATA_WIDTH-1 downto 0);
GPO_1    : out std_logic_vector(GPO_DATA_WIDTH-1 downto 0);
GPO_2    : out std_logic_vector(GPO_DATA_WIDTH-1 downto 0);
GPO_3    : out std_logic_vector(GPO_DATA_WIDTH-1 downto 0);
GPO_4    : out std_logic_vector(GPO_DATA_WIDTH-1 downto 0);
GPO_5    : out std_logic_vector(GPO_DATA_WIDTH-1 downto 0);
GPO_6    : out std_logic_vector(GPO_DATA_WIDTH-1 downto 0);
--oGPO_DATA : out GPO_ARRAY;
IRQ      : in std_logic_vector (IRQ_NUM-1 downto 0);      
GPI_0    : in std_logic_vector (GPI_DATA_WIDTH-1 downto 0);
--GPI_1    : in std_logic_vector (GPI_DATA_WIDTH-1 downto 0);
--GPI_2    : in std_logic_vector (GPI_DATA_WIDTH-1 downto 0);
--GPI_3    : in std_logic_vector (GPI_DATA_WIDTH-1 downto 0);
Enable   : out std_logic;
INTQ     : out std_logic:='1';
RST_N    : in std_logic;
CLK      : in std_logic;
MEM_CLK  : out std_logic;
MEM_WR   : out std_logic;
MEM_ADDR : out std_logic_vector(MEM_ADDR_WIDTH-1 downto 0);        
MEM_WD   : out std_logic_vector(7 downto 0);
MEM_RD   : in std_logic_vector(7 downto 0)
);
end component;


component serial_divider is
	generic
	(
	gInput_Width :					integer := 32 -- Maximum 64	
	);
    port
	(
	-- Main clock
	iMCLK : 							in std_logic;	
	
	-- Inputs
	iStart_Pulse :						in std_logic;
    iNum : 								in std_logic_vector(gInput_Width-1 downto 0);
	iDen : 								in std_logic_vector(gInput_Width-1 downto 0);

	-- Outputs
	oDivRes :							out std_logic_vector((2*gInput_Width)-1 downto 0);
	oWhole_DivRes :						out std_logic_vector(gInput_Width-1 downto 0);
	oFinish_Pulse :						out std_logic
	);	
end component;


begin

Instance_50MHZ_Diff_Input: ILVDS
PORT MAP (A=> iOSC_50MHz_P,
AN => iOSC_50MHz_N,
Z => sOSC_50MHz
);

Instance_Int_OSC: OSCH
PORT MAP (STDBY=> '0',
OSC => sInternal_OSC_26M60,
SEDSTDBY => open
);


inst_i2c_gpio_P034_Rev6: i2c_gpio_P034_Rev6 
PORT MAP
	(
	SDA => ioSDA,
    SCL => ioSCL,
    INTQ => open,   
	GPI_0 => x"FF",
	--GPI_1 => x"FF",
	--GPI_2 => x"FF",
	--GPI_3 => x"FF",
	GPO_0 => svRequested_Position_LSByte,
	GPO_1 => svRequested_Position_MByte,
	GPO_2 => svRequested_Position_MSByte,
	GPO_3 => svSD_Step_Count_LSByte,
	GPO_4 => svSD_Step_Count_MByte,
	GPO_5 => svSD_Step_Count_MSByte,
	GPO_6 => svEnable_RGB_LEDs_Byte,
	IRQ  => "0000",
	Enable => open,
	MEM_CLK => open,
	MEM_WR => open , 
	MEM_ADDR => open ,
	MEM_WD => open ,
	MEM_RD => "00000000" ,
	RST_N => sReset_Arduino_N,
--	CLK => iOSC_25MHz
	CLK => sInternal_OSC_26M60
	);


inst_serial_divider: serial_divider 
	PORT MAP
	(
 --   iMCLK => iOSC_25MHz,
    iMCLK => sInternal_OSC_26M60,
    iStart_Pulse => sStart_Pulse_Division,
    iNum => svNum_Division,      
    iDen => svDen_Division,
    oDivRes => open,
    oWhole_DivRes => svResult_Division,
    oFinish_Pulse => sFinish_Pulse_Division
    );


--OSC_50MHz_Process: process (sOSC_50MHz) 
OSC_26_6MHz_Process: process (sInternal_OSC_26M60) 
begin
--if rising_edge(sOSC_50MHz) then
if rising_edge(sInternal_OSC_26M60) then

-- *** BEGIN Red Indicator LED ***

	if (svRedIndLED_Cntr > 100000000) then
		sRedIndLED_Cntr_RST <= '1';
	else
		sRedIndLED_Cntr_RST <= '0';
	end if;
	
	if (sRedIndLED_Cntr_RST = '1') then
		svRedIndLED_Cntr <= (others => '0');
	else
		svRedIndLED_Cntr <= svRedIndLED_Cntr + 1;
	end if;
	
	if (svRedIndLED_Cntr > 50000000) then
		sRed_Ind_LED <= '1';
	else
		sRed_Ind_LED <= '0';
	end if;
	
	sRed_Ind_LED_1D <= sRed_Ind_LED;
	oRed_Ind_LED <= not sRed_Ind_LED_1D;

	
-- *** END Red Indicator LED ***

end if;
end process; 





Main_Process: process (sInternal_OSC_26M60) 
begin
if rising_edge(sInternal_OSC_26M60) then


	if (svOneMilliSec_Cntr = cOne_MilliSec-1) then
		sOneMilliSec <= '1';
	else
		sOneMilliSec <= '0';
	end if;
	
	if (sOneMilliSec = '1') then
		svOneMilliSec_Cntr <= (others => '0');
	else
		svOneMilliSec_Cntr <= svOneMilliSec_Cntr + 1;
	end if;
	


-- ***** START of Resets *****

--	sRST_Ard_FromGalil_N <= iRST_Ard_FromGalil_N;
	sRST_Ard_FromGalil_N <= '1';
	sRST_Ard_FromGalil_N_1D <= sRST_Ard_FromGalil_N;
			
	sReset_N_Button <= iReset_N_Button;
	sReset_N_Button_1D <= sReset_N_Button;
	
	if (sRST_Ard_FromGalil_N_1D = '0' or sReset_N_Button_1D = '0') then	 
		sRequest_Ext_Reset <= '1';
	else
		sRequest_Ext_Reset <= '0';
	end if;
	
	if (svExt_RST_Cntr = cExt_Reset_Time_In_mS) then
		sExt_Reset <= '1';
	else	
		sExt_Reset <= '0';
	end if;
	
	if (sRequest_Ext_Reset = '0') then
		sEnable_Ext_RST_Cntr <= '0';
	else
		if (svExt_RST_Cntr > cExt_Reset_Time_In_mS) then
			sEnable_Ext_RST_Cntr <= '0';
		elsif (svExt_RST_Cntr <= cExt_Reset_Time_In_mS) then
			sEnable_Ext_RST_Cntr <= '1';
		else	
			null;
		end if;
	end if;
	
	if (sRequest_Ext_Reset = '0') then
		svExt_RST_Cntr <= (others => '0');
	else
		if (sEnable_Ext_RST_Cntr and sOneMilliSec) = '1' then
			svExt_RST_Cntr <= svExt_RST_Cntr + 1;
		else
			null;
		end if;
	end if;
	
	sStart_Reset_Arduino <= sExt_Reset or sMain_24V_SW_ON_RE;
	sStart_Reset_Arduino_1D <= sStart_Reset_Arduino;
	sStart_Reset_Arduino_RE <= sStart_Reset_Arduino and (not sStart_Reset_Arduino_1D);
	
	if (sStart_Reset_Arduino_RE = '1') then
		sEnable_RSTArduino_Cntr <= '1';
	elsif (svRSTArduino_Cntr > cArduino_ResetTime_In_mS) then
		sEnable_RSTArduino_Cntr <= '0';
	else
		null;
	end if;
	
	if (sEnable_RSTArduino_Cntr = '0') then
		svRSTArduino_Cntr <= (others => '0');
	elsif (sEnable_RSTArduino_Cntr and sOneMilliSec) = '1' then
		svRSTArduino_Cntr <= svRSTArduino_Cntr + 1;
	else
		null;
	end if;
		
	if (sStart_Reset_Arduino_RE = '1') then
		sReset_Ethernet <= '1';
	elsif (svRSTArduino_Cntr >= cEthernet_ResetTime_In_mS) then
		sReset_Ethernet <= '0';
	else
		null;
	end if;
	
	if (sStart_Reset_Arduino_RE = '1') then
		sReset_Arduino <= '1';
	elsif (svRSTArduino_Cntr >= cArduino_ResetTime_In_mS) then
		sReset_Arduino <= '0';
	else
		null;
	end if;
	
	sReset_Arduino_N <= not sReset_Arduino;
	
	if (sReset_Ethernet = '1') then
		oReset_N_ETH <= '0';
	else
		oReset_N_ETH <= 'Z';
	end if;
			
	if (sReset_Arduino = '1') then
		oReset_N <= '0';
	else
		oReset_N <= 'Z';
	end if;
			
-- ***** END of Resets *****



-- ***** START of power control *****

	sFront_SW_ON <= iFront_Panel_SW_ON;
	sFront_SW_ON_1D <= sFront_SW_ON;
	
	if (sFront_SW_ON_1D = '1') then
		if (svFront_SW_Cntr <= cFront_SW_Threshold_In_mS) and (sOneMilliSec = '1') then
			svFront_SW_Cntr <= svFront_SW_Cntr + 1;
		else
			null;
		end if;
	else
		if (svFront_SW_Cntr > 0) and (sOneMilliSec = '1') then
			svFront_SW_Cntr <= svFront_SW_Cntr - 1;
		else
			null;
		end if;
	end if;
	
	if (svFront_SW_Cntr = 0) then
		sTurn_System_ON <= '0';
	elsif (svFront_SW_Cntr >= cFront_SW_Threshold_In_mS) then
		sTurn_System_ON <= '1';
	else
		null;
	end if;

	sSoft_Power_Reset_Request <= '0';
--	sSoft_Power_Reset_Request <= (not sRST_Ard_FromGalil_N_1D) or sSoft_Power_Reset_InProgress;

	if (sSoft_Power_Reset_Request = '0') then		
		svSoftPower_RST_Cntr <= (others => '0');
	else
		if (sSoft_Power_Reset_Request = '1' or sSoft_Power_Reset_InProgress = '1') and (sOneMilliSec = '1') then
			svSoftPower_RST_Cntr <= svSoftPower_RST_Cntr + 1;
		else
			null;
		end if;
	end if;
	
	if (svSoftPower_RST_Cntr > cSoft_Power_RST_In_mSec) then
		sSoft_Power_Reset_InProgress <= '0';
	elsif (svSoftPower_RST_Cntr > cInitiate_Soft_Power_RST_In_mS) then
		sSoft_Power_Reset_InProgress <= '1';
	end if;
	
	sMain_24V_SW_ON <= sTurn_System_ON and (not sSoft_Power_Reset_InProgress);
	sMain_24V_SW_ON_1D <= sMain_24V_SW_ON;
	sMain_24V_SW_ON_RE <= sMain_24V_SW_ON and (not sMain_24V_SW_ON_1D);
	
	sSystem_ON <= sMain_24V_SW_ON_1D;
	sSystem_ON_N <= not sMain_24V_SW_ON_1D;
	
	oSystem_ON <= sSystem_ON;
	oSystem_ON_N <= sSystem_ON_N;
		
	sPin37_Micro_Online <= iPin37_Micro_Online;
	sPin37_Micro_Online_1D <= sPin37_Micro_Online;
	
	if (sMain_24V_SW_ON_1D = '0' or sPin37_Micro_Online_1D = '0') then
		sStable_System <= '0';
	else
		if (svResetStability_Cntr > cStability_Time_In_mS) then
			sStable_System <= '1';
		else
			null;
		end if;
	end if;
	
	sStable_System_1D <= sStable_System;
	sStable_System_RE <= sStable_System and (not sStable_System_1D);
	
	if (sMain_24V_SW_ON_1D = '0') then
		sResetStability_Cntr_Enable <= '0';
	else
		if (svResetStability_Cntr > cStability_Time_In_mS) then
			sResetStability_Cntr_Enable <= '0';
		else
			sResetStability_Cntr_Enable <= '1';
		end if;
	end if;
	
	if (sResetStability_Cntr_Enable = '1') and (sOneMilliSec = '1') then
		svResetStability_Cntr <= svResetStability_Cntr + 1;
	elsif (sResetStability_Cntr_Enable = '0') then
		svResetStability_Cntr <= (others => '0');
	else
		null;
	end if;
	
		
-- *****  END of power control  *****


	
-- *** BEGIN Frame Grabber Trigger ***

	sEncoder <= iEncoder_Trig;
	sEncoder_1D <= sEncoder;
	svEncoder_Del <= svEncoder_Del(svEncoder_Del'left-1 downto 0) & sEncoder_1D;
	sEncoder_PD <= svEncoder_Del(svEncoder_Del'left);
	sEncoder_PD_1D <= sEncoder_PD;
	sEncoder_PD_FE <= not sEncoder_PD and sEncoder_PD_1D;
	
	if (svEncoder_Del = 0 and sEncoder_PD_FE = '1') then
		sFE_Trig_NoGlitch <= '1';
	else
		sFE_Trig_NoGlitch <= '0';
	end if;

	if (sFE_Trig_NoGlitch = '1' and svTrig_Cntr < cReset_Trig_Cntr) then
		sEnable_Trig_Cntr <= '1';
	elsif (svTrig_Cntr = cReset_Trig_Cntr) then
		sEnable_Trig_Cntr <= '0';
	else
		null;
	end if;
	
	if (sEnable_Trig_Cntr = '0') then
		svTrig_Cntr <= (others => '0');
	else
		svTrig_Cntr <= svTrig_Cntr + 1;
	end if;
	
	if (svTrig_Cntr = 0) then
		sTrigger <= '1';
	else
		if (svTrig_Cntr = cStart_Trig) then
			sTrigger <= '0';
		elsif (svTrig_Cntr = cEnd_Trig) then
			sTrigger <= '1';
		else
			null;
		end if;
	end if;
	
	sFrame_Grab_Trig <= sTrigger;
	oFrame_Grab_Trig <= sFrame_Grab_Trig;

-- *** END Frame Grabber Trigger ***



-- *** BEGIN Main White Illumination ***

	-- Registers for RIO control inputs
	sWhite_LED_ENA_N <= iPin42_WhiteLED_ENA_N;
	sWhite_LED_ENA_N_1D <= sWhite_LED_ENA_N;
	
	if (svTrig_Cntr = 0) then
		sEnable_LED <= '0';
	else
		if (svTrig_Cntr = cStart_LED) then
			sEnable_LED <= '1';
		elsif (svTrig_Cntr = cEnd_LED) then
			sEnable_LED <= '0';
		else
			null;
		end if;
	end if;
	
	sPulsing_Mode <= iPin38_Pulsing_Mode;
		
	if cRGB_Emissions_Test then
		sPulsing_Mode_1D <= '1';
	else
		sPulsing_Mode_1D <= sPulsing_Mode;
	end if;

	if (sStable_System = '0') or (sWhite_LED_ENA_N_1D = '1') or cRGB_Emissions_Test then
		sWhite_LED_Pulse <= '1';
	else
		if (sPulsing_Mode_1D = '1') then
			sWhite_LED_Pulse <= not sEnable_LED;
		else
			sWhite_LED_Pulse <= '0';
		end if;
	end if;
	
	-- Registers for outputs to illumination control FETs
	-- High-impedence at startup avoids a startup glitch
	if (sStable_System = '0') then 
		oWhite_LED_Pulse <= 'Z';
	else
		oWhite_LED_Pulse <= sWhite_LED_Pulse;
	end if;

-- *** END Main White Illumination ***



-- *** BEGIN Main RGB Illumination ***

	-- Registers for RIO control inputs
	sRed_LED_ENA_Arduino <= svEnable_RGB_LEDs_Byte(0);
	sGreen_LED_ENA_Arduino <= svEnable_RGB_LEDs_Byte(1);
	sBlue_LED_ENA_Arduino <= svEnable_RGB_LEDs_Byte(2);

	sRed_LED_ENA_Arduino_1D <= sRed_LED_ENA_Arduino;
	sGreen_LED_ENA_Arduino_1D <= sGreen_LED_ENA_Arduino;
	sBlue_LED_ENA_Arduino_1D <= sBlue_LED_ENA_Arduino;
	
	if cRGB_Emissions_Test then
		svPulse_Mode_SEL <= "000";
	else
		svPulse_Mode_SEL <= sRed_LED_ENA_Arduino_1D & sGreen_LED_ENA_Arduino_1D & sBlue_LED_ENA_Arduino_1D;
	end if;
	
	if (sPulsing_Mode_1D = '1') then
	
		if (sStable_System = '0') then
			svRGB_Pulse_Cntr <= (others => '0');
		elsif (svTrig_Cntr = cEnd_LED) then
			if (svRGB_Pulse_Cntr = 2) then
				svRGB_Pulse_Cntr <= (others => '0');
			else
				svRGB_Pulse_Cntr <= svRGB_Pulse_Cntr + 1;
			end if;
		end if;
		
		CASE svRGB_Pulse_Cntr IS
			WHEN "01" => -- Green
				sRed_LED_Enable <=  '0';
				sGreen_LED_Enable <=  '1';
				sBlue_LED_Enable <=  '0';
				
			WHEN "10" => -- Blue
				sRed_LED_Enable <=  '0';
				sGreen_LED_Enable <=  '0';
				sBlue_LED_Enable <=  '1';
				
			WHEN OTHERS => -- Red
				sRed_LED_Enable <=  '1';
				sGreen_LED_Enable <=  '0';
				sBlue_LED_Enable <=  '0';
		END CASE;				
				
		CASE svPulse_Mode_SEL IS
			WHEN "100" => -- Red pulse used within 3-pass brightfield
				sRed_LED_Pulse <=  sEnable_LED;
				sGreen_LED_Pulse <=  '0';
				sBlue_LED_Pulse <=  '0';

			WHEN "010" => -- Green pulse used within 3-pass brightfield
				sRed_LED_Pulse <=  '0';
				sGreen_LED_Pulse <=  sEnable_LED;
				sBlue_LED_Pulse <=  '0';

			WHEN "001" => -- Blue pulse used within 3-pass brightfield
				sRed_LED_Pulse <=  '0';
				sGreen_LED_Pulse <=  '0';
				sBlue_LED_Pulse <=  sEnable_LED;

			WHEN "000" => -- RGB rolling pulse used within single pass brightfield
				sRed_LED_Pulse <=  sEnable_LED and sRed_LED_Enable;
				sGreen_LED_Pulse <=  sEnable_LED and sGreen_LED_Enable;
				sBlue_LED_Pulse <=  sEnable_LED and sBlue_LED_Enable;

			WHEN OTHERS => -- all LEDs off
				sRed_LED_Pulse <=  '0';
				sGreen_LED_Pulse <=  '0';
				sBlue_LED_Pulse <=  '0';
				
		END CASE;

	else
	
		sRed_LED_Pulse <= sRed_LED_ENA_Arduino_1D;
		sGreen_LED_Pulse <= sGreen_LED_ENA_Arduino_1D;
		sBlue_LED_Pulse <= sBlue_LED_ENA_Arduino_1D;
		svRGB_Pulse_Cntr <= (others => '0');
		
	end if;

	-- Registers for outputs to illumination control FETs
	-- High-impedence at startup avoids a startup glitch
	if (sStable_System = '0') then
		oRed_LED_Pulse <= 'Z';
		oGreen_LED_Pulse <= 'Z';
		oBlue_LED_Pulse <= 'Z';
	else
		oRed_LED_Pulse <= sRed_LED_Pulse;
		oGreen_LED_Pulse <= sGreen_LED_Pulse;
		oBlue_LED_Pulse <= sBlue_LED_Pulse;
--		oRed_LED_Pulse <= '0';
--		oGreen_LED_Pulse <= sRed_Ind_LED_1D;
--		oBlue_LED_Pulse <= '0';
	end if;

-- *** END Main RGB Illumination ***



-- *** BEGIN RFID Serializer ***

	if (svSPI_Cntr > cSPI_Period_Counts) then
		sReset_SPI_Cntr <= '1';
	else
		sReset_SPI_Cntr <= '0';
	end if;
	
	if (sReset_SPI_Cntr = '1') then
		svSPI_Cntr <= (others => '0');
	else
		svSPI_Cntr <= svSPI_Cntr + 1;
	end if;
	
	if (svSPI_Cntr > cSPI_Period_Counts/2) then
		sSPI_clk <= '1';
	else
		sSPI_clk <= '0';
	end if;

	sSPI_clk_1D <= sSPI_clk;
	sSPI_clk_2D <= sSPI_clk_1D;
	sSPI_clk_RE <= sSPI_clk_1D and (not sSPI_clk_2D);
	sSPI_clk_FE <= (not sSPI_clk_1D) and sSPI_clk_2D;
	
	oRFID_CLK <= sSPI_clk_2D;

	sAna_Mux_Sel2 <= iPin46_RFID_S2;
	sAna_Mux_Sel1 <= iPin45_RFID_S1;
	sAna_Mux_Sel0 <= iPin44_RFID_S0;
	sMux_ENA_N <= iPin47_RFID_EN_N;
	sRFID_RST_N <= iPin36_RFID_Reset_N;

	sAna_Mux_Sel2_1D <= sAna_Mux_Sel2;
	sAna_Mux_Sel1_1D <= sAna_Mux_Sel1;
	sAna_Mux_Sel0_1D <= sAna_Mux_Sel0;
	sMux_ENA_N_1D <= sMux_ENA_N;
	sRFID_RST_N_1D <= sRFID_RST_N;

	
	if (svOutput_SPI_Data_Counter = 13) then  
		sEnd_SPI_Data_Output <= '1';
	else
		sEnd_SPI_Data_Output <= '0'; 
	end if;	 
	
	sEnd_SPI_Data_Output_1D <= sEnd_SPI_Data_Output;
	
	if (sEnd_SPI_Data_Output = '1') then
		sOutput_SPI_Data <= '0';
	else
		if (sStable_System_RE = '1' and sOutput_SPI_Data = '0') or (sStable_System_RE = '0' and sEnd_SPI_Data_Output_1D = '1') then
			sOutput_SPI_Data <= '1';
		end if;	  
	end if;
	
	sOutput_SPI_Data_1D <= sOutput_SPI_Data;
	sOutput_SPI_Data_FE <= (not sOutput_SPI_Data) and sOutput_SPI_Data_1D;
	sOutput_SPI_Data_RE <= sOutput_SPI_Data and (not sOutput_SPI_Data_1D);
	
	if (sOutput_SPI_Data_RE = '1') then
		sStart_Outputting_SPI_Data <= '1';
	elsif (sSPI_clk_FE = '1') then
		sStart_Outputting_SPI_Data <= '0';
	end if;	
	
	if (sSPI_clk_FE = '1') then
		if (sStart_Outputting_SPI_Data = '1') then
			svOutput_SPI_Data_Counter <= (others => '0');
			svOutput_SPI_Data_Vector <= cvSyncIn_Code & sAna_Mux_Sel2_1D & sAna_Mux_Sel1_1D & sAna_Mux_Sel0_1D & sMux_ENA_N_1D & sRFID_RST_N_1D;
		else
			svOutput_SPI_Data_Counter <= svOutput_SPI_Data_Counter + 1;
			svOutput_SPI_Data_Vector <= svOutput_SPI_Data_Vector(svOutput_SPI_Data_Vector'left-1 downto 0) & '0';
		end if;
	end if;
	
	sRFID_Data_OUT <= svOutput_SPI_Data_Vector(svOutput_SPI_Data_Vector'left);
	
	oRFID_Data_OUT <= sRFID_Data_OUT;


	sSPI_Data_IN <= iRFID_Data_IN;
	sSPI_Data_IN_1D <= sSPI_Data_IN;
	
	if (sSPI_clk_RE = '1') then
		svSyncIn_Code <= svSyncIn_Code(svSyncIn_Code'left-1 downto 0) & sSPI_Data_IN_1D;
	else
		null;
	end if;
	
	if (svIncoming_Data_Cntr = 5) then
		sCapture_Incoming_Data <= '1';
	else
		sCapture_Incoming_Data <= '0';
	end if;

	if (svSyncIn_Code = cvSyncIn_Code) and (sCapture_Incoming_Data = '0') then
		sWait_For_Incoming_Data <= '1';
	elsif (sCapture_Incoming_Data = '1') then
		sWait_For_Incoming_Data <= '0';
	end if;

	if (sWait_For_Incoming_Data = '0') then
		svIncoming_Data_Cntr <= (others => '0');
	else
		if (sSPI_clk_RE = '1') then
			svIncoming_Data_Cntr <= svIncoming_Data_Cntr + 1;
		end if;
	end if;
	
	if (sSPI_clk_RE = '1') then
		svIncoming_Data_Vector <= svIncoming_Data_Vector(svIncoming_Data_Vector'left-1 downto 0) & sSPI_Data_IN_1D;
	end if;
	
	if (sCapture_Incoming_Data = '1') then
		svCartridge_Data <= svIncoming_Data_Vector;
	end if;
	
	sCartridge5 <= svCartridge_Data(4);
	sCartridge4 <= svCartridge_Data(3);
	sCartridge3 <= svCartridge_Data(2);
	sCartridge2 <= svCartridge_Data(1);
	sCartridge1 <= svCartridge_Data(0);
	
	sCartridge1_1D <= sCartridge1;
	sCartridge2_1D <= sCartridge2;
	sCartridge3_1D <= sCartridge3;
	sCartridge4_1D <= sCartridge4;
	sCartridge5_1D <= sCartridge5;
	
    oPin48_CartridgeSW1 <= sCartridge1_1D;
    oPin49_CartridgeSW2	<= sCartridge2_1D; 
    oPin50_CartridgeSW3	<= sCartridge3_1D;  
    oPin51_CartridgeSW4	<= sCartridge4_1D;  
    oPin43_CartridgeSW5	<= sCartridge5_1D;  
	
	
-- *** END RFID Serializer ***




-- *** BEGIN  Motor driver, sensor, and encoders ***

-- Slide release encoders and position
	sSlideRel_ENCA <= iSlideRel_ENCA;
	sSlideRel_ENCA_1D <= sSlideRel_ENCA;
	sSlideRel_ENCA_2D <= sSlideRel_ENCA_1D;
	sSlideRel_ENCA_RE <= sSlideRel_ENCA_1D and (not sSlideRel_ENCA_2D);
	sSlideRel_ENCA_FE <= (not sSlideRel_ENCA_1D) and sSlideRel_ENCA_2D;
	
	sSlideRel_ENCB <= iSlideRel_ENCB;
	sSlideRel_ENCB_1D <= sSlideRel_ENCB;
	sSlideRel_ENCB_2D <= sSlideRel_ENCB_1D;
	sSlideRel_ENCB_RE <= sSlideRel_ENCB_1D and (not sSlideRel_ENCB_2D);
	sSlideRel_ENCB_FE <= (not sSlideRel_ENCB_1D) and sSlideRel_ENCB_2D;
	
	sSlideRel_ENCX <= iSlideRel_ENCX;
	sSlideRel_ENCX_1D <= sSlideRel_ENCX;
	sSlideRel_ENCX_2D <= sSlideRel_ENCX_1D;
	
	sSL_Encoder_Phase_Changed <= sSlideRel_ENCA_RE or sSlideRel_ENCA_FE or sSlideRel_ENCB_RE or sSlideRel_ENCB_FE;
	sSL_Encoder_Phase_Changed_1D <= sSL_Encoder_Phase_Changed;
	
	if (sSL_Encoder_Phase_Changed = '1') then
	
		if (sSlideRel_ENCA_1D = '0' and sSlideRel_ENCB_1D = '0') then
			svSL_Current_Encoder_Phase <= "00";
		elsif (sSlideRel_ENCA_1D = '0' and sSlideRel_ENCB_1D = '1') then
			svSL_Current_Encoder_Phase <= "01";
		elsif (sSlideRel_ENCA_1D = '1' and sSlideRel_ENCB_1D = '1') then
			svSL_Current_Encoder_Phase <= "10";
		else
			svSL_Current_Encoder_Phase <= "11";
		end if;
		
		svSL_Previous_Encoder_Phase <= svSL_Current_Encoder_Phase;
		
	else
		null;
	end if;

	if (sHoming = '1' and sMoveDone = '1') then
		svSlideRel_LivePosition <= std_logic_vector(to_unsigned(cZero_Position, svSlideRel_LivePosition'length));
	elsif (sSL_Encoder_Phase_Changed_1D = '1') then
		if (((svSL_Current_Encoder_Phase > svSL_Previous_Encoder_Phase) and (not(svSL_Previous_Encoder_Phase = 0 and svSL_Current_Encoder_Phase = 3)))
			or (svSL_Current_Encoder_Phase = 0 and svSL_Previous_Encoder_Phase = 3)) then
			svSlideRel_LivePosition <= svSlideRel_LivePosition - 1;
		else
			svSlideRel_LivePosition <= svSlideRel_LivePosition + 1;
		end if;
	else
		null;
	end if;
	
	if (svSlideRel_LivePosition <= svRequested_Position+1) and (svSlideRel_LivePosition >= svRequested_Position-1) then
		sReached_Requested_Position <= '1';
	else
		sReached_Requested_Position <= '0';
	end if;
		
	svPre_Requested_Position <= svRequested_Position_MSByte & svRequested_Position_MByte & svRequested_Position_LSByte; 		
	svPre_StartDecel_StepCount <= svSD_Step_Count_MSByte & svSD_Step_Count_MByte & svSD_Step_Count_LSByte;
		
	if (sMoving = '0') then
	
		if cSimulation then
			svRequested_Position <= std_logic_vector(to_unsigned(20100, svRequested_Position'length)); 	-- Move distance from home = 50k-20k = 30k, Steps = 30k/2.5 = 12,000 steps
			svStartDecel_StepCount <= std_logic_vector(to_unsigned(7500, svStartDecel_StepCount'length)); -- Requested Steps - Accel distance = [(40k-10k)/2.5] - 4500 = 12k-4500 = 7500
		else
			svRequested_Position <= svPre_Requested_Position; 		
			svStartDecel_StepCount <= svPre_StartDecel_StepCount;
		end if;
		
	else
		null;
	end if;
	
	svStopDecel_StepCount <= svStartDecel_StepCount + cAccelDistance_NumSteps;
	
	sENCX_RE <= sSlideRel_ENCX_1D and (not sSlideRel_ENCX_2D);

	-- Slide Release Home Sensor
	sSlideRel_Home_Sensor <= iSlideRel_Home_Sens;
	sSlideRel_IsAtHomeSensor <= not sSlideRel_Home_Sensor;
		
	sAtHome_Sensor <= sSlideRel_IsAtHomeSensor;
		
	-- Control Bits and movement 
	sPin39_GO <= iPin39_GO;
	sGO <= sPin39_GO;
	sGO_1D <= sGO;
	sGO_StartPulse <= sGO and (not sGO_1D);

	sPin41_Home <= iPin41_Home;
	sHome_Cntrl_Bit <= sPin41_Home;

	if (sHome_Cntrl_Bit = '1') then
		sStart_Homing <= sGO_StartPulse and (not sMoving) and (not sHoming);
		sStart_Moving <= '0';
	else
		sStart_Homing <= '0';
		sStart_Moving <= sGO_StartPulse and (not sMoving) and (not sHoming);
	end if;
	
	if (sStart_Homing = '1' or sStart_Moving = '1') then
		sPin34_ACK <= not sPin34_ACK;
	else
		null;
	end if;
	oPin34_ACK <= sPin34_ACK;

	if (sStart_Homing = '1') then
		sHoming <= '1';
	elsif (sMoveDone = '1') then
		sHoming <= '0';
	else
		null;
	end if;
	
	sPin32_DONE <= (not sHoming) and (not sMoving);
	oPin32_DONE <= sPin32_DONE;
	
	svCLK_Cntr_End_Div2 <= '0' & svCLK_Cntr_End(svCLK_Cntr_End'left downto 1);
	
	if (sStart_Homing = '1' or sMoveDone = '1') then
		sMovingTo_HomeSensor <= '1';
	elsif (sHoming = '1' and sAtHome_Sensor = '1' and svCLK_Cntr = svCLK_Cntr_End_Div2) then
		sMovingTo_HomeSensor <= '0';
	else
		null;
	end if;
		
	if (sStart_Moving = '1') then
		sMoving <= '1';
	elsif (sMoveDone = '1') then
		sMoving <= '0';
	else
		null;
	end if;
	
	if (sMoving = '0') then  
		svPresent_Position <= svSlideRel_LivePosition;
	else
		null;
	end if;
 		
	if (sHoming = '1') then
	
		if (sMovingTo_HomeSensor = '1') then
			sSD_DIR <= '1';
		else -- Moving to Index
			sSD_DIR <= '0';
		end if;
		
	else
	
		if (svRequested_Position >= svPresent_Position) then
			sSD_DIR <= '0';
		else
			sSD_DIR <= '1';
		end if;	
		
	end if;
	oSlideRel_DIR <= sSD_DIR;

	-- Disable motor when not moving
	if (sHoming = '1' or sMoving = '1') then
		sSlideRel_ENA <= '1';
	else
		sSlideRel_ENA <= '0';
	end if;
	oSlideRel_ENA <= sSlideRel_ENA;
	
	-- 10us HIGH pulse for Step input to motor driver
	if (svCLK_Cntr > 0) and (svCLK_Cntr < cStep_HI_NumCounts+1) then
		sStep <= '1';
	else
		sStep <= '0';
	end if;
	oSlideRel_Step <= sStep;	

	if (svRequested_Position >= svPresent_Position) then
		svMove_Distance <= svRequested_Position - svPresent_Position;
	else
		svMove_Distance <= svPresent_Position - svRequested_Position;
	end if;
	
	if (svMove_Distance > cMinimum_Position_Distance) then
		sMove_Below_Minimum <= '0';
	else
		sMove_Below_Minimum <= '1';
	end if;
	
	if (sHoming = '1') then
		sAccelerating <= '0';
		sDecelerating <= '0';
		sSet_To_MaxSpeed <= '0';
	else
	
		if (svStep_Cntr > cStartAccel_StepCount and svStep_Cntr < cStopAccel_StepCount) then
			sAccelerating <= '1';
			sDecelerating <= '0';
			sSet_To_MaxSpeed <= '0';
		elsif (svStep_Cntr >= cStopAccel_StepCount and svStep_Cntr <= svStartDecel_StepCount) then
			sAccelerating <= '0';
			sDecelerating <= '0';
			sSet_To_MaxSpeed <= '1';
		elsif (svStep_Cntr > svStartDecel_StepCount and svStep_Cntr < svStopDecel_StepCount) then
			sAccelerating <= '0';
			sDecelerating <= '1';
			sSet_To_MaxSpeed <= '0';
		else
			sAccelerating <= '0';
			sDecelerating <= '0';
			sSet_To_MaxSpeed <= '0';
		end if;
		
	end if;
	
	
	svPre_Den_Division_Accel <= std_logic_vector(to_unsigned(cB, svDen_Division'length)) + cvAcceleration_Cntr;
	svPre_Den_Division_Decel <= std_logic_vector(to_unsigned(cC, svDen_Division'length)) - cvAcceleration_Cntr;

	if (sAccelerating = '1') then
		if (svPre_Den_Division_Accel >= cC) then
			svDen_Division <= std_logic_vector(to_unsigned(cC, svDen_Division'length));
		else
			svDen_Division <= svPre_Den_Division_Accel;
		end if;
	elsif (sDecelerating = '1') then	
		if (svPre_Den_Division_Decel <= cB) then
			svDen_Division <= std_logic_vector(to_unsigned(cB, svDen_Division'length));
		else
			svDen_Division <= svPre_Den_Division_Decel;
		end if;
	elsif (sSet_To_MaxSpeed = '1') then
		svDen_Division <= std_logic_vector(to_unsigned(cC, svDen_Division'length));
	else -- Set to Min Speed
		svDen_Division <= std_logic_vector(to_unsigned(cB, svDen_Division'length));
	end if;

	if (svUpdate_CLK_Cntr = cUpdate_CLK_Cntr_NumCounts) then
		sUpdate_Cntr_RST <= '1';
	else
		sUpdate_Cntr_RST <= '0';
	end if;
	
	-- This update counter determines when the end count on the counter that determines step frequency gets changed
	if (sHoming = '0' and sMoving = '0') then
		svUpdate_CLK_Cntr <= (others => '0'); 
	elsif (sUpdate_Cntr_RST = '1') then
		svUpdate_CLK_Cntr <= (others => '0'); 
	else
		svUpdate_CLK_Cntr <= svUpdate_CLK_Cntr + 1;
	end if;

	-- This is the end count for the counter that determines step frequency (svCLK_Cntr)
	if (sHoming = '0' and sMoving = '0') or (sHoming = '1' or sMove_Below_Minimum = '1') then
		svCLK_Cntr_End <= std_logic_vector(to_unsigned(cStart_Period_NumCounts-4, svCLK_Cntr_End'length));
	elsif (sUpdate_Cntr_RST = '1') then
		svCLK_Cntr_End <= svResult_Division - 4;
	else
		null;
	end if;
	
	if (svCLK_Cntr > svCLK_Cntr_End) then
		sCLK_Cntr_RST <= '1';
	else
		sCLK_Cntr_RST <= '0';
	end if;
	
	sCLK_Cntr_RST_1D <= sCLK_Cntr_RST;
	sCLK_Cntr_RST_RE <= sCLK_Cntr_RST and (not sCLK_Cntr_RST_1D);

	if (sHoming = '0' and sMoving = '0') then
		svCLK_Cntr <= (others => '0'); 
	elsif (sCLK_Cntr_RST_RE = '1') then
		svCLK_Cntr <= (others => '0'); 
	else
		svCLK_Cntr <= svCLK_Cntr + 1;
	end if;
	
	if (sHoming = '0' and sMoving = '0') then
		cvAcceleration_Cntr <= (others => '0');
	elsif (sAccelerating = '0' and sDecelerating = '0') then
		cvAcceleration_Cntr <= (others => '0');
	elsif (sUpdate_Cntr_RST = '1') then
		cvAcceleration_Cntr <= cvAcceleration_Cntr + 1;
	else
		null;
	end if;

	sStart_Pulse_Division <= sMoving and (sUpdate_Cntr_RST or sStart_Moving);

	-- This is the counter that counts the number of steps that have occurred
	if (sHoming = '0' and sMoving = '0') then
		svStep_Cntr <= (others => '0');
	else
		if (sCLK_Cntr_RST_RE = '1') then
			svStep_Cntr <= svStep_Cntr + 1;
		else
			null;
		end if;
	end if;
		
	if (svStep_Cntr >= cTimeout_NumSteps) then
		sMoveDone <= '1';
	elsif (sHoming = '1' and sENCX_RE = '1' and sMovingTo_HomeSensor = '0' and sAtHome_Sensor = '0') then
		sMoveDone <= '1';
	elsif (sMoving = '1' and sReached_Requested_Position = '1') then
		sMoveDone <= '1';
	else
		sMoveDone <= '0';
	end if;

-- -- *** END Slide Motor Driver and Sensors ***


end if;
end process; 


-- Start Asynchronous

oErase_N <= not iERASE_CMD; -- Needed for Arduino

svNum_Division <= std_logic_vector(to_unsigned(cA, svNum_Division'length));

oOSC_Enable <= '0'; -- 50MHz external oscillator is permanently disabled, using internal 26.6MHz

oDoor_IN1 <= iPin23_IN1_Door;
oDoor_IN2 <= iPin24_IN2_Door;

oPreview_Cam_Trig <= iPin11_Prev_Cam_Trig;
oPreview_Top_Illumination <= iPin29_Top_Illumination;
oPreview_Bot_Illumination <= iPin40_Bot_Illumination;

-- End Asynchronous
		 
end Behavioral;





