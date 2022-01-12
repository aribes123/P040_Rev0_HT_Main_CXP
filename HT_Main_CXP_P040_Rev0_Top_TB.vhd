----------------------------------------------------------------------------------------------------
-- FILE NAME: HT_Main_P034_Rev6_Top_TB.vhd
----------------------------------------------------------------------------------------------------
-- COPYRIGHT Huron Digital Pathology
----------------------------------------------------------------------------------------------------
-- Project:              HT   
-- Board:                P034 Rev 6 HT Main
-- FPGA:                 Lattice LCMXO2-2000HC-XTG100C
-- Designer:             Alfonso Ribes
-- Date of Origin:       May 11, 2021
--
----------------------------------------------------------------------------------------------------
--  General Description:
--
--  


library ieee;
use ieee.std_logic_1164.all;
use ieee.std_logic_unsigned.all;
use IEEE.NUMERIC_STD.ALL;

library MACHXO2;
use MACHXO2.components.all;

 
ENTITY HT_Main_P034_Rev6_Top_TB IS
END HT_Main_P034_Rev6_Top_TB;
 
ARCHITECTURE behavior OF HT_Main_P034_Rev6_Top_TB IS 
 
-- Component Declaration for the Unit Under Test (UUT)
 
component HT_Main_P034_Rev6_Top is
Port 
	(
    iOSC_50MHz_P     			: in STD_LOGIC;       -- PIN 88 - 50MHz LVDS oscillator
    iOSC_50MHz_N     			: in STD_LOGIC;       -- PIN 87 - 50MHz oscillator not used at the moment
	
	oOSC_Enable					: out STD_LOGIC;  	  -- PIN 1 	- HI = enable 50MHz LVDS oscillator
	oReset_N_ETH				: out STD_LOGIC;  	  -- PIN 2 	- Reset_N signal to ethernet board
	iReset_N_Button				: in  STD_LOGIC;   	  -- PIN 3 	- Connected to button. Normally HI
	iPin23_IN1_Door				: in  STD_LOGIC;   	  -- PIN 4 	- Signal from Arduino which controls door motor driver
	oRed_Ind_LED   				: out STD_LOGIC;      -- PIN 7  - Red indicator LED on board (HI = OFF) 
	oSystem_ON  				: out STD_LOGIC;      -- PIN 8  - Main system signal that turns on 24V_SW (HI = ON) 
	iFront_Panel_SW_ON  		: in STD_LOGIC;       -- PIN 10 - Input from front panel switch (HI = ON) 
    oReset_N     				: out STD_LOGIC;      -- PIN 12 - Active low reset for Arduino (LO = Reset)
    oErase_N    				: out STD_LOGIC;      -- PIN 13 - Active low erase signal used to program Arduino
    oWhite_LED_Pulse    		: out STD_LOGIC;      -- PIN 14 - White LED control (HI = ON)
    oFrame_Grab_Trig    		: out STD_LOGIC;      -- PIN 15 - Output trigger to frame grabber 
    iEncoder_Trig       		: in STD_LOGIC;       -- PIN 16 - Input trigger from Galil encoder
	iPin11_Arduino     			: in STD_LOGIC;       -- PIN 17 - 
	iPin29_Arduino      		: in STD_LOGIC;       -- PIN 18 - 
--	oOut_To_Galil	    		: out STD_LOGIC;      -- PIN 19 - Spare signal out to DMC 
	iSlideRel_Home_Sens			: in STD_LOGIC;       -- PIN 20 - Slide release home sensor
    oSlideRel_Step				: out STD_LOGIC;      -- PIN 21 - Step output to TI slide release motor driver
--    iRST_Ard_FromGalil_N		: in STD_LOGIC;       -- PIN 24 - Active low reset signal for Arduino from Galil DMC
--    iIn_From_Galil			: in STD_LOGIC;       -- PIN 25 - Spare signal in from Galil
    iERASE_CMD   				: in STD_LOGIC;       -- PIN 28 - Input from Arduino micro helper
    oSlideRel_DIR				: out STD_LOGIC;      -- PIN 29 - DIR output to TI slide release motor driver
    oSlideRel_ENA      			: out STD_LOGIC;      -- PIN 30 - Enable signal out to TI slide release motor driver
    iPin18_Arduino		    	: in STD_LOGIC;       -- PIN 34 - 
--    iPin13_Arduino			: out STD_LOGIC;      -- PIN 35 - 
--    iPin31_Arduino			: out STD_LOGIC;      -- PIN 36 - 
--    iPin19_Arduino			: in STD_LOGIC;       -- PIN 37 - 
    oPreview_Cam_Trig			: out STD_LOGIC;      -- PIN 38 - Spare signal to pin
    oPreview_Top_Illumination	: out STD_LOGIC;      -- PIN 39 - Spare signal to pin
    oPreview_Bot_Illumination	: out STD_LOGIC;      -- PIN 40 - Spare signal to pin
    oDoor_ENA					: out STD_LOGIC;      -- PIN 41 - Spare signal to pin
    iPin24_IN2_Door				: in STD_LOGIC;       -- PIN 42 - Signal from Arduino which controls door motor driver
	oDoor_IN1					: out STD_LOGIC;      -- PIN 43 - Signal connected to door motor driver
	oDoor_IN2					: out STD_LOGIC;      -- PIN 45 - Signal connected to door motor driver
    iRFID_Data_IN				: in STD_LOGIC;       -- PIN 47 - Serial data from RFID board
    oRFID_CLK					: out STD_LOGIC;      -- PIN 51 - Clock for serial data to RFID board
    oRFID_Data_OUT				: out STD_LOGIC;      -- PIN 52 - Serial data to RFID board
    iPin36_RFID_Reset_N			: in STD_LOGIC;       -- PIN 53 - 
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
--    iPin40_Position_Cntrl1	: in STD_LOGIC;   	  -- PIN 67 - 
    iPin39_GO					: in STD_LOGIC;       		  -- PIN 68 - 'GO' rising edge initiates slide release mechanism move
    iPin38_Pulsing_Mode			: in STD_LOGIC;       -- PIN 69 - Enables long pulsing on white LED (Enable pulsing = HI)
    iPin37_Micro_Online			: in STD_LOGIC;       -- PIN 70 - 
--    iPin35_Position_Cntrl2	: in STD_LOGIC;       -- PIN 71 - 
    oPin34_ACK					: out STD_LOGIC;      -- PIN 74 - 
    oPin32_DONE					: out STD_LOGIC;      -- PIN 75 - 
	iPin12_Select_SlideRel		: in STD_LOGIC;       -- PIN 78 - 
    iSlideRel_ENCA 				: in STD_LOGIC;       -- PIN 83 - Step signal to Trinamic motor driver
    iSlideRel_ENCB 				: in STD_LOGIC;       -- PIN 84 - Step signal to Trinamic motor driver
	ioSCL						: inout STD_LOGIC;    -- PIN 86 - I2C clock signal
	ioSDA						: inout STD_LOGIC;    -- PIN 85 - I2C data signal
    iSlideRel_ENCX 				: in STD_LOGIC       -- PIN 96 - Enable signal to Trinamic motor driver
--	iDoor_ENCA				    : in STD_LOGIC;       -- PIN 97 - Encoder A signal input
--	iDoor_ENCB					: in STD_LOGIC;       -- PIN 98 - Encoder B signal input
--	iDoor_ENCX					: in STD_LOGIC        -- PIN 99 - Encoder Index signal input
    );
end component;
 
 
--component HT_RFID_P029_Top is
    --Port 
    --(
    --iCartridge1         : in  STD_LOGIC;      -- PIN 4  - Cartridge 1 switch (Present = LO)
    --iCartridge2         : in  STD_LOGIC;      -- PIN 5  - Cartridge 2 switch (Present = LO)
    --iCartridge3	        : in  STD_LOGIC;      -- PIN 8  - Cartridge 3 switch (Present = LO) 
    --iCartridge4         : in  STD_LOGIC;      -- PIN 9  - Cartridge 4 switch (Present = LO)
    --iCartridge5         : in  STD_LOGIC;      -- PIN 10 - Cartridge 5 switch (Present = LO)
    --oAna_Mux_Sel2       : out STD_LOGIC;      -- PIN 11 - Analog mux select bit 2 
    --oAna_Mux_Sel1       : out STD_LOGIC;      -- PIN 12 - Analog mux select bit 1 
    --oAna_Mux_Sel0       : out STD_LOGIC;      -- PIN 13 - Analog mux select bit 0 
    --oMux_ENA_N      	: out STD_LOGIC;      -- PIN 14 - Analog mux enable (active LO)
    --oRFID_RST_N      	: out STD_LOGIC;      -- PIN 16 - RFID reader reset (active LO) 
    --iSPI_Data      		: in  STD_LOGIC;      -- PIN 17 - SPI data in from main motherboard FPGA
    --iSPI_Clk      		: in  STD_LOGIC;      -- PIN 20 - SPI clock from main motherboard FPGA
    --oSPI_Data		   	: out STD_LOGIC       -- PIN 21 - SPI data out to main motherboard FPGA
    --);
--end component;



-- Main Module
signal iOSC_50MHz_P 				: std_logic := '0';
signal iOSC_50MHz_N 				: std_logic := '0';

signal oOSC_Enable 				: std_logic := '0';
signal oReset_N_ETH 				: std_logic := '0';
signal iReset_N_Button 				: std_logic := '0';
signal iPin23_IN1_Door 				: std_logic := '0';
signal oRed_Ind_LED 			: std_logic := '0';
signal oSystem_ON 				: std_logic := '0';
signal iFront_Panel_SW_ON 		: std_logic := '0';
signal oReset_N 				: std_logic := '0';
signal oErase_N 				: std_logic := '0';
signal oWhite_LED_Pulse 		: std_logic := '0';
signal oFrame_Grab_Trig 		: std_logic := '0';
signal iEncoder_Trig 			: std_logic := '0';
signal iPin11_Arduino 			: std_logic := '0';
signal iPin29_Arduino 			: std_logic := '0';
--signal oOut_To_Galil 			: std_logic := '0';
signal iSlideRel_Home_Sens 			: std_logic := '0';
signal oSlideRel_Step 				: std_logic := '0';
--signal iRST_Ard_FromGalil_N 	: std_logic := '0';
--signal iIn_From_Galil 			: std_logic := '0';
signal iERASE_CMD 				: std_logic := '0';
signal oSlideRel_DIR 			: std_logic := '0';
signal oSlideRel_ENA 					: std_logic := '0';
signal iPin18_Arduino 		: std_logic := '0';
--signal iPin13_Arduino 		: std_logic := '0';
--signal iPin31_Arduino 		: std_logic := '0';
--signal iPin19_Arduino 		: std_logic := '0';
signal oPreview_Cam_Trig 				: std_logic := '0';
signal oPreview_Top_Illumination 				: std_logic := '0';
signal oPreview_Bot_Illumination 				: std_logic := '0';
signal oDoor_ENA 		: std_logic := '0';
signal iPin24_IN2_Door 			: std_logic := '0';
signal oDoor_IN1 			: std_logic := '0';
signal oDoor_IN2 			: std_logic := '0';
signal iRFID_Data_IN 			: std_logic := '0';
signal oRFID_CLK 			: std_logic := '0';
signal oRFID_Data_OUT 			: std_logic := '0';
signal iPin36_RFID_Reset_N 		: std_logic := '0';
signal iPin44_RFID_S0 			: std_logic := '0';
signal iPin45_RFID_S1 			: std_logic := '0';
signal iPin46_RFID_S2 			: std_logic := '0';
signal iPin47_RFID_EN_N 			: std_logic := '0';
signal oPin48_CartridgeSW1 		: std_logic := '0';
signal oPin49_CartridgeSW2 		: std_logic := '0';
signal oPin50_CartridgeSW3 		: std_logic := '0';
signal oPin51_CartridgeSW4 		: std_logic := '0';
signal oPin43_CartridgeSW5 		: std_logic := '0';
signal iPin42_WhiteLED_ENA_N 	: std_logic := '0';
signal iPin41_Home 				: std_logic := '0';
--signal iPin40_Position_Cntrl1 				: std_logic := '0';
signal iPin39_GO 			: std_logic := '0';
signal iPin38_Pulsing_Mode 		: std_logic := '0';
signal iPin37_Micro_Online 		: std_logic := '0';
--signal iPin35_Position_Cntrl2 			: std_logic := '0';
signal oPin34_ACK 			: std_logic := '0';
signal oPin32_DONE 			: std_logic := '0';
signal iPin12_Select_SlideRel 			: std_logic := '0';
signal iSlideRel_ENCA 				: std_logic := '0';
signal iSlideRel_ENCB 				: std_logic := '0';
signal ioSCL 				: std_logic := '0';
signal ioSDA 				: std_logic := '0';
signal iSlideRel_ENCX 				: std_logic := '0';
--signal iDoor_ENCA 				: std_logic := '0';
--signal iDoor_ENCB 				: std_logic := '0';
--signal iDoor_ENCX 				: std_logic := '0';

--RFID Module
signal iCartridge1 				: std_logic := '0';
signal iCartridge2 				: std_logic := '0';
signal iCartridge3 				: std_logic := '0';
signal iCartridge4 				: std_logic := '0';
signal iCartridge5 				: std_logic := '0';
signal oAna_Mux_Sel2 			: std_logic := '0';
signal oAna_Mux_Sel1 			: std_logic := '0';
signal oAna_Mux_Sel0 			: std_logic := '0';
signal oMux_ENA_N 				: std_logic := '0';
signal oRFID_RST_N 				: std_logic := '0';
signal sSPI_Data_To_Main 		: std_logic := '0';
signal sSPI_Data_From_Main 		: std_logic := '0';
signal sSPI_Clk 				: std_logic := '0';


signal Test_Number 				: natural := 0; 
signal Marker 					: std_logic := '0';

constant cOSC_25MHz_period		: time := 40 ns;

constant cRot_Encoder_period		: time := 400 us;

signal test_counter		:  STD_LOGIC_VECTOR (9 downto 0) := (others => '0');




BEGIN


HT_Main_P034_Rev6_Top_inst: HT_Main_P034_Rev6_Top 
	PORT MAP
	(
    iOSC_50MHz_P => iOSC_50MHz_P,
    iOSC_50MHz_N => iOSC_50MHz_N,
	
    oOSC_Enable => oOSC_Enable,
    oReset_N_ETH => oReset_N_ETH,
    iReset_N_Button => iReset_N_Button,
    iPin23_IN1_Door => test_counter(0),
    oRed_Ind_LED => oRed_Ind_LED,
    oSystem_ON => oSystem_ON,
    iFront_Panel_SW_ON => iFront_Panel_SW_ON,
    oReset_N => oReset_N,
    oErase_N => oErase_N,
    oWhite_LED_Pulse => oWhite_LED_Pulse,
    oFrame_Grab_Trig => oFrame_Grab_Trig,
    iEncoder_Trig => iEncoder_Trig,
    iPin11_Arduino => test_counter(0),
    iPin29_Arduino => test_counter(1),
--    oOut_To_Galil => oOut_To_Galil,
    iSlideRel_Home_Sens => iSlideRel_Home_Sens,
    oSlideRel_Step => oSlideRel_Step,
--    iRST_Ard_FromGalil_N => iRST_Ard_FromGalil_N,
--    iIn_From_Galil => iIn_From_Galil,
    iERASE_CMD => iERASE_CMD,
    oSlideRel_DIR => oSlideRel_DIR,
    oSlideRel_ENA => oSlideRel_ENA,
    iPin18_Arduino => test_counter(2),
--    iPin13_Arduino => iPin13_Arduino,
--    iPin31_Arduino => iPin31_Arduino,
--    iPin19_Arduino => iPin19_Arduino,
    oPreview_Cam_Trig => oPreview_Cam_Trig,
    oPreview_Top_Illumination => oPreview_Top_Illumination,
    oPreview_Bot_Illumination => oPreview_Bot_Illumination,
    oDoor_ENA => oDoor_ENA,
    iPin24_IN2_Door => test_counter(1),
    oDoor_IN1 => oDoor_IN1,
    oDoor_IN2 => oDoor_IN2,
    iRFID_Data_IN => sSPI_Data_To_Main,
    oRFID_CLK => sSPI_Clk,
    oRFID_Data_OUT => sSPI_Data_From_Main,
    iPin36_RFID_Reset_N => iPin36_RFID_Reset_N,
    iPin44_RFID_S0 => iPin44_RFID_S0,
    iPin45_RFID_S1 => iPin45_RFID_S1,
    iPin46_RFID_S2 => iPin46_RFID_S2,
    iPin47_RFID_EN_N => iPin47_RFID_EN_N,
    oPin48_CartridgeSW1 => oPin48_CartridgeSW1,
    oPin49_CartridgeSW2 => oPin49_CartridgeSW2,
    oPin50_CartridgeSW3 => oPin50_CartridgeSW3,
    oPin51_CartridgeSW4 => oPin51_CartridgeSW4,
    oPin43_CartridgeSW5 => oPin43_CartridgeSW5,
    iPin42_WhiteLED_ENA_N => iPin42_WhiteLED_ENA_N,
    iPin41_Home => iPin41_Home,
--    iPin40_Position_Cntrl1 => iPin40_Position_Cntrl1,
    iPin39_GO => iPin39_GO,
    iPin38_Pulsing_Mode => iPin38_Pulsing_Mode,
    iPin37_Micro_Online => iPin37_Micro_Online,
--    iPin35_Position_Cntrl2 => iPin35_Position_Cntrl2,
    oPin34_ACK => oPin34_ACK,
    oPin32_DONE => oPin32_DONE,
    iPin12_Select_SlideRel => iPin12_Select_SlideRel,
    iSlideRel_ENCA => iSlideRel_ENCA,
    iSlideRel_ENCB => iSlideRel_ENCB,
    ioSCL => ioSCL,
    ioSDA => ioSDA,
    iSlideRel_ENCX => iSlideRel_ENCX
--    iDoor_ENCA => iDoor_ENCA,
--	iDoor_ENCB => iDoor_ENCB,
--	iDoor_ENCX => iDoor_ENCX

    );


--HT_RFID_P029_Top_inst: HT_RFID_P029_Top 
	--PORT MAP
	--(
    --iCartridge1 => iCartridge1,
    --iCartridge2 => iCartridge2,
    --iCartridge3 => iCartridge3,      
    --iCartridge4 => iCartridge4,
    --iCartridge5 => iCartridge5,
    --oAna_Mux_Sel2 => oAna_Mux_Sel2,
    --oAna_Mux_Sel1 => oAna_Mux_Sel1,
    --oAna_Mux_Sel0 => oAna_Mux_Sel0,
    --oMux_ENA_N => oMux_ENA_N,
    --oRFID_RST_N => oRFID_RST_N,
    --iSPI_Data => sSPI_Data_From_Main,	 
	--iSPI_Clk => sSPI_Clk,
    --oSPI_Data => sSPI_Data_To_Main
    --);


iEncoder_process: process
begin 
	
	iSlideRel_ENCA <= '0';
	iSlideRel_ENCB <= '0';
	wait for cRot_Encoder_period/4;	
	
	iSlideRel_ENCA <= '1';
	iSlideRel_ENCB <= '0';
	wait for cRot_Encoder_period/4;

	iSlideRel_ENCA <= '1';
	iSlideRel_ENCB <= '1';
	wait for cRot_Encoder_period/4;
	
	iSlideRel_ENCA <= '0';
	iSlideRel_ENCB <= '1';
	wait for cRot_Encoder_period/4;
	
	test_counter <= test_counter + 1;
	
end process;


-- Stimulus process
stim_proc: process
begin
 
	iReset_N_Button <= '1';	  
	iFront_Panel_SW_ON <= '0';
	wait for 3 ms;

	iFront_Panel_SW_ON <= '1';
	wait for 10 ms;
	iFront_Panel_SW_ON <= '0';
	wait for 5 ms;
	iFront_Panel_SW_ON <= '1';
	wait for 700 ms; 
	iFront_Panel_SW_ON <= '0';
	wait for 10 ms;
	iFront_Panel_SW_ON <= '1';
	wait for 300 ms; 
	iFront_Panel_SW_ON <= '0';
	wait for 10 ms;
	iFront_Panel_SW_ON <= '1';
	wait for 500 ms;
	iFront_Panel_SW_ON <= '0';
	wait for 500 ms;  
	iFront_Panel_SW_ON <= '1';
	wait for 1000 ms;
	
	
	iReset_N_Button <= '0';	  
	wait for 300 ms;
	iReset_N_Button <= '1';	  
	
wait;
	
end process;


END;
