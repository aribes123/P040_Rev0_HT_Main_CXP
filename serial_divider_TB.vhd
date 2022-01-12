----------------------------------------------------------------------------------------------------
-- FILE NAME: serial_divider_TB.vhd
----------------------------------------------------------------------------------------------------
-- COPYRIGHT Huron Digital Pathology
----------------------------------------------------------------------------------------------------
-- Project:              OAK-ASC-SFL   
-- Board:                P036
-- FPGA:                 Lattice LCMXO2-1200HC-XTG100C
-- Designer:             Alfonso Ribes
-- Date of Origin:       Sept 11, 2019
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

 
ENTITY serial_divider_TB IS
END serial_divider_TB;
 
ARCHITECTURE behavior OF serial_divider_TB IS 
 
-- Component Declaration for the Unit Under Test (UUT)
 
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


-- Constants
constant gInput_Width	: integer := 32;

--Inputs
signal iMCLK 			: std_logic := '0';
signal iStart_Pulse 	: std_logic := '0';
signal iNum 			: std_logic_vector(gInput_Width-1 downto 0) := (others => '0');
signal iDen 			: std_logic_vector(gInput_Width-1 downto 0) := (others => '0');
signal oDivRes 			: std_logic_vector((2*gInput_Width)-1 downto 0) := (others => '0');
signal oWhole_DivRes 	: std_logic_vector(gInput_Width-1 downto 0) := (others => '0');
signal oFinish_Pulse 	: std_logic := '0';


-- Clock period definitions
constant cMCLK_Period 		: time := 40 ns; -- 25MHz


signal Test_Number 				: natural := 0; 
signal Marker 					: std_logic := '0';





BEGIN
 

serial_divider_uut: serial_divider 
	PORT MAP
	(
    iMCLK => iMCLK,
    iStart_Pulse => iStart_Pulse,
    iNum => iNum,      
    iDen => iDen,
    oDivRes => oDivRes,
    oWhole_DivRes => oWhole_DivRes,
    oFinish_Pulse => oFinish_Pulse
    );


-- 10kHz SPI clock
iMCLK_process: process
begin
	iMCLK <= '0';
	wait for cMCLK_Period/2;
	iMCLK <= '1';
	wait for cMCLK_Period/2;
end process;


-- Stimulus process
stim_proc: process
begin	
	
	iNum <= std_logic_vector(to_unsigned(15258789, iNum'length));
	iDen <= std_logic_vector(to_unsigned(3907, iDen'length));
	iStart_Pulse <= '0';
	
	wait for cMCLK_Period*10;
	iStart_Pulse <= '1';
	wait for cMCLK_Period*2;
	iStart_Pulse <= '0';

	wait;
	
end process;


END;
