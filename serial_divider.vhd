----------------------------------------------------------------------------------------------------
-- FILE NAME: serial_divider.vhd
----------------------------------------------------------------------------------------------------
-- COPYRIGHT Huron Digital Pathology
----------------------------------------------------------------------------------------------------
-- Project:              OAK ASC SFL   
-- Board:                P036
-- FPGA:                 Lattice LCMXO2-1200HC-XTG100C
-- Designer:             Alfonso Ribes
-- Date of Origin:       Sept 11, 2019
--
----------------------------------------------------------------------------------------------------
--  General Description:
--  Divider function that produces result after a series of clocks (serially with low resource usage)
--




library ieee;
use ieee.std_logic_1164.all;
use ieee.std_logic_unsigned.all;
use IEEE.NUMERIC_STD.ALL;

library MACHXO2;
use MACHXO2.components.all;



entity serial_divider is

	generic
	(
	gInput_Width :					integer := 28	-- Maximum 64
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

end serial_divider;


architecture Behavioral of serial_divider is

constant cDone : integer := 8*gInput_Width;



signal bTwo_Count : 				std_logic := '0';
signal bPositive_Res : 				std_logic := '0';
signal bLoad_SubRes :				std_logic := '0';
signal bShift :						std_logic := '0';
signal bEnable_SubRes :				std_logic := '0';
signal bSubRes_Shiftin :			std_logic := '0';
signal bSubRes :					std_logic_vector(gInput_Width downto 0) := (others => '0');
signal bSubtractor_Res :			std_logic_vector(gInput_Width downto 0) := (others => '0');
signal bDivCnt : 					std_logic_vector(9 downto 0) := (others => '0'); -- 1 + RoundUP[LOG2(8*gInput_Width)]
signal bContinue_Div :				std_logic := '0';
signal bEnable_DivCnt :				std_logic := '0';
signal bDivRes_Shiftin :			std_logic := '0';
signal bDivRes :					std_logic_vector((2*gInput_Width)-1 downto 0) := (others => '0');
signal bStart_Pulse :				std_logic := '0';
signal bDen :						std_logic_vector(gInput_Width-1 downto 0) := (others => '0');
signal svRounded_WDR :				std_logic_vector(gInput_Width downto 0) := (others => '0');
signal sFinish_Pulse :				std_logic := '0';
signal sPreFinish_Pulse :			std_logic := '0';



begin



Main_Process : process (iMCLK)
begin
if rising_edge(iMCLK) then

	if (iStart_Pulse = '1') then
		bDen <= iDen;
	else
		null;
	end if;	
	
	bTwo_Count <= bDivCnt(1) and not(bDivCnt(0));
	
	if (bSubRes >= ('0' & bDen)) then
		bPositive_Res <= '1';
	else
		bPositive_Res <= '0';
	end if;

	bShift <= not(bDivCnt(1)) and bEnable_DivCnt and not(bDivCnt(0));
		
	bStart_Pulse <= iStart_Pulse;

	if (bEnable_SubRes = '1') then
		if (bStart_Pulse = '1') then
			bSubRes <= (others => '0');
		elsif (bLoad_SubRes = '1') then
			bSubRes <= bSubtractor_Res;
		else
			bSubRes <= bSubRes(gInput_Width-1 downto 0) & bSubRes_Shiftin;
		end if;
	else
		null;
	end if;
	
	bSubtractor_Res <= bSubRes - ('0' & bDen);	
	
	if (bEnable_DivCnt = '0') then
		bDivCnt <= (others => '0');
	else
		bDivCnt <= bDivCnt + 1;
	end if;

	if (bDivCnt = cDone) then
		bContinue_Div <= '0';	
	else
		bContinue_Div <= '1';
	end if;

	sPreFinish_Pulse <= not(bContinue_Div);
	sFinish_Pulse <= sPreFinish_Pulse;
	oFinish_Pulse <= sFinish_Pulse;

	bEnable_DivCnt <= (bEnable_DivCnt and bContinue_Div) or bStart_Pulse;

	bDivRes_Shiftin <= bPositive_Res;
	
	if ((bShift = '1') or (iStart_Pulse = '1')) then
		
		if (iStart_Pulse = '1') then
			bDivRes((2*gInput_Width)-1 downto (2*gInput_Width)-gInput_Width) <= iNum;
			bDivRes(gInput_Width-1 downto 0) <= (others => '0');
		else
			bDivRes <= bDivRes((2*gInput_Width)-2 downto 0) & bDivRes_Shiftin;
		end if;
	
	else
		null;
	end if;
	
	oDivRes <= bDivRes;
	
	svRounded_WDR <= bDivRes(2*gInput_Width-1 downto gInput_Width-1) + 1;

end if;	
end process Main_Process;



-- Start Asynchronous

bLoad_SubRes <= bTwo_Count and bPositive_Res;
bEnable_SubRes <= bStart_Pulse or bShift or bLoad_SubRes;
bSubRes_Shiftin <= bDivRes((2*gInput_Width)-1);
oWhole_DivRes <= svRounded_WDR(gInput_Width downto 1);

-- End Asynchronous


end Behavioral;
