		--************************************************************************************************
-- Top entity for AVR microcontroller (for synthesis) with JTAG OCD and DMAs
-- Version 0.5 (Version for Xilinx)
-- Designed by Ruslan Lepetenok 
-- Modified 31.05.2006
--************************************************************************************************

--************************************************************************************************
--Adapted for the Papilio FPGA development board. To learn more visit http://papilio.cc
--Gadget Factory Note: This project is currently configured for the Papilio One board Version 2.03 or greater. It assumes a 32Mhz oscillator and a ucf with a period of 31.25.
--*************************************************************************************************

library IEEE;
use IEEE.std_logic_1164.all;

use WORK.AVRuCPackage.all;
use WORK.AVR_uC_CompPack.all;

use WORK.SynthCtrlPack.all; -- Synthesis control

use WORK.XMemCompPack.all;  -- Xilinx RAM components  

use WORK.MemAccessCtrlPack.all;
use WORK.MemAccessCompPack.all;

library vn_lib;
use vn_lib.display_pack.all;
use vn_lib.vn_pack.all;

Library UNISIM;
use UNISIM.vcomponents.all;

entity Papilio_AVR8 is
    generic (
      TWO_COLOR_ONLY   : boolean := FALSE;
--      DIS_DESC_NAME    : string := "640x480_80x40x256"
--      DIS_DESC_NAME    : string := "800x600_100x50x256"
      DIS_DESC_NAME    : string := "1024x768_128x64x256"
--      DIS_DESC_NAME    : string := "1152x864_144x72x256"

--      DIS_DESC_NAME    : string := "640x480_80x30x128"
--      DIS_DESC_NAME    : string := "1024x768_128x48x128"
--      DIS_DESC_NAME    : string := "1152x864_144x54x128"
--      DIS_DESC_NAME    : string := "1280x1024_160x64x128"
--      DIS_DESC_NAME    : string := "1600x1200_200x75x128"
  );
  port(
	 nrst_i : in    std_logic;						--Uncomment this to connect reset to an external pushbutton. Must be defined in ucf.
	 clk    : in    std_logic;
	 porta  : inout std_logic_vector(7 downto 0);
--	 portb  : inout std_logic_vector(7 downto 0);
--	 portc  : inout std_logic_vector(7 downto 0);
--	 portd  : inout std_logic_vector(7 downto 0);
--	 porte  : inout std_logic_vector(7 downto 0);
--	 portf  : inout std_logic_vector(7 downto 0);

	-- UART 
	rxd    : in    std_logic;
	txd    : out   std_logic

	);

end Papilio_AVR8;

architecture Struct of Papilio_AVR8 is

function string2Dis(key : string) return Display_type is
variable ret : Display_type := DIS_1024x768_128x64x256;
begin
  if key = "640x480_80x40x256" then
    ret := DIS_640x480_80x40x256;
  elsif key = "800x600_100x50x256" then
    ret := DIS_800x600_100x50x256;
  elsif key = "1024x768_128x64x256" then
    ret := DIS_1024x768_128x64x256;
  elsif key = "1152x864_144x72x256" then
    ret := DIS_1152x864_144x72x256;
  elsif key = "640x480_80x30x128" then
    ret := DIS_640x480_80x30x128;
  elsif key = "1024x768_128x48x128" then
    ret := DIS_1024x768_128x48x128;
  elsif key = "1152x864_144x54x128" then
    ret := DIS_1152x864_144x54x128_FONT2; -- so we can demo the other font
--    ret := DIS_1152x864_144x54x128;
  elsif key = "1280x1024_160x64x128" then
    ret := DIS_1280x1024_160x64x128;
  elsif key = "1600x1200_200x75x128" then
    ret := DIS_1600x1200_200x75x128;
  else
  end if;
  return ret;
end function string2Dis;

constant DIS_DESC : Display_type := string2Dis(DIS_DESC_NAME);
  
-- Use these setting to control which peripherals you want to include with your custom AVR8 implementation.
constant CImplPORTA			            : boolean := FALSE;
constant CImplPORTB			            : boolean := FALSE;
constant CImplPORTC							    : boolean := FALSE;
constant CImplPORTD    			        : boolean := FALSE;
constant CImplPORTE      			      : boolean := FALSE;
constant CImplPORTF           			: boolean := FALSE;
constant CImplUART      			      : boolean := TRUE;	--AVR8 UART peripheral
constant CImplTmrCnt     					  : boolean := TRUE;	--AVR8 Timer
constant CImplpapilio_core_template : boolean := TRUE;	--An example User Core, use this template to make your own custom peripherals.


component XDM4Kx8	port(
	                    cp2       : in  std_logic;
						ce      : in  std_logic;
	                    address   : in  std_logic_vector(CDATAMEMSIZE downto 0); 
					    din       : in  std_logic_vector(7 downto 0);		                
					    dout      : out std_logic_vector(7 downto 0);
					    we        : in  std_logic
					   );
end component;

component XPM8Kx16 port(
	                  cp2     : in  std_logic;
					  ce      : in  std_logic;
	                  address : in  std_logic_vector(CPROGMEMSIZE downto 0); 
					  din     : in  std_logic_vector(15 downto 0);		                
					  dout    : out std_logic_vector(15 downto 0);
					  we     : in  std_logic
					  );
end component;

--COMPONENT DCM32to16
--PORT(
--	CLKIN_IN : IN std_logic;          
--	CLKFX_OUT : OUT std_logic;
--	CLKFX180_OUT : OUT std_logic;
--	CLKIN_IBUFG_OUT : OUT std_logic;
--	CLK0_OUT : OUT std_logic
--	);
--END COMPONENT;

-- ############################## Define Components for User Cores ##################################################

-- Example Core - core9 
--COMPONENT papilio_core_template
--PORT(
--			-- begin Signals required by AVR8 for this core, do not modify.
--			nReset 		: in  STD_LOGIC;
--			clk 			: in  STD_LOGIC;
--			adr 			: in  STD_LOGIC_VECTOR (15 downto 0);
--			dbus_in 		: in  STD_LOGIC_VECTOR (7 downto 0);
--			dbus_out 	: out  STD_LOGIC_VECTOR (7 downto 0);
--			iore 			: in  STD_LOGIC;
--			iowe 			: in  STD_LOGIC;
--			out_en		: out STD_LOGIC;
--			-- end Signals required by AVR8 for this core, do not modify.
--
--			--Define signals that you want to go in or out of the peripheral. These are usually going to be connected to extenal pins of the Papilio board.
--			--Two Output Signals
--			output_sig	: out std_logic_vector (1 downto 0);
--			
--			--Two Input Signals
--			input_sig		: in std_logic_vector (1 downto 0)
--	);
--END COMPONENT;

-- ###############################################################################################################

--signal	 porta  : std_logic_vector(7 downto 0);
signal	 portb  : std_logic_vector(7 downto 0);
signal	 portc  : std_logic_vector(7 downto 0);
signal	 portd  : std_logic_vector(7 downto 0);
signal	 porte  : std_logic_vector(7 downto 0);
signal	 portf  : std_logic_vector(7 downto 0);

-- ############################## Signals connected directly to the core ##########################################

signal core_cpuwait  : std_logic;                    

-- Program memory
signal core_pc   : std_logic_vector (15 downto 0); -- PROM address
signal core_inst : std_logic_vector (15 downto 0); -- PROM data

-- I/O registers
signal core_adr  : std_logic_vector (15 downto 0);
signal core_iore : std_logic;                    
signal core_iowe : std_logic;

-- Data memery
signal core_ramadr : std_logic_vector (15 downto 0);
signal core_ramre  : std_logic;
signal core_ramwe  : std_logic;

signal core_dbusin   : std_logic_vector (7 downto 0);
signal core_dbusout  : std_logic_vector (7 downto 0);

-- Interrupts
signal core_irqlines : std_logic_vector(22 downto 0);
signal core_irqack   : std_logic;
signal core_irqackad : std_logic_vector(4 downto 0);

-- ###############################################################################################################

-- ############################## Signals connected directly to the SRAM controller ###############################

signal ram_din       : std_logic_vector(7 downto 0);

-- ###############################################################################################################

-- ####################### Signals connected directly to the external multiplexer ################################
signal   io_port_out     : ext_mux_din_type;
signal   io_port_out_en  : ext_mux_en_type;
signal   ind_irq_ack     : std_logic_vector(core_irqlines'range);
-- ###############################################################################################################

-- ################################## Reset signals #############################################
signal core_ireset        : std_logic;

-- ##############################################################################################

-- Port signals
signal PortAReg : std_logic_vector(porta'range);
signal DDRAReg  : std_logic_vector(porta'range);

signal PortBReg : std_logic_vector(portb'range);
signal DDRBReg  : std_logic_vector(portb'range);

signal PortCReg : std_logic_vector(portc'range);
signal DDRCReg  : std_logic_vector(portc'range);

signal PortDReg : std_logic_vector(portd'range);
signal DDRDReg  : std_logic_vector(portd'range);

signal PortEReg : std_logic_vector(porte'range);
signal DDREReg  : std_logic_vector(porte'range);

signal PortFReg : std_logic_vector(portf'range);
signal DDRFReg  : std_logic_vector(portf'range);

-- Added for Synopsys compatibility
signal gnd   : std_logic;
signal vcc    : std_logic;

-- Sleep support
signal core_cp2  : std_logic; -- Global clock signal after gating(and global primitive)
signal sleep_en  : std_logic;

signal sleepi   : std_logic;
signal irqok    : std_logic;
signal globint  : std_logic;

signal nrst_clksw : std_logic; -- Separate reset for clock gating module 

-- Watchdog related signals
signal wdtmout 	  : std_logic; -- Watchdog overflow
signal core_wdri  : std_logic; -- Watchdog clear

-- **********************  JTAG and memory **********************************************
-- PM address,data and control
signal pm_adr         : std_logic_vector(core_pc'range);
signal pm_h_we        : std_logic;
signal pm_l_we        : std_logic;
signal pm_din         : std_logic_vector(core_inst'range);

signal pm_dout        : std_logic_vector(core_inst'range);

signal TDO_Out        : std_logic;
signal TDO_OE         : std_logic;

signal JTAG_Rst       : std_logic;

-- **********************  JTAG and memory **********************************************

signal nrst_cp64m_tmp   : std_logic;

signal ram_cp2_n        : std_logic;

signal sleep_mode       : std_logic; 

-- "EEPROM" related signals
signal EEPrgSel : std_logic; 
signal EEAdr    : std_logic_vector(11 downto 0);
signal EEWrData : std_logic_vector(7 downto 0);
signal EERdData : std_logic_vector(7 downto 0);
signal EEWr     : std_logic; 


-- New
signal busmin   : MastersOutBus_Type;                            
signal busmwait : std_logic_vector(CNumOfBusMasters-1 downto 0) := (others => '0'); 

signal slv_outs : SlavesOutBus_Type;

signal ram_sel  : std_logic;

-- UART DMA
signal udma_mack    : std_logic;


signal mem_mux_out   : std_logic_vector (7 downto 0);

-- Place Holder Signals for JTAG instead of connecting them externally
signal TRSTn         : std_logic;     
signal TMS         : std_logic;     
signal TCK          : std_logic;      
signal TDI           : std_logic;
signal TDO           : std_logic;

-- AES

signal aes_mack         : std_logic;        


-- Address decoder
signal stb_IO        : std_logic;   
signal stb_IOmod     : std_logic_vector (CNumOfSlaves-1 downto 0);

signal ram_ce      	 : std_logic;

signal slv_cpuwait   : std_logic;

-- Memory i/f
signal mem_ramadr       : std_logic_vector (15 downto 0);  
signal mem_ram_dbus_in  : std_logic_vector (7 downto 0);
signal mem_ram_dbus_out : std_logic_vector (7 downto 0);
signal mem_ramwe        : std_logic;
signal mem_ramre        : std_logic;

-- RAM
signal ram_ramwe         : std_logic;

-- Clock generation/distribution
signal clk16M             : std_logic; 
signal clk32M             : std_logic; 
--signal clk16M180          : std_logic; 
--signal ibufgClk           : std_logic; 
signal vgaRst : std_logic;
signal vgaClk : std_logic;
signal vgaClk180 : std_logic;

-- nrst
signal nrst               : std_logic;  		--Comment this to connect reset to an external pushbutton.
signal rst_in             : std_logic;
signal rst16M             : std_logic;

-- ############################## Signals connected directly to the I/O registers ################################
-- PortA
signal porta_dbusout : std_logic_vector (7 downto 0);
signal porta_out_en  : std_logic;

-- PortB
signal portb_dbusout : std_logic_vector (7 downto 0);
signal portb_out_en  : std_logic;

-- PortC
signal portc_dbusout : std_logic_vector (7 downto 0);
signal portc_out_en  : std_logic;

-- PortD
signal portd_dbusout : std_logic_vector (7 downto 0);
signal portd_out_en  : std_logic;

-- PortE
signal porte_dbusout : std_logic_vector (7 downto 0);
signal porte_out_en  : std_logic;

-- PortF
signal portf_dbusout : std_logic_vector (7 downto 0);
signal portf_out_en  : std_logic;


-- Timer/Counter
signal tc_dbusout    : std_logic_vector (7 downto 0);
signal tc_out_en     : std_logic;

-- UART
signal uart_dbusout  : std_logic_vector (7 downto 0);
signal uart_out_en   : std_logic;


-- ###############################################################################################################

-- ############################## Define Signals for User Cores ##################################################
-- Example Core - - core9
--signal core9_input_sig : std_logic_vector(1 downto 0);		--Define a signal for the inputs.
signal core9_dbusout  : std_logic_vector (7 downto 0);
signal core9_out_en   : std_logic;

-- ###############################################################################################################

begin

-- Added for Synopsys compatibility
gnd <= '0';
vcc  <= '1';
-- Added for Synopsys compatibility	

--nrst <= '1';										--Comment this to connect reset to an external pushbutton.
rst_in <= not nrst_i;

--	Inst_DCM32to16: DCM32to16
--  PORT MAP(
--		CLKIN_IN => clk,
--		CLKFX_OUT => clk16M,
--		CLKFX180_OUT => clk16M180,
--		CLKIN_IBUFG_OUT => ibufgClk,
--		CLK0_OUT => clk32M
--	);

syscon_inst: entity vn_lib.syscon(structure)
  generic map (
          VGA_CLK_OUT_PERIOD => DIS_DESC.Vga.PixelClockPeriod
  )
  port map (
          sysClk_i       => clk,      -- system clock input
          rst_i          => rst_in,   -- external async. reset input
          clkVga_o       => vgaClk,   -- VGA pixel clock
          clkVga180_o    => vgaClk180,-- VGA pixel clock
          clk_o          => clk32M,   -- sysClk
          clk2x_o        => open,     -- sysClk * 2
          clkDiv2_o      => clk16M,   -- sysClk / 2 output
          rst_o          => rst16M,   -- sysClk domain reset
          vgaRst_o       => vgaRst,   -- VGA Pixel clock domain reset
          locked_o       => open);    -- DCM locked signal

nrst <= not rst16M;

core_inst <= pm_dout;

-- ******************  User Cores - Instantiate User Cores Here **************************

-- Example Core - core9 - This is an example of implenting a custom User core.
Inst_papilio_core_template:if CImplpapilio_core_template generate
  constant IO_BASE_ADDRESS : std_logic_vector (15 downto 0) := x"0FE0"; -- _SFR_IO8(0x0FE0) / _SFR_MEM8(0x1000)

  signal color     : std_logic_vector(2 downto 0);
  signal hSync     : std_logic;
  signal vSync     : std_logic;
  signal addrMatch : std_logic;
  signal we        : std_logic;
  signal stb       : std_logic;
  signal ack       : std_logic;
  signal lock      : std_logic;
  signal addr      : std_logic_vector(7 downto 0);
  
begin

vga_inst : entity vn_lib.vnVga(rtl)
  generic map(
    DIS_DESC          => DIS_DESC,
    TWO_COLOR_ONLY    => TWO_COLOR_ONLY
  )
  port map (
    rst_i       =>  rst16M,
    clk_i       =>  clk16M,
    vgaRst_i    =>  vgaRst,
    vgaClk_i    =>  vgaClk,
    -- wishbone bus
    we_i        =>  we,
    stb_i       =>  stb,
    ack_o       =>  ack,
    adr_i       =>  addr,
    dat_i       =>  core_dbusout,
    dat_o       =>  core9_dbusout,
    -- VGA Outputs
    color_o     =>  color,     -- Video color triplet (2=>Red, 1=>Green, 0=>Blue)
    hSync_o     =>  hSync,
    vSync_o     =>  vSync
    );   
    
  addrMatch    <= '1' when IO_BASE_ADDRESS(15 downto 5) =  core_adr(15 downto 5) else '0';
  stb          <= (core_iowe or core_iore) and addrMatch;
  we           <= core_iowe                and addrMatch;
  addr         <= "000" & core_adr(4 downto 0);
  core9_out_en <= ack and core_iore and addrMatch;
  
  porta(7) <= color(2);  -- Red
  porta(6) <= color(1);  -- Green
  porta(2) <= color(0);  -- Blue
  porta(1) <= hSync;
  porta(0) <= vSync;

  porta(3) <= '0';
  porta(4) <= '0';
  porta(5) <= '0';

-- Example Core - core9 connection to the external multiplexer
io_port_out(9) <= core9_dbusout;
io_port_out_en(9) <= core9_out_en;

--In order to avoid a conflict of the GPIO core and Example core both trying to drive outputs either disable PortA and PortB or uncomment the lines below that set the DDR Registers to make the pins inputs.
--DDRAReg(0)<='0';
--DDRAReg(1)<='0';
--DDRBReg(0)<='0';
--DDRBReg(1)<='0';
end generate;


-- ******************  END User Cores - Instantiate User Cores Here **************************


-- Unused IRQ lines
core_irqlines(7 downto 4) <= ( others => '0');
core_irqlines(3 downto 0) <= ( others => '0');
core_irqlines(13 downto 10) <= ( others => '0');
core_irqlines(16) <= '0';
core_irqlines(22 downto 20) <= ( others => '0');
-- ************************

-- Unused out_en
io_port_out_en(10 to 15) <= (others => '0');
io_port_out(10 to 15) <= (others => (others => '0'));

AVR_Core_Inst:component AVR_Core port map(
	--Clock and reset
		cp2      => core_cp2,
		cp2en    => vcc,
		ireset   => core_ireset,
	-- JTAG OCD support
		valid_instr => open,
		insert_nop  => gnd,
		block_irq   => gnd,
		change_flow => open,
	-- Program Memory
		pc       => core_pc,
		inst     => core_inst,
	-- I/O control
		adr      => core_adr,
		iore     => core_iore,
		iowe     => core_iowe,
	-- Data memory control
		ramadr   => core_ramadr,
		ramre    => core_ramre,
		ramwe    => core_ramwe,
		cpuwait  => core_cpuwait,
	-- Data paths
		dbusin   => core_dbusin,
		dbusout  => core_dbusout,
	-- Interrupts
		irqlines => core_irqlines, 
		irqack   => core_irqack,
		irqackad => core_irqackad, 
	--Sleep Control
		sleepi   => sleepi,
		irqok	   => irqok,
		globint  => globint,
	--Watchdog
		wdri	   => core_wdri);
										  

RAM_Data_Register:component RAMDataReg port map(	                   
               ireset      => core_ireset,
               cp2	       => clk16M, -- clk,
               cpuwait     => core_cpuwait,
			   RAMDataIn   => core_dbusout,
			   RAMDataOut  => ram_din
	                     );



EXT_MUX:component external_mux port map(
		  ramre              => mem_ramre,		   -- ramre output of the arbiter and multiplexor
		  dbus_out           => core_dbusin,       -- Data input of the core 
		  ram_data_out       => mem_mux_out,       -- Data output of the RAM mux(RAM or memory located I/O)
		  io_port_bus        => io_port_out,       -- Data outputs of the I/O
		  io_port_en_bus     => io_port_out_en,    -- Out enable outputs of I/O
		  irqack             => core_irqack,		  
		  irqackad			 => core_irqackad,
		  ind_irq_ack		 =>	ind_irq_ack		  -- Individual interrupt acknolege for the peripheral
                                            );


-- ******************  PORTA **************************				
PORTA_Impl:if CImplPORTA generate
PORTA_COMP:component pport  
	generic map(PPortNum => 0)
	port map(
	                   -- AVR Control
               ireset     => core_ireset,
               cp2	      => clk16M, -- clk,
               adr        => core_adr,
               dbus_in    => core_dbusout,
               dbus_out   => porta_dbusout,
               iore       => core_iore,
               iowe       => core_iowe,
               out_en     => porta_out_en,
			            -- External connection
			   portx      => PortAReg,
			   ddrx       => DDRAReg,
			   pinx       => porta);

-- PORTA connection to the external multiplexer
io_port_out(0) <= porta_dbusout;
io_port_out_en(0) <= porta_out_en;

-- Tri-state control for PORTA
PortAZCtrl:for i in porta'range generate
porta(i) <= PortAReg(i) when DDRAReg(i)='1' else 'Z'; 	
end generate;

end generate;

PORTA_Not_Impl:if not CImplPORTA generate
 porta <= (others => 'Z');	
end generate; 

-- ******************  PORTB **************************		
PORTB_Impl:if CImplPORTB generate
PORTB_COMP:component pport 
	generic map (PPortNum => 1)
	port map(
	                   -- AVR Control
               ireset     => core_ireset,
               cp2	      => clk16M, -- clk, 
               adr        => core_adr,
               dbus_in    => core_dbusout,
               dbus_out   => portb_dbusout,
               iore       => core_iore,
               iowe       => core_iowe,
               out_en     => portb_out_en,
			            -- External connection
			   portx      => PortBReg,
			   ddrx       => DDRBReg,
			   pinx       => portb);

-- PORTB connection to the external multiplexer
io_port_out(1) <= portb_dbusout;
io_port_out_en(1) <= portb_out_en;

-- Tri-state control for PORTB
PortBZCtrl:for i in portb'range generate
portb(i) <= PortBReg(i) when DDRBReg(i)='1' else 'Z'; 	
end generate;

end generate;

PORTB_Not_Impl:if not CImplPORTB generate
 portb <= (others => 'Z');	
end generate; 
	
-- ************************************************

-- ******************  PORTC **************************				
PORTC_Impl:if CImplPORTC generate
PORTC_COMP:component pport  
	generic map(PPortNum => 2)
	port map(
	                   -- AVR Control
               ireset     => core_ireset,
               cp2	      => clk16M, -- clk,
               adr        => core_adr,
               dbus_in    => core_dbusout,
               dbus_out   => portc_dbusout,
               iore       => core_iore,
               iowe       => core_iowe,
               out_en     => portc_out_en,
			            -- External connection
			   portx      => PortCReg,
			   ddrx       => DDRCReg,
			   pinx       => portc);

-- PORTC connection to the external multiplexer
io_port_out(5) <= portc_dbusout;
io_port_out_en(5) <= portc_out_en;

-- Tri-state control for PORTC
PortCZCtrl:for i in portc'range generate
portc(i) <= PortCReg(i) when DDRCReg(i)='1' else 'Z'; 	
end generate;

end generate;

PORTC_Not_Impl:if not CImplPORTC generate
 portc <= (others => 'Z');	
end generate; 

-- ******************  PORTD **************************		
PORTD_Impl:if CImplPORTD generate
PORTD_COMP:component pport 
	generic map (PPortNum => 3)
	port map(
	                   -- AVR Control
               ireset     => core_ireset,
               cp2	      => clk16M, -- clk, 
               adr        => core_adr,
               dbus_in    => core_dbusout,
               dbus_out   => portd_dbusout,
               iore       => core_iore,
               iowe       => core_iowe,
               out_en     => portd_out_en,
			            -- External connection
			   portx      => PortDReg,
			   ddrx       => DDRDReg,
			   pinx       => portd);

-- PORTD connection to the external multiplexer
io_port_out(6) <= portd_dbusout;
io_port_out_en(6) <= portd_out_en;

-- Tri-state control for PORTD
PortDZCtrl:for i in portd'range generate
portd(i) <= PortDReg(i) when DDRDReg(i)='1' else 'Z'; 	
end generate;

end generate;

PORTD_Not_Impl:if not CImplPORTD generate
 portd <= (others => 'Z');	
end generate; 
	
-- ************************************************

-- ******************  PORTE **************************				
PORTE_Impl:if CImplPORTE generate
PORTE_COMP:component pport  
	generic map(PPortNum => 4)
	port map(
	                   -- AVR Control
               ireset     => core_ireset,
               cp2	      => clk16M, -- clk,
               adr        => core_adr,
               dbus_in    => core_dbusout,
               dbus_out   => porte_dbusout,
               iore       => core_iore,
               iowe       => core_iowe,
               out_en     => porte_out_en,
			            -- External connection
			   portx      => PortEReg,
			   ddrx       => DDREReg,
			   pinx       => porte);

-- PORTE connection to the external multiplexer
io_port_out(7) <= porte_dbusout;
io_port_out_en(7) <= porte_out_en;

-- Tri-state control for PORTE
PortEZCtrl:for i in porte'range generate
porte(i) <= PortEReg(i) when DDREReg(i)='1' else 'Z'; 	
end generate;

end generate;

PORTE_Not_Impl:if not CImplPORTE generate
 porte <= (others => 'Z');	
end generate; 

-- ******************  PORTF **************************		
PORTF_Impl:if CImplPORTF generate
PORTF_COMP:component pport 
	generic map (PPortNum => 5)
	port map(
	                   -- AVR Control
               ireset     => core_ireset,
               cp2	      => clk16M, -- clk, 
               adr        => core_adr,
               dbus_in    => core_dbusout,
               dbus_out   => portf_dbusout,
               iore       => core_iore,
               iowe       => core_iowe,
               out_en     => portf_out_en,
			            -- External connection
			   portx      => PortFReg,
			   ddrx       => DDRFReg,
			   pinx       => portf);

-- PORTF connection to the external multiplexer
io_port_out(8) <= portf_dbusout;
io_port_out_en(8) <= portf_out_en;

-- Tri-state control for PORTF
PortFZCtrl:for i in portf'range generate
portf(i) <= PortFReg(i) when DDRFReg(i)='1' else 'Z'; 	
end generate;

end generate;

PORTF_Not_Impl:if not CImplPORTF generate
 portf <= (others => 'Z');	
end generate; 
	
-- ************************************************




--****************** Timer/Counter **************************
TmrCnt_Impl:if CImplTmrCnt generate
TmrCnt_Inst:component Timer_Counter port map(
	           -- AVR Control
               ireset     => core_ireset,
               cp2	      => clk16M, -- clk,
			   cp2en	  => vcc,
			   tmr_cp2en  => vcc,
			   stopped_mode   => gnd,
			   tmr_running    => gnd,
               adr        => core_adr,
               dbus_in    => core_dbusout,
               dbus_out   => tc_dbusout, 
               iore       => core_iore,
               iowe       => core_iowe,
               out_en     => tc_out_en,
			   -- External inputs/outputs
               EXT1           => gnd,
               EXT2           => gnd,
			   OC0_PWM0       => open,
			   OC1A_PWM1A     => open,
			   OC1B_PWM1B     => open,
			   OC2_PWM2       => open,
			   -- Interrupt related signals
               TC0OvfIRQ      => core_irqlines(15),  -- Timer/Counter0 overflow ($0020)
			   TC0OvfIRQ_Ack  => ind_irq_ack(15),
			   TC0CmpIRQ      => core_irqlines(14),  -- Timer/Counter0 Compare Match ($001E)
			   TC0CmpIRQ_Ack  => ind_irq_ack(14),
			   TC2OvfIRQ      => core_irqlines(9),	-- Timer/Counter2 overflow ($0014)
			   TC2OvfIRQ_Ack  => ind_irq_ack(9),
			   TC2CmpIRQ      => core_irqlines(8),	-- Timer/Counter2 Compare Match ($0012)
			   TC2CmpIRQ_Ack  => ind_irq_ack(8),
			   TC1OvfIRQ      => open,
			   TC1OvfIRQ_Ack  => gnd,
			   TC1CmpAIRQ     => open,
			   TC1CmpAIRQ_Ack => gnd,
			   TC1CmpBIRQ     => open,
			   TC1CmpBIRQ_Ack => gnd,
			   TC1ICIRQ       => open,
			   TC1ICIRQ_Ack   => gnd);

-- Timer/Counter connection to the external multiplexer							  
io_port_out(4)    <= tc_dbusout;
io_port_out_en(4) <= tc_out_en;
end generate;

-- Watchdog is not implemented
wdtmout <= '0';


-- Reset generator						 
ResetGenerator_Inst:component ResetGenerator port map(
	                            -- Clock inputs
								cp2	       => clk16M, -- clk,
								cp64m	   => gnd,
								-- Reset inputs
	                            nrst       => nrst,
								npwrrst    => vcc,
								wdovf      => wdtmout,
			                    jtagrst    => JTAG_Rst,
      							-- Reset outputs
					            nrst_cp2   => core_ireset,
			                    nrst_cp64m => nrst_cp64m_tmp,
								nrst_clksw => nrst_clksw
								);

						   
ClockGatingDis:if not CImplClockSw generate
 core_cp2 <=  clk16M;
end generate;

-- **********************  JTAG and memory **********************************************

ram_cp2_n <= not clk16M;

---- Data memory(8-bit)					   
DRAM_Inst:component XDM4Kx8 
port map(
	                    cp2       => ram_cp2_n,
						ce        => vcc,
	                    address   => mem_ramadr(CDATAMEMSIZE downto 0), 
					    din       => mem_ram_dbus_in, 
					    dout      => mem_ram_dbus_out, 
					    we        => ram_ramwe
					   );

-- Program memory					   
PM_Inst:component XPM8Kx16 
port map(
	                  cp2     => ram_cp2_n, 
					  ce      => vcc,
	                  address => pm_adr(CPROGMEMSIZE downto 0),
					  din     => pm_din,
					  dout    => pm_dout,
					  we     => pm_l_we
					  );  
					   					   
-- **********************  JTAG and memory **********************************************

-- Sleep mode is not implemented
sleep_mode <= '0';


JTAGOCDPrgTop_Inst:component JTAGOCDPrgTop port map(
	                      -- AVR Control
                          ireset       => core_ireset,
                          cp2	       => core_cp2,
						  -- JTAG related inputs/outputs
						  TRSTn        => TRSTn, -- Optional
	                      TMS          => TMS,
                          TCK	       => TCK,
                          TDI          => TDI,
                          TDO          => TDO_Out,
						  TDO_OE       => TDO_OE,
						  -- From the core
                          PC           => core_pc,
						  -- To the PM("Flash")
						  pm_adr       => pm_adr,
						  pm_h_we      => pm_h_we,
						  pm_l_we      => pm_l_we,
						  pm_dout      => pm_dout,
						  pm_din       => pm_din,
						  -- To the "EEPROM" 
						  EEPrgSel     => EEPrgSel,
						  EEAdr        => EEAdr,
						  EEWrData     => EEWrData,
						  EERdData     => EERdData,
						  EEWr         => EEWr,
						  -- CPU reset
						  jtag_rst     => JTAG_Rst
                          );

-- JTAG OCD module connection to the external multiplexer
io_port_out(3) <= (others => '0');
io_port_out_en(3) <= gnd;						  
						  
TDO <= TDO_Out when TDO_OE='1' else 'Z'; 						  

-- *******************************************************************************************************	
-- DMA, Memory decoder, ...
-- *******************************************************************************************************	


uart_Inst:component uart port map(
	                -- AVR Control
                    ireset     => core_ireset,
                    cp2	       => core_cp2,
                    adr        => core_adr,
                    dbus_in    => core_dbusout,
                    dbus_out   => uart_dbusout,
                    iore       => core_iore,
                    iowe       => core_iowe,
                    out_en     => uart_out_en,
                    -- UART
                    rxd        => rxd,
                    rx_en      => open,
                    txd        => txd,
                    tx_en      => open,
                    -- IRQ
                    txcirq     => core_irqlines(19),
                    txc_irqack => ind_irq_ack(19),
                    udreirq    => core_irqlines(18),
			        rxcirq     => core_irqlines(17)
		            );


-- UART connection to the external multiplexer							  
io_port_out(2)    <= uart_dbusout;
io_port_out_en(2) <= uart_out_en;


-- Arbiter and mux
ArbiterAndMux_Inst:component ArbiterAndMux port map(
                        --Clock and reset
						ireset      => core_ireset,
						cp2         => core_cp2,
					    -- Bus masters
                        busmin		=> busmin,
						busmwait	=> busmwait,
						-- Memory Address,Data and Control
						ramadr     => mem_ramadr,
						ramdout    => mem_ram_dbus_in,
                        ramre      => mem_ramre,
                        ramwe      => mem_ramwe,
						cpuwait    => slv_cpuwait
						);

-- cpuwait 
slv_cpuwait <= '0';
						
-- Core connection						
busmin(0).ramadr <= core_ramadr; 						
busmin(0).dout   <=	ram_din; -- !!!
busmin(0).ramre  <=	core_ramre;
busmin(0).ramwe  <=	core_ramwe;				
core_cpuwait     <=	busmwait(0);

-- UART DMA connection						
busmin(1).ramadr <= (others => '0'); 						
busmin(1).dout   <=	(others => '0'); -- !!!
busmin(1).ramre  <=	gnd;
busmin(1).ramwe  <=	gnd;				
udma_mack        <=  not busmwait(1);

-- AES DMA connection
busmin(2).ramadr <= (others => '0');		
busmin(2).dout   <=	(others => '0');
busmin(2).ramre  <=	gnd;
busmin(2).ramwe  <=	gnd;
aes_mack         <=  not busmwait(2);

-- UART DMA slave part
slv_outs(0).dout    <= (others => '0');
slv_outs(0).out_en 	<= gnd;	

-- AES DMA slave part
slv_outs(1).dout    <= (others => '0');
slv_outs(1).out_en 	<= gnd;	


-- Memory read mux
MemRdMux_inst:component MemRdMux port map(
	                    slv_outs  =>  slv_outs,
						ram_sel   =>  ram_sel,    -- Data RAM selection(optional input)
	                    ram_dout  =>  mem_ram_dbus_out,            -- Data memory output (From RAM)
						dout      =>  mem_mux_out -- Data output (To the core and other bus masters)
						);



-- Address decoder
RAMAdrDcd_Inst:component RAMAdrDcd port map(
                         ramadr    => mem_ramadr, 
		                 ramre     => mem_ramre,
		                 ramwe     => mem_ramwe,
		                 -- Memory mapped I/O i/f
		                 stb_IO	   => stb_IO,
		                 stb_IOmod => stb_IOmod,
	                     -- Data memory i/f
		                 ram_we    => ram_ramwe,
		                 ram_ce    => ram_ce,
						 ram_sel   => ram_sel
		                );

end Struct;
