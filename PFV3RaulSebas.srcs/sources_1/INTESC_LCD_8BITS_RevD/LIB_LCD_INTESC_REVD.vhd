---------------------------------------


library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.numeric_std.ALL;
use IEEE.std_logic_unsigned.ALL;






USE WORK.COMANDOS_LCD_REVD.ALL;

entity LIB_LCD_INTESC_REVD is

GENERIC(
			FPGA_CLK : INTEGER := 100_000_000
);


PORT(CLK: IN STD_LOGIC;

-----------------------------------------------------
------------------PUERTOS DE LA LCD------------------
	  RS 		  : OUT STD_LOGIC;							--
	  RW		  : OUT STD_LOGIC;							--
	  ENA 	  : OUT STD_LOGIC;							--
	  DATA_LCD : OUT STD_LOGIC_VECTOR(7 DOWNTO 0);   --
-----------------------------------------------------
-----------------------------------------------------
	  
	  
-----------------------------------------------------------
--------------Temperatura--------------------	
      OS,botonCambioPassword: IN STD_LOGIC;
      SDA, SCL : INOUT STD_LOGIC;
      RDY_O,ERR_O: OUT STD_LOGIC;
--    temp_o: OUT STD_LOGIC_VECTOR(15 DOWNTO 0);
       teste : OUT STD_LOGIC_VECTOR(15 DOWNTO 0);
      
    
-----------------------------------------------------------
-----------------------------------------------------------

-----------------------------------------------------------
--------------Seguridad--------------------	
     
      inputPassword:  STD_LOGIC_VECTOR(3 DOWNTO 0);
     
     
     
     cambioPassword: STD_LOGIC_VECTOR(3 DOWNTO 0); 
    
-----------------------------------------------------------
-----------------------------------------------------------

--------------SERVO --------------------	
     
  PWM1, PWM2, PWM3, PWM4 :  out  STD_LOGIC--terminal donde sale la señal de PWM
-----------------------------------------------------------
-----------------------------------------------------------

	  );

end LIB_LCD_INTESC_REVD;

architecture Behavioral of LIB_LCD_INTESC_REVD is


CONSTANT NUM_INSTRUCCIONES : INTEGER := 38; 	--INDICAR EL N?MERO DE INSTRUCCIONES PARA LA LCD


--------------------------------------------------------------------------------
-------------------------SE?ALES DE LA LCD (NO BORRAR)--------------------------
																										--
component PROCESADOR_LCD_REVD is																--
																										--
GENERIC(																								--
			FPGA_CLK : INTEGER := 50_000_000;												--
			NUM_INST : INTEGER := 38																--
);																										--
																										--
PORT( CLK 				 : IN  STD_LOGIC;														--
	   VECTOR_MEM 		 : IN  STD_LOGIC_VECTOR(8  DOWNTO 0);							--
	   C1A,C2A,C3A,C4A : IN  STD_LOGIC_VECTOR(39 DOWNTO 0);							--
	   C5A,C6A,C7A,C8A : IN  STD_LOGIC_VECTOR(39 DOWNTO 0);							--
	   RS 				 : OUT STD_LOGIC;														--
	   RW 				 : OUT STD_LOGIC;														--
	   ENA 				 : OUT STD_LOGIC;														--
	   BD_LCD 			 : OUT STD_LOGIC_VECTOR(7 DOWNTO 0);			         	--
	   DATA 				 : OUT STD_LOGIC_VECTOR(7 DOWNTO 0);							--
	   DIR_MEM 			 : OUT INTEGER RANGE 0 TO NUM_INSTRUCCIONES					--
	);																									--
																										--
end component PROCESADOR_LCD_REVD;															--
																										--
COMPONENT CARACTERES_ESPECIALES_REVD is													--
																										--
PORT( C1,C2,C3,C4 : OUT STD_LOGIC_VECTOR(39 DOWNTO 0);								--
		C5,C6,C7,C8 : OUT STD_LOGIC_VECTOR(39 DOWNTO 0)									--
	 );																								--
																										--
end COMPONENT CARACTERES_ESPECIALES_REVD;													--
                                              
component clk_wiz_0
port
 (-- Clock in ports
  -- Clock out ports
  clk_out1          : out    std_logic;
  -- Status and control signals
  reset             : in     std_logic;
  locked            : out    std_logic;
  clk_in1           : in     std_logic
 );
end component;
                                                          --
                                                                                                        
component one_shot is
    Port ( Din : in STD_LOGIC;
           clk : in STD_LOGIC;
           Qout : out STD_LOGIC);
end component;

component binary_bcd is
    generic(N: positive := 16);
    port(
        clk, reset: in std_logic;
        binary_in: in std_logic_vector(N-1 downto 0);
        bcd0, bcd1, bcd2, bcd3, bcd4: out std_logic_vector(3 downto 0)
    );
end component;

component BCDtoASCII is
  Port (num: in std_logic_vector(3 downto 0);
  asciinum: out std_logic_vector(7 downto 0 ) );
end component;

component TempSensorCtl is
	Generic (CLOCKFREQ : natural := 100); -- input CLK frequency in MHz
	Port (
		TMP_SCL : inout STD_LOGIC;
		TMP_SDA : inout STD_LOGIC;
      -- The Interrupt and Critical Temperature Signals
      -- from the ADT7420 Temperature Sensor are not used in this design
--		TMP_INT : in STD_LOGIC;
--		TMP_CT : in STD_LOGIC;		
		TEMP_O : out STD_LOGIC_VECTOR(12 downto 0); --12-bit two's complement temperature with sign bit
		RDY_O : out STD_LOGIC;	--'1' when there is a valid temperature reading on TEMP_O
		ERR_O : out STD_LOGIC; --'1' if communication error
		CLK_I : in STD_LOGIC;
		SRST_I : in STD_LOGIC
	);
end component;
                                                                                                        
CONSTANT CHAR1 : INTEGER := 1;																--
CONSTANT CHAR2 : INTEGER := 2;																--
CONSTANT CHAR3 : INTEGER := 3;																--
CONSTANT CHAR4 : INTEGER := 4;																--
CONSTANT CHAR5 : INTEGER := 5;																--
CONSTANT CHAR6 : INTEGER := 6;																--
CONSTANT CHAR7 : INTEGER := 7;																--
CONSTANT CHAR8 : INTEGER := 8;																--
																										--
type ram is array (0 to  NUM_INSTRUCCIONES) of std_logic_vector(8 downto 0); 	--
signal INST : ram := (others => (others => '0'));										--
																										--
signal blcd 			  : std_logic_vector(7 downto 0):= (others => '0');		--																										
signal vector_mem 	  : STD_LOGIC_VECTOR(8  DOWNTO 0) := (others => '0');		--
signal c1s,c2s,c3s,c4s : std_logic_vector(39 downto 0) := (others => '0');		--
signal c5s,c6s,c7s,c8s : std_logic_vector(39 downto 0) := (others => '0'); 	--
signal dir_mem 		  : integer range 0 to NUM_INSTRUCCIONES := 0;				

																										--
--------------------------------------------------------------------------------
--------------------------------------------------------------------------------


--------------------------------------------------------------------------------
---------------------------AGREGA TUS SE?ALES AQU?------------------------------

--------------------------------------------------------------------------------
--------------------------------------------------------------------------------
TYPE CONTROL IS(abierto, cerrado,   advertenciaalCerrar, advertenciaalAbrir, cambiodePassword,PasswordIncorrectoalCerrar,PasswordIncorrectoalAbrir);
SIGNAL state : CONTROL := abierto;
SIGNAL P1, P2, P3, P4, B1 : STD_LOGIC;
SIGNAL CH1, CH2, CH3, CH4, CH5, CH6, CH7, CH8, CH9, CH10, CH11, CH12, CH13, CH14, CH15,
       CH16, CH17, CH18, CH19, CH20, CH21, CH22, CH23, CH24, CH25, CH26, CH27, CH28,
       CH29, CH30, CH31, CH32 : std_logic_vector(7 downto 0);
signal temp : std_logic_vector(12 downto 0);
signal reset: std_logic;
signal temp_aux: std_logic_vector(12 downto 0);--salida del tempsensor
signal tautemp: integer:=625;-- constante para conseguir la temp_aux en celsius
signal punto: integer:=100;-- dividimos 
signal temp_unsigned: unsigned(23 downto 0);-- conserva la temp real en celsius pero unsigned
signal temp_celsiusb: std_logic_vector(15 downto 0);-- temp real 
signal bcd0, bcd1, bcd2, bcd3, bcd4:  std_logic_vector(3 downto 0);--temp en bcd
--signal temp_o: std_logic_vector(15 downto 0);
signal temp0ch,temp1ch,temp2ch,temp3ch: std_logic_vector(7 downto 0);
signal temp_registrada: std_logic_vector(15 downto 0):="0000000000000000";

constant a : STD_LOGIC_VECTOR(7 DOWNTO 0) := "01100001";   constant MA : STD_LOGIC_VECTOR(7 DOWNTO 0) := "01000001";
constant b : STD_LOGIC_VECTOR(7 DOWNTO 0) := "01100010";   constant MB : STD_LOGIC_VECTOR(7 DOWNTO 0) := "01000010";
constant c : STD_LOGIC_VECTOR(7 DOWNTO 0) := "01100011";   constant MC : STD_LOGIC_VECTOR(7 DOWNTO 0) := "01000011";
constant d : STD_LOGIC_VECTOR(7 DOWNTO 0) := "01100100";   constant MD : STD_LOGIC_VECTOR(7 DOWNTO 0) := "01000100";
constant ee :STD_LOGIC_VECTOR(7 DOWNTO 0) := "01100101";   constant ME : STD_LOGIC_VECTOR(7 DOWNTO 0) := "01000101";
constant f : STD_LOGIC_VECTOR(7 DOWNTO 0) := "01100110";   constant MF : STD_LOGIC_VECTOR(7 DOWNTO 0) := "01000110";
constant g : STD_LOGIC_VECTOR(7 DOWNTO 0) := "01100111";   constant MG : STD_LOGIC_VECTOR(7 DOWNTO 0) := "01000111";
constant h : STD_LOGIC_VECTOR(7 DOWNTO 0) := "01101000";   constant MH : STD_LOGIC_VECTOR(7 DOWNTO 0) := "01001000";
constant i : STD_LOGIC_VECTOR(7 DOWNTO 0) := "01101001";   constant MI : STD_LOGIC_VECTOR(7 DOWNTO 0) := "01001001";
constant j : STD_LOGIC_VECTOR(7 DOWNTO 0) := "01101010";   constant MJ : STD_LOGIC_VECTOR(7 DOWNTO 0) := "01001010";
constant k : STD_LOGIC_VECTOR(7 DOWNTO 0) := "01101011";   constant MK : STD_LOGIC_VECTOR(7 DOWNTO 0) := "01001011";
constant l : STD_LOGIC_VECTOR(7 DOWNTO 0) := "01101100";   constant ML : STD_LOGIC_VECTOR(7 DOWNTO 0) := "01001100";	
constant m : STD_LOGIC_VECTOR(7 DOWNTO 0) := "01101101";   constant MM : STD_LOGIC_VECTOR(7 DOWNTO 0) := "01001101";
constant n : STD_LOGIC_VECTOR(7 DOWNTO 0) := "01101110";   constant MN : STD_LOGIC_VECTOR(7 DOWNTO 0) := "01001110";
constant o : STD_LOGIC_VECTOR(7 DOWNTO 0) := "01101111";   constant MO : STD_LOGIC_VECTOR(7 DOWNTO 0) := "01001111";
constant p : STD_LOGIC_VECTOR(7 DOWNTO 0) := "01110000";   constant MP : STD_LOGIC_VECTOR(7 DOWNTO 0) := "01010000";
constant q : STD_LOGIC_VECTOR(7 DOWNTO 0) := "01110001";   constant MQ : STD_LOGIC_VECTOR(7 DOWNTO 0) := "01010001";
constant r : STD_LOGIC_VECTOR(7 DOWNTO 0) := "01110010";   constant MR : STD_LOGIC_VECTOR(7 DOWNTO 0) := "01010010";
constant s : STD_LOGIC_VECTOR(7 DOWNTO 0) := "01110011";   constant MS : STD_LOGIC_VECTOR(7 DOWNTO 0) := "01010011";
constant t : STD_LOGIC_VECTOR(7 DOWNTO 0) := "01110100";   constant MT : STD_LOGIC_VECTOR(7 DOWNTO 0) := "01010100";
constant u : STD_LOGIC_VECTOR(7 DOWNTO 0) := "01110101";   constant MU : STD_LOGIC_VECTOR(7 DOWNTO 0) := "01010101";
constant v : STD_LOGIC_VECTOR(7 DOWNTO 0) := "01110110";   constant MV : STD_LOGIC_VECTOR(7 DOWNTO 0) := "01010110";
constant y : STD_LOGIC_VECTOR(7 DOWNTO 0) := "01111001";   constant MY : STD_LOGIC_VECTOR(7 DOWNTO 0) := "01011001";
constant x : STD_LOGIC_VECTOR(7 DOWNTO 0) := "01111000";   constant MX : STD_LOGIC_VECTOR(7 DOWNTO 0) := "01011000";
constant z : STD_LOGIC_VECTOR(7 DOWNTO 0) := "01111010";   constant MZ : STD_LOGIC_VECTOR(7 DOWNTO 0) := "01011010";
constant w : STD_LOGIC_VECTOR(7 DOWNTO 0) := "01110111";

constant uno : STD_LOGIC_VECTOR(7 DOWNTO 0) := "00110001";
constant dos : STD_LOGIC_VECTOR(7 DOWNTO 0) := "00110010";
constant tres : STD_LOGIC_VECTOR(7 DOWNTO 0) := "00110011";
constant cuatro : STD_LOGIC_VECTOR(7 DOWNTO 0) := "00110100";
constant cinco : STD_LOGIC_VECTOR(7 DOWNTO 0) := "00110101";
constant seis : STD_LOGIC_VECTOR(7 DOWNTO 0) := "00110110";
constant siete : STD_LOGIC_VECTOR(7 DOWNTO 0) := "00110111";
constant ocho : STD_LOGIC_VECTOR(7 DOWNTO 0) := "00111000";
constant nueve : STD_LOGIC_VECTOR(7 DOWNTO 0) := "00111001";
constant cero : STD_LOGIC_VECTOR(7 DOWNTO 0) := "00110000";

constant espacio : STD_LOGIC_VECTOR(7 DOWNTO 0) := "00100000";
signal password : std_logic_vector (3 downto 0 ) :="0000";

--Señales para servos 
signal Contador :integer:=1;
signal Contador2 :integer:=1;
signal PWM_Count: integer range 1 to 500000;--500000;
signal selectorPosicion : STD_LOGIC_VECTOR (2 downto 0);--selecciona las 4 posiciones

signal Clock50 : std_logic;
signal locked :std_logic;
signal resett :std_logic;

--signal tempMax : std_logic_vector (15 downto 0):= "0000100000000000"; -- 20 grados
signal tempMax : std_logic_vector (15 downto 0):=   "0000100111000100"; -- 20 grados
signal bitcambiodePassword0 :  std_logic_vector (7 downto 0);
signal bitcambiodePassword1 :  std_logic_vector (7 downto 0);
signal bitcambiodePassword2 :  std_logic_vector (7 downto 0);
signal bitcambiodePassword3 :  std_logic_vector (7 downto 0);

signal counterincorrecto: integer range 0 to 10000001:=0;




--signal c1 : integer := 0;
--signal c2 : integer:= 0;
--signal c3 : integer:= 0;
--signal c4 : integer:= 0;
--signal rb : std_logic;
--signal templimite
--signal test : std_logic ;
--signal COUNT :  integer range 0 to 50000;
--signal testt1 : std_logic_vector (7 downto 0);
--signal testt2 : std_logic_vector (7 downto 0);
--signal testt3: std_logic_vector (7 downto 0);
--signal testt4 : std_logic_vector (7 downto 0);




--------------------------------------------------------------------------------
---------------------------Senales Servos------------------------------


subtype u20 is unsigned(19 downto 0);
signal counter      : u20 := x"00000";

constant clk_freq   : integer := 50_000_000;        -- Clock frequency in Hz (20 ns)
constant pwm_freq   : integer := 50;                -- PWM sigmal frequency in Hz (20 ms)
constant period     : integer := clk_freq/pwm_freq; -- Número de pulsos de reloj en un ciclo PWM
constant duty_cycle : integer := 125_000;           -- Número de pulsos de reloj para ciclo de trabajo
                                                    --  25,000 -> -90°, 50,000 -> -45°, 75,000 -> 0°
  

signal pwm_counter  : std_logic := '0';
signal stateHigh    : std_logic := '1';



signal posicion : integer := 0;
signal mult : integer := 0;
  
  signal xsignal : std_logic;
--------------------------------------------------------------------------------
--------------------------------------------------------------------------------
begin



Clock50_MHZ : clk_wiz_0
   port map ( 
  -- Clock out ports  
   clk_out1 => Clock50,
  -- Status and control signals                
   reset => resett,
   locked => locked,
   -- Clock in ports
   clk_in1 => CLK
 );
 
---------------------------------------------------------------
-------------------COMPONENTES PARA LCD------------------------
																				 --
u1: PROCESADOR_LCD_REVD													 --
GENERIC map( FPGA_CLK => FPGA_CLK,									 --
				 NUM_INST => NUM_INSTRUCCIONES )						 --
																				 --
PORT map( CLK,VECTOR_MEM,C1S,C2S,C3S,C4S,C5S,C6S,C7S,C8S,RS, --
			 RW,ENA,BLCD,DATA_LCD, DIR_MEM );						 --
																				 --
U2 : CARACTERES_ESPECIALES_REVD 										 --
PORT MAP( C1S,C2S,C3S,C4S,C5S,C6S,C7S,C8S );				 		 --
																				 --
VECTOR_MEM <= INST(DIR_MEM);											 --
																				 --
---------------------------------------------------------------
---------------------------------------------------------------
OS1 : one_shot port map (Din => OS, clk => clk, Qout => B1);
TC1: TempSensorCtl port map (TMP_SCL => SCL, TMP_SDA => SDA, TEMP_O => temp, RDY_O => RDY_O,
                             ERR_O => ERR_O, CLK_I => clk, SRST_I => '0');
                             
                             
               temp_unsigned<=(to_unsigned(natural(tautemp),12)*unsigned(temp(11 downto 0)))/(to_unsigned(natural(punto),12));
                                                         
                                                         temp_celsiusb<= std_logic_vector(temp_unsigned(15 downto 0));
                                                         teste<=temp_celsiusb;
                                                         bcdtemp: binary_bcd
                                                         generic map (N => 16)
                                                         port map (
                                                         clk =>clk,
                                                         reset=>reset,
                                                         binary_in=>temp_celsiusb,
                                                         bcd0=>bcd0,
                                                         bcd1=>bcd1,
                                                         bcd2=>bcd2,
                                                         bcd3=> bcd3,
                                                         bcd4=> bcd4);
                                                         
                                                         
--                                                         temp_o<=bcd3&bcd2&bcd1&bcd0;
                                                         
                            temp0ascii: BCDtoASCII port map(bcd0, temp0ch);
                            temp1ascii: BCDtoASCII port map(bcd1, temp1ch);
                            temp2ascii: BCDtoASCII port map(bcd2, temp2ch);
                            temp3ascii: BCDtoASCII port map(bcd3, temp3ch);
                

                INST(0) <= LCD_INI("10");
                INST(1) <= BUCLE_INI(1);
                INST(2) <= POS(1,1);
                INST(3) <= CHAR_ASCII(CH1);
                INST(4) <= CHAR_ASCII(CH2);
                INST(5) <= CHAR_ASCII(CH3);
                INST(6) <= CHAR_ASCII(CH4);
                INST(7) <= CHAR_ASCII(CH5);
                INST(8) <= CHAR_ASCII(CH6);
                INST(9) <= CHAR_ASCII(CH7);
                INST(10) <= CHAR_ASCII(CH8);
                INST(11) <= CHAR_ASCII(CH9);
                INST(12) <= CHAR_ASCII(CH10);
                INST(13) <= CHAR_ASCII(CH11);
                INST(14) <= CHAR_ASCII(CH12);
                INST(15) <= CHAR_ASCII(CH13);
                INST(16) <= CHAR_ASCII(CH14);
                INST(17) <= CHAR_ASCII(CH15);
                INST(18) <= CHAR_ASCII(CH16);
                INST(19) <= POS(2,1);
                INST(20) <= CHAR_ASCII(CH17);
                INST(21) <= CHAR_ASCII(CH18);
                INST(22) <= CHAR_ASCII(CH19);
                INST(23) <= CHAR_ASCII(CH20);
                INST(24) <= CHAR_ASCII(CH21);
                INST(25) <= CHAR_ASCII(CH22);
                INST(26) <= CHAR_ASCII(CH23);
                INST(27) <= CHAR_ASCII(CH24);
                INST(28) <= CHAR_ASCII(CH25);
                INST(29) <= CHAR_ASCII(CH26);
                INST(30) <= CHAR_ASCII(CH27);
                INST(31) <= CHAR_ASCII(CH28);
                INST(32) <= CHAR_ASCII(CH29);
                INST(33) <= CHAR_ASCII(CH30);
                INST(34) <= CHAR_ASCII(CH31);
                                INST(35) <= CHAR_ASCII(CH32);
                                INST(36) <= BUCLE_FIN(1);
                                INST(37) <= CODIGO_FIN(1);
                                
                                
                                
--         process(Clock50)
--         begin
         
--         if (Clock50'event and Clock50 = '1') then
--            if c1<100000000 and c2=0 then
--            c1 <= c1 + 1;
--            rb<='0';
--             elsif c1=100000000 then
--            c1 <= c1 + 1;
--             rb<='1';
            
--             elsif c1=100000001 then
--             c2<=3;
--             c1<=0;
--             rb<='0';
             
              
--             end if;  
--             end if;         
--         end process;
         

        process(Clock50)
        
        
        
      
        
        
        
                    VARIABLE char  :  INTEGER RANGE 0 TO 31 := 0;   
                    VARIABLE char2  :  INTEGER RANGE 0 TO 31 := 0;
                    constant pos1: integer := 75000;  --representa a 1.5ms = 0° - -90
                    constant reposo1: integer := 1000000;  --representa a 1.5ms = 0° - -90
                     constant total1: integer := 10000000;  --representa a 1.5ms = 0° - -90
                   Variable counterincorrecto: integer range 0 to 100000001:=0;
                    
                    
                    constant pos2: integer := 125000;  --representa a 2.5 ms = 90 grados 
                    constant total2: integer := 1000000;  --representa a 1.5ms = 0° - -90  
            begin
            
            
                                                                    
--              COUNT <= COUNT + 1;
--              if COUNT>1000  and COUNT>1002then
--              state<=cerrado;
--              else
--              testt1<=temp3ch;
--              testt2<=temp2ch;
--              testt3<=temp1ch;
--              testt4<=temp0ch;
--              end if;
                                                               
                                       
            if (Clock50'event and Clock50 = '1') then
                                
         
               case state is

                                            when abierto => 
                                            
                         
                                          CH1 <= MA;--S
                                           CH2 <= c;--i
                                           CH3 <= ee;--s
                                           CH4 <= r;--t
                                           CH5 <= c;--e
                                           CH6 <= a;--temp3ch;--temp en bcd 3
                                           CH7 <= r;--temp2ch;--temp en bcd 2
                                           CH8 <= espacio;--punto decimal
--                                           CH10 <= temp1ch;--temp1ch;--temp en bcd 1
--                                           CH11 <= temp0ch;--temp0ch;--temp en bcd 0
--                                           CH6 <= MS;--m
 --                                            CH7 <=y;--a
 --                                           -- CH9 <= "00101110";--punto decimal
--                                            CH8 <=s ;--spacio
                                            CH9 <= espacio;--A
                                             CH10 <= espacio;--b
                                           CH11 <= espacio;--i
                                           CH12 <= espacio;--e       
                                           CH13 <= espacio;--r
                                           CH14 <=  espacio;--t
                                           CH15 <= espacio;--o
                                           CH16 <=espacio;--spacio
                                           CH17 <=MS;--I
                                           CH18 <= u;--n
                                           CH19 <= espacio;--g
                                            CH20 <=MD;--r
                                           CH21 <=  ee;--e   
                                            CH22 <= d;--s   
                                             CH23 <= o;--e   
                                              CH24 <=espacio;--spacio 
                                           CH25 <= espacio;--P   
                                           CH26 <= espacio;--i
                                           CH27 <=  MT;--n
                                           CH28 <="00111010";--spacio
                                           CH29 <=temp3ch;--T
                                           CH30 <=temp2ch;--: 
                                           CH31 <=  "00101110";--
                                           CH32 <= temp1ch;--spacio
                                  
                                                                      
                                          
                                          
                        
                 if(botonCambioPassword = '1') then
                 state<= cambiodePassword;
                 end if;
                 
                   if( B1= '1')  and (inputPassword = password)
                    then   
                     if (temp_celsiusb<tempMax) then ---22 grados
                     state<=cerrado;
                     elsif (temp_celsiusb>tempMax) then --22
                     state<=advertenciaalCerrar;
                     end if;
                     elsif  ( B1= '1') and (inputPassword /= password) then 
                    state<= PasswordIncorrectoalCerrar; 
                     end if;
                      counterincorrecto:=1; 
                                                                 xsignal <= '1';
                     
                     
  when PasswordIncorrectoalAbrir =>
                                                                         
                                                                                                      CH1 <= MP;--S
                                                                                                      CH2 <= a;--i
                                                                                                      CH3 <= s;--s
                                                                                                      CH4 <= s;--t
                                                                                                      CH5 <= w;--e
                                                           --                                           
                                                                                                      CH6 <= o;--m
                                                                                                      CH7 <=r;--m
                                                                                                      CH8 <= d;--m
                                                                                                        CH9 <=espacio;--a
                                                                                                      
                                                                                                       CH10 <=espacio ;--spacio
                                                                                                        CH11 <= espacio;--A
                                                                                                        CH12 <= espacio;--b
                                                                                                      CH13 <= espacio;--i
                                                                                                      CH14 <= espacio;--e       
                                                                                                      CH15 <= espacio;--r
                                                                                                      CH16 <=  espacio;--t
                                                                                                      
                                                                                                      CH17 <=MI;--I
                                                                                                      CH18 <= n;--n
                                                                                                      CH19 <= c;--g
                                                                                                       CH20 <=o;--r
                                                                                                      CH21 <=  r;--e   
                                                                                                       CH22 <= r;--s   
                                                                                                        CH23 <= ee;--e   
                                                                                                         CH24 <=c;--spacio 
                                                                                                      CH25 <= t;--P   
                                                                                                      CH26 <= o;--i
                                                                                                      CH27 <=  espacio;--n
                                                                                                      CH28 <=espacio;--spacio
                                                                                                      CH29 <=espacio;--T
                                                                                                      CH30 <=espacio;--: 
                                                                                                      CH31 <=  espacio;--
                                                                                                      CH32 <= espacio;--spacio
                                                                                       counterincorrecto:=counterincorrecto + 1;               
                                                                                   if counterincorrecto>100000000 then 
                                                                                   state<=cerrado;   
                                                                                     end if; 
                                                                                     
                                                                                     
                                                        
                                                                                        
 when PasswordIncorrectoalCerrar =>
                                                     
                                                                                  CH1 <= ML;--S
                                                                                  CH2 <= i;--i
                                                                                  CH3 <= m;--s
                                                                                  CH4 <= i;--t
                                                                                  CH5 <= t;--e
                                       --                                           
                                                                                  CH6 <= ee;--m
                                                                                  CH7 <=espacio;--m
                                                                                  CH8 <= d;--m
                                                                                    CH9 <=ee;--a
                                                                                  
                                                                                   CH10 <=espacio ;--spacio
                                                                                    CH11 <= espacio;--A
                                                                                    CH12 <= espacio;--b
                                                                                  CH13 <= espacio;--i
                                                                                  CH14 <= espacio;--e       
                                                                                  CH15 <= espacio;--r
                                                                                  CH16 <=  espacio;--t
                                                                                  
                                                                                  CH17 <=MP;--I
                                                                                  CH18 <= ee;--n
                                                                                  CH19 <= r;--g
                                                                                   CH20 <=s;--r
                                                                                 CH21 <=  o;--e   
                                                                                    CH22 <= n;--s   
                                                                                     CH23 <= a;--e   
                                                                                     CH24 <=s;--spacio 
                                                                                  CH25 <= espacio;--P   
                                                                                       CH26 <= espacio;--i
                                                                                    CH27 <=  espacio;--n
                                                                        CH28 <=espacio;--spacio
                                                                  CH29 <=espacio;--T
                                                                  CH30 <=espacio;--: 
                                                                              CH31 <= espacio;--
                                                                                  CH32 <= espacio;--spacio
                                 
                                                         IF (B1='1') THEN
                        state<=abierto;
                        end if;
                         
                                                                              
                    when cerrado =>
                    
                                 
                                                             CH1 <= MP;--S
                                                              CH2 <= u;--i
                                                              CH3 <= ee;--s
                                                              CH4 <= d;--t
                                                              CH5 <= ee;--e
                   --                                           CH7 <= temp3ch;--temp3ch;--temp en bcd 3
                   --                                           CH8 <= temp2ch;--temp2ch;--temp en bcd 2
                   --                                           CH9 <= "00101110";--punto decimal
                   --                                           CH10 <= temp1ch;--temp1ch;--temp en bcd 1
                   --                                           CH11 <= temp0ch;--temp0ch;--temp en bcd 0
                                                              CH6 <= espacio;--m
                                                              CH7 <=a;--m
                                                              CH8 <= c;--m
                                                                CH9 <=c;--a
                                                               -- CH9 <= "00101110";--punto decimal
                                                               CH10 <=ee ;--spacio
                                                                CH11 <= s;--A
                                                                CH12 <= a;--b
                                                              CH13 <= r;--i
                                                              CH14 <= espacio;--e       
                                                              CH15 <= a;--r
                                                              CH16 <=  l;--t
                                                              
                                                              CH17 <=ME;--I
                                                              CH18 <= d;--n
                                                              CH19 <= i;--g
                                                               CH20 <=f;--r
                                                              CH21 <=  i;--e   
                                                               CH22 <=c;--s   
                                                                CH23 <= i;--e   
                                                                 CH24 <=o;--spacio 
                                                              CH25 <= espacio;--P   
                                                              CH26 <= espacio;--i
                                                              CH27 <=  MT;--n
                                                              CH28 <="00111010";--spacio
                                                              CH29 <=temp3ch;--T
                                                              CH30 <=temp2ch;--: 
                                                              CH31 <=  "00101110";--
                                                              CH32 <= temp1ch;--spacio
                                                              
                                                              
                                                                 
                                                                                                    
                                                                                                    
                                                                                                    
                                                                                                    
                                               if( B1= '1') and (inputPassword = password)  then 
                                               temp_registrada<=temp_celsiusb;
                                                 if (temp_registrada<tempMax) then  --22
                                                     state<=abierto;
                                                 elsif (temp_registrada>tempMax) then --22
                                                    state<=abierto;
                                                 end if;
                                               elsif  ( B1= '1') and (inputPassword /= password) then 
                                                  state<= PasswordIncorrectoalAbrir; 
                                                 
                                                 end if;
                                                  counterincorrecto:=1;
                                                                     xsignal <= '0'; 
                    when advertenciaalAbrir =>

                        
                                                  CH1 <= ME;--S
                                                              CH2 <= x;--i
                                                              CH3 <= c;--s
                                                              CH4 <= ee;--t
                                                              CH5 <= s;--e
                   --                                           CH7 <= temp3ch;--temp3ch;--temp en bcd 3
                   --                                           CH8 <= temp2ch;--temp2ch;--temp en bcd 2
                   --                                           CH9 <= "00101110";--punto decimal
                   --                                           CH10 <= temp1ch;--temp1ch;--temp en bcd 1
                   --                                           CH11 <= temp0ch;--temp0ch;--temp en bcd 0
                                                              CH6 <= o;--m
                                                              CH7 <=espacio;--m
                                                              CH8 <= d;--m
                                                                CH9 <=ee;--a
                                                               -- CH9 <= "00101110";--punto decimal
                                                               CH10 <=espacio ;--spacio
                                                                CH11 <= espacio;--A
                                                                CH12 <= espacio;--b
                                                              CH13 <= espacio;--i
                                                              CH14 <= espacio;--e       
                                                              CH15 <= espacio;--r
                                                              CH16 <=  espacio;--t
                                                              
                                                              CH17 <=MT;--I
                                                              CH18 <= ee;--n
                                                              CH19 <= m;--g
                                                               CH20 <=p;--r
                                                              CH21 <=  ee;--e   
                                                               CH22 <=r;--s   
                                                                CH23 <= a;--e   
                                                                 CH24 <=t;--spacio 
                                                              CH25 <= u;--P   
                                                              CH26 <= r;--i
                                                              CH27 <=  a;--n
                                                              CH28 <="00111010";--spacio
                                                              CH29 <=temp3ch;--T
                                                              CH30 <=temp2ch;--: 
                                                              CH31 <=  "00101110";--
                                                              CH32 <= temp1ch;--spacio 
                                                   

                           
                       

                 
                        IF (B1='1') THEN
                        state<=abierto;
                        end if;

                when advertenciaalCerrar =>
 
                 
                    
     CH1 <= ME;--S
                                                              CH2 <= x;--i
                                                              CH3 <= c;--s
                                                              CH4 <= ee;--t
                                                              CH5 <= s;--e
                   --                                           CH7 <= temp3ch;--temp3ch;--temp en bcd 3
                   --                                           CH8 <= temp2ch;--temp2ch;--temp en bcd 2
                   --                                           CH9 <= "00101110";--punto decimal
                   --                                           CH10 <= temp1ch;--temp1ch;--temp en bcd 1
                   --                                           CH11 <= temp0ch;--temp0ch;--temp en bcd 0
                                                              CH6 <= o;--m
                                                              CH7 <=espacio;--m
                                                              CH8 <= d;--m
                                                                CH9 <=ee;--a
                                                               -- CH9 <= "00101110";--punto decimal
                                                               CH10 <=espacio ;--spacio
                                                                CH11 <= espacio;--A
                                                                CH12 <= espacio;--b
                                                              CH13 <= espacio;--i
                                                              CH14 <= espacio;--e       
                                                              CH15 <= espacio;--r
                                                              CH16 <=  espacio;--t
                                                              
                                                              CH17 <=MT;--I
                                                              CH18 <= ee;--n
                                                              CH19 <= m;--g
                                                               CH20 <=p;--r
                                                              CH21 <=  ee;--e   
                                                               CH22 <=r;--s   
                                                                CH23 <= a;--e   
                                                                 CH24 <=t;--spacio 
                                                              CH25 <= u;--P   
                                                              CH26 <= r;--i
                                                              CH27 <=  a;--n
                                                              CH28 <="00111010";--spacio
                                                              CH29 <=temp3ch;--T
                                                              CH30 <=temp2ch;--: 
                                                              CH31 <=  "00101110";--
                                                              CH32 <= temp1ch;--spacio                                               
                    
                                 IF (B1='1') THEN
                                        state<=abierto;
                                        end if;
                                        
                                        
                         when cambiodePassword =>
                                   if cambioPassword(0)='1' then
                                                       bitcambiodePassword0 <= "00110001";
                                                       else 
                                                        bitcambiodePassword0 <= "00110000";
                                                        end if;
                                                        
                                                         if cambioPassword(1)='1' then
                                                                bitcambiodePassword1 <= "00110001";
                                                            else 
                                                           bitcambiodePassword1 <= "00110000";
                                                           end if;
                                                           
                                                            if cambioPassword(2)='1' then
                                                                              bitcambiodePassword2 <= "00110001";
                                                                              else 
                                                                               bitcambiodePassword2 <= "00110000";
                                                                               end if;
                                       if cambioPassword(3)='1' then
                                                  bitcambiodePassword3 <= "00110001";
                                                   else 
                                                         bitcambiodePassword3 <= "00110000";
                                                     end if;
                                                       
                                                                           
                                                                         CH1 <= ME;--
                                                                                                                     CH2 <= n;--: 
                                                                                                                     CH3 <= t;--1
                                                                                                                     CH4 <= ee;--2
                                                                                                                     CH5 <= r;
                                                                                                                     CH6 <= espacio;--2
                                                                                                                     CH7 <= MN;-->.
                                                                                                                     CH8 <= ee;--2.
                                                                                                                     CH9 <= w;--5.
                                                                                                                      CH10 <= espacio;--atn
                                                                                                                     CH11 <= MN;--sr3
                                                                                                                     CH12 <= i;--ta
                                                                                                                     CH13 <= p;--atn
                                                                                                                     CH14 <= espacio;--duo
                                                                                                                     CH15 <= espacio; --ro
                                                                                                                     
                                                                                                                     CH16 <= espacio;--atn
                                                                                                                     CH17 <= MN; --ce
                                                                                                                     CH18 <= ee; --oa
                                                                                                                     CH19 <= w;--nlr
                                                                                                                     CH20 <= espacio;--spteacio
                                                                                                                     CH21 <= Me; --aac
                                                                                                                     CH22 <= n;--lo
                                                                                                                     CH23 <= t;--gnm
                                                                 CH24 <= r;--uo i
                                                 CH25 <= y; --nse
                                                CH26 <= "00111010"; --puntos
                                              CH27 <= espacio; --cd
                                                      CH28 <= bitcambiodePassword0;--ora
                                  CH29 <=bitcambiodePassword1; --nes
                                             CH30 <=bitcambiodePassword2; --tca
                                                  CH31 <= bitcambiodePassword3;--aol
                                    CH32 <= espacio;--gmi                                    
                                                     
                                                     
                                                     if( B1= '1') then
                                                    Password <=cambioPassword;
                                                     state <=abierto;                                                 
                                                       end if;
                when others => null;
				end case;
            end if;
        end process;
     
     
     
     process(Clock50) is
      variable cur : u20 := counter;
      begin
          if (rising_edge(Clock50)) then
              cur := cur + 1;  
              counter <= cur;
              if (xsignal='0') then
                  mult <= 4;
                  
             -- elsif (temp < "0000000010100") then
              elsif (xsignal ='1') then
                  mult <= 2;
                  
             
              end if;
              
              
              posicion <= duty_cycle - (25000 * mult);
              if (cur <= posicion) then
                  pwm_counter <= '1'; 
              elsif (cur > posicion) then
                  pwm_counter <= '0';
              elsif (cur = period) then
                  cur := x"00000";
             end if;
          end if;
      end process pwm_generator;  
          PWM1<=      pwm_counter;     
               --                                                    PWM2<=      pwm_counter;     
        --                                                           PWM3<=      pwm_counter;     
                          --                                    PWM4<=      pwm_counter; 
end Behavioral;