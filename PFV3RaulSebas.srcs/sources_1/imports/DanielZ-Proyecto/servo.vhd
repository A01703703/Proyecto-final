library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;

entity main_pwm is  
port (
    clk100m : in std_logic;
    temp    : in std_logic_vector (12 downto 0);
    pwm_out : out std_logic;
    rgb1_red_o : out std_logic;
    rgb1_green_o : out std_logic;
    rgb1_blue_o: out std_logic
);
end main_pwm;

architecture Behavioral of main_pwm is

subtype u20 is unsigned(19 downto 0);
signal counter      : u20 := x"00000";

constant clk_freq   : integer := 50_000_000;        -- Clock frequency in Hz (20 ns)
constant pwm_freq   : integer := 50;                -- PWM sigmal frequency in Hz (20 ms)
constant period     : integer := clk_freq/pwm_freq; -- Número de pulsos de reloj en un ciclo PWM
constant duty_cycle : integer := 125_000;           -- Número de pulsos de reloj para ciclo de trabajo
                                                    --  25,000 -> -90°, 50,000 -> -45°, 75,000 -> 0°
 signal verde : std_logic;
 signal rojo : std_logic;                                                   -- 100,000 ->  45°, 125,000 -> 90°
 signal azul : std_logic; 

signal pwm_counter  : std_logic := '0';
signal stateHigh    : std_logic := '1';

signal clk50m       : std_logic;
signal reset        : std_logic;
signal locked       : std_logic;

signal posicion : integer := 0;
signal mult : integer := 0;

component clk_wiz_50 port (-- Clock in ports
  -- Clock out ports
  clk_50          : out    std_logic;
  -- Status and control signals
  reset             : in     std_logic;
  locked            : out    std_logic;
  clk_in1           : in     std_logic
 );
end component;
begin

clock_instance : clk_wiz_50 port map ( 
  -- Clock out ports  
   clk_50 => clk50m,
  -- Status and control signals                
   reset => reset,
   locked => locked,
   -- Clock in ports
   clk_in1 => clk100m );
 
 
 
 
 
pwm_generator : process(clk50m) is
variable cur : u20 := counter;
begin
    if (rising_edge(clk50m)) then
        cur := cur + 1;  
        counter <= cur;
        if (temp > "0000000011001") then
            mult <= 1;
            azul <= '1';
            rojo <= '0';
            verde <= '0';
       -- elsif (temp < "0000000010100") then
        elsif (temp <= "0000000011000") then
            mult <= 3;
            rojo <= '1';
            azul <= '0';
            verde <= '0';
       -- elsif (temp <= "0000000011001" and temp >= "0000000010100") then
        elsif (temp <= "0000000011001" and temp > "0000000011000") then
            mult <= 2;
            verde <= '1';
            azul <= '0';
            rojo <= '0';
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

pwm_out <= pwm_counter;
rgb1_red_o <= rojo;
rgb1_green_o <= verde;
rgb1_blue_o <= azul;

end Behavioral;