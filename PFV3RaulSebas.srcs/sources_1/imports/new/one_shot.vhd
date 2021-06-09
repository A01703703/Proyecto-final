----------------------------------------------------------------------------------
-- Company: 
-- Engineer: 
-- 
-- Create Date: 11/20/2020 11:53:06 PM
-- Design Name: 
-- Module Name: one_shot - Behavioral
-- Project Name: 
-- Target Devices: 
-- Tool Versions: 
-- Description: 
-- 
-- Dependencies: 
-- 
-- Revision:
-- Revision 0.01 - File Created
-- Additional Comments:
-- 
----------------------------------------------------------------------------------


library IEEE;
use IEEE.STD_LOGIC_1164.ALL;

-- Uncomment the following library declaration if using
-- arithmetic functions with Signed or Unsigned values
--use IEEE.NUMERIC_STD.ALL;

-- Uncomment the following library declaration if instantiating
-- any Xilinx leaf cells in this code.
--library UNISIM;
--use UNISIM.VComponents.all;

entity one_shot is
    Port ( Din : in STD_LOGIC;
           clk : in STD_LOGIC;
           Qout : out STD_LOGIC);
end one_shot;

architecture Behavioral of one_shot is
    signal Q1, Q2, Q3 : std_logic;
begin
    process(clk)
    begin
        if clk'event and clk = '1' then 
            Q1 <= Din;
            Q2 <= Q1;
            Q3 <= Q2;
        end if;
    end process;
 Qout <= Q1 and Q2 and (not Q3);
 end behavioral;


