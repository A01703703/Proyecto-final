
library IEEE;
use IEEE.STD_LOGIC_1164.ALL;



entity BCDtoASCII is
  Port (num: in std_logic_vector(3 downto 0);
  asciinum: out std_logic_vector(7 downto 0 ) );
end BCDtoASCII;

architecture Behavioral of BCDtoASCII is

begin
asciinum<=  "00110000" when num= "0000" else
            "00110001" when num= "0001" else
            "00110010" when num= "0010" else
            "00110011" when num= "0011" else
            "00110100" when num= "0100" else
            "00110101" when num= "0101" else 
            "00110110" when num= "0110" else
            "00110111" when num= "0111" else
            "00111000" when num= "1000" else
            "00111001" when num= "1001" else
            "00100001";

end Behavioral;
