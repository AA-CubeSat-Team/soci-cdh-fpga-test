	component NIOS_2 is
		port (
			clk_clk : in std_logic := 'X'  -- clk
		);
	end component NIOS_2;

	u0 : component NIOS_2
		port map (
			clk_clk => CONNECTED_TO_clk_clk  -- clk.clk
		);

