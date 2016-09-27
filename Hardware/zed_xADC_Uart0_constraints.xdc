#Y11  -> JA1
#AA11 -> JA2
#Y10  -> JA3
#AA9  -> JA4
#set_property PACKAGE_PIN Y11 [get_ports xadc_out[0]]        
#set_property PACKAGE_PIN AA11 [get_ports xadc_out[1]]       
#set_property PACKAGE_PIN Y10 [get_ports xadc_out[2]]        
#set_property PACKAGE_PIN AA9 [get_ports xadc_out[3]]        
#set_property IOSTANDARD LVCMOS33 [get_ports xadc_out[0]]
#set_property IOSTANDARD LVCMOS33 [get_ports xadc_out[1]]
#set_property IOSTANDARD LVCMOS33 [get_ports xadc_out[2]]
#set_property IOSTANDARD LVCMOS33 [get_ports xadc_out[3]]

# AB11 -> JA7
# AB10 -> JA8
# AB9  -> JA9
# AA8  -> JA10
#set_property PACKAGE_PIN AB11 [get_ports dma_dout[0]]       
#set_property PACKAGE_PIN AB10 [get_ports dma_dout[1]]       
#set_property PACKAGE_PIN AB9 [get_ports dma_dout[2]]        
#set_property PACKAGE_PIN AA8 [get_ports dma_dout[3]]        
#set_property IOSTANDARD LVCMOS33 [get_ports dma_dout[0]]
#set_property IOSTANDARD LVCMOS33 [get_ports dma_dout[1]]
#set_property IOSTANDARD LVCMOS33 [get_ports dma_dout[2]]
#set_property IOSTANDARD LVCMOS33 [get_ports dma_dout[3]]
#
# W12  -> JB1
# V10  -> JB3(x)
# W8   -> JB4(x)
# V12  -> JB7
# W11  -> JB2 (uart0_txd)
# V9   -> JB9 (uart0_rxd)
# W10  -> JB8
# V8   -> JB10
set_property PACKAGE_PIN V9 [get_ports UART0_rxd]
set_property IOSTANDARD LVCMOS33 [get_ports UART0_rxd]
set_property PACKAGE_PIN W11 [get_ports UART0_txd]
set_property IOSTANDARD LVCMOS33 [get_ports UART0_txd]
set_property SLEW FAST [get_ports UART0_txd]       
             
#set_property PACKAGE_PIN V10 [get_ports xadcfifo_adapter_TLAST]             
#set_property IOSTANDARD LVCMOS33 [get_ports xadcfifo_adapter_TLAST]

#set_property PACKAGE_PIN W8 [get_ports m_axi_s2mm_wlast]                                  
#set_property IOSTANDARD LVCMOS33 [get_ports m_axi_s2mm_wlast]

#set_property PACKAGE_PIN W12 [get_ports FCLK_CLK0]                                  
#set_property IOSTANDARD LVCMOS33 [get_ports FCLK_CLK0]
#set_property SLEW FAST [get_ports FCLK_CLK0]   

#set_property PACKAGE_PIN V12 [get_ports dma_s2mm_introut]                                  
#set_property IOSTANDARD LVCMOS33 [get_ports dma_s2mm_introut]

set_property PACKAGE_PIN W10 [get_ports gpio_tri_o[0]]    
set_property PACKAGE_PIN V8 [get_ports gpio_tri_o[1]]                                 
set_property IOSTANDARD LVCMOS33 [get_ports gpio_tri_o[0]]
set_property IOSTANDARD LVCMOS33 [get_ports gpio_tri_o[1]]

set_property PACKAGE_PIN E16 [get_ports Vaux0_v_n]
set_property PACKAGE_PIN F16 [get_ports Vaux0_v_p]
set_property IOSTANDARD LVCMOS18 [get_ports Vaux0_v_n]
set_property IOSTANDARD LVCMOS18 [get_ports Vaux0_v_p]


