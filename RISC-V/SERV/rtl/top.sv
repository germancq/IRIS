/**
 * @ Author: German Cano Quiveu, germancq
 * @ Create Time: 2023-01-19 11:54:56
 * @ Modified by: German Cano Quiveu, germancq
 * @ Modified time: 2023-02-14 13:15:25
 * @ Description:
 */

module top (
    input wb_clk,
    input rst,
    input center_button,
    output logic [0:0] led,

    //SPI
	output	   sclk,
	output	   mosi,
	input	   miso,
	output 	   cs,
	output SD_RESET,
	output SD_DAT_1,
	output SD_DAT_2,

    //7-seg
    output [6:0] seg,
    output [7:0] AN,
    output DP
);


    //SD in SPI_MODE
    assign SD_RESET = 1'b0;
    assign SD_DAT_1 = 1'b1;
    assign SD_DAT_2 = 1'b1;

    localparam memsize = 8192;
    localparam BOOT_ON = 1;

    logic timer_irq;

    logic [31:0] wb_ibus_adr;
    logic wb_ibus_cyc;
    logic [31:0] wb_ibus_rdt;
    logic wb_ibus_ack;

    logic [31:0] wb_dbus_adr;
    logic [31:0] wb_dbus_dat;
    logic [3:0] wb_dbus_sel;
    logic wb_dbus_we;
    logic wb_dbus_cyc;
    logic [31:0] wb_dbus_rdt;
    logic wb_dbus_ack;

    logic [31:0] mdu_rs1;
    logic [31:0] mdu_rs2;
    logic [2:0] mdu_op;
    logic [31:0] mdu_rd;
    logic mdu_ready;
    logic mdu_valid;

    logic [31:0] wb_mem_adr;
    logic [31:0] wb_mem_dat;
    logic [3:0] wb_mem_sel;
    logic wb_mem_we;
    logic wb_mem_cyc;
    logic [31:0] wb_mem_rdt;
    logic wb_mem_ack;

    logic [31:0] wb_timer_dat;
    logic wb_timer_we;
    logic wb_timer_cyc;
    logic [31:0] wb_timer_rdt;

    logic wb_gpio_dat;
    logic wb_gpio_we;
    logic wb_gpio_cyc;
    logic wb_gpio_rdt;

    logic [31:0] wb_dmem_adr;
    logic [31:0] wb_dmem_dat;
    logic [3:0] wb_dmem_sel;
    logic wb_dmem_we;
    logic wb_dmem_cyc;
    logic [31:0] wb_dmem_rdt;
    logic wb_dmem_ack; 


    logic wb_bootloader_ack;
    logic [31:0] wb_bootloader_adr;
    logic [1:0] wb_bootloader_bte;
    logic [2:0] wb_bootloader_cti;
    logic wb_bootloader_cyc;
    logic wb_bootloader_stb;
    logic [3:0] wb_bootloader_sel;
    logic wb_bootloader_we;
    logic [31:0] wb_bootloader_dat;
    logic [31:0] wb_bootloader_rdt;
    logic wb_bootloader_rty;
    logic wb_bootloader_err;

    logic wb_eluks_ack;
    logic [31:0] wb_eluks_adr;
    logic [1:0] wb_eluks_bte;
    logic [2:0] wb_eluks_cti;
    logic wb_eluks_cyc;
    logic wb_eluks_stb;
    logic [3:0] wb_eluks_sel;
    logic wb_eluks_we;
    logic [31:0] wb_eluks_dat;
    logic [31:0] wb_eluks_rdt;
    logic wb_eluks_rty;
    logic wb_eluks_err;
    

    logic [31:0] bootloader_debug_data;
    logic bus_rst;
    logic [63:0] exec_timer;
    logic boot_cpu_rst;


    logic wb_rst;

    assign wb_rst = rst | bus_rst;

    serv_rf_top #(
        .RESET_PC(0),
        .RESET_STRATEGY("MINI")
    )
    proccessor(
      .clk      (wb_clk),
      .i_rst    (wb_rst | boot_cpu_rst),
      .i_timer_irq  (timer_irq),

      .o_ibus_adr   (wb_ibus_adr),
      .o_ibus_cyc   (wb_ibus_cyc),
      .i_ibus_rdt   (wb_ibus_rdt),
      .i_ibus_ack   (wb_ibus_ack),

      .o_dbus_adr   (wb_dbus_adr),
      .o_dbus_dat   (wb_dbus_dat),
      .o_dbus_sel   (wb_dbus_sel),
      .o_dbus_we    (wb_dbus_we),
      .o_dbus_cyc   (wb_dbus_cyc),
      .i_dbus_rdt   (wb_dbus_rdt),
      .i_dbus_ack   (wb_dbus_ack),
      
      //Extension
      .o_ext_rs1    (mdu_rs1),
      .o_ext_rs2    (mdu_rs2),
      .o_ext_funct3 (mdu_op),
      .i_ext_rd     (mdu_rd),
      .i_ext_ready  (mdu_ready),
      //MDU
      .o_mdu_valid  (mdu_valid)
    );


    assign mdu_ready = 1'b0;
    assign mdu_rd = 32'b0;

    servant_ram #(
        .depth(memsize),
        .RESET_STRATEGY("NONE")
        /*.memfile("blinky.hex")*/
    ) ram (
        .i_wb_clk (wb_clk),
        .i_wb_rst (wb_rst),
        .i_wb_adr (wb_mem_adr[$clog2(memsize)-1:2]),
        .i_wb_cyc (wb_mem_cyc),
        .i_wb_we  (wb_mem_we) ,
        .i_wb_sel (wb_mem_sel),
        .i_wb_dat (wb_mem_dat),
        .o_wb_rdt (wb_mem_rdt),
        .o_wb_ack (wb_mem_ack)
    );



    servant_timer #(
        .WIDTH(32)
    ) timer (
        .i_clk    (wb_clk),
	    .i_rst    (wb_rst),
	    .o_irq    (timer_irq),
	    .i_wb_cyc (wb_timer_cyc),
	    .i_wb_we  (wb_timer_we) ,
	    .i_wb_dat (wb_timer_dat),
	    .o_wb_dat (wb_timer_rdt)
    );



    servant_gpio gpio
     (.i_wb_clk (wb_clk),
      .i_wb_dat (wb_gpio_dat),
      .i_wb_we  (wb_gpio_we),
      .i_wb_cyc (wb_gpio_cyc),
      .o_wb_rdt (wb_gpio_rdt),
      .o_gpio   (led));


    ////////////////////////////////////////////////////////////////////////
//
// BOOTLOADER MODULE
//
////////////////////////////////////////////////////////////////////////


    

    bootloaderModule #(
        .WB_DATA(32),
        .ELUKS_WB_ADDR(32'hA0000000),
        .RAM_WB_ADDR(32'h00000000)
    )
    boot0(
        .wb_clk(wb_clk),
        .clk_ctr(wb_clk),
        .rst(center_button),
        .bus_rst(bus_rst),

        .wb_rst_o(),
        .wb_ack_i(wb_bootloader_ack),
        .wb_err_i(wb_bootloader_err),
        .wb_dat_i(wb_bootloader_rdt),
        .wb_rty_i(wb_bootloader_rty),
        .wb_dat_o(wb_bootloader_dat),
        .wb_cyc_o(wb_bootloader_cyc),
        .wb_stb_o(wb_bootloader_stb),
        .wb_sel_o(wb_bootloader_sel),
        .wb_we_o(wb_bootloader_we),
        .wb_cti_o(wb_bootloader_cti),
        .wb_bte_o(wb_bootloader_bte),
        .wb_adr_o(wb_bootloader_adr),

        .start(BOOT_ON),
        .cpu_rst(boot_cpu_rst),
        .psw(64'h1122334455667788),
        .start_block(32'h0),
        .hmac_enable(1),

        .dbg_btn(),
        .error(),
        .debug(bootloader_debug_data[15:0]),
        .exec_timer(exec_timer)
    );
   //assign bootloader_debug_data[15:8] = {wb_bootloader_stb,wb_eluks0_stb};

    ////////////////////////////////////////////////////////////////////////
    //
    // ELUKS MODULE
    //
    ////////////////////////////////////////////////////////////////////////

    wb_eluks #(
        .CLK_FQ_KHZ(100000),
        .WB_ADDR_DIR(8),
        .WB_DATA_WIDTH(32),
        .PSW_WIDTH(64),
        .SALT_WIDTH(64),
        .COUNT_WIDTH(32),
        .BLOCK_SIZE(64),
        .KEY_SIZE(80),
        .N(88),
        .c(80),
        .r(8),
        .R(45),
        .lCounter_initial_state(6'h05),
        .lCounter_feedback_coeff(7'h61),
        .N_kdf(88),
        .c_kdf(80),
        .r_kdf(8),
        .R_kdf(45),
        .lCounter_initial_state_kdf(6'h05),
        .lCounter_feedback_coeff_kdf(7'h61)
    ) wb_eluks_inst(
        .wb_clk(wb_clk),
        .wb_rst(wb_rst),
        .wb_adr_i(wb_eluks_adr),
        .wb_dat_i(wb_eluks_dat),
        .wb_we_i(wb_eluks_we),
        .wb_cyc_i(wb_eluks_cyc),
        .wb_stb_i(wb_eluks_cyc),
        .wb_sel_i(wb_eluks_sel),
        .wb_cti_i(wb_eluks_cti),
        .wb_bte_i(wb_eluks_bte),
        .wb_ack_o(wb_eluks_ack),
        .wb_err_o(wb_eluks_err),
        .wb_rty_o(wb_eluks_rty),
        .wb_dat_o(wb_eluks_rdt),
        .sclk(sclk),
        .cs(cs),
        .mosi(mosi),
        .miso(miso),
        .sclk_speed(4'h1),
        .debug(bootloader_debug_data[31:16]),
        .dbg_btn()
    );

    ///////////////////////////////////////////////////////////
    ///WISHBONE INTERCONECTION///

    whisbone_if data_bus(.wb_clk(wb_clk),.wb_rst(wb_rst));
    whisbone_if instruction_bus(.wb_clk(wb_clk),.wb_rst(wb_rst));
    whisbone_if boot_bus(.wb_clk(wb_clk),.wb_rst(wb_rst));
    whisbone_if ram_bus(.wb_clk(wb_clk),.wb_rst(wb_rst));
    whisbone_if gpio_bus(.wb_clk(wb_clk),.wb_rst(wb_rst));
    whisbone_if timer_bus(.wb_clk(wb_clk),.wb_rst(wb_rst));
    whisbone_if eluks_bus(.wb_clk(wb_clk),.wb_rst(wb_rst));


    assign data_bus.wb_adr = wb_dbus_adr;
    assign data_bus.wb_dat = wb_dbus_dat;
    assign data_bus.wb_ack = wb_dbus_ack;
    assign data_bus.wb_cyc = wb_dbus_cyc;
    assign data_bus.wb_we  = wb_dbus_we;
    assign data_bus.wb_sel = wb_dbus_sel;
    assign data_bus.wb_rdt = wb_dbus_rdt;


    assign instruction_bus.wb_adr = wb_ibus_adr;
    assign instruction_bus.wb_dat = 0;
    assign instruction_bus.wb_ack = wb_ibus_ack;
    assign instruction_bus.wb_cyc = wb_ibus_cyc;
    assign instruction_bus.wb_we  = 0;
    assign instruction_bus.wb_sel = 0;
    assign instruction_bus.wb_rdt = wb_ibus_rdt;

    assign boot_bus.wb_adr = wb_bootloader_adr;
    assign boot_bus.wb_dat = wb_bootloader_dat;
    assign boot_bus.wb_ack = wb_bootloader_ack;
    assign boot_bus.wb_cyc = wb_bootloader_cyc | wb_bootloader_stb;
    assign boot_bus.wb_we  = wb_bootloader_we;
    assign boot_bus.wb_sel = wb_bootloader_sel;
    assign boot_bus.wb_rdt = wb_bootloader_rdt;

    assign ram_bus.wb_adr = wb_mem_adr;
    assign ram_bus.wb_dat = wb_mem_dat;
    assign ram_bus.wb_ack = wb_mem_ack;
    assign ram_bus.wb_cyc = wb_mem_cyc;
    assign ram_bus.wb_we  = wb_mem_we;
    assign ram_bus.wb_sel = wb_mem_sel;
    assign ram_bus.wb_rdt = wb_mem_rdt;


    assign gpio_bus.wb_adr = 0;
    assign gpio_bus.wb_dat = wb_gpio_dat;
    assign gpio_bus.wb_ack = 1;
    assign gpio_bus.wb_cyc = wb_gpio_cyc;
    assign gpio_bus.wb_we  = wb_gpio_we;
    assign gpio_bus.wb_sel = 0;
    assign gpio_bus.wb_rdt = wb_gpio_rdt;


    assign timer_bus.wb_adr = 0;
    assign timer_bus.wb_dat = wb_timer_dat;
    assign timer_bus.wb_ack = 1;
    assign timer_bus.wb_cyc = wb_timer_cyc;
    assign timer_bus.wb_we  = wb_timer_we;
    assign timer_bus.wb_sel = 0;
    assign timer_bus.wb_rdt = wb_timer_rdt;

    assign eluks_bus.wb_adr = wb_eluks_adr;
    assign eluks_bus.wb_dat = wb_eluks_dat;
    assign eluks_bus.wb_ack = wb_eluks_ack;
    assign eluks_bus.wb_cyc = wb_eluks_cyc;
    assign eluks_bus.wb_we  = wb_eluks_we;
    assign eluks_bus.wb_sel = wb_eluks_sel;
    assign eluks_bus.wb_rdt = wb_eluks_rdt;
    
    wb_arbiter conn(
        .data_bus(data_bus),
        .instruction_bus(instruction_bus),
        .boot_bus(boot_bus),
        .ram_bus(ram_bus),
        .gpio_bus(gpio_bus),
        .timer_bus(timer_bus),
        .eluks_bus(eluks_bus)
    );
    
    /*
    whisbone_if [1:0] master_bus;
    assign master_bus[0] = data_bus;
    assign master_bus[1] = instruction_bus;

    whisbone_if [2:0] slave_bus;
    assign slave_bus[0] = ram_bus;
    assign slave_bus[1] = gpio_bus;
    assign slave_bus[2] = timer_bus;

    logic [2:0][1:0] s_address;
    assign s_address[0] = 2'b00;  //0x00000000
    assign s_address[1] = 2'b01;  //0x40000000
    assign s_address[2] = 2'b10;  //0x80000000


    wb_generic_conn #(.MSB(2)) conn(
        .wb_clk(wb_clk),
        .wb_rst(wb_rst),
        .master_bus(master_bus),
        .slave_bus(slave_bus),
        .s_address(s_address)
    );
    */
    /*
    localparam NUM_MASTERS = 3;
    localparam NUM_SLAVES = 4;
    localparam WB_DATA_WIDTH = 32;
    localparam MSB_SLAVES_ADDR = 3;

    logic [NUM_MASTERS-1:0][WB_DATA_WIDTH-1:0] m_wb_adr;
    assign m_wb_adr[0] = wb_ibus_adr;
    assign m_wb_adr[1] = wb_dbus_adr;
    assign m_wb_adr[2] = wb_bootloader_adr;
    logic [NUM_MASTERS-1:0][WB_DATA_WIDTH-1:0] m_wb_dat;
    assign m_wb_dat[0] = 0;
    assign m_wb_dat[1] = wb_dbus_dat;
    assign m_wb_dat[2] = wb_bootloader_dat;
    logic [NUM_MASTERS-1:0][(WB_DATA_WIDTH>>3)-1:0] m_wb_sel;
    assign m_wb_sel[0] = 0;
    assign m_wb_sel[1] = wb_dbus_sel;
    assign m_wb_sel[2] = wb_bootloader_sel;
    logic [NUM_MASTERS-1:0][0:0] m_wb_we;
    assign m_wb_we[0] = 0;
    assign m_wb_we[1] = wb_dbus_we;
    assign m_wb_we[2] = wb_bootloader_we;
    logic [NUM_MASTERS-1:0][0:0] m_wb_cyc;
    assign m_wb_cyc[0] = wb_ibus_cyc;
    assign m_wb_cyc[1] = wb_dbus_cyc;
    assign m_wb_cyc[2] = 0;//wb_bootloader_cyc;
    logic [NUM_MASTERS-1:0][WB_DATA_WIDTH-1:0] m_wb_rdt;
    assign m_wb_rdt[0] = wb_ibus_rdt;
    assign m_wb_rdt[1] = wb_dbus_rdt;
    assign m_wb_rdt[2] = wb_bootloader_rdt;
    logic [NUM_MASTERS-1:0][0:0] m_wb_ack;
    assign m_wb_ack[0] = wb_ibus_ack;
    assign m_wb_ack[1] = wb_dbus_ack;
    assign m_wb_ack[2] = wb_bootloader_ack;


    logic [NUM_SLAVES-1:0][WB_DATA_WIDTH-1:0] s_wb_rdt;
    assign s_wb_rdt[0] = wb_mem_rdt;
    assign s_wb_rdt[1] = wb_gpio_rdt;
    assign s_wb_rdt[2] = wb_timer_rdt;
    assign s_wb_rdt[3] = wb_eluks_rdt;
    logic [NUM_SLAVES-1:0][WB_DATA_WIDTH-1:0] s_wb_dat;
    assign s_wb_dat[0] = wb_mem_dat;
    assign s_wb_dat[1] = wb_gpio_dat;
    assign s_wb_dat[2] = wb_timer_dat;
    assign s_wb_dat[3] = wb_eluks_dat;
    logic [NUM_SLAVES-1:0][WB_DATA_WIDTH-1:0] s_wb_adr;
    assign s_wb_adr[0] = wb_mem_adr;
    assign s_wb_adr[1] = 0;
    assign s_wb_adr[2] = 0;
    assign s_wb_adr[3] = wb_eluks_adr;
    logic [NUM_SLAVES-1:0][(WB_DATA_WIDTH>>3)-1:0] s_wb_sel;
    assign s_wb_sel[0] = wb_mem_sel;
    assign s_wb_sel[1] = 0;
    assign s_wb_sel[2] = 0;
    assign s_wb_sel[3] = wb_eluks_sel;
    logic [NUM_SLAVES-1:0][0:0] s_wb_we;
    assign s_wb_we[0] = wb_mem_we;
    assign s_wb_we[1] = wb_gpio_we;
    assign s_wb_we[2] = wb_timer_we;
    assign s_wb_we[3] = wb_eluks_we;
    logic [NUM_SLAVES-1:0][0:0] s_wb_cyc;
    assign s_wb_cyc[0] = wb_mem_cyc;
    assign s_wb_cyc[1] = wb_gpio_cyc;
    assign s_wb_cyc[2] = wb_timer_cyc;
    assign s_wb_cyc[3] = wb_eluks_cyc;
   
    logic [NUM_SLAVES-1:0][MSB_SLAVES_ADDR-1:0] s_address;
    assign s_address[0] = 3'b000;  //0x00000000
    assign s_address[1] = 3'b010;  //0x40000000
    assign s_address[2] = 3'b100;  //0x80000000
    assign s_address[3] = 3'b101;  //0x50000000



    wb_simple_connection #(
        .NUM_MASTERS(NUM_MASTERS),
        .NUM_SLAVES(NUM_SLAVES),
        .MSB_SLAVES_ADDR(MSB_SLAVES_ADDR),
        .WB_DATA_WIDTH(WB_DATA_WIDTH)
    )wb_conn(
        .wb_clk(wb_clk),
        .wb_rst(wb_rst),

        .m_wb_adr(m_wb_adr),
        .m_wb_dat(m_wb_dat),
        .m_wb_sel(m_wb_sel),
        .m_wb_we(m_wb_we),
        .m_wb_cyc(m_wb_cyc),
        .m_wb_rdt(m_wb_rdt),
        .m_wb_ack(m_wb_ack),

        .s_wb_rdt(s_wb_rdt),
        .s_wb_dat(s_wb_dat),
        .s_wb_adr(s_wb_adr),
        .s_wb_sel(s_wb_sel),
        .s_wb_we(s_wb_we),
        .s_wb_cyc(s_wb_cyc),

        .s_address(s_address)
    );
    */
    

    ////////////////////////////////////////////////////////////////////////
//
// 7 seg
//
////////////////////////////////////////////////////////////////////////
logic [31:0] debug_7_seg;
display #(.N(32),.CLK_HZ(100000000)) seg7(
    .clk(wb_clk),
	.rst(wb_rst),
	.din(debug_7_seg),
	.seg(seg),
	.an(AN)
);
assign debug_7_seg = bootloader_debug_data;//switch_i[15] == 1 ? exec_timer[63:32] : exec_timer[31:0];


endmodule : top