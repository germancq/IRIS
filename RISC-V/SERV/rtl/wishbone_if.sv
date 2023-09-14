/**
 * @ Author: German Cano Quiveu, germancq
 * @ Create Time: 2023-02-03 12:01:50
 * @ Modified by: German Cano Quiveu, germancq
 * @ Modified time: 2023-02-07 18:52:11
 * @ Description:
 */

interface whisbone_if #(
    parameter DATA_WIDTH = 32,
    parameter ADDR_WIDTH = 32
)(input wb_clk, input wb_rst);

    logic [DATA_WIDTH-1:0] wb_dat;
    logic [DATA_WIDTH-1:0] wb_rdt;
    logic [ADDR_WIDTH-1:0] wb_adr;
    logic [(DATA_WIDTH>>3)-1:0] wb_sel;
    logic wb_cyc;
    logic wb_ack;
    logic wb_we;

    

    modport master (
    input wb_ack, wb_rdt, wb_clk, wb_rst,
    output wb_dat, wb_adr, wb_sel, wb_cyc, wb_we
    );

    modport slaveconn (
    input wb_ack, wb_rdt, wb_clk, wb_rst,
    output wb_dat, wb_adr, wb_sel, wb_cyc, wb_we,
    import slaveconnToStruct
    );

    modport slave (
    input wb_dat, wb_adr, wb_cyc, wb_we, wb_sel, wb_clk, wb_rst,
    output wb_rdt, wb_ack
    );

    modport masterconn (
    input wb_dat, wb_adr, wb_cyc, wb_we, wb_sel, wb_clk, wb_rst,
    output wb_rdt, wb_ack,
    import assign_slave_v,
    import assign_slave
    );
    
    typedef struct packed {
            logic [DATA_WIDTH-1:0] wb_dat;
            logic [DATA_WIDTH-1:0] wb_rdt;
            logic [ADDR_WIDTH-1:0] wb_adr;
            logic [(DATA_WIDTH>>3)-1:0] wb_sel;
            logic wb_cyc;
            logic wb_ack;
            logic wb_we;  
       } whisboneInf;

    function automatic slaveconnToStruct();
       

       whisboneInf conn;
       conn.wb_rdt = wb_rdt;
       conn.wb_ack = wb_ack;
       wb_adr = conn.wb_adr;
       wb_sel = conn.wb_sel;
       wb_ack = conn.wb_ack;
       wb_cyc = conn.wb_cyc;
       

       return conn;
        
    endfunction: slaveconnToStruct

    task automatic assign_slave(ref whisboneInf slave);
        
                    wb_rdt = slave.wb_rdt;
                    wb_ack = slave.wb_ack;
                    slave.wb_we   = wb_we;
                    slave.wb_sel  = wb_sel;
                    slave.wb_cyc  = wb_cyc;
                    slave.wb_dat  = wb_dat;
                    slave.wb_adr  = wb_adr;

    endtask //automatic

    task automatic assign_slave_v(ref logic [DATA_WIDTH-1:0] s_wb_dat,
                                  ref logic [DATA_WIDTH-1:0] s_wb_rdt,
                                  ref logic [ADDR_WIDTH-1:0] s_wb_adr,
                                  ref logic [(DATA_WIDTH>>3)-1:0] s_wb_sel,
                                  ref logic s_wb_cyc,
                                  ref logic s_wb_ack,
                                  ref logic s_wb_we );
        
                    wb_rdt = s_wb_rdt;
                    wb_ack = s_wb_ack;
                    s_wb_we   = wb_we;
                    s_wb_sel  = wb_sel;
                    s_wb_cyc  = wb_cyc;
                    s_wb_dat  = wb_dat;
                    s_wb_adr  = wb_adr;

    endtask //automatic

endinterface:whisbone_if