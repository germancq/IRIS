/**
 * @ Author: German Cano Quiveu, germancq
 * @ Create Time: 2023-01-31 16:05:11
 * @ Modified by: German Cano Quiveu, germancq
 * @ Modified time: 2023-01-31 16:21:08
 * @ Description:
 */

module wb_simple_connection #(
    parameter NUM_MASTERS = 3,
    parameter NUM_SLAVES = 3,
    parameter WB_DATA_WIDTH = 32,
    parameter MSB_SLAVES_ADDR = 3
) (
    input wb_clk,
    input wb_rst,

    //masters interfaces
    input [NUM_MASTERS-1:0][WB_DATA_WIDTH-1:0] m_wb_adr,
    input [NUM_MASTERS-1:0][WB_DATA_WIDTH-1:0] m_wb_dat,
    input [NUM_MASTERS-1:0][(WB_DATA_WIDTH>>3)-1:0] m_wb_sel, 
    input [NUM_MASTERS-1:0][0:0] m_wb_we,
    input [NUM_MASTERS-1:0][0:0] m_wb_cyc,
    output logic [NUM_MASTERS-1:0][WB_DATA_WIDTH-1:0] m_wb_rdt,
    output logic [NUM_MASTERS-1:0][0:0] m_wb_ack,

    //slaves interfaces
    input [NUM_SLAVES-1:0][WB_DATA_WIDTH-1:0] s_wb_rdt,
    output logic [NUM_SLAVES-1:0][WB_DATA_WIDTH-1:0] s_wb_dat,
    output logic [NUM_SLAVES-1:0][WB_DATA_WIDTH-1:0] s_wb_adr,
    output logic [NUM_SLAVES-1:0][(WB_DATA_WIDTH>>3)-1:0] s_wb_sel,
    output logic [NUM_SLAVES-1:0][0:0] s_wb_we,
    output logic [NUM_SLAVES-1:0][0:0] s_wb_cyc,

    //slaves addr
    input [NUM_SLAVES-1:0][MSB_SLAVES_ADDR-1:0] s_address
);

    logic [WB_DATA_WIDTH-1:0] selected_m_wb_adr;
    logic [WB_DATA_WIDTH-1:0] selected_m_wb_dat;
    logic [WB_DATA_WIDTH-1:0] selected_m_wb_rdt;
    logic [(WB_DATA_WIDTH>>3)-1:0] selected_m_wb_sel;
    logic selected_m_wb_cyc;
    logic selected_m_wb_we; 


    //by priority 0: lowest priority
    logic [31:0] i;
    logic [WB_DATA_WIDTH-1:0] msb;
    always_ff @(posedge wb_clk) begin
        //default values
        selected_m_wb_adr = 0;
        selected_m_wb_dat = 0;
        selected_m_wb_sel = 0;
        selected_m_wb_we = 0;
        selected_m_wb_cyc = 0;
        selected_m_wb_rdt = 0;


        for (i = 0;i<NUM_MASTERS-1 ;i=i+1 ) begin
            m_wb_ack[i] = (m_wb_ack[i] == 0 && m_wb_cyc[i] == 1)? 1 : 0 ;
            if(wb_rst) begin
                m_wb_ack[i] = 0;
            end
            else if(m_wb_cyc[i]) begin
                selected_m_wb_dat = m_wb_dat[i];
                selected_m_wb_adr = m_wb_adr[i];
                selected_m_wb_sel = m_wb_sel[i];
                selected_m_wb_we = m_wb_we[i];
                selected_m_wb_cyc = m_wb_cyc[i];
            end
        end


        for (i = 0;i<NUM_SLAVES ;i=i+1 ) begin
            msb = selected_m_wb_adr>>(WB_DATA_WIDTH-MSB_SLAVES_ADDR);  
            s_wb_adr[i] = msb == s_address[i] ? selected_m_wb_adr : 0;
            s_wb_dat[i] = msb == s_address[i] ? selected_m_wb_dat : 0;
            s_wb_sel[i] = msb == s_address[i] ? selected_m_wb_sel : 0;
            s_wb_we[i] = msb == s_address[i] ? selected_m_wb_we : 0;
            s_wb_cyc[i] = msb == s_address[i] ? selected_m_wb_cyc : 0;
            selected_m_wb_rdt = msb == s_address[i] ? s_wb_rdt[i] : 0;
        end

    end
    
endmodule: wb_simple_connection