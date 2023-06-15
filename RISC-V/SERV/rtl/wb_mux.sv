/**
 * @ Author: German Cano Quiveu, germancq
 * @ Create Time: 2023-01-25 12:31:49
 * @ Modified by: German Cano Quiveu, germancq
 * @ Modified time: 2023-01-31 15:27:48
 * @ Description:
 */

module wb_mux #(
    parameter NUM_SLAVES = 3,
    parameter WB_DATA_WIDTH = 32,
    parameter MSB_SLAVES_ADDR = 3
)
(
    input wb_clk,
    input wb_rst,

    //slaves addr
    input [NUM_SLAVES-1:0][MSB_SLAVES_ADDR-1:0] s_address,

    //slaves interfaces
    input [NUM_SLAVES-1:0][WB_DATA_WIDTH-1:0] s_wb_rdt,
    output [NUM_SLAVES-1:0][WB_DATA_WIDTH-1:0] s_wb_dat,
    output [NUM_SLAVES-1:0][WB_DATA_WIDTH-1:0] s_wb_adr,
    output [NUM_SLAVES-1:0][(WB_DATA_WIDTH>>3)-1:0] s_wb_sel,
    output [NUM_SLAVES-1:0][0:0] s_wb_we,
    output logic [NUM_SLAVES-1:0][0:0] s_wb_cyc,

    //master interface
    input [WB_DATA_WIDTH-1:0] m_wb_adr,
    input [WB_DATA_WIDTH-1:0] m_wb_dat,
    input [(WB_DATA_WIDTH>>3)-1:0] m_wb_sel, 
    input m_wb_we,
    input m_wb_cyc,
    output logic [WB_DATA_WIDTH-1:0] m_wb_rdt,
    output logic m_wb_ack
);

    logic msb;
    assign msb = m_wb_adr>>(WB_DATA_WIDTH-MSB_SLAVES_ADDR);
    
    genvar i;
    for (i = 0;i<NUM_SLAVES ;i=i+1 ) begin
        assign s_wb_dat[i] = msb == s_address[i] ? m_wb_dat : 0;
        assign s_wb_adr[i] = msb == s_address[i] ? m_wb_adr : 0;
        assign s_wb_sel[i] = msb == s_address[i] ? m_wb_sel : 0;
        assign s_wb_we[i]  = msb == s_address[i] ? m_wb_we  : 0;
        assign s_wb_cyc[i] = msb == s_address[i] ? m_wb_cyc : 0;
    end

    logic [31:0] j;
    always_comb begin     
        for (j = 0;j<NUM_SLAVES ;j=j+1 ) begin
            if(msb == s_address[j]) begin
                m_wb_rdt = s_wb_rdt[j];
            end
        end
    end

    always_ff @(posedge wb_clk) begin
        if(wb_rst) begin
            m_wb_ack = 0;
        end
        else begin
            m_wb_ack = (m_wb_ack == 0 && m_wb_cyc == 1)? 1 : 0 ;
        end
    end

endmodule: wb_mux