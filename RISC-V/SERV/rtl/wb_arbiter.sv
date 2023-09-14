/**
 * @ Author: German Cano Quiveu, germancq
 * @ Create Time: 2023-01-30 12:29:11
 * @ Modified by: German Cano Quiveu, germancq
 * @ Modified time: 2023-02-08 12:15:41
 * @ Description:
 */

module wb_arbiter(
    whisbone_if.masterconn data_bus,
    whisbone_if.masterconn instruction_bus,
    whisbone_if.masterconn boot_bus,
    whisbone_if.slaveconn ram_bus,
    whisbone_if.slaveconn gpio_bus,
    whisbone_if.slaveconn timer_bus,
    whisbone_if.slaveconn eluks_bus
);
    always_comb 
    begin
        //ver a quien se le concede el bus
        //segun quien sea, que slave esta seleccionado
        //hacer las conexiones hasta que cambie el bus
        ram_bus.wb_cyc = 0;
        gpio_bus.wb_cyc = 0;
        timer_bus.wb_cyc = 0;
        eluks_bus.wb_cyc = 0;
        instruction_bus.wb_ack = 0;
        instruction_bus.wb_rdt = 0;
        data_bus.wb_ack = 0;
        data_bus.wb_rdt = 0;
        boot_bus.wb_ack = 0;
        boot_bus.wb_rdt = 0;

        if(boot_bus.wb_cyc) begin
            case(boot_bus.wb_adr[31:29])
                3'b000: begin
                    
                    boot_bus.wb_rdt = ram_bus.wb_rdt;
                    boot_bus.wb_ack = ram_bus.wb_ack;
                    ram_bus.wb_we = boot_bus.wb_we;
                    ram_bus.wb_sel = boot_bus.wb_sel;
                    ram_bus.wb_cyc = boot_bus.wb_cyc;
                    ram_bus.wb_dat = boot_bus.wb_dat;
                    ram_bus.wb_adr = boot_bus.wb_adr;
                    
                    //data_bus.assign_slave(ram_bus.toStruct());
                    //data_bus.assign_slave_v(.s_wb_dat(ram_bus.wb_dat),
                    //                        .s_wb_rdt(ram_bus.wb_rdt),
                    //                        .s_wb_adr(ram_bus.wb_adr),
                    //                        .s_wb_sel(ram_bus.wb_sel),
                    //                        .s_wb_cyc(ram_bus.wb_cyc),
                    //                        .s_wb_ack(ram_bus.wb_ack),
                    //                        .s_wb_we(ram_bus.wb_we));
                end
                3'b010: begin
                    
                    boot_bus.wb_rdt = gpio_bus.wb_rdt;
                    boot_bus.wb_ack = gpio_bus.wb_ack;
                    gpio_bus.wb_we =  boot_bus.wb_we;
                    gpio_bus.wb_sel = boot_bus.wb_sel;
                    gpio_bus.wb_cyc = boot_bus.wb_cyc;
                    gpio_bus.wb_dat = boot_bus.wb_dat;
                    gpio_bus.wb_adr = boot_bus.wb_adr;
                    
                    //data_bus.assign_slave(gpio_bus.slaveconnToStruct());
                    /*
                    data_bus.assign_slave_v(.s_wb_dat(gpio_bus.wb_dat),
                                            .s_wb_rdt(gpio_bus.wb_rdt),
                                            .s_wb_adr(gpio_bus.wb_adr),
                                            .s_wb_sel(gpio_bus.wb_sel),
                                            .s_wb_cyc(gpio_bus.wb_cyc),
                                            .s_wb_ack(gpio_bus.wb_ack),
                                            .s_wb_we(gpio_bus.wb_we));
                                            */
                end
                3'b100: begin
                    
                    boot_bus.wb_rdt = timer_bus.wb_rdt;
                    boot_bus.wb_ack = timer_bus.wb_ack;
                    timer_bus.wb_we = boot_bus.wb_we;
                    timer_bus.wb_sel = boot_bus.wb_sel;
                    timer_bus.wb_cyc = boot_bus.wb_cyc;
                    timer_bus.wb_dat = boot_bus.wb_dat;
                    timer_bus.wb_adr = boot_bus.wb_adr;
                    
                    //data_bus.assign_slave(.slave(timer_bus.toStruct()));
                end
                3'b101: begin
                    boot_bus.wb_rdt = eluks_bus.wb_rdt;
                    boot_bus.wb_ack = eluks_bus.wb_ack;
                    eluks_bus.wb_we =  boot_bus.wb_we;
                    eluks_bus.wb_sel = boot_bus.wb_sel;
                    eluks_bus.wb_cyc = boot_bus.wb_cyc;
                    eluks_bus.wb_dat = boot_bus.wb_dat;
                    eluks_bus.wb_adr = boot_bus.wb_adr;
                end
                default: begin
                    boot_bus.wb_rdt = 0;
                    boot_bus.wb_ack = 0;
                    
                end
            endcase
        end

        if(data_bus.wb_cyc) begin

            
            
            case(data_bus.wb_adr[31:29])
                3'b000: begin
                    
                    data_bus.wb_rdt = ram_bus.wb_rdt;
                    data_bus.wb_ack = ram_bus.wb_ack;
                    ram_bus.wb_we = data_bus.wb_we;
                    ram_bus.wb_sel = data_bus.wb_sel;
                    ram_bus.wb_cyc = data_bus.wb_cyc;
                    ram_bus.wb_dat = data_bus.wb_dat;
                    ram_bus.wb_adr = data_bus.wb_adr;
                    
                    //data_bus.assign_slave(ram_bus.toStruct());
                    //data_bus.assign_slave_v(.s_wb_dat(ram_bus.wb_dat),
                    //                        .s_wb_rdt(ram_bus.wb_rdt),
                    //                        .s_wb_adr(ram_bus.wb_adr),
                    //                        .s_wb_sel(ram_bus.wb_sel),
                    //                        .s_wb_cyc(ram_bus.wb_cyc),
                    //                        .s_wb_ack(ram_bus.wb_ack),
                    //                        .s_wb_we(ram_bus.wb_we));
                end
                3'b010: begin
                    
                    data_bus.wb_rdt = gpio_bus.wb_rdt;
                    data_bus.wb_ack = gpio_bus.wb_ack;
                    gpio_bus.wb_we = data_bus.wb_we;
                    gpio_bus.wb_sel = data_bus.wb_sel;
                    gpio_bus.wb_cyc = data_bus.wb_cyc;
                    gpio_bus.wb_dat = data_bus.wb_dat;
                    gpio_bus.wb_adr = data_bus.wb_adr;
                    
                    //data_bus.assign_slave(gpio_bus.slaveconnToStruct());
                    /*
                    data_bus.assign_slave_v(.s_wb_dat(gpio_bus.wb_dat),
                                            .s_wb_rdt(gpio_bus.wb_rdt),
                                            .s_wb_adr(gpio_bus.wb_adr),
                                            .s_wb_sel(gpio_bus.wb_sel),
                                            .s_wb_cyc(gpio_bus.wb_cyc),
                                            .s_wb_ack(gpio_bus.wb_ack),
                                            .s_wb_we(gpio_bus.wb_we));
                                            */
                end
                3'b100: begin
                    
                    data_bus.wb_rdt = timer_bus.wb_rdt;
                    data_bus.wb_ack = timer_bus.wb_ack;
                    timer_bus.wb_we = data_bus.wb_we;
                    timer_bus.wb_sel = data_bus.wb_sel;
                    timer_bus.wb_cyc = data_bus.wb_cyc;
                    timer_bus.wb_dat = data_bus.wb_dat;
                    timer_bus.wb_adr = data_bus.wb_adr;
                    
                    //data_bus.assign_slave(.slave(timer_bus.toStruct()));
                end
                3'b101: begin
                    data_bus.wb_rdt = eluks_bus.wb_rdt;
                    data_bus.wb_ack = eluks_bus.wb_ack;
                    eluks_bus.wb_we = data_bus.wb_we;
                    eluks_bus.wb_sel = data_bus.wb_sel;
                    eluks_bus.wb_cyc = data_bus.wb_cyc;
                    eluks_bus.wb_dat = data_bus.wb_dat;
                    eluks_bus.wb_adr = data_bus.wb_adr;
                end
                default: begin
                    data_bus.wb_rdt = 0;
                    data_bus.wb_ack = 0;
                    
                end
            endcase
        end
        else if(instruction_bus.wb_cyc) begin

            

            case(instruction_bus.wb_adr[31:29])
                3'b000: begin
                    
                    instruction_bus.wb_rdt = ram_bus.wb_rdt;
                    instruction_bus.wb_ack = ram_bus.wb_ack;
                    ram_bus.wb_we = instruction_bus.wb_we;
                    ram_bus.wb_sel = instruction_bus.wb_sel;
                    ram_bus.wb_cyc = instruction_bus.wb_cyc;
                    ram_bus.wb_dat = instruction_bus.wb_dat;
                    ram_bus.wb_adr = instruction_bus.wb_adr;
                    
                    //instruction_bus.assign_slave(.slave(ram_bus.toStruct()));
                end
                3'b010: begin
                    
                    instruction_bus.wb_rdt = gpio_bus.wb_rdt;
                    instruction_bus.wb_ack = gpio_bus.wb_ack;
                    gpio_bus.wb_we = instruction_bus.wb_we;
                    gpio_bus.wb_sel = instruction_bus.wb_sel;
                    gpio_bus.wb_cyc = instruction_bus.wb_cyc;
                    gpio_bus.wb_dat = instruction_bus.wb_dat;
                    gpio_bus.wb_adr = instruction_bus.wb_adr;
                    
                    //instruction_bus.assign_slave(.slave(gpio_bus.toStruct()));
                end
                3'b100: begin
                    
                    instruction_bus.wb_rdt = timer_bus.wb_rdt;
                    instruction_bus.wb_ack = timer_bus.wb_ack;
                    timer_bus.wb_we = instruction_bus.wb_we;
                    timer_bus.wb_sel = instruction_bus.wb_sel;
                    timer_bus.wb_cyc = instruction_bus.wb_cyc;
                    timer_bus.wb_dat = instruction_bus.wb_dat;
                    timer_bus.wb_adr = instruction_bus.wb_adr;
                    
                    //instruction_bus.assign_slave(.slave(timer_bus.toStruct()));
                end
                3'b101: begin
                    instruction_bus.wb_rdt = eluks_bus.wb_rdt;
                    instruction_bus.wb_ack = eluks_bus.wb_ack;
                    eluks_bus.wb_we = instruction_bus.wb_we;
                    eluks_bus.wb_sel = instruction_bus.wb_sel;
                    eluks_bus.wb_cyc = instruction_bus.wb_cyc;
                    eluks_bus.wb_dat = instruction_bus.wb_dat;
                    eluks_bus.wb_adr = instruction_bus.wb_adr;
                end
                default: begin
                    instruction_bus.wb_rdt = 0;
                    instruction_bus.wb_ack = 0;
                    
                end
            endcase
        end 
        else begin
            //not one selected
        end
    end
    
    
    


endmodule: wb_arbiter

/*

module wb_generic_conn #(
    parameter DATA_WIDTH = 32,
    parameter ADDR_WIDTH = 32,
    parameter NUM_MASTERS = 2,
    parameter NUM_SLAVES = 3,
    parameter MSB = 3
)
(
    input wb_clk,
    input wb_rst,
    whisbone_if.masterconn [NUM_MASTERS-1:0] master_bus,
    whisbone_if.slaveconn [NUM_SLAVES-1:0] slave_bus,
    input [NUM_SLAVES-1:0][MSB_SLAVES_ADDR-1:0] s_address 
);

    logic [NUM_MASTERS - 1:0] select;

    logic [31:0] i;
    always_ff @( posedge wb_clk ) begin
        
        select = 0;
        for (i = 0;i<NUM_MASTERS ;i=i+1 ) begin
            if(master_bus[i].wb_cyc) begin
                select = i;
            end
        end

        for (i = 0;i<NUM_SLAVES ;i=i+1 ) begin
            if(master_bus[select].wb_adr[ADDR_WIDTH-1:ADDR_WIDTH-MSB] == s_address[i]) begin
                master_bus[select].wb_rdt = slave_bus[i].wb_rdt;
                master_bus[select].wb_ack = slave_bus[i].wb_ack;
                slave_bus[i].wb_we = master_bus[select];
                slave_bus[i].wb_cyc = master_bus[select];
                slave_bus[i].wb_dat = master_bus[select];
                slave_bus[i].wb_adr = master_bus[select];
            end
        end
        
    end
    
endmodule : wb_generic_conn
*/