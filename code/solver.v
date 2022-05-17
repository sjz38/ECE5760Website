module scheduler#(parameter NUM_ITERS=4) (
    cycles, zoom_x, zoom_y, next_x, next_y, rd_color, x_pos, y_pos, clk, rst
);
input wire         clk;
input wire         rst;
input wire  [26:0] zoom_x;
input wire  [26:0] zoom_y;
input wire  [9:0]  next_x;
input wire  [9:0]  next_y;
output wire [7:0]  rd_color;
input wire  [26:0] x_pos;
input wire  [26:0] y_pos;
output reg  [31:0] cycles;
 
// "GLOBAL"
parameter XSIZE     = 639;
parameter YSIZE     = 479;
parameter XbyY      = 307199;
parameter MAX_COUNT = 1000;
 
// FSM state names
parameter START = 3'b0;
parameter WAIT  = 3'd1;
parameter DONE  = 3'd2;
 
reg  [2:0]  state;
reg  [31:0] cycles_reg;
wire [31:0] cycles_wire;
assign cycles_wire = cycles_reg;
 
reg                    start;
wire [NUM_ITERS-1:0]   done;
reg                    all_done;
 
wire [7:0] pixel_read [NUM_ITERS-1:0];
 
wire [20:0] rd_addr;
 
// Create look up table for address modulo,
// since every iterator increments address by one every num_iters pixel
reg [9:0] addresses [XSIZE:0];
integer ii, iii;
initial
begin
    for (ii=0; ii <= XSIZE; ii=ii+1) begin
        addresses[ii] = ii % NUM_ITERS;
    end
end
   
assign rd_addr  = ( ( next_y * (XSIZE+1) ) + next_x ) / NUM_ITERS;
assign rd_color = pixel_read[addresses[next_x]];
// Instantiate solvers
generate
genvar i;
    for (i=0; i < NUM_ITERS; i=i+1) begin: iter
        solver #(NUM_ITERS) s(.all_done   (all_done),
                              .zoom_x(zoom_x),
                              .zoom_y(zoom_y),
                              .x_pos      (x_pos),
                              .y_pos      (y_pos),
                              .rd_addr    (rd_addr),
                              .pixel      (pixel_read[i]),
                              .id         (i),
                              .start      (start),
                              .done       (done[i]),
                              .clk        (clk),
                              .rst        (rst)
            );
    end
 
endgenerate
 
always @ (posedge clk) begin
    if (rst) begin
        start <= 0;
        state <= START;
    end    
    else begin
    case (state)
        START: begin // Tell solvers to start
            start      <= 1;
            cycles_reg <= 0;
            all_done   <= 0;
            state      <= WAIT;
        end
        WAIT: begin // Get input from solvers
            start      <= 1;
            cycles_reg <= cycles_reg + 32'd1; // Time elapsed to compute set
 
            if (&done) begin // check if all bits are high
                state <= DONE;
                all_done <= 1;
            end
        end
        DONE: begin // Output time elapsed and restart
            state  <= START;
            cycles <= cycles_wire;
        end
        default: begin
            state <= START;
            start <= 0;
        end
    endcase
    end
end
endmodule
 
// Solves a subset of the mandelbrot set
module solver#(parameter NUM_ITERS=4) (
    all_done, zoom_x, zoom_y, x_pos, y_pos,
    rd_addr, pixel, id, start, done, clk, rst
);
 
input wire [9:0]   id;
input wire         start;
output reg         done;
input wire         clk;
input wire         rst;
input wire  [26:0] zoom_x;
input wire  [26:0] zoom_y;
input wire  [26:0] x_pos;
input wire  [26:0] y_pos;
input wire  [20:0] rd_addr;
output wire  [7:0] pixel;
input wire         all_done;
 
// "GLOBAL"
parameter XSIZE     = 639;
parameter YSIZE     = 479;
parameter XbyY      = 307199;
parameter MAX_COUNT = 1000;
 
// Useful constants
parameter signed NEGTWO       = 27'b1110_00000000000000000000000;
parameter signed TWO          = 27'b0010_00000000000000000000000;
parameter signed FOUR         = 27'b0100_00000000000000000000000;
 
// First bulb:
// left 320 (-2+3*(left)/640 -> -.5)
// right 482 (.259375)
// top 122 (-1+2*(top/480) -> .49166666667)
// bottom 480-122 (-0.4916666667)
parameter signed BOTTOM       = 27'b1111_01111101110111011101110;
parameter signed TOP          = 27'b0000_01111101110111011101110;
parameter signed LEFT         = 27'b1111_10000000000000000000000;
parameter signed RIGHT        = 27'b0000_01000010011001100110011;
 
// Coordinates of Second bulb:
// left 186 (-1.128125)
// right 241 (-.8703125)
// top 189 (.2125)
// bot 292 (-.2125)
localparam signed BOT2        = -27'sb0000_00110110011001100110011;
localparam signed TOP2        =  27'sb0000_00110110011001100110011;
localparam signed LEFT2       = -27'sb0001_00100000110011001100110;
localparam signed RIGHT2      = -27'sb0000_11011110110011001100110;
 
reg         [18:0] i;
reg         [9:0]  count;
reg         [9:0]  pix_i;
reg         [8:0]  pix_j;
reg signed  [26:0] zim_reg;
reg signed  [26:0] zre_reg;
reg signed  [26:0] zre_reg_p;
reg signed  [26:0] zim_reg_p;
// Full multiplication output
reg signed  [53:0] zim_tmp;
reg signed  [53:0] zre_tmp;
reg signed  [53:0] zimzre_tmp;
 
wire signed [26:0] zre;
wire signed [26:0] zim;
 
wire signed [26:0] zre_sq;
wire signed [26:0] zim_sq;
wire signed [26:0] zimzre;
reg signed  [26:0] cre;
reg signed  [26:0] cim;
 
reg         [20:0] wr_addr;
reg                we;
 
// State machine states
parameter INIT       = 3'd1;
parameter COMPUTE1   = 3'd2;
parameter PIXEL_DONE = 3'd3;
parameter FIN        = 3'd4;
parameter COMPUTE2   = 3'd5;
parameter COMPUTE3   = 3'd6;
reg [2:0] state;
 
// Convert intermediate outputs multiplication back to 4.23 fixed point
assign zre_sq = {zre_tmp[53], zre_tmp[48:23]};
assign zim_sq = {zim_tmp[53], zim_tmp[48:23]};
assign zimzre = {zimzre_tmp[53], zimzre_tmp[48:23]};
 
always @ (posedge clk) begin
    if (rst) begin // Initialize everything on reset
        i       <= {9'b0, id};
        pix_i   <= id;
        pix_j   <= 9'b0;
        we      <= 0;
        wr_addr <= 0;
        done    <= 0;
        state   <= INIT;
    end
    else begin
    case (state)
        INIT: begin // initialize computation variables and wait for scheduler
            done       <= 0;
            cre        <= (x_pos) + ($signed({17'b0, pix_i})*zoom_x);
            cim        <= (y_pos) + ($signed({18'b0, pix_j})*zoom_y);
            count      <= 0;
            zre_reg    <= 0;
            zim_reg    <= 0;
            zre_tmp    <= 0;
            zim_tmp    <= 0;
            zimzre_tmp <= 0;
            state      <= (start) ? COMPUTE1 : INIT;
        end
        COMPUTE1: begin // first stage of computation
            if ( ((cre > LEFT) && (cre < RIGHT) && (cim > BOTTOM) && (cim < TOP)) ||
                ((cre > LEFT2) && (cre < RIGHT2) && (cim > BOT2) && (cim < TOP2)) )
                begin
                state   <= PIXEL_DONE;
                count   <= MAX_COUNT;
                we      <= 1;
                i       <= i + NUM_ITERS;
            end else begin
                count   <= count + 1;
                zre_reg <= zre_sq - zim_sq + cre;
                zim_reg <= (zimzre <<< 1) + cim;
                state   <= COMPUTE2;
            end
        end
        COMPUTE2: begin
            zre_tmp     <= zre_reg * zre_reg;
            zim_tmp     <= zim_reg * zim_reg;
            state       <= COMPUTE3;
        end
        COMPUTE3: begin // finish computation
            if (zre_reg >= TWO || zre_reg <= NEGTWO ||
                zim_reg >= TWO || zim_reg <= NEGTWO ||
                (zre_sq + zim_sq) >= FOUR || count >= MAX_COUNT) begin
                state      <= PIXEL_DONE;
                we         <= 1;
                i          <= i + NUM_ITERS;
            end
            else begin
                zimzre_tmp <= zre_reg * zim_reg;
                state      <= COMPUTE1;
            end
        end
        PIXEL_DONE: begin // Handle next pixel to solve
            we <= 0;
            if (i <= XbyY) begin
                if (pix_i > XSIZE-NUM_ITERS) begin
                    pix_i <= id; // Pixel x-value to start on
                    pix_j <= pix_j + 1;
                end
                else begin
                    pix_i <= pix_i + NUM_ITERS; // evenly apportion pixels to iterators
                end
                wr_addr   <= wr_addr + 1;
                state     <= INIT;
            end
            else state    <= FIN;
        end
        FIN: begin
            wr_addr <= 0;
            pix_i   <= id;
            pix_j   <= 9'b0;
            done    <= 1;
            i       <= {9'd0, id};
            state   <= all_done ? INIT : FIN; // recompute when all iterators done
        end
        default: begin
            state <= INIT;
        end
    endcase
    end
end
 
// Instantiate solver memory
M10K_ITER_8 #(NUM_ITERS) vga_mem (.q(pixel),
    .d(count[9:2]),
    .write_address(wr_addr),
    .read_address(rd_addr),
    .we(we),
    .clk(clk)
);
endmodule
 
// Individual solver memory module, creates M10K memory
module M10K_ITER_8#(parameter NUM_ITERS=4) (q, d, write_address, read_address, we, clk);
    output reg [7:0] q;
    input [7:0] d;
    input [20:0] write_address, read_address;
    input we, clk;
 
    reg [7:0] mem [307200/NUM_ITERS:0] /*synthesis ramstyle = "no_rw_check, M10K" */;
   
    always @ (posedge clk) begin
        if (we) begin
            mem[write_address] <= d;
        end
        q <= mem[read_address];
    end
endmodule
