 
module Traffic_led_controller(
//Global 
input sys_clk,
input sys_rst_n,
 
//
input [1:0] sw,			//2 bits switch to control four states of vehicles
 
 
 
//
output R1,G1,Y1,		//main road traffic light
output R2,G2,Y2			//trunk road traffic light
	
 
	);
//
 
reg [5:0] led_reg;
assign G1 = led_reg[5];
assign R1 = led_reg[4];
assign Y1 = led_reg[3];
assign G2 = led_reg[2];
assign R2 = led_reg[1];
assign Y2 = led_reg[0];
 
 
 
 
//
parameter time_cycle1 = 45;//main road release time
parameter time_cycle2 = 25;//trunk road release time
parameter time_cycle3 = 5; //yellow light time
 
//Three-stage state machine
 
reg [3:0] current_state;
reg [3:0] next_state;
 
 
reg state_change_flag;  //状态转移标志位，同时作为黄灯亮标志位
reg [1:0]state_alt; //表示在“有有”时，交替同行状态  主干道-> 支路 支路->主干道
 
parameter IDLE = 4'b0001;  //无无
parameter S1 = 4'b0010;		//无有
parameter S2 = 4'b0100;		//有无
parameter S3 = 4'b1000;		//有有
//三段式状态机第一步：时序逻辑实现状态转移
always @(posedge sys_clk or negedge sys_rst_n) begin
	if (!sys_rst_n) begin
		current_state <= IDLE;
	end
	else begin
		current_state <= next_state;
	end
end
 
//组合逻辑描述状态转移条件
always @(*) begin
	if (!sys_rst_n) begin
		next_state = IDLE;
	end
	else begin
		case(current_state)
			IDLE: begin
				if (sw == 2'b00) begin
					next_state = IDLE;
				end
				else if (sw == 2'b01) begin
					next_state = S1;
				end
				else if (sw == 2'b10) begin
					next_state = S2;
				end
				else if (sw == 2'b11) begin
					next_state = S3;
 
				end				
			end
			S1:begin
				if (sw == 2'b00) begin
					next_state = IDLE;
				end
				else if (sw == 2'b01) begin
					next_state = S1;
				end
				else if (sw == 2'b10) begin
					next_state = S2;
				end
				else if (sw == 2'b11) begin
					next_state = S3;
				end				
			end
			S2:begin
				if (sw == 2'b00) begin
					next_state = IDLE;
				end
				else if (sw == 2'b01) begin
					next_state = S1;
				end
				else if (sw == 2'b10) begin
					next_state = S2;		
				end
				else if (sw == 2'b11) begin
					next_state = S3;
				end				
			end
			S3:begin
				if (sw == 2'b00) begin
					next_state = IDLE;
				end
				else if (sw == 2'b01) begin
					next_state = S1;
				end
				else if (sw == 2'b10) begin
					next_state = S2;				
				end
				else if (sw == 2'b11) begin
					next_state = S3;
				end			
			end
			default: next_state = IDLE;
		endcase
	end
end
 
 
//状态机第三步：时序逻辑描述输出
 
reg [6:0] G_time_cnt;  //green light time count 用于“有有状态”
reg [5:0] Y_time_cnt;   //yellow light time count 用于前三个状态
 
//第四个状态的绿灯黄灯计数器
always @(posedge sys_clk or negedge sys_rst_n) begin
	if (!sys_rst_n) begin
		G_time_cnt <= time_cycle1 + time_cycle2 + time_cycle3 + time_cycle3;
		state_alt <= 2'b00;
	end
	else if (state_change_flag == 1'b0)begin
		if (G_time_cnt > 7'd0) begin
			G_time_cnt <= G_time_cnt - 1'b1;
			if (G_time_cnt == time_cycle2 + 2 * time_cycle3) begin
				state_alt <= 2'b01;
			end
			else if (G_time_cnt == time_cycle2 + time_cycle3)begin
				state_alt <= 2'b10;
			end
			else if (G_time_cnt == time_cycle3) begin
				state_alt <= 2'b11;
			end
		end
		else if (G_time_cnt == 7'd0) begin
			state_alt <= 2'b00;
			G_time_cnt <= time_cycle1 + time_cycle2 + 2* time_cycle3;
		end
	end
end
//前三个状态的黄灯计数器
always @(posedge sys_clk or negedge sys_rst_n) begin
	if (!sys_rst_n) begin
		Y_time_cnt <= time_cycle3;
	end
	else if (state_change_flag == 1'b1 )begin
		if (Y_time_cnt > 6'd0) 
			Y_time_cnt <= Y_time_cnt - 1'b1;
		else if (Y_time_cnt == 6'd0)begin
			Y_time_cnt <= time_cycle3;
			// state_change_flag <= ~state_change_flag;
		end
	end
end
//state_change_flag 变化
always @(*) begin
	if (!sys_rst_n) begin
		state_change_flag = 1'b0;
	end
	else if (Y_time_cnt == 6'd0 && state_change_flag == 1'b1) begin
		state_change_flag <= ~state_change_flag;
	end
	else begin
		case(next_state)
			IDLE: begin
				if (current_state == IDLE || current_state == S2 || current_state == S3) begin
					state_change_flag = state_change_flag;
				end
				else if (current_state == S1) begin
					state_change_flag = ~state_change_flag;
				end
			end
			S1: begin
				if (current_state == IDLE || current_state == S2 || current_state == S3) begin
					state_change_flag = ~state_change_flag;
				end
				else if (current_state == S1) begin
					state_change_flag = state_change_flag;
				end
			end
			S2: begin
				if (current_state == IDLE || current_state == S2 || current_state == S3) begin
					state_change_flag = state_change_flag;
				end
				else if (current_state == S1) begin
					state_change_flag = ~state_change_flag;
				end
			end
			S3: begin
				if (current_state == IDLE || current_state == S2 || current_state == S3) begin
					state_change_flag = state_change_flag;
				end
				else if (current_state == S1) begin
					state_change_flag = ~state_change_flag;
				end
			end
			default: state_change_flag = 1'b0;
		endcase
	end
end
//状态机第三步：根据不同状态输出
always @(posedge sys_clk or negedge sys_rst_n) begin
	if (!sys_rst_n) begin
		led_reg <= 6'b000_000;
	end
	else begin
		case(next_state)
			IDLE: begin
				if (state_change_flag == 1'b1 && Y_time_cnt < time_cycle3) begin
					led_reg <= 6'b010_001;
				end
				else if (state_change_flag == 1'b0) begin
					led_reg <= 6'b100_010;
				end
			end
			S1: begin
				if (state_change_flag == 1'b1 && Y_time_cnt < time_cycle3) begin
					led_reg <= 6'b001_010;
				end
				else if (state_change_flag == 1'b0) begin
					led_reg <= 6'b010_100;
				end
			end
			S2: begin
				if (state_change_flag == 1'b1 && Y_time_cnt < time_cycle3) begin
					led_reg <= 6'b010_001;
				end
				else if (state_change_flag == 1'b0) begin
					led_reg <= 6'b100_010;
				end
			end
			S3: begin
				if (state_change_flag == 1'b1 && Y_time_cnt < time_cycle3) begin
					led_reg <= 6'b010_001;
				end
				else if (state_change_flag == 1'b0) begin
					if (state_alt == 2'b00) begin
						led_reg <= 6'b100_010;
					end
					else if (state_alt == 2'b01) begin
						led_reg <= 6'b001_010;
					end
					else if (state_alt == 2'b10) begin
						led_reg <= 6'b010_100;
					end
					else if (state_alt == 2'b11) begin
						led_reg <= 6'b010_001;
					end
				end
			end
			default: led_reg <= 6'b000_000;
		endcase
	end
end
 
endmodule
	
 
 
 
 
// 	else if (next_state == IDLE) begin
// 		if ((state_change_flag == 1'b1) && (Y_time_cnt < time_cycle3))
// 			led_reg <= 6'b010_001;
// 		else
// 			led_reg <= 6'b100_010;
// 	end
// 	else if (next_state == S1) begin
// 		if ((state_change_flag == 1'b1) && (Y_time_cnt < time_cycle3))
// 			led_reg <= 6'b001_010;
// 		else
// 			led_reg <= 6'b010_100;		
// 	end
// 	else if (next_state == S2) begin
// 		if ((state_change_flag == 1'b1) && (Y_time_cnt < time_cycle3))
// 			led_reg <= 6'b010_001;
// 		else 
// 			led_reg <= 6'b100_010;		
// 	end
// 	else if (next_state == S3) begin
// 		led_reg <= 6'b100_010;
// 	end
// 	else
// 		led_reg <= 6'b000_000;
// end
			// IDLE: begin
			// 	if ((state_change_flag == 1'b1) && (Y_time_cnt < time_cycle3))
			// 		led_reg <= 6'b010_001;
			// 	else
			// 		led_reg <= 6'b100_010;
			// end
			// S1: begin
			// 	if ((state_change_flag == 1'b1) && (Y_time_cnt < time_cycle3))
			// 		led_reg <= 6'b001_010;
			// 	else
			// 		led_reg <= 6'b010_100;
			// end
			// S2: begin
			// 	if ((state_change_flag == 1'b1) && (Y_time_cnt < time_cycle3))
			// 		led_reg <= 6'b010_001;
			// 	else 
			// 		led_reg <= 6'b100_010;
			// end
			// S3: begin
			// 	if ((G_time_cnt < time_cycle1) && (G1_flag == 1'b1))
			// 		led_reg <= 6'b100_010;
			// 	else if (G_time_cnt == time_cycle1) 
			// 		led_reg <= 6'b001_100;
			// end
			// default: led_reg <= 6'b000_000;
