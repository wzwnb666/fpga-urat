// 基于正点原子设计的简化UART回环测试
// 采用经过验证的UART收发逻辑

module verified_uart_test (
    input wire sys_clk,            // 50MHz系统时钟
    input wire sys_rst_n,          // 系统复位(低有效)
    
    output wire uart_txd,          // UART发送
    input wire uart_rxd,           // UART接收
    
    output wire [3:0] debug_led    // LED指示
);

    // 参数定义
    parameter CLK_FREQ = 50000000;  // 系统时钟频率
    parameter UART_BPS = 115200;    // 串口波特率
    localparam BPS_CNT = CLK_FREQ/UART_BPS; // 波特率计数值 = 434
    
    //=========================================================================
    // UART接收器 (基于正点原子设计)
    //=========================================================================
    
    // 接收相关信号
    reg uart_rxd_d0, uart_rxd_d1;
    reg [15:0] rx_clk_cnt;
    reg [3:0] rx_cnt;
    reg rx_flag;
    reg [7:0] rxdata;
    reg [7:0] uart_recv_data;
    reg uart_recv_done;
    
    wire start_flag;
    
    // 捕获接收端口下降沿(起始位)
    assign start_flag = uart_rxd_d1 & (~uart_rxd_d0);
    
    // 对UART接收端口的数据延迟两个时钟周期
    always @(posedge sys_clk or negedge sys_rst_n) begin 
        if (!sys_rst_n) begin 
            uart_rxd_d0 <= 1'b0;
            uart_rxd_d1 <= 1'b0;          
        end
        else begin
            uart_rxd_d0 <= uart_rxd;                   
            uart_rxd_d1 <= uart_rxd_d0;
        end   
    end
    
    // 接收过程标志
    always @(posedge sys_clk or negedge sys_rst_n) begin         
        if (!sys_rst_n)                                  
            rx_flag <= 1'b0;
        else begin
            if(start_flag)                          // 检测到起始位
                rx_flag <= 1'b1;                   // 进入接收过程
            else if((rx_cnt == 4'd9) && (rx_clk_cnt == BPS_CNT/2))
                rx_flag <= 1'b0;                   // 接收过程结束
            else
                rx_flag <= rx_flag;
        end
    end
    
    // 接收时钟计数器
    always @(posedge sys_clk or negedge sys_rst_n) begin         
        if (!sys_rst_n)                             
            rx_clk_cnt <= 16'd0;                                  
        else if (rx_flag) begin                     // 处于接收过程
            if (rx_clk_cnt < BPS_CNT - 1)
                rx_clk_cnt <= rx_clk_cnt + 1'b1;
            else
                rx_clk_cnt <= 16'd0;               // 计数达一个波特率周期后清零
        end
        else                              				
            rx_clk_cnt <= 16'd0;                   // 接收过程结束，计数器清零
    end
    
    // 接收数据计数器
    always @(posedge sys_clk or negedge sys_rst_n) begin         
        if (!sys_rst_n)                             
            rx_cnt <= 4'd0;
        else if (rx_flag) begin                     // 处于接收过程
            if (rx_clk_cnt == BPS_CNT - 1)         // 计数达一个波特率周期
                rx_cnt <= rx_cnt + 1'b1;           // 接收数据计数器加1
            else
                rx_cnt <= rx_cnt;       
        end
        else
            rx_cnt <= 4'd0;                        // 接收过程结束，计数器清零
    end
    
    // 数据接收
    always @(posedge sys_clk or negedge sys_rst_n) begin 
        if (!sys_rst_n)  
            rxdata <= 8'd0;                                     
        else if(rx_flag)                            // 系统处于接收过程
            if (rx_clk_cnt == BPS_CNT/2) begin     // 在数据位中间采样
                case (rx_cnt)
                 4'd1 : rxdata[0] <= uart_rxd_d1;  // 数据位最低位
                 4'd2 : rxdata[1] <= uart_rxd_d1;
                 4'd3 : rxdata[2] <= uart_rxd_d1;
                 4'd4 : rxdata[3] <= uart_rxd_d1;
                 4'd5 : rxdata[4] <= uart_rxd_d1;
                 4'd6 : rxdata[5] <= uart_rxd_d1;
                 4'd7 : rxdata[6] <= uart_rxd_d1;
                 4'd8 : rxdata[7] <= uart_rxd_d1;  // 数据位最高位
                 default:;                                    
                endcase
            end
            else 
                rxdata <= rxdata;
        else
            rxdata <= 8'd0;
    end
    
    // 接收完成标志和数据输出
    always @(posedge sys_clk or negedge sys_rst_n) begin        
        if (!sys_rst_n) begin
            uart_recv_data <= 8'd0;                               
            uart_recv_done <= 1'b0;
        end
        else if(rx_cnt == 4'd9) begin               // 计数到停止位时           
            uart_recv_data <= rxdata;               // 输出接收到的数据
            uart_recv_done <= 1'b1;                // 接收完成标志
        end
        else begin
            uart_recv_data <= 8'd0;                                   
            uart_recv_done <= 1'b0; 
        end    
    end
    
    //=========================================================================
    // UART发送器 (基于正点原子设计)
    //=========================================================================
    
    // 发送相关信号
    reg uart_send_en;
    reg [7:0] uart_send_data;
    reg uart_en_d0, uart_en_d1;
    reg [15:0] tx_clk_cnt;
    reg [3:0] tx_cnt;
    reg tx_flag;
    reg [7:0] tx_data;
    reg uart_txd_reg;
    
    wire en_flag;
    wire uart_tx_busy;
    
    // 发送忙状态标志
    assign uart_tx_busy = tx_flag;
    assign uart_txd = uart_txd_reg;
    
    // 捕获发送使能上升沿
    assign en_flag = (~uart_en_d1) & uart_en_d0;
    
    // 对发送使能信号延迟两个时钟周期
    always @(posedge sys_clk or negedge sys_rst_n) begin         
        if (!sys_rst_n) begin
            uart_en_d0 <= 1'b0;                                  
            uart_en_d1 <= 1'b0;
        end                                                      
        else begin                                               
            uart_en_d0 <= uart_send_en;                               
            uart_en_d1 <= uart_en_d0; 		  
        end
    end
    
    // 发送过程控制
    always @(posedge sys_clk or negedge sys_rst_n) begin         
        if (!sys_rst_n) begin                                  
            tx_flag <= 1'b0;
            tx_data <= 8'd0;
        end 
        else if (en_flag) begin                     // 检测到发送使能上升沿                      
                tx_flag <= 1'b1;                   // 进入发送过程				
                tx_data <= uart_send_data;         // 寄存待发送的数据
            end
            else if ((tx_cnt == 4'd9) && (tx_clk_cnt == BPS_CNT - (BPS_CNT/16))) begin                                       
                tx_flag <= 1'b0;                   // 发送过程结束				
                tx_data <= 8'd0;
            end
            else begin
                tx_flag <= tx_flag;
                tx_data <= tx_data;
            end 
    end
    
    // 发送时钟计数器
    always @(posedge sys_clk or negedge sys_rst_n) begin         
        if (!sys_rst_n)                             
            tx_clk_cnt <= 16'd0;                                  
        else if (tx_flag) begin                     // 处于发送过程
            if (tx_clk_cnt < BPS_CNT - 1)
                tx_clk_cnt <= tx_clk_cnt + 1'b1;
            else
                tx_clk_cnt <= 16'd0;               // 计数达一个波特率周期后清零
        end
        else                             
            tx_clk_cnt <= 16'd0; 				    // 发送过程结束
    end
    
    // 发送数据计数器
    always @(posedge sys_clk or negedge sys_rst_n) begin         
        if (!sys_rst_n)                             
            tx_cnt <= 4'd0;
        else if (tx_flag) begin                     // 处于发送过程
            if (tx_clk_cnt == BPS_CNT - 1)         // 计数达一个波特率周期
                tx_cnt <= tx_cnt + 1'b1;           // 发送数据计数器加1
            else
                tx_cnt <= tx_cnt;       
        end
        else                              
            tx_cnt <= 4'd0;				            // 发送过程结束
    end
    
    // 根据发送数据计数器给uart发送端口赋值
    always @(posedge sys_clk or negedge sys_rst_n) begin        
        if (!sys_rst_n)  
            uart_txd_reg <= 1'b1;        
        else if (tx_flag)
            case(tx_cnt)
                4'd0: uart_txd_reg <= 1'b0;         // 起始位 
                4'd1: uart_txd_reg <= tx_data[0];   // 数据位最低位
                4'd2: uart_txd_reg <= tx_data[1];
                4'd3: uart_txd_reg <= tx_data[2];
                4'd4: uart_txd_reg <= tx_data[3];
                4'd5: uart_txd_reg <= tx_data[4];
                4'd6: uart_txd_reg <= tx_data[5];
                4'd7: uart_txd_reg <= tx_data[6];
                4'd8: uart_txd_reg <= tx_data[7];   // 数据位最高位
                4'd9: uart_txd_reg <= 1'b1;         // 停止位
                default: ;
            endcase
        else 
            uart_txd_reg <= 1'b1;                   // 空闲时为高电平
    end
    
    //=========================================================================
    // 串口环回逻辑 (基于正点原子设计)
    //=========================================================================
    
    reg recv_done_d0, recv_done_d1;
    wire recv_done_flag;
    
    // 捕获接收完成上升沿
    assign recv_done_flag = (~recv_done_d1) & recv_done_d0;
    
    // 对接收完成信号延迟两个时钟周期
    always @(posedge sys_clk or negedge sys_rst_n) begin         
        if (!sys_rst_n) begin
            recv_done_d0 <= 1'b0;                                  
            recv_done_d1 <= 1'b0;
        end                                                      
        else begin                                               
            recv_done_d0 <= uart_recv_done;                               
            recv_done_d1 <= recv_done_d0; 		  
        end
    end
    
    // 环回逻辑
    always @(posedge sys_clk or negedge sys_rst_n) begin         
        if (!sys_rst_n) begin
            uart_send_en <= 1'b0;                                  
            uart_send_data <= 8'd0;
        end                                                      
        else if (recv_done_flag && !uart_tx_busy) begin  // 收到数据且发送器不忙                              
            uart_send_en <= 1'b1;                               
            uart_send_data <= uart_recv_data;   // 直接回环		  
        end
        else begin
            uart_send_en <= 1'b0;
            uart_send_data <= uart_send_data;
        end
    end
    
    //=========================================================================
    // UART控制LED验证模块
    //=========================================================================
    
    reg [2:0] led_ctrl;  // 控制LED0-LED2
    
    // 根据接收到的UART数据控制LED
    always @(posedge sys_clk or negedge sys_rst_n) begin         
        if (!sys_rst_n) begin
            led_ctrl <= 3'b000;
        end                                                      
        else if (recv_done_flag) begin  // 收到UART数据时                              
            case (uart_recv_data)
                8'h61: led_ctrl <= 3'b001;    // 收到0x61，LED0亮起
                8'h62: led_ctrl <= 3'b010;    // 收到0x62，LED1亮起
                8'h63: led_ctrl <= 3'b100;    // 收到0x63，LED2亮起
                8'h64: led_ctrl <= 3'b000;    // 收到0x64，LED0-LED2全部熄灭
                default: led_ctrl <= led_ctrl;  // 其他数据不改变LED状态
            endcase
        end
        else begin
            led_ctrl <= led_ctrl;  // 保持当前LED状态
        end
    end
    
    // LED3收发数据闪烁逻辑
    reg [19:0] blink_cnt;
    reg led3_blink;
    reg uart_activity;
    
    // 检测UART收发活动
    always @(posedge sys_clk or negedge sys_rst_n) begin
        if (!sys_rst_n) begin
            uart_activity <= 1'b0;
        end
        else begin
            uart_activity <= rx_flag | tx_flag;  // 收发过程中置高
        end
    end
    
    // 闪烁计数器和LED3控制
    always @(posedge sys_clk or negedge sys_rst_n) begin
        if (!sys_rst_n) begin
            blink_cnt <= 20'd0;
            led3_blink <= 1'b0;
        end
        else if (uart_activity) begin  // 收发数据时闪烁
            if (blink_cnt < 20'd250000) begin  // 约5ms闪烁周期
                blink_cnt <= blink_cnt + 1'b1;
            end
            else begin
                blink_cnt <= 20'd0;
                led3_blink <= ~led3_blink;  // 翻转LED3状态
            end
        end
        else begin
            blink_cnt <= 20'd0;
            led3_blink <= 1'b0;  // 无活动时熄灭
        end
    end
    
    //=========================================================================
    // LED指示
    //=========================================================================
    assign debug_led[0] = led_ctrl[0];      // LED0 - 由UART控制 (0x61)
    assign debug_led[1] = led_ctrl[1];      // LED1 - 由UART控制 (0x62) 
    assign debug_led[2] = led_ctrl[2];      // LED2 - 由UART控制 (0x63)
    assign debug_led[3] = led3_blink;       // LED3 - 收发数据时闪烁

endmodule
