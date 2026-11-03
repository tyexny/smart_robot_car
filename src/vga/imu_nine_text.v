`timescale 1ns/1ps
// Shared character decoder for the SW1 panel and the wheel-page sidebar.
// Fields are AX AY AZ GX GY GZ ROLL PITCH RELATIVE YAW, in that order.
// The existing frame-snapshotted telemetry formatter supplies the BCD words.
module imu_nine_text #(parameter integer GROUPED=0)(
    input wire [3:0] text_row,
    input wire [6:0] text_column,
    input wire [215:0] values_bcd,
    input wire [8:0] values_negative,values_valid,
    output reg [6:0] ascii
);
    wire [3:0] data_row=GROUPED ? (text_row[3:2]*3+{2'b0,text_row[1:0]}) : text_row;
    wire heading_row=GROUPED ? text_row[1:0]==0 : text_row==0;
    wire [62:0] field_ascii;
    imu_nine_fields fields(.text_column(text_column),.values_bcd(values_bcd),
        .values_negative(values_negative),.values_valid(values_valid),.field_ascii(field_ascii));
    reg [159:0] group_title;
    integer row_index;
    always @* begin
        group_title=text_row[3:2]==0?"ACCELERATION        ":text_row[3:2]==1?
            "ANGULAR RATE        ":"RPY / RELATIVE YAW  ";
        ascii=0;
        for(row_index=0;row_index<9;row_index=row_index+1)
            ascii=ascii | ({7{data_row==row_index+1}} & field_ascii[row_index*7+:7]);
        if(data_row<1 || data_row>9)ascii=32;
        if(GROUPED && heading_row)begin
            ascii=32;
            if(text_column<20)ascii=group_title[(19-text_column)*8+:7];
        end else if(!GROUPED && heading_row) begin
            ascii=32;
            case(text_column)
                0:ascii="I";1:ascii="M";2:ascii="U";
                4:ascii="/";6:ascii="Y";7:ascii="A";8:ascii="W";
                10:ascii="R";11:ascii="E";12:ascii="L";
                default:;
            endcase
        end
    end
endmodule

// Parallel field decoder shared by the combinational panel and the pipelined
// wheel sidebar. Each output owns fixed BCD slices, labels and units.
module imu_nine_fields(
    input wire [6:0] text_column,
    input wire [215:0] values_bcd,
    input wire [8:0] values_negative,values_valid,
    output wire [62:0] field_ascii
);
    // Constant digit slices are evaluated before the row mux, rather than
    // shifting a row-selected 24-bit number through another variable mux.
    function [6:0] field_glyph;
        input [6:0] column;
        input [23:0] value;
        input negative,valid,acceleration;
        input [47:0] label;
        input [39:0] unit_text;
        begin
            field_glyph=32;
            case(column)
                0:field_glyph=label[46:40];1:field_glyph=label[38:32];
                2:field_glyph=label[30:24];3:field_glyph=label[22:16];
                4:field_glyph=label[14:8];5:field_glyph=label[6:0];
                6:field_glyph=!valid ? "-" : negative ? "-" : "+";
                7:field_glyph=!valid ? "-" : 7'd48+{3'd0,value[23:20]};
                8:field_glyph=!valid ? "-" : 7'd48+{3'd0,value[19:16]};
                9:field_glyph=!valid ? "-" : 7'd48+{3'd0,value[15:12]};
                10:field_glyph=!valid ? "-" : acceleration ? "." : 7'd48+{3'd0,value[11:8]};
                11:field_glyph=!valid ? "-" : 7'd48+{3'd0,(acceleration ? value[11:8] : value[7:4])};
                12:field_glyph=!valid ? "-" : acceleration ? 7'd48+{3'd0,value[7:4]} : ".";
                13:field_glyph=!valid ? "-" : 7'd48+{3'd0,value[3:0]};
                15:field_glyph=unit_text[38:32];16:field_glyph=unit_text[30:24];
                17:field_glyph=unit_text[22:16];18:field_glyph=unit_text[14:8];
                19:field_glyph=unit_text[6:0];
                default:;
            endcase
        end
    endfunction
    genvar field;
    generate for(field=0;field<9;field=field+1)begin:g_fields
        localparam [47:0] LABEL=field==0 ? "AX    " : field==1 ? "AY    " : field==2 ? "AZ    " :
            field==3 ? "GX    " : field==4 ? "GY    " : field==5 ? "GZ    " :
            field==6 ? "ROLL  " : field==7 ? "PITCH " : "YAW   ";
        localparam [39:0] UNIT_TEXT=field<3 ? "g    " : field<6 ? "deg/s" : "deg  ";
        assign field_ascii[field*7+:7]=field_glyph(text_column,values_bcd[field*24+:24],
            values_negative[field],values_valid[field],field<3,LABEL,UNIT_TEXT);
    end endgenerate
endmodule
