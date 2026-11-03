`timescale 1ns/1ps
// Deployed YOLO labels, generic VLM label, and independent HSV ball labels.
module detection_class_label (
    input wire [6:0] class_id,
    input wire [3:0] char_index,
    input wire [119:0] custom_text,
    input wire [3:0] custom_length,
    output reg supported,
    output reg [3:0] text_length,
    output wire [6:0] label_width,
    output reg [6:0] ascii
);
    assign label_width = {text_length,3'b000};
    always @* begin
        supported = 1'b1;
        text_length = 4'd0;
        case (class_id)
            7'd79:begin // VLM TARGET fallback for older Pi software
                if(custom_length!=0)text_length=custom_length;else text_length=4'd10;
            end
            7'd0: text_length = 4'd6; // person
            7'd24: text_length = 4'd8; // backpack
            7'd25: text_length = 4'd8; // umbrella
            7'd26: text_length = 4'd7; // handbag
            7'd32: text_length = 4'd11; // sports ball
            7'd39: text_length = 4'd6; // bottle
            7'd41: text_length = 4'd3; // cup
            7'd46: text_length = 4'd6; // banana
            7'd47: text_length = 4'd5; // apple
            7'd56: text_length = 4'd5; // chair
            7'd64: text_length = 4'd5; // mouse
            7'd67: text_length = 4'd10; // cell phone
            7'd73: text_length = 4'd4; // book
            7'd80: text_length = 4'd8; // Red Ball
            7'd81: text_length = 4'd11; // Yellow Ball
            7'd82: text_length = 4'd10; // Green Ball
            7'd83: text_length = 4'd9; // Blue Ball
            default: supported = 1'b0;
        endcase
    end
    always @* begin
        ascii = 7'd32;
        if(class_id==79)begin
            if(custom_length!=0)begin
                if(char_index<custom_length)ascii=custom_text[118-char_index*8-:7];
            end else case(char_index)
                0:ascii="V";1:ascii="L";2:ascii="M";4:ascii="T";5:ascii="A";
                6:ascii="R";7:ascii="G";8:ascii="E";9:ascii="T";default:;
            endcase
        end else
        case ({class_id,char_index})
            // person
            11'h000: ascii = 7'd112; // p
            11'h001: ascii = 7'd101; // e
            11'h002: ascii = 7'd114; // r
            11'h003: ascii = 7'd115; // s
            11'h004: ascii = 7'd111; // o
            11'h005: ascii = 7'd110; // n
            // backpack
            11'h180: ascii = 7'd98; // b
            11'h181: ascii = 7'd97; // a
            11'h182: ascii = 7'd99; // c
            11'h183: ascii = 7'd107; // k
            11'h184: ascii = 7'd112; // p
            11'h185: ascii = 7'd97; // a
            11'h186: ascii = 7'd99; // c
            11'h187: ascii = 7'd107; // k
            // umbrella
            11'h190: ascii = 7'd117; // u
            11'h191: ascii = 7'd109; // m
            11'h192: ascii = 7'd98; // b
            11'h193: ascii = 7'd114; // r
            11'h194: ascii = 7'd101; // e
            11'h195: ascii = 7'd108; // l
            11'h196: ascii = 7'd108; // l
            11'h197: ascii = 7'd97; // a
            // handbag
            11'h1a0: ascii = 7'd104; // h
            11'h1a1: ascii = 7'd97; // a
            11'h1a2: ascii = 7'd110; // n
            11'h1a3: ascii = 7'd100; // d
            11'h1a4: ascii = 7'd98; // b
            11'h1a5: ascii = 7'd97; // a
            11'h1a6: ascii = 7'd103; // g
            // sports ball
            11'h200: ascii = 7'd115; // s
            11'h201: ascii = 7'd112; // p
            11'h202: ascii = 7'd111; // o
            11'h203: ascii = 7'd114; // r
            11'h204: ascii = 7'd116; // t
            11'h205: ascii = 7'd115; // s
            11'h207: ascii = 7'd98; // b
            11'h208: ascii = 7'd97; // a
            11'h209: ascii = 7'd108; // l
            11'h20a: ascii = 7'd108; // l
            // bottle
            11'h270: ascii = 7'd98; // b
            11'h271: ascii = 7'd111; // o
            11'h272: ascii = 7'd116; // t
            11'h273: ascii = 7'd116; // t
            11'h274: ascii = 7'd108; // l
            11'h275: ascii = 7'd101; // e
            // cup
            11'h290: ascii = 7'd99; // c
            11'h291: ascii = 7'd117; // u
            11'h292: ascii = 7'd112; // p
            // banana
            11'h2e0: ascii = 7'd98; // b
            11'h2e1: ascii = 7'd97; // a
            11'h2e2: ascii = 7'd110; // n
            11'h2e3: ascii = 7'd97; // a
            11'h2e4: ascii = 7'd110; // n
            11'h2e5: ascii = 7'd97; // a
            // apple
            11'h2f0: ascii = 7'd97; // a
            11'h2f1: ascii = 7'd112; // p
            11'h2f2: ascii = 7'd112; // p
            11'h2f3: ascii = 7'd108; // l
            11'h2f4: ascii = 7'd101; // e
            // chair
            11'h380: ascii = 7'd99; // c
            11'h381: ascii = 7'd104; // h
            11'h382: ascii = 7'd97; // a
            11'h383: ascii = 7'd105; // i
            11'h384: ascii = 7'd114; // r
            // mouse
            11'h400: ascii = 7'd109; // m
            11'h401: ascii = 7'd111; // o
            11'h402: ascii = 7'd117; // u
            11'h403: ascii = 7'd115; // s
            11'h404: ascii = 7'd101; // e
            // cell phone
            11'h430: ascii = 7'd99; // c
            11'h431: ascii = 7'd101; // e
            11'h432: ascii = 7'd108; // l
            11'h433: ascii = 7'd108; // l
            11'h435: ascii = 7'd112; // p
            11'h436: ascii = 7'd104; // h
            11'h437: ascii = 7'd111; // o
            11'h438: ascii = 7'd110; // n
            11'h439: ascii = 7'd101; // e
            // book
            11'h490: ascii = 7'd98; // b
            11'h491: ascii = 7'd111; // o
            11'h492: ascii = 7'd111; // o
            11'h493: ascii = 7'd107; // k
            11'h500: ascii = 7'd82; // R
            11'h501: ascii = 7'd101; // e
            11'h502: ascii = 7'd100; // d
            11'h503: ascii = 7'd32; //  
            11'h504: ascii = 7'd66; // B
            11'h505: ascii = 7'd97; // a
            11'h506: ascii = 7'd108; // l
            11'h507: ascii = 7'd108; // l
            11'h510: ascii = 7'd89; // Y
            11'h511: ascii = 7'd101; // e
            11'h512: ascii = 7'd108; // l
            11'h513: ascii = 7'd108; // l
            11'h514: ascii = 7'd111; // o
            11'h515: ascii = 7'd119; // w
            11'h516: ascii = 7'd32; //  
            11'h517: ascii = 7'd66; // B
            11'h518: ascii = 7'd97; // a
            11'h519: ascii = 7'd108; // l
            11'h51a: ascii = 7'd108; // l
            11'h520: ascii = 7'd71; // G
            11'h521: ascii = 7'd114; // r
            11'h522: ascii = 7'd101; // e
            11'h523: ascii = 7'd101; // e
            11'h524: ascii = 7'd110; // n
            11'h525: ascii = 7'd32; //  
            11'h526: ascii = 7'd66; // B
            11'h527: ascii = 7'd97; // a
            11'h528: ascii = 7'd108; // l
            11'h529: ascii = 7'd108; // l
            11'h530: ascii = 7'd66; // B
            11'h531: ascii = 7'd108; // l
            11'h532: ascii = 7'd117; // u
            11'h533: ascii = 7'd101; // e
            11'h534: ascii = 7'd32; //  
            11'h535: ascii = 7'd66; // B
            11'h536: ascii = 7'd97; // a
            11'h537: ascii = 7'd108; // l
            11'h538: ascii = 7'd108; // l
            default: ascii = 7'd32;
        endcase
    end
endmodule
