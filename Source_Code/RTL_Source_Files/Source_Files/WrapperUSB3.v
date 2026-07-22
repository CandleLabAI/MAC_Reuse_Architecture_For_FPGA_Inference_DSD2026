
module WrapperUSB3 #(
  parameter DATA_WIDTH = 16
) (
  input   wire                     rst_ni     ,
  input   wire                     ClkUSB_i   ,

// Video input signals
  input   wire                     Fv_i       ,
  input   wire                     Lv_i       ,
  input   wire          [24 - 1:0] DatYCbCr_i ,

// USB3 output signals
  output  wire                     Fv_o       ,
  output  wire                     Lv_o       ,
  output  wire  [DATA_WIDTH - 1:0] Data_o
);

  reg                      Fv          ;
  reg                      Lv          ;
  reg   [DATA_WIDTH - 1:0] Data        ;
  wire  [DATA_WIDTH - 1:0] Data_c      ;

  reg           [24 - 1:0] DatYCbCr_d1 ;
  reg                      data_en     ;
  wire                     data_en_c   ;

  //Output
  assign Fv_o       = Fv   ;
  assign Lv_o       = Lv   ;
  assign Data_o     = Data ;

  assign data_en_c  = Lv_i & ~data_en;
  assign Data_c     = ({DATA_WIDTH{ data_en}} & {DatYCbCr_d1[23:16], DatYCbCr_i[7:0]}) |
                      ({DATA_WIDTH{~data_en}} & {DatYCbCr_i[15:08],  DatYCbCr_i[7:0]}) ;

  always @ (posedge ClkUSB_i or negedge rst_ni)
  begin
    if(~rst_ni)
    begin
      DatYCbCr_d1 <= 0          ;
      data_en     <= 0          ;
      Fv          <= 0          ;
      Lv          <= 0          ;
      Data        <= 0          ;
    end
    else
    begin
      DatYCbCr_d1 <= DatYCbCr_i ;
      data_en     <= data_en_c  ;
      Fv          <= Fv_i       ;
      Lv          <= Lv_i       ;
      Data        <= Data_c     ;
    end
  end

endmodule

