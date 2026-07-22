
module reset_synchronizer
(
  input  wire  clk_i  ,
  input  wire  rst_ni ,
  output wire  rst_no
);

  reg rst_sync_d0;
  reg rst_sync_d1;

  assign rst_no = rst_sync_d1;

  always @ (posedge clk_i or negedge rst_ni)
  begin
    if (~rst_ni)
    begin
      rst_sync_d0 <= 0;
      rst_sync_d1 <= 0;
    end
    else
    begin
      rst_sync_d0 <= 1;
      rst_sync_d1 <= rst_sync_d0;
    end
  end

endmodule

