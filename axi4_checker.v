`include "Macros.v"
`include "axi4dummy.v"
/*
AXI 4 Testing Utility
==================================
Stores and loads data to/from a specified address using full AXI4 interface
Parameters
-------------
| Parameter | Default | Explanation |
|-----------|:-------:|------------:|
| `AWIDTH` | 64 | Bitwidth of the address bus |
| `DWIDTH` | 64 | Bitwidth of the data bus |
| `IDWIDTH` | 4 | Bitwidth of the ID bus |
| `UDWIDTH` | 0 | Bitwidth of the user data bus |
| `NUM_WORDS` | 24 | Depth of the AXI4dummy internal memory |
Interface
------------
                                 +--------------------+
                                 |   AXI4             |
    AXI4 Utility (Master) <>---<>|   DUT (Slave)      |
                                 |                    |
                                 +--------------------+
Notes
-----
- Can't pass signals/nets into tasks to make them generalizable =(
*/

module axi_tester ();

  parameter
    AWIDTH = 64,
    DWIDTH = 64,
    IDWIDTH = 4,
    UWIDTH =  0,
    NUM_WORDS = 24;

  localparam

    DBYTES = (DWIDTH/8),

    AWID = IDWIDTH,
    AWADDR = AWIDTH,
    AWLEN = 8, //# beats in a burst (up to 256 beats for INCR, 16 for WRAP)
    AWSIZE =  3, //# bytes to transfer per beat (power of 2 up to 128)
    AWBURST = 2, // burst type (2'b00 - FIXED, 2'b01 - INCR, OR 2'b10 - WRAP)
    AWCACHE = 4, //NOT USED
    AWPROT = 3, //NOT USED
    AWQOS = 4, //NOT USED
    AWREGION = 4, //NOT USED
    AWUSER = UWIDTH, //NOT USED
    WDATA = DWIDTH,
    WSTRB = DBYTES, //one write strobe bit for each byte of data bus
    WUSER = UWIDTH, //NOT USED
    BID = IDWIDTH,
    BRESP = 2, // write response (2'b00 - OK, 2'b01 - EXOKAY, 2'b10 - SLVERR, 2'b11 - DECERR),
    BUSER = UWIDTH, //NOT USED
    ARID = IDWIDTH,
    ARADDR = AWIDTH,
    ARLEN = 8, //# beats in a burst (up to 256 beats for INCR, 16 for WRAP)
    ARSIZE = 3, //# bytes to transfer per beat
    ARBURST = 2, // burst type (FIXED, INCR, OR WRAP)
    ARCACHE = 4, //NOT USED
    ARPROT = 3, //NOT USED
    ARQOS = 4, //NOT USED
    ARREGION = 4, //NOT USED
    ARUSER = UWIDTH, //NOT USED
    RID = IDWIDTH,
    RDATA = DWIDTH,
    RRESP = 2,
    RUSER = UWIDTH, //NOT USED

    DEBUG = 0;

  reg [AWADDR-1:0]              store_m_axi_awaddr;
  reg [AWID-1:0]                store_m_axi_awid;         // write address ID
  reg [AWLEN-1:0]               store_m_axi_awlen;        // write address burst length
  reg [AWSIZE-1:0]              store_m_axi_awsize;       // write address burst size
  reg [AWBURST-1:0]             store_m_axi_awburst;      // write address burst type
  reg                           store_m_axi_awlock;       // write address lock type
  reg [AWCACHE-1:0]             store_m_axi_awcache;      // write address memory type
  reg [AWPROT-1:0]              store_m_axi_awprot;       // write address protection type
  reg [AWQOS-1:0]               store_m_axi_awqos;        // write address quality of service
  reg [AWREGION-1:0]            store_m_axi_awregion;     // write address region identifier
  reg [AWUSER-1:0]              store_m_axi_awuser;       // write address user defined junk
  reg                           store_m_axi_awvalid;      // write address valid
  wire                          store_s_axi_awready;      // write address ready

  reg [WDATA-1:0]               store_m_axi_wdata;
  reg [WSTRB-1:0]               store_m_axi_wstrb;        // write data mask
  reg                           store_m_axi_wlast;        // write data last (end of burst)
  reg [WUSER-1:0]               store_m_axi_wuser;        // write data user defined junk
  reg                           store_m_axi_wvalid;       // write valid
  wire                          store_s_axi_wready;       // write ready

  wire [BID-1:0]                store_s_axi_bid;          // write response id
  wire [BRESP-1:0]              store_s_axi_bresp;        // write response
  wire [BUSER-1:0]              store_s_axi_buser;        // write response user defined junk
  wire                          store_s_axi_bvalid;       // write response valid
  reg                           store_m_axi_bready;       // write response ready

  reg [ARADDR-1:0]              load_m_axi_araddr;
  reg [ARID-1:0]                load_m_axi_arid;         // read address id
  reg [ARLEN-1:0]               load_m_axi_arlen;        // read address burst length
  reg [ARSIZE-1:0]              load_m_axi_arsize;       // read address burst size
  reg [ARBURST-1:0]             load_m_axi_arburst;      // read address burst type
  reg                           load_m_axi_arlock;       // read address lock type
  reg [ARCACHE-1:0]             load_m_axi_arcache;      // read address memory type
  reg [ARPROT-1:0]              load_m_axi_arprot;       // read address protection type
  reg [ARQOS-1:0]               load_m_axi_arqos;        // read address quality of service
  reg [ARREGION-1:0]            load_m_axi_arregion;     // read address region identifier
  reg [ARUSER-1:0]              load_m_axi_aruser;       // read address user defined junk
  reg                           load_m_axi_arvalid;      // read address valid
  wire                          load_s_axi_arready;      // read address ready

  wire [RID-1:0]                load_s_axi_rid;          // read response id
  reg [RID-1:0] read_id;
  wire [RDATA-1:0]              load_s_axi_rdata;        // read response data
  reg [RDATA-1:0] read_data;
  wire [RRESP-1:0]              load_s_axi_rresp;        // read response
  wire                          load_s_axi_rlast;        // read response last (end of burst)
  wire [RUSER-1:0]              load_s_axi_ruser;        // read response user defined junk
  wire                          load_s_axi_rvalid;       // read valid
  reg                           load_m_axi_rready;       // read ready

  reg [0:0]  clock_axi;
  reg [0:0]  reset_axi_n;


  /* ==== INSTANTIATE AXI4 SLAVE DUT HERE ==== */
    axi4dummy #(
      .NUM_WORDS(             NUM_WORDS),
      .AWIDTH(                AWIDTH),
      .DWIDTH(                DWIDTH),
      .IDWIDTH(               IDWIDTH),
      .INITIAL_VALUE(         {(NUM_WORDS*DWIDTH){1'b0}})
    ) DUT (
      .clock_axi(             clock_axi),
      .reset_axi_n(           reset_axi_n),

      .s_axi_awaddr(          store_m_axi_awaddr),
      .s_axi_awid(            store_m_axi_awid),
      .s_axi_awlen(           store_m_axi_awlen),
      .s_axi_awsize(          store_m_axi_awsize),
      .s_axi_awburst(         store_m_axi_awburst),
      .s_axi_awlock(          store_m_axi_awlock),
      .s_axi_awcache(         store_m_axi_awcache),
      .s_axi_awprot(          store_m_axi_awprot),
      .s_axi_awqos(           store_m_axi_awqos),
      .s_axi_awregion(        store_m_axi_awregion),
      .s_axi_awvalid(         store_m_axi_awvalid),
      .s_axi_awready(         store_s_axi_awready),
      .s_axi_awuser(          store_m_axi_awuser),

      .s_axi_wdata(           store_m_axi_wdata),
      .s_axi_wstrb(           store_m_axi_wstrb),
      .s_axi_wlast(           store_m_axi_wlast),
      .s_axi_wuser(           store_m_axi_wuser),
      .s_axi_wvalid(          store_m_axi_wvalid),
      .s_axi_wready(          store_s_axi_wready),

      .s_axi_bid(             store_s_axi_bid),
      .s_axi_bresp(           store_s_axi_bresp),
      .s_axi_buser(           store_s_axi_buser),
      .s_axi_bvalid(          store_s_axi_bvalid),
      .s_axi_bready(          store_m_axi_bready),

      .s_axi_araddr(          load_m_axi_araddr),
      .s_axi_arid(            load_m_axi_arid),
      .s_axi_arlen(           load_m_axi_arlen),
      .s_axi_arsize(          load_m_axi_arsize),
      .s_axi_arburst(         load_m_axi_arburst),
      .s_axi_arlock(          load_m_axi_arlock),
      .s_axi_arcache(         load_m_axi_arcache),
      .s_axi_arprot(          load_m_axi_arprot),
      .s_axi_arqos(           load_m_axi_arqos),
      .s_axi_arregion(        load_m_axi_arregion),
      .s_axi_arvalid(         load_m_axi_arvalid),
      .s_axi_arready(         load_s_axi_arready),

      .s_axi_rid(             load_s_axi_rid),
      .s_axi_rdata(           load_s_axi_rdata),
      .s_axi_rresp(           load_s_axi_rresp),
      .s_axi_rlast(           load_s_axi_rlast),
      .s_axi_ruser(           load_s_axi_ruser),
      .s_axi_rvalid(          load_s_axi_rvalid),
      .s_axi_rready(          load_m_axi_rready)
      );

  // This block is for gtkwave
  initial
   begin
      $dumpfile("axi_tester.vcd");
      $dumpvars(0,axi_tester);
   end

   //initialize signals to known state
   initial
    begin
      clock_axi = 0;
      store_m_axi_wlast = 0;
      store_m_axi_bready = 1;
      forever #1 clock_axi = !clock_axi;
    end

  /* ==== TEST BEGINS HERE ==== */
  initial begin
    reset(2);
    //Burst Length = AxLEN[7:0] + 1;
    store(4'b0000,8'b00,3'b011, 2'b00, 64'h0000_0000_0000_0000, 64'hDEAD_BEEF_DEAD_BEEF);
        // ( awid, awlen, awsize, awburst, awaddr, wdata)
    #10
    load(4'b0000,8'b00,3'b011, 2'b00, 64'h0000_0000_0000_0000);
    #10
    $finish;
  end
  /* ==== TEST ENDS HERE ==== */

// ======= RESET TASK =========================
/*
During reset the following interface requirements apply:
• a master interface must drive ARVALID, AWVALID, and WVALID LOW
*/

  task reset;
    input [31:0] reset_cycles;

    begin
      reset_axi_n = 0;
      repeat (reset_cycles)
        @(posedge clock_axi) begin
          load_m_axi_arvalid <= 0;
          store_m_axi_awvalid <= 0;
          store_m_axi_wvalid <= 0;
          load_m_axi_rready <= 0;
        end
      reset_axi_n = 1;
      $display ("");
      $display("==== Reset complete, Time = %0d units", $time);
      $display ("");
    end

  endtask

// =========== STORE TASK =========================
/* TODO: Writing data strobes is gonna get a little tricky...
*/
  task automatic store;
    input [AWID-1:0]                awid;         // write address ID
    input [AWLEN-1:0]               awlen;        // write address burst length
    input [AWSIZE-1:0]              awsize;       // write address burst size
    input [AWBURST-1:0]             awburst;      // write address burst type
    input [AWADDR-1:0]              awaddr;
    input [WDATA-1:0]               wdata;

    begin
      store_m_axi_awid = awid;         // write address ID
      store_m_axi_awlen = awlen;        // write address burst length
      store_m_axi_awsize = awsize;       // write address burst size
      store_m_axi_awburst = awburst;      // write address burst type

      // Address Info Valid
        store_m_axi_bready <= 1;
        store_m_axi_awvalid <= 1;
        store_m_axi_awaddr <= awaddr;

      // Data Info valid_out
        store_m_axi_wdata <= wdata;
        store_m_axi_wstrb <= 8'hFF; // 1 bit per byte lane
        store_m_axi_wlast <= 1'b1;
        store_m_axi_wvalid <= 1'b1;


        fork //Wait for three threads all to finish

          // Wait for awready to assert for address handshake
          begin
            while (store_s_axi_awready != 1)
              begin
                @(clock_axi);
              end
            if (DEBUG == 1) $display ("AWREADY = %0d @ Time = %0d units", store_s_axi_awready, $time);
            #1
           store_m_axi_awvalid <= 0;
          end

          // Wait for wready to assert for data handshake
          begin
            while (store_s_axi_wready != 1)
              begin
                @(clock_axi);
            end
            if (DEBUG == 1) $display ("WREADY = %0d @ Time = %0d units", store_s_axi_wready, $time);
            #1
            store_m_axi_wlast <= 0;
            store_m_axi_wvalid <= 0;
          end

          // Wait for write data response
          begin
            while (store_s_axi_bvalid !=1)
              @(clock_axi);
            if (DEBUG == 1) if (store_s_axi_bresp === 2'b00) $display ("BRESP = %0d (OK) @ Time = %0d units", store_s_axi_bresp, $time);
          end

       join

       #1
       $display ("==== Store data = 0x%h, Address = 0x%h", wdata, awaddr);
       $display ("");
    end

   endtask

// LOAD TASK =========================
  task load;
    input  [ARID-1:0]               arid;
    input  [ARLEN-1:0]              arlen;
    input  [ARSIZE-1:0]             arsize;
    input  [ARBURST-1:0]            arburst;
    input  [ARADDR-1:0]             araddr;

    begin
      load_m_axi_arid = arid;         // write address ID
      load_m_axi_arlen = arlen;        // write address burst length
      load_m_axi_arsize = arsize;       // write address burst size
      load_m_axi_arburst = arburst;      // write address burst type

      // Read Address Info Valid
        load_m_axi_arvalid <= 1;
        load_m_axi_araddr <= araddr;

        fork //Wait for two threads all to finish

          // Wait for awready to assert for address handshake, then assert rready for data handshake
          begin
            while (load_s_axi_arready != 1)
              begin
                @(clock_axi);
              end
            if (DEBUG == 1) $display ("ARREADY = %0d @ Time = %0d units", load_s_axi_arready, $time);
            #1
            load_m_axi_arvalid <= 0;
            load_m_axi_rready <= 1;
             #1
            if (DEBUG == 1) begin
              if (load_s_axi_rresp === 2'b00) $display ("RRESP = %0d (OK) @ Time = %0d units", load_s_axi_rresp, $time);
            end
            $display ("==== Loaded data = 0x%h, Address = 0x%h", load_s_axi_rdata, araddr);
            $display ("");
            #1
            load_m_axi_rready <= 0;
          end
       join
     end

  endtask

endmodule
