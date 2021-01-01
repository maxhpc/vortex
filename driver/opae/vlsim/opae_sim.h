#pragma once

#include "Vvortex_afu_shim.h"
#include "Vvortex_afu_shim__Syms.h"
#include "verilated.h"

#ifdef VCD_OUTPUT
#include <verilated_vcd_c.h>
#endif

#include <VX_config.h>
#include "ram.h"

#include <ostream>
#include <future>
#include <list>
#include <unordered_map>

#define CACHE_BLOCK_SIZE 64

class opae_sim {
public:
  
  opae_sim();
  virtual ~opae_sim();

  int prepare_buffer(uint64_t len, void **buf_addr, uint64_t *wsid, int flags);

  void release_buffer(uint64_t wsid);

  void get_io_address(uint64_t wsid, uint64_t *ioaddr);

  void write_mmio64(uint32_t mmio_num, uint64_t offset, uint64_t value);

  void read_mmio64(uint32_t mmio_num, uint64_t offset, uint64_t *value);

  void flush();

private: 

  typedef struct {
    int cycles_left;  
    std::array<uint8_t, CACHE_BLOCK_SIZE> data;
    uint32_t addr;
  } dram_rd_req_t;

  typedef struct {
    int cycles_left;  
    std::array<uint8_t, CACHE_BLOCK_SIZE> data;
    uint64_t addr;
    uint32_t mdata;
  } cci_rd_req_t;

  typedef struct {
    int cycles_left;  
    uint32_t mdata;
  } cci_wr_req_t;

  typedef struct {    
    uint64_t* data;
    size_t    size;
    uint64_t  ioaddr;  
  } host_buffer_t;

  void reset();

  void eval();

  void step();

  void sRxPort_bus();
  void sTxPort_bus();
  void avs_bus();

  std::future<void> future_;
  bool stop_;

  std::unordered_map<int64_t, host_buffer_t> host_buffers_;

  std::list<dram_rd_req_t> dram_reads_;

  std::list<cci_rd_req_t> cci_reads_;

  std::list<cci_wr_req_t> cci_writes_;

  std::mutex mutex_;

  RAM ram_;
  Vvortex_afu_shim *vortex_afu_;
#ifdef VCD_OUTPUT
  VerilatedVcdC *trace_;
#endif
};