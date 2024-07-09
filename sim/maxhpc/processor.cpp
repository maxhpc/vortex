// Copyright © 2019-2023
// 
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
// http://www.apache.org/licenses/LICENSE-2.0
// 
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "processor.h"

#include <verilated.h>

#include "VVortex.h"
#include "VVortex__Syms.h"

#if VM_TRACE_VCD
#include <verilated_vcd_c.h>
#endif

#include <iostream>
#include <fstream>
#include <iomanip>
#include <mem.h>

#include <VX_config.h>
#include <ostream>
#include <list>
#include <queue>
#include <vector>
#include <sstream> 
#include <unordered_map>

#ifndef MEM_CYCLE_RATIO
#define MEM_CYCLE_RATIO -1
#endif

#ifndef TRACE_START_TIME
#define TRACE_START_TIME 0ull
#endif

#ifndef TRACE_STOP_TIME
#define TRACE_STOP_TIME -1ull
#endif

#ifndef VERILATOR_RESET_VALUE
#define VERILATOR_RESET_VALUE 2
#endif

#if (XLEN == 32)
typedef uint32_t Word;
#elif (XLEN == 64)
typedef uint64_t Word;
#else
#error unsupported XLEN
#endif

#define VL_WDATA_GETW(lwp, i, n, w) \
  VL_SEL_IWII(0, n * w, 0, 0, lwp, i * w, w)

using namespace vortex;

///////////////////////////////////////////////////////////////////////////////

static bool trace_enabled = false;
static uint64_t trace_start_time = TRACE_START_TIME;
static uint64_t trace_stop_time  = TRACE_STOP_TIME;

void sim_trace_enable(bool enable) {
  trace_enabled = enable;
}

VerilatedContext *Vcnxt_;
bool sim_trace_enabled() {
 if (Vcnxt_->time() >= trace_start_time 
  && Vcnxt_->time() < trace_stop_time)
   return true;
 return trace_enabled;
}

///////////////////////////////////////////////////////////////////////////////

class Processor::Impl {
public:
  Impl() {
   // force random values for unitialized signals  
   Verilated::randReset(VERILATOR_RESET_VALUE);
   Verilated::randSeed(50);

   // turn off assertion before reset
   Verilated::assertOn(false);

   // create RTL module instance
   Vcnxt_ = new VerilatedContext;
   device_ = new VVortex{Vcnxt_};
    #if VM_TRACE_VCD
     Verilated::traceEverOn(true);
     trace_ = new VerilatedVcdC();
     device_->trace(trace_, 99);
     trace_->open("trace.vcd");
    #endif

   // reset the device
   this->reset();
    
   // Turn on assertion after reset
   Verilated::assertOn(true);
  }

  ~Impl() {
   this->cout_flush();

   #if VM_TRACE_VCD
    trace_->close();
    delete trace_;
   #endif
    
   delete device_;
   delete Vcnxt_;
  }

  void cout_flush() {
    for (auto& buf : print_bufs_) {
      auto str = buf.second.str();
      if (!str.empty()) {
        std::cout << "#" << buf.first << ": " << str << std::endl;
      }
    }
  }

  void attach_ram(void* ram) {
   ram_ = ram;
  }

  int run() {
    int exitcode = 0;

  #ifndef NDEBUG
    std::cout << std::dec << Vcnxt_->time() << ": [sim] run()" << std::endl;
  #endif

    // start execution
    running_ = true;
    device_->reset = 0;

    // wait on device to go busy
    while (!device_->busy) {
      this->tick();
    }

    // wait on device to go idle
    while (device_->busy) {
      if (get_ebreak()) {
        exitcode = (int)get_last_wb_value(3);
        break;  
      }
      this->tick();
    }
    
    // reset device
    this->reset();

    this->cout_flush();

    return exitcode;
  }

  void write_dcr(uint32_t addr, uint32_t value) {
    device_->dcr_wr_valid = 1;
    device_->dcr_wr_addr  = addr;
    device_->dcr_wr_data  = value;
    this->tick();
    device_->dcr_wr_valid = 0;
  }

private:

  void reset() {
   running_ = 0;

   print_bufs_.clear();

   device_->reset = 1;
   device_->dcr_wr_valid = 0;

   for (uint32_t i = 0; i < RESET_DELAY; ++i) {
    this->tick();
   }
  }

  void tick() {
   // clk rising edge
   device_->clk = 1;
   this->eval();
   // clk falling edge
   device_->clk = 0;
   this->eval();

   #ifndef NDEBUG
    fflush(stdout);
   #endif
  }

  void eval() {
   this->eval_avs_bus();
   device_->eval();
   #if VM_TRACE_VCD
    if (sim_trace_enabled()) {
     trace_->dump(Vcnxt_->time());
    } else {
     exit(-1);
    }
   #endif
   Vcnxt_->timeInc(1);
  }

bool t[10] = {0};
  void eval_avs_bus() {
   if (device_->reset) {
    device_->mem_req_ready = 0;
    device_->mem_rsp_valid = 0;
   }
    else {
     if (device_->clk) {
      if (device_->mem_rsp_ready) {
       device_->mem_rsp_valid = 0;
      }
      //
      if (device_->mem_req_valid) {
       if (!device_->mem_req_rw) {
        if (ram_ != nullptr) {
         memcpy(device_->mem_rsp_data.data(), ram_+(device_->mem_req_addr*64), 64);
         device_->mem_rsp_tag = device_->mem_req_tag;
         device_->mem_rsp_valid = 1;
        }
       }
        else {
         uint64_t byteen = device_->mem_req_byteen;
         for (int i=0; i<64; i++) {
          if (byteen & (1ul<<i)) {
           *((uint8_t*)ram_+(device_->mem_req_addr*64) +i) = *((uint8_t*)device_->mem_req_data.data() +i);
          }
         }
        }
      }
      device_->mem_req_ready = device_->mem_req_valid;
     }
    }
  }

  void wait(uint32_t cycles) {
    for (int i = 0; i < cycles; ++i) {
      this->tick();
    }
  }

  bool get_ebreak() const {
    return (bool)device_->Vortex->sim_ebreak;
  }

  uint64_t get_last_wb_value(int reg) const {
    return ((Word*)device_->Vortex->sim_wb_value.data())[reg];
  }

private:

  typedef struct {    
    bool ready;  
    std::array<uint8_t, MEM_BLOCK_SIZE> block;
    uint64_t addr;
    uint64_t tag;
    bool write;
  } mem_req_t;

  VVortex* device_;
#if VM_TRACE_VCD
  VerilatedVcdC *trace_;
#endif

  std::unordered_map<int, std::stringstream> print_bufs_;

  bool running_;

  void* ram_ = nullptr;
};

///////////////////////////////////////////////////////////////////////////////

Processor::Processor() 
  : impl_(new Impl())
{}

Processor::~Processor() {
  delete impl_;
}

void Processor::attach_ram(void* ram) {
  impl_->attach_ram(ram);
}

int Processor::run() {
  return impl_->run();
}

void Processor::write_dcr(uint32_t addr, uint32_t value) {
  return impl_->write_dcr(addr, value);
}
