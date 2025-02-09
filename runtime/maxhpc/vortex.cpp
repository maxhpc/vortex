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

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include <iostream>
#include <future>
#include <list>
#include <chrono>

#include <vortex.h>
#include <malloc.h>
#include <utils.h>
#include <VX_config.h>
#include <VX_types.h>

#include <util.h>

#include <maxhpc.h>


#ifndef NDEBUG
#define DBGPRINT(format, ...) do { printf("[VXDRV] " format "", ##__VA_ARGS__); } while (0)
#else
#define DBGPRINT(format, ...) ((void)0)
#endif

using namespace vortex;

///////////////////////////////////////////////////////////////////////////////

class vx_device {    
public:
    vx_device() {
     processor_ = new maxhpc("/dev/ttyUSB1");
//      if (!processor_->dev_ok) {
//       delete processor_;
//       exit(-1);
//      }
    }

    ~vx_device() {    
     if (future_.valid()) {
         future_.wait();
     }
     delete processor_;
    }

    uint64_t free_loc = 0x00000040;
    int mem_alloc(uint64_t size, uint64_t* dev_addr) {
     *dev_addr = free_loc;
     free_loc += size;
     if ((free_loc&0x3F) != 0) {
      free_loc += 0x40-free_loc&0x3F;
     }
     return 0;
    }

    int upload(uint64_t dest_addr, const void* src, uint64_t size) {
     uint64_t asize = aligned_size(size, CACHE_BLOCK_SIZE);
     if (dest_addr + asize > GLOBAL_MEM_SIZE) {
      return -1;
     }
     return processor_->ddrW(dest_addr, src, size);
    }

    int download(void* dest, uint64_t src_addr, uint64_t size) {
     uint64_t asize = aligned_size(size, CACHE_BLOCK_SIZE);
     if (src_addr + asize > GLOBAL_MEM_SIZE) {
      return -1;
     }
     return processor_->ddrR(dest, src_addr, size);
    }

    int start(uint64_t krnl_addr, uint64_t args_addr) {   
        // ensure prior run completed
        if (future_.valid()) {
            future_.wait();
        }

        // set kernel info
        this->write_dcr(VX_DCR_BASE_STARTUP_ADDR0, krnl_addr & 0xffffffff);
        this->write_dcr(VX_DCR_BASE_STARTUP_ADDR1, krnl_addr >> 32);
        this->write_dcr(VX_DCR_BASE_STARTUP_ARG0, args_addr & 0xffffffff);
        this->write_dcr(VX_DCR_BASE_STARTUP_ARG1, args_addr >> 32);

        // start new run
        future_ = std::async(std::launch::async, [&]{
            processor_->run();
        });
        return 0;
    }

    int wait(uint64_t timeout) {
        if (!future_.valid())
            return 0;
        uint64_t timeout_sec = timeout / 1000;
        std::chrono::seconds wait_time(1);
        for (;;) {
            // wait for 1 sec and check status
            auto status = future_.wait_for(wait_time);
            if (status == std::future_status::ready 
             || 0 == timeout_sec--)
                break;
        }
        return 0;
    }

    int write_dcr(uint32_t addr, uint32_t value) {
        if (future_.valid()) {
            future_.wait(); // ensure prior run completed
        }
        dcrs_.write(addr, value);
        return processor_->dcrW(addr, value);
    }

    uint64_t read_dcr(uint32_t addr) const {
        return dcrs_.read(addr);
    }

private:
 maxhpc*           processor_;
 DeviceConfig      dcrs_;
 std::future<void> future_;
};

///////////////////////////////////////////////////////////////////////////////

extern int vx_dev_caps(vx_device_h hdevice, uint32_t caps_id, uint64_t *value) {
   if (nullptr == hdevice)
        return  -1;

    //vx_device *device = ((vx_device*)hdevice);

    switch (caps_id) {
    case VX_CAPS_VERSION:
        *value = IMPLEMENTATION_ID;
        break;
    case VX_CAPS_NUM_THREADS:
        *value = NUM_THREADS;
        break;
    case VX_CAPS_NUM_WARPS:
        *value = NUM_WARPS;
        break;
    case VX_CAPS_NUM_CORES:
        *value = NUM_CORES * NUM_CLUSTERS;
        break;
    case VX_CAPS_CACHE_LINE_SIZE:
        *value = CACHE_BLOCK_SIZE;
        break;
    case VX_CAPS_GLOBAL_MEM_SIZE:
        *value = GLOBAL_MEM_SIZE;
        break;
    case VX_CAPS_LOCAL_MEM_SIZE:
        *value = (1 << LMEM_LOG_SIZE);
        break;    
    case VX_CAPS_LOCAL_MEM_ADDR:
        *value = LMEM_BASE_ADDR;
        break;
    case VX_CAPS_ISA_FLAGS:
        *value = ((uint64_t(MISA_EXT))<<32) | ((log2floor(XLEN)-4) << 30) | MISA_STD;
        break;
    default:
        std::cout << "invalid caps id: " << caps_id << std::endl;
        std::abort();
        return -1;
    }

    return 0;
}

extern int vx_dev_open(vx_device_h* hdevice) {
    if (nullptr == hdevice)
        return  -1;

    auto device = new vx_device();
    if (device == nullptr)
        return -1;

    int err = dcr_initialize(device);
    if (err != 0) {
        delete device;
        return err;
    }

#ifdef DUMP_PERF_STATS
    perf_add_device(device);
#endif

    *hdevice = device;

    return 0;
}

extern int vx_dev_close(vx_device_h hdevice) {
    if (nullptr == hdevice)
        return -1;

    vx_device *device = ((vx_device*)hdevice);
    
#ifdef DUMP_PERF_STATS
    perf_remove_device(hdevice);
#endif

    delete device;

    return 0;
}

extern int vx_mem_alloc(vx_device_h hdevice, uint64_t size, uint64_t* dev_addr) {
    if (nullptr == hdevice 
     || nullptr == dev_addr
     || 0 == size)
        return -1;

    DBGPRINT("MEM_ALLOC: size=%ld\n", size);

    vx_device *device = ((vx_device*)hdevice);
    return device->mem_alloc(size, dev_addr);
}

extern int vx_mem_free(vx_device_h hdevice, uint64_t dev_addr) {
    if (nullptr == hdevice)
        return -1;

    if (0 == dev_addr)
        return 0;

    DBGPRINT("MEM_FREE: dev_addr=0x%lx\n", dev_addr);

    return 0;
}

extern int vx_mem_info(vx_device_h hdevice, uint64_t* mem_free, uint64_t* mem_used) {
    if (nullptr == hdevice)
        return -1;

    DBGPRINT("%s\n", "MEM_INFO");

    return 0;
}

extern int vx_copy_to_dev(vx_device_h hdevice, uint64_t dev_addr, const void* host_ptr, uint64_t size) {
    if (nullptr == hdevice)
        return -1;

    auto device = (vx_device*)hdevice;

    DBGPRINT("COPY_TO_DEV: dev_addr=0x%lx, host_addr=0x%p, size=%ld\n", dev_addr, host_ptr, size);

    return device->upload(dev_addr, host_ptr, size);
}

extern int vx_copy_from_dev(vx_device_h hdevice, void* host_ptr, uint64_t dev_addr, uint64_t size) {
    if (nullptr == hdevice)
        return -1;

    auto device = (vx_device*)hdevice;

    DBGPRINT("COPY_FROM_DEV: dev_addr=0x%lx, host_addr=0x%p, size=%ld\n", dev_addr, host_ptr, size); 

    return device->download(host_ptr, dev_addr, size);
}

extern int vx_start(vx_device_h hdevice, uint64_t krnl_addr, uint64_t args_addr) {
    if (nullptr == hdevice)
        return -1;    
    
    DBGPRINT("START: krnl_addr=0x%lx, args_addr=0x%lx\n", krnl_addr, args_addr);

    vx_device *device = ((vx_device*)hdevice);
    return device->start(krnl_addr, args_addr);
}

extern int vx_ready_wait(vx_device_h hdevice, uint64_t timeout) {
    if (nullptr == hdevice)
        return -1;    
    
    DBGPRINT("%s\n", "WAIT");

    vx_device *device = ((vx_device*)hdevice);
    return device->wait(timeout);
}

extern int vx_dcr_read(vx_device_h hdevice, uint32_t addr, uint32_t* value) {
    if (nullptr == hdevice || NULL == value)
        return -1;

    vx_device *device = ((vx_device*)hdevice);

    // Ensure ready for new command
    if (vx_ready_wait(hdevice, -1) != 0)
        return -1;

    *value = device->read_dcr(addr);

    DBGPRINT("DCR_READ: addr=0x%x, value=0x%x\n", addr, *value);

    return 0;
}

extern int vx_dcr_write(vx_device_h hdevice, uint32_t addr, uint32_t value) {
    if (nullptr == hdevice)
        return -1;

    vx_device *device = ((vx_device*)hdevice);

    // Ensure ready for new command
    if (vx_ready_wait(hdevice, -1) != 0)
        return -1;  

    DBGPRINT("DCR_WRITE: addr=0x%x, value=0x%x\n", addr, value);

    return device->write_dcr(addr, value);
}
