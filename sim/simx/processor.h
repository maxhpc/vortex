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

#pragma once

#include <stdint.h>

namespace vortex {

class Arch;
class RAM;
class ProcessorImpl;

class Processor {
public:
  Processor(const Arch& arch);
  ~Processor();

  void attach_ram(RAM* mem);

  int run();

  void write_dcr(uint32_t addr, uint32_t value);

private:
  ProcessorImpl* impl_;
};

}
