# PIMSimulator

## Contents

  [1. Overview](#1-overview)  
  [2. HW Description](#2-hw-description)  
  [3. Setup](#3-setup)  
  [4. Programming Guide](#4-programming-guide)

## 1. Overview

PIMSimulator is a cycle accurate model that Single Instruction, Multiple Data (SIMD) execution units
that uses the bank-level parallelism in PIM Block to boost performance that would have otherwise used
multiple times of bandwidth from simultaneous access of all bank.
The simulator include memory and have embedded within it a PIM block, which consist of programmable
command registers, general purpose register files and execution units.

Based on https://github.com/umd-memsys/DRAMSim2, the simulator includes

* PIM Block:
  * Register files including CRF (for command), GRF (for vector value), SRF (for scalar value)
  * ALU (ADD, MUL, MAC, MAD, MOVE, FILL, NOP, JUMP, EXIT)
* PIM Kernel:
  * Generate a set of memory transactions for enabling PIM operation
* HBM2 support (refer to `ini/HBM2_samsung_2M_16B_x64.ini`)

## 2. HW description

PIM is a HBM stack that is pin compatible with HBM2 and have embedded within it a PIM block

### 2.1 Base Architecture

```C
|--------------|          |--------------|          |--------------|
|              |    (A)   |              |   (B)    |              |
|    HOST      |----------| Controller   |----------|    Memory    |
|              |          |              |          |              |
|--------------|          |--------------|          |--------------|
```

* Each channel is logically independent memory, so it has a dedicated independent controller.
* (A): Read [Addr], Write [Addr]
* (B): Activate, Read, Write, Precharge, Refresh, Activate_pim, ALU_pim, Precharge_pim, READ_pim

#### HBM2

* System Specification: system_hbm.ini
* HBM Specification: ini/HBM2_samsung_2M_16B_x64.ini
  * 1 PIM block per 2 banks, 4 Bank per Bankgroup, 4 Bank group per pseudo channel, 4 pseudo channel per die, 4 die per stack.
  * Prefetch size : 256bit
  * burst length: 4n
  * Pin speed: 2Gbps
  * The simulator supports the pseudo-channel mode only, and we assume that each pseudo-channel is totally independent.

### 2.2 Address mapping

* The address mapping is used when the memory controller decodes the address from host.
* Use Scheme8 addressing mode for PIM functionality.

```C
|<-rank->|<-row->|<-col high->|<-bg->|<-bank->|<-chan->|<-col low->|<-offset ->|
```
* the length of col_low is log(BL * JEDEC_DATA_BUS_BUTS/8), which are 5b both for HBM2
* You can also change the current addressing mode dynamically (Not recommended, though)

```C
// Static Setting in system_*.ini
ADDRESS_MAPPING_SCHEME=Scheme8
```
### 2.3 PIM Block Placement

* BANKS_PER_PIM_BLOCK = NUM_BANKS / NUM_PIM_BLOCKS

#### HBM2 case
```C
|--------|  |--------|
|        |  |        |
| BANK_0 |  | BANK_2 |
|        |  |        |
|--------|  |--------|
|  PB_0  |  |  PB_1  |
|--------|  |--------|
|        |  |        |
| BANK_1 |  | BANK_3 |
|        |  |        |
|--------|  |--------|
```

* if 2 * NUM_PIM_BLOCKS == NUM_BANK, a PIM block is located per two banks.
  * NUM_PIM_BLOCKS = 8, NUM_BANKS = 16

* if NUM_PIM_BLOCKS == NUM_BANKS, a PIM Block (PB) is located per banks.
  * NUM_PIM_BLOCKS = 8, NUM_BANKS = 8


### 2.3 PIM Instruction-Set Architecture
|Type|Command|Description|Result (DST)|Operand (SRC0)|Operand (SRC1)|
|---|---|---|---|---|---|
|Arithmetic|ADD|addition |GRF|GRF, BANK, SRF|GRF, BANK, SRF|
|Arithmetic|MUL|multiplication |GRF|GRF, BANK|GRF, BANK, SRF|
|Arithmetic|MAC|multiply-accumulate |GRF_B|GRF, BANK|GRF, BANK, SRF|
|Arithmetic|MAD|multiply-and-add |GRF|GRF, BANK|GRF, BANK, SRF|
|Data|MOV|load or store data from register to bank|GRF, SRF|GRF, BANK||
|Data|FILL|copy data from bank to register|GRF, BANK|GRF, BANK||
|Control|NOP|do nothing||||
|Control|JUMP|jump instruction||||
|Control|EXIT|exit instruction||||

* Supports RISC-style 32-bit instructions
* Three instructions types
  * 4 Arithmetic: ADD, MUL, MAC, MAD
  * 2 Data transfer: MOV, FILL
  * 3 control flows: NOP, JUMP, EXIT
* JUMP instruction
  * Zero-cycle static branch: supports only a pre-programmed numbers of iterations
* Operand type:
  * Vector Register (GRF_A, GRF_B)
  * Scalar Register (SRF)
  * Bank Row Buffer
* PIM instructions are stored in the Command Register File (CRF), and memory command triggers a CRF to perform a target instruction
  * each memory command increments the CRF PC
* DRAM commands decide where to retrieve data from DRAM for PIM arithmetic operations

### 2.4 Movement of Data
|Mode|Transaction|PIM Instruction|Operation|
|---|---|---|---|
|SB|Read|-|Normal Memory Read|
|SB|Write|-|Normal Memory Write|
|HAB|Write|-| PIM Write (Host to PIM Register)|
|PIM|-|MOV|read or write from bank to PIM Register|
|PIM|-|FILL|write from bank to PIM Registers|

* SB mode: standard DRAM operation
* HAB mode: Allowing concurrent accesses to multiple banks with a single DRAM command
* PIM mode: Triggers the execution of PIM commands on the CRF by DRAM Command


## 3. Setup

### 3.1 Prerequisites
* `Scons` tool for compiling PIMSimulator:
```bash
sudo apt install scons
```
* `gtest` for running test cases:
```bash
sudo apt install libgtest-dev
```

### 3.2 Installing
* To Install PIMSimulator:
```bash
# compile
scons
```

### 3.3 Launch a Test Run
* Show a list of test cases
```bash
./sim --gtest_list_tests

# Example
PIMKernelFixture.
  gemv_tree
  gemv
  mul
  add
  relu
MemBandwidthFixture.
  hbm_read_bandwidth
  hbm_write_bandwidth
PIMBenchFixture.
  gemv
  mul
  add
  relu
```

* Test Running
```bash
# Running: functionality test (GEMV)
./sim --gtest_filter=PIMKernelFixture.gemv

# Running: functionality test (MUL)
./sim --gtest_filter=PIMKernelFixture.mul

# Running: performance test (GEMV)
./sim --gtest_filter=PIMBenchFixture.gemv

# Running: performance test (ADD)
./sim --gtest_filter=PIMBenchFixture.add
```

If you want to functionality test for other dimensions, generate a new dimension in `./data`
and add generated dimension to the source of `src/tests/KernelTestCases.cpp`.
Use the gen script in `./data` to generate data of the dimension to be changed.

### 3.4 Configuration

#### Turning on/off verbose mode
* You can select what kinds of log you want to see by modifying system_*.ini

#### Turning on/off data mode
* Data mode
  * build without -DNO_STORAGE option
* No-data mode
  * build with -DNO_STORAGE option
```bash
# build to No-data mode
scons NO_STORAGE=1
```

## 4 Programming Guide
Highly recommend you to refer to `src/tests/*` (especially, `src/tests/PIMKernel.cpp` and `src/tests/PIMBenchTestCases.cpp`)
To attach to host simulator, refer to `src/tests/PIMKernel.cpp`.
You can see commands that request memory transactions to the memory controller for GEMV or Eltwise operations on PIM.
It include a basic PIM procedure for GEMV operation in the `PIMKernel::executeGemv()`,
and also for Eltwise operation (add, mul, relu) in the `PIMKernel::executeEltwise()`

### 4.1 Primitive Function

```C
mem->addTransaction(is_read, address, tag, buffer);
```
* is_read: memory request types between READ('false') and WRITE('true')
* address: address used for memory / PIM transaction
* tag: Used for log or set to barrier. If not used, only three parameters are available,
        as `addTransaction(is_read, address, buffer)`
* buffer : used to verify pim functionality using data. Here, the buffer is at least 256-bit sized container.
If you do not want to use the data buffer, you can use it as below:
    ```C
    BurstType nullBst
    mem->addTransaction(isWrite, addr, &nullBst);
    ```

#### Memory transaction

* read
    ```C
    mem->addTransaction(false, addr, tag, buffer);
    ```
* write
    ```C
    mem->addTransaction(true, addr, tag, buffer);
    ```
Here, the buffer must be at least 256bit size container.

#### PIM transaction

* alu_pim (dataflow is similar to normal write)
    ```C
    mem->addTransaction(true, addr, tag, buffer);
    ```
  * Highly recommend you to refer to simple PIM operations using PIM ISA (`src/tests/PIMCmdGen.h`) and procedures using them(`src/tests/PIMKernel.cpp`)
  * The buffer must contain data to be broadcasted to all pim blocks of a specific channel.
  * In GEMV cacse, the corresponding weight is supplied from specific row, a specific col of multiple banks of a specific memory channel.
    * If each bank in a channel has unique ID, and bank addr in the transaction is BA, the banks satisfying (ID % BANKS_PER_PIM_BLOCK == BA) supply the weight to PIM blocks.
      * if BANKS_PER_PIM_BLOCK == 2, and BA = 0, bank 0,2,4,6,... supply the weight to pim-block 0,1,2,3,..., respectively.
      * if BANKS_PER_PIM_BLOCK == 2, and BA = 1, bank 1,3,5,7,... supply the weight to pim-block 0,1,2,3,..., respectively.
    * As a result, data broadcasted and data from multiple banks of a specific channel are multiplied and accumulated.

* read_pim (dataflow is similar to normal read)
    ```C
    mem->addTransaction(false, addr, tag, buffer);
    ```
   * Read the accumulated partial sum in the pim block. Then reset the buffer.
      * If each pim block in a channel has unique ID, and bank addr in the transaction is BA, the PIM block satisfying (ID == BA) supply partial sums to DQ.
   * Highly recommend to use the address that alu_pim command used at the last

### 4.2 PIM High-level Steps
* The following shows the high level steps of a generic PIM operation.
   * Place data in DRAM
   * Switch to HAB mode
   * Program CRF
   * Enable PIM
   * Execute PIM
   * Disable PIM
   * Switch to SB mode

* A similar procedure at the source level can be found in `src/tests/PIMKernel.cpp`.
```C
    /* Example Code - PIMKernel::executeELtwise() */

    parkIn();
    changePIMMode(dramMode::SB, dramMode::HAB);      // Switch to HAB
    programCrf(pim_cmds);                            // Program CRF
    changePIMMode(dramMode::HAB, dramMode::HAB_PIM); // Enable PIM

    if (ktype == KernelType::ADD || ktype == KernelType::MUL)
        computeAddOrMul(num_tile, input0_row, result_row, input1_row); // Execute PIM
    else if (ktype == KernelType::RELU)
        computeRelu(num_tile, input0_row, result_row);

    changePIMMode(dramMode::HAB_PIM, dramMode::HAB); // Disable PIM
    changePIMMode(dramMode::HAB, dramMode::SB);      // Switch to SB mode
    parkOut();

```

* The other basic operation flow on PIM for GEMV(Matrix Vector multiplication), Element-wise operation are described in the `src/tests/PIMKernel.cpp`.

### Contact
* Shin-haeng Kang (s-h.kang@samsung.com)
* Sanghoon Cha (s.h.cha@samsung.com)
* Seungwoo Seo (sgwoo.seo@samsung.com)
* Jin-seong kim (jseong82.kim@samsung.com)
