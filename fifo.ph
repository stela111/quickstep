#define PRU0_ARM_INTERRUPT 19

#define PRUSS_PRU_CTRL 0x22000
#define CONTROL 0x00
#define CYCLECOUNT 0x0C

#define COUNTER_ENABLE_BIT 3

#define FIFO_LENGTH 1024

.struct FifoDefs
  .u32 front
  .u32 back
.ends

.macro init_fifo
.mparam tmp
  // Enable OCP master ports
  lbco tmp, C4, 4, 4
  clr  tmp, tmp, 4
  sbco tmp, C4, 4, 4

  // Init fifo pointers
  mov Fifo.front, SIZE(Fifo)
.endm

.macro fifo_get
.mparam fifo_addr
.mparam reg
.mparam len
.mparam tmp

wait_available:
  lbbo Fifo.back, fifo_addr, OFFSET(Fifo.back), SIZE(Fifo.back)
  qbeq wait_available, Fifo.back, Fifo.front

  // Read data into reg
  lbbo reg, fifo_addr, Fifo.front, len
  add Fifo.front, Fifo.front, len

  // Check for fifo wraparound
  mov tmp, FIFO_LENGTH
  qblt no_wrap, tmp, Fifo.front
  mov Fifo.front, SIZE(Fifo)

no_wrap:
  // Store front in ram
  sbbo Fifo.front, fifo_addr, OFFSET(Fifo.front), SIZE(Fifo.front)
.endm

.macro check_end_command
  // 0 signals end
  qbne not_end, Command, 0
  mov R31.b0, PRU0_ARM_INTERRUPT+16
  halt
not_end:
.endm

.macro reset_cyclecount
.mparam addr
.mparam ctrl
.mparam tmp
  mov addr, PRUSS_PRU_CTRL

  lbbo ctrl, addr, CONTROL, 4
  clr ctrl, COUNTER_ENABLE_BIT
  sbbo ctrl, addr, CONTROL, 4

  mov tmp, 4 // make up for disabled cycles
  sbbo tmp, addr, CYCLECOUNT, 4

  set ctrl, COUNTER_ENABLE_BIT
  sbbo ctrl, addr, CONTROL, 4
.endm

.macro get_cyclecount
.mparam counts
.mparam addr = r0
  mov addr, PRUSS_PRU_CTRL
  lbbo counts, addr, CYCLECOUNT, 4
.endm
