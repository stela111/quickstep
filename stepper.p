#include "fifo.ph"
#include "division.pm"

.origin 0
.entrypoint START

.struct InitDef
  .u32 fifo_addr
  .u32 axes
.ends

.struct AxisDefs
  .u32 gpio_addr
  .u32 pin_mask
  .u32 steps
  .u32 err
.ends

.struct CommandDefs
  .u32 timesteps
  .u32 c
  .u32 n
  .u32 acc_steps
  .u32 dec_steps
  .u32 n_dec
.ends

#define tmp r0
#define tmp2 r1
#define tmp3 r2
#define nom r3
#define den r4
#define quot r5
#define rem r6
#define steps_left r7
.assign InitDef, r8, r9, Init
.assign FifoDefs, r10, r11, Fifo
.assign CommandDefs, r12, r17, Command
.assign AxisDefs, r20, r23, Axis
#define cur_axis r24

#define GPIO0 0x44E07000
#define GPIO_CLEARDATAOUT 	0x190
#define GPIO_SETDATAOUT 	0x194
#define STEP (1<<27)

START:
  // Load setup from ram
  mov tmp, 0
  lbbo Init, tmp, 0, SIZE(Init)

  // Init must be followed by Init.axes number of
  // AxisDefs structs with initialized
  // gpio_addr and pin_mask
  init_fifo tmp
  mov rem, 0

NEXT_COMMAND:
  mov tmp, Command.c // Store current c
  fifo_get Init.fifo_addr, Command, SIZE(Command), tmp2
  // If new c is 0, reuse last c
  qbne keep_c, Command.c, 0
  mov Command.c, tmp
keep_c:
  call init_axes
  check_end_command

  reset_cyclecount tmp, tmp2, tmp3
  mov steps_left, Command.timesteps
wait:
  get_cyclecount tmp, tmp2
  // Apply c-scale factor
  lsl tmp, tmp, 10
  qbgt wait, tmp, Command.c
  call do_bresenham
  reset_cyclecount tmp, tmp2, tmp3

  call calc_next_delay
  sub steps_left, steps_left, 1
  qblt wait, steps_left, 0

  mov tmp, Init.fifo_addr
  mov tmp2, FIFO_LENGTH
  sbbo Command, tmp, tmp2, SIZE(Command)

  jmp NEXT_COMMAND

// Subroutine
calc_next_delay:

  // When reached start of dec, set n to n_dec
  qbne no_start_dec, steps_left, Command.dec_steps
  mov Command.n, Command.n_dec
no_start_dec:

  // Calculate new c only if acc or dec
  // Count down acc_steps to zero to keep track of acc
  qblt acc, Command.acc_steps, 0

  // Decelerating if steps_left <= dec_steps
  qble dec, Command.dec_steps, steps_left

  // Constant speed, keep c
  jmp no_ramp
acc:
  sub Command.acc_steps, Command.acc_steps, 1
dec:

  // c = c - (c*2+rem)/(4*(++n)+1)
  lsl nom, Command.c, 1
  add nom, nom, rem
  add Command.n, Command.n, 1
  lsl den, Command.n, 2
  add den, den, 1
  divide nom, den, quot, rem, tmp
  sub Command.c, Command.c, quot

no_ramp:
  ret

// Subroutine
init_axes:
  mov cur_axis, 0
  mov tmp, SIZE(Init)
init_axes_loop:
  lbbo Axis, tmp, 0, SIZE(Axis)
  fifo_get Init.fifo_addr, Axis.steps, SIZE(Axis.steps), tmp2

  // We will toggle output, therefore double steps
  lsl Axis.steps, Axis.steps, 1

  // Init bresenham error
  lsr Axis.err, Command.timesteps, 1

  // Store in ram
  sbbo Axis, tmp, 0, SIZE(Axis)

  add tmp, tmp, SIZE(Axis)
  add cur_axis, cur_axis, 1
  qblt init_axes_loop, Init.axes, cur_axis
  ret

// Subroutine
do_bresenham:
  mov cur_axis, 0
  mov tmp, SIZE(Init)
bresenham_axis:
  lbbo Axis, tmp, 0, SIZE(Axis)
  sub Axis.err, Axis.err, Axis.steps
  qbbc no_step, Axis.err, 31 // step only if signed err < 0

  // toggle step
  sbbo Axis.pin_mask, Axis.gpio_addr, 0, 4

  // Toggle set/clear bit
  xor Axis.gpio_addr, Axis.gpio_addr, 4

  add Axis.err, Axis.err, Command.timesteps
no_step:
  sbbo Axis, tmp, 0, SIZE(Axis)
  add tmp, tmp, SIZE(Axis)
  add cur_axis, cur_axis, 1
  qblt bresenham_axis, Init.axes, cur_axis
  ret

