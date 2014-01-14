#include "fifo.ph"
#include "division.pm"

.origin 0
.entrypoint START

.struct InitDef
  .u32 fifo_addr
  .u32 axes
.ends

.struct AxisDefs
  .u32 step_addr
  .u32 step_pin
  .u32 dir_addr
  .u32 dir_pin
  .u32 steps
  .u32 err
.ends

.struct CommandDefs
  .u32 timesteps
  .u32 c
  .u32 n
  .u32 acc_steps
  .u32 dec_steps
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
.assign CommandDefs, r12, r16, Command
.assign AxisDefs, r17, r22, Axis
#define cur_axis r23

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
  fifo_get Init.fifo_addr, Command, SIZE(Command), tmp2
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

  // Debug output
  mov tmp, Init.fifo_addr
  mov tmp2, FIFO_LENGTH
  sbbo Command, tmp, tmp2, SIZE(Command)

  jmp NEXT_COMMAND

// Subroutine
calc_next_delay:

  // When reached start of dec, negate n
  qbne no_start_dec, steps_left, Command.dec_steps
  not Command.n, Command.n
  add Command.n, Command.n, 1
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
  // Fetch axis definition
  lbbo Axis, tmp, 0, SIZE(Axis)

  // Read step number from fifo
  fifo_get Init.fifo_addr, Axis.steps, SIZE(Axis.steps), tmp2

  // Set dir pin, based on steps sign
  // steps = abs(steps)
  mov tmp2, Axis.dir_addr
  qbbc forwards, Axis.steps, 31
  not Axis.steps, Axis.steps
  add Axis.steps, Axis.steps, 1
  xor tmp2, tmp2, 4
forwards:
  sbbo Axis.dir_pin, tmp2, 0, 4

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
  sbbo Axis.step_pin, Axis.step_addr, 0, 4

  // Toggle set/clear bit
  xor Axis.step_addr, Axis.step_addr, 4

  add Axis.err, Axis.err, Command.timesteps
no_step:
  sbbo Axis, tmp, 0, SIZE(Axis)
  add tmp, tmp, SIZE(Axis)
  add cur_axis, cur_axis, 1
  qblt bresenham_axis, Init.axes, cur_axis
  ret

