import pypruss
import sys
import math
from fifo import Fifo

gpio0 = 0x44E07000
gpio1 = 0x4804C000
gpio2 = 0x481AC000
gpio3 = 0x481AE000

axes = ((gpio0, 27, gpio1, 14),)

def init(pru_bin):
  pypruss.modprobe(1024)
  ddr_addr = pypruss.ddr_addr()
  ddr_size = pypruss.ddr_size()
  print "DDR memory address is 0x%x and the size is 0x%x"%(ddr_addr, ddr_size)
  fifo = Fifo(ddr_addr, ddr_size)

  pypruss.init()
  pypruss.open(0)
  pypruss.pruintc_init()

  pru_init = [ddr_addr, len(axes)]
  for step_addr, step_pin, dir_addr, dir_pin in axes:
    pru_init += [step_addr+0x194, 1 << step_pin,
                  dir_addr+0x194, 1 << dir_pin,
                  0, 0]

  pypruss.pru_write_memory(0,0,pru_init)
  pypruss.exec_program(0, pru_bin)
  return fifo

class Stepper:
  def __init__(self, fifo):
    self.fifo = fifo
    self.cscale = 1024
    self.a = 2.0*math.pi/200/32
    self.f = 2e8
    self.speed = 0

  def acc_steps(self, speed_diff, acc):
    return int(speed_diff**2/(2*self.a*acc))

  def move(self, steps, speed, acc, endspeed = 0):
    tf = 2 # timesteps must be at least 2*steps
    timesteps = abs(steps*tf)

    if not self.speed == 0 and steps*self.dir < 0:
      print 'Cannot change direction'
      return
    if self.speed > speed:
      print 'Cannot start with deceleration'
      return
    if speed < endspeed:
      print 'Cannot end with acceleration'
      return

    cut_acc_steps = tf*self.acc_steps(self.speed, acc)
    cut_dec_steps = tf*self.acc_steps(endspeed, acc)
    acc_steps = tf*self.acc_steps(speed, acc)
    dec_steps = acc_steps - cut_dec_steps
    acc_steps -= cut_acc_steps

    acc_meets_dec_steps = (timesteps+cut_acc_steps+cut_dec_steps)/2-cut_acc_steps
    if acc_meets_dec_steps < 0 or acc_meets_dec_steps > timesteps:
      print 'Specified velocity change not possible with given acceleration'
      return

    if self.speed == 0:
      c = int(round(0.676*self.f*math.sqrt(2*self.a/acc/tf)*self.cscale))
    else:
      c = int(round(self.a*self.f/self.speed/tf*self.cscale))

    if acc_meets_dec_steps < acc_steps:
      self.fifo.write([timesteps, c, cut_acc_steps, 
        acc_meets_dec_steps, timesteps-acc_meets_dec_steps,
        -timesteps+acc_meets_dec_steps-cut_dec_steps, 
        steps], 'l')
    else:
      self.fifo.write([timesteps, c, cut_acc_steps, acc_steps, 
        dec_steps, -dec_steps-cut_dec_steps, steps],'l')

    self.speed = endspeed
    self.dir = 1 if steps > 0 else -1

#cmd = [float(arg) for arg in sys.argv[1:8]]

fifo = init('./stepper.bin')

stepper = Stepper(fifo)
stepper.move(200*16, 10, 25, 2)
stepper.move(100*16, 2, 25, 0)
stepper.move(-200*16, 50, 200, 5)
stepper.move(-100*16, 5, 50, 0)
fifo.write([0]*7)

olda = fifo.front()
print 'front:',olda
while True:
  a = fifo.front()
  if not olda == a:
    print fifo.memread(1024, 6)
    print 'front:',a
    olda = a
  if a == fifo.back:
    break

pypruss.wait_for_event(0)
pypruss.clear_event(0)
pypruss.pru_disable(0)
pypruss.exit()
