import pypruss
import sys
import math
from fifo import Fifo

axes = 1

def init(pru_bin):
  pypruss.modprobe(1024)
  ddr_addr = pypruss.ddr_addr()
  ddr_size = pypruss.ddr_size()
  print "DDR memory address is 0x%x and the size is 0x%x"%(ddr_addr, ddr_size)
  fifo = Fifo(ddr_addr, ddr_size)

  pypruss.init()
  pypruss.open(0)
  pypruss.pruintc_init()

  axes_init = []
  for axis in range(axes):
    axes_init += [0x44E07194, 1 << (27+axis), 0, 0]

  pypruss.pru_write_memory(0,0,[ddr_addr, axes]+axes_init)
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
    if self.speed > speed:
      print 'Cannot start with deceleration'
      return
    if speed < endspeed:
      print 'Cannot end with acceleration'
      return

    tf = 2 # timesteps must be at least 2*steps

    cut_acc_steps = tf*self.acc_steps(self.speed, acc)
    cut_dec_steps = tf*self.acc_steps(endspeed, acc)
    acc_steps = tf*self.acc_steps(speed, acc)
    dec_steps = acc_steps - cut_dec_steps
    acc_steps -= cut_acc_steps

    acc_meets_dec_steps = (steps+cut_acc_steps+cut_dec_steps)/2-cut_acc_steps
    acc_meets_dec_steps *= tf

    if self.speed == 0:
      c = int(round(0.676*self.f*math.sqrt(2*self.a/acc/tf)*self.cscale))
    else:
      c = int(round(self.a*self.f/self.speed/tf*self.cscale))

    if acc_meets_dec_steps < acc_steps:
      self.fifo.write([steps*tf, c, cut_acc_steps, 
        acc_meets_dec_steps, steps-acc_meets_dec_steps,
        -steps+acc_meets_dec_steps-cut_dec_steps, 
        steps], 'l')
    else:
      self.fifo.write([steps*tf, c, cut_acc_steps, acc_steps, 
        dec_steps, -dec_steps-cut_dec_steps, steps],'l')

    self.speed = endspeed

#cmd = [float(arg) for arg in sys.argv[1:8]]

fifo = init('./stepper.bin')

stepper = Stepper(fifo)
stepper.move(200*32, 10, 25, 5)
stepper.move(200*32, 5, 25, 5)
stepper.move(200*32, 20, 50, 0)
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
