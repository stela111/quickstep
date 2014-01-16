#!/usr/bin/env python

import pypruss
import sys
import math
import time
from fifo import Fifo

gpio0 = 0x44E07000
gpio1 = 0x4804C000
gpio2 = 0x481AC000
gpio3 = 0x481AE000

axes = ((gpio0, 27, gpio1, 14),
        (gpio1, 15, gpio0, 26))

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

def vdiff(v1, v2):
  """Vector difference""" 
  return tuple(a-b for a, b in zip(v1, v2))

def vlen(v):
  """Euclidean vector length"""
  return sum(float(a)**2 for a in v)**.5

class Stepper:
  def __init__(self, fifo):
    self.fifo = fifo
    self.cscale = 1024
    self.a = 2.0*math.pi/200/32
    self.f = 2e8
    self.speed = 0
    self.num_axes = len(axes)

  def acc_steps(self, speed_diff, acc):
    return int(speed_diff**2/(2*self.a*acc))

  def move(self, steps, speed, acc, endspeed = 0):
    tf = 2 # timesteps must be at least 2*steps
    timesteps = vlen(steps)*tf

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
      print timesteps, acc_meets_dec_steps
      return

    if self.speed == 0:
      c = int(round(0.676*self.f*math.sqrt(2*self.a/acc/tf)*self.cscale))
    else:
      c = int(round(self.a*self.f/self.speed/tf*self.cscale))

    if acc_meets_dec_steps < acc_steps:
      self.fifo.write([timesteps, c, cut_acc_steps, acc_meets_dec_steps,
        timesteps-acc_meets_dec_steps] + steps, 'l')
    else:
      self.fifo.write([timesteps, c, cut_acc_steps, acc_steps, 
        dec_steps] + steps,'l')

    self.speed = endspeed

  def stop(self):
    self.fifo.write([0]*(5+self.num_axes))

#cmd = [float(arg) for arg in sys.argv[1:8]]

fifo = init('./stepper.bin')

stepper = Stepper(fifo)

r = 200*16/5
s = 5
j = 2
a = 50

ax = len(axes)
steps = 20
for t in range(steps):
  x = math.sin(t*2*math.pi/steps)
  y = math.cos(t*2*math.pi/steps)
  stepper.move([r*x, r*y], s, a, s if t < steps-1 else 0)
stepper.stop()

olda = fifo.front()
print 'front:',olda
while True:
  a = fifo.front()
  if not olda == a:
    print fifo.memread(1024, 5,'l')
    print 'front:',a
    olda = a
  if a == fifo.back:
    break
  time.sleep(0.1)

pypruss.wait_for_event(0)
pypruss.clear_event(0)
pypruss.pru_disable(0)
pypruss.exit()
