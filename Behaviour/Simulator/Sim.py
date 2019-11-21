#simulate the behaviour algorithm

from p5 import setup, draw, size, background, run
import numpy as np
from Boid_sim import Boid

width = 1000
height = 1000

flock = [Boid(*np.random.rand(2)*1000, width, height) for _ in range(20)] #creating 50 boids randomly around the map

def setup():
    size(width, height) #setup map

def draw():
    global flock
    background('black') #coloring the map
    for boid in flock: 
        boid.edges() 
        boid.apply_behaviour(flock) 
        boid.update()
        boid.show()

run()