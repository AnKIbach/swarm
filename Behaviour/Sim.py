#simulate the behaviour algorithm

from p5 import setup, draw, size, background, run
import numpy as np
from Boid import Boid

width = 800
height = 800

flock = [Boid(*np.random.rand(2)*800, width, height) for _ in range(5)] #creating 50 boids randomly around the map

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