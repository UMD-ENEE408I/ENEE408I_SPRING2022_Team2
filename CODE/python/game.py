
import pygame
import random
from enum import Enum
from collections import namedtuple
import numpy as np
from numpy import linalg as LA
import copy
#import math
default_maze = [[1,   1,   1,   1,   0,   0,   0,   0,   0,   0],
        [0,   0,   0,   1,   0,   0,   0,   0,   0,   0],
        [0,   1,   1,   1,   1,   1,   1,   1,   1,   0],
        [0,   0,   0,   1,   0,   0,   0,   1,   0,   0],
        [0,   0,   0,   1,   1,   1,   1,   1,   1,  0],
        [0,   0,   0,   1,   0,   0,   0,   0,   1,   0],
        [0,   0,   0,   1,   0,   0,   0,   0,   1,   0],
        [0,   0,   0,   1,   0,   0,   0,   0,   1,   0],
        [0,   0,   0,   1,   1,   1,   1,   1,   1,   0],
        [0,   0,   0,   1,   0,   0,   0,   0,   1,   1]]

pygame.init()
font = pygame.font.SysFont('arial', 25)
screen = pygame.display.set_mode([25,25])
MARGIN = 5
WIDTH = 20
HEIGHT = 20
#WINDOW_SIZE = [255, 255]
class Direction(Enum):
    RIGHT = 1
    LEFT = 2
    UP = 3
    DOWN = 4
Point = namedtuple('Point', 'row, column')
# rgb colors
WHITE = (255, 255, 255) #maze
BLACK = (0,0,0)         #not the maze 
BLUE1 = (0, 0, 255)
BLUE2 = (0, 100, 255)
BLACK = (0,0,0)
GREEN = (0, 255, 0)
RED = (255, 0, 0)
BLOCK_SIZE =20
SPEED = 500
END_R = 180
END_C = 180
class mazeAI:
    

    def __init__(self, w=250, h=250):
        self.w = w
        self.h = h
        # init display
        self.display = pygame.display.set_mode((self.w, self.h))
        pygame.display.set_caption('Maze')
        self.clock = pygame.time.Clock()
        
        # init game state
        self.direction = Direction.RIGHT
        self.possibleMoves = [0, 0, 0, 0]
        self.head = Point(0,0)
        self.snake = [self.head]
        self.score = 0
        self.maze1 = [[1,   1,   1,   1,   0,   0,   0,   0,   0,   0],
        [0,   0,   0,   1,   0,   0,   0,   0,   0,   0],
        [0,   1,   1,   1,   1,   1,   1,   1,   1,   0],
        [0,   0,   0,   1,   0,   0,   0,   1,   0,   0],
        [0,   0,   0,   1,   1,   1,   1,   1,   1,  0],
        [0,   0,   0,   1,   0,   0,   0,   0,   1,   0],
        [0,   0,   0,   1,   0,   0,   0,   0,   1,   0],
        [0,   0,   0,   1,   0,   0,   0,   0,   1,   0],
        [0,   0,   0,   1,   1,   1,   1,   1,   1,   0],
        [0,   0,   0,   1,   0,   0,   0,   0,   1,   1]]
        self.maze = [[1,   1,   1,   1,   0,   0,   0,   0,   0,   0],
        [0,   0,   0,   1,   0,   0,   0,   0,   0,   0],
        [0,   1,   1,   1,   1,   1,   1,   1,   1,   0],
        [0,   0,   0,   1,   0,   0,   0,   1,   0,   0],
        [0,   0,   0,   1,   1,   1,   1,   1,   1,  0],
        [0,   0,   0,   1,   0,   0,   0,   0,   1,   0],
        [0,   0,   0,   1,   0,   0,   0,   0,   1,   0],
        [0,   0,   0,   1,   0,   0,   0,   0,   1,   0],
        [0,   0,   0,   1,   1,   1,   1,   1,   1,   0],
        [0,   0,   0,   1,   0,   0,   0,   0,   1,   1]]
        self.discoveredMaze = [([0]*10) for i in range(10)]
        self.clock = pygame.time.Clock()
        self.reset()

    def reset(self):
        # init game state
        self.direction = Direction.RIGHT
        self.head = Point(0, 0)
        self.score = 0
        self.frame_iteration = 0
        self.maze = copy.deepcopy(self.maze1)
        #print(self.maze)
        #self._update_ui()

    
    def play_step(self,action):
        # 1. collect user input
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                quit()

        
        # 2. move
        self.possibleMoves = [0,0,0,0]#self._options()
        self._move(action) # update the head
        row_ = self.head.row
        column_ = self.head.column      
        self.score+=1
        # 3. check if game over
        reward = 0#(40-((row_-END_R)/20+(column_-END_C)/20))/10-5#10/int(LA.norm([row_-END_R,column_-END_C])+1)
        #if (40-((row_-END_R)/20+(column_-END_C)/20))/10:
        print(reward)
        if (self.head.column==END_C  and self.head.row == END_R):
            game_over = True 
            
            print("Solved")
            reward= 100+ int(5000/self.score)#100/int(self.score/50+1)
            print(reward)
        elif(self.score > 100):
            print("Failed to Solve")
            #print(row_-END_R)
            #print(column_-END_C)
            #print(LA.norm([row_-END_R,column_-END_C])+1)
            reward = -5 
            #print(norm([(row_-END_R)/20,(column_-END_C)/20]))
            #if norm([(row_-END_R)/20,(column_-END_C)/20])<10:
            #    reward = 10
            #15-int(1000/(int(LA.norm([row_-END_R,column_-END_C]))+1)**.5)
            game_over = True
            print(reward)
        else:
            game_over = False
            

        
        # 5. update ui and clock
        self._update_ui()
        self.clock.tick(SPEED)
        # 6. return game over and score
        return reward, game_over, self.score
    
    def _update_ui(self):
        self.display.fill(BLACK)
        
        WIDTH = 20
        HEIGHT = 20
        #pygame.draw.rect(self.display, RED, pygame.Rect(self.food.x, self.food.y, BLOCK_SIZE, BLOCK_SIZE))
        # Draw the grid 
        #print(maze)
        for row in range(10):
            for column in range(10):
                color = WHITE
                if self.maze[row][column] == 1:
                    color = WHITE
                if self.maze[row][column] == 0:
                    color = BLACK
                if self.maze[row][column] == .5:
                    color = GREEN
                if self.maze[row][column] == .75:
                    color = RED
                pygame.draw.rect(screen,
                                color,
                                [(MARGIN + WIDTH) * column + MARGIN,
                                  (MARGIN + HEIGHT) * row + MARGIN,
                                  WIDTH,
                                  HEIGHT])
        pygame.display.flip()


    def _option(self):
        row_ = self.head.row
        column_ = self.head.column
        op = [0, 0, 0, 0]
        if(int(row_/20)>=0 and int(column_/20)+1>=0) and ( int(row_/20)<10 and int(column_/20)+1<10):
            if(self.maze[int(row_/20)][int(column_/20)+1] == 1): #right open
                op[0] = 1
        if (int(row_/20)>=0 and int(column_/20)-1>=0) and ( int(row_/20)<10 and int(column_/20)<10):
            if(self.maze[int(row_/20)][int(column_/20)-1] == 1 ): #left open
                op[2] = 1
        if (int(row_/20)+1>=0 and int(column_/20)>=0) and ( int(row_/20)+1<10 and int(column_/20)<10):
            if(self.maze[int(row_/20)+1][int(column_/20)] == 1):   #down open
                op[1] = 1
        if ( int(row_/20)-1>=0 and int(column_/20)>=0) and ( int(row_/20)<10 and int(column_/20)<10):
            if(self.maze[int(row_/20)-1][int(column_/20)] == 1 ):   # up open
                op[3] = 1
        opt =[0, 0, 0, 0]
        if self.direction == Direction.RIGHT:
            temp3 = op[3]
            opt= op
            opt[3]=opt[2]
            opt[2]=temp3
        if self.direction == Direction.LEFT:
            temp3 = op[3]
            opt= op
            opt[3]=opt[2]
            opt[2]=temp3
            opt.reverse()
        if self.direction == Direction.DOWN:
            temp0 = op[0]
            temp3 = op[3]
            opt[0]=op[1]
            opt[1]=op[2]
            opt[2]=temp0
            opt[3]=temp3
            
        if self.direction == Direction.UP:
            temp0 = op[0]
            temp3 = op[3]
            opt[0]=op[1]
            opt[1]=op[2]
            opt[2]=temp0
            opt[3]=temp3
            
            opt.reverse()
        #print("opt:")
        #print(opt)

            
        return opt

    def _move(self, action):
        #print("action:")
        #print(action)
        row_ = self.head.row
        column_ = self.head.column
        #print(row_)
        clock_wise = [Direction.RIGHT, Direction.DOWN, Direction.LEFT, Direction.UP]
        idx = clock_wise.index(self.direction)
        op = self._option()
        pause = 0
        if np.array_equal(action, [1, 0, 0, 0]) and op[0]==1:
            new_dir = clock_wise[idx] # no change
            #print('straight')
        elif np.array_equal(action, [0, 1, 0 , 0]) and op[1]==1:
            next_idx = (idx + 1) % 4

            new_dir = clock_wise[next_idx] # right turn r -> d -> l -> u
            #print('turn right')
        elif np.array_equal(action, [0, 0, 1, 0]) and op[2]==1:
            next_idx = (idx -1 ) % 4
            new_dir = clock_wise[next_idx] # left turn r -> d -> l -> u
            #print('turn left')
        elif np.array_equal(action, [0, 0, 0, 1]) and op[3]==1: # [0, 0, 1]
            next_idx = (idx - 2) % 4
            new_dir = clock_wise[next_idx] # turn around r -> u -> l -> d
            #print('turn around')
        elif np.array_equal(op, [0, 0, 0, 1]) and op[3]==1: # [0, 0, 1]
            next_idx = (idx - 2) % 4
            new_dir = clock_wise[next_idx] # turn around r -> u -> l -> d
            #print('turn around')
        else: # pause
            #print('should pause')
            pause =1
            next_idx = (idx) % 4
            new_dir = clock_wise[next_idx] #Pause	

        self.direction = new_dir	
        if self.direction == Direction.RIGHT and pause==0:
            self.maze[int(row_/20)][int(column_/20)+1] = .75
            self.maze[int(row_/20)][int(column_/20)] = default_maze[int(row_/20)][int(column_/20)]
            column_ += BLOCK_SIZE		
            #print('RIGHT')
        elif self.direction == Direction.LEFT and pause==0:
            self.maze[int(row_/20)][int(column_/20)-1] = .75
            self.maze[int(row_/20)][int(column_/20)] = default_maze[int(row_/20)][int(column_/20)]
            column_ -= BLOCK_SIZE
            #print('LEFT')
        elif self.direction == Direction.DOWN and pause==0:
            self.maze[int(row_/20)+1][int(column_/20)] = .75
            self.maze[int(row_/20)][int(column_/20)] = default_maze[int(row_/20)][int(column_/20)]
            row_ += BLOCK_SIZE
            #print('down')
        elif self.direction == Direction.UP and pause==0:
            self.maze[int(row_/20)-1][int(column_/20)] = .75
            self.maze[int(row_/20)][int(column_/20)] = default_maze[int(row_/20)][int(column_/20)]
            row_ -= BLOCK_SIZE
            #print('up')
        
        self.head = Point(row_, column_)

        #print(self.head)