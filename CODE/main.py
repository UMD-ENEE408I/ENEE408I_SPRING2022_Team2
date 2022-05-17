import pygame
import random
from enum import Enum
from collections import namedtuple
import numpy as np
default_maze = [[1,   1,   1,   1,   0,   0,   0,   0,   0,   0],
        [0,   0,   0,   1,   0,   0,   0,   0,   0,   0],
        [0,   1,   1,   1,   1,   1,   1,   1,   1,   0],
        [0,   0,   0,   1,   0,   0,   0,   1,   0,   0],
        [0,   0,   0,   1,   1,   1,   1,   1,   1,  0],
        [0,   0,   0,   1,   0,   0,   0,   0,   1,   0],
        [0,   0,   0,   1,   0,   0,   0,   0,   1,   0],
        [0,   0,   0,   1,   0,   0,   0,   0,   1,   0],
        [0,   0,   0,   1,   0,   0,   0,   0,   1,   0],
        [0,   0,   0,   1,   0,   0,   0,   0,   1,   .5]]

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
SPEED = .5
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
        self.maze = [[1,   1,   1,   1,   0,   0,   0,   0,   0,   0],
        [0,   0,   0,   1,   0,   0,   0,   0,   0,   0],
        [0,   1,   1,   1,   1,   1,   1,   1,   1,   0],
        [0,   0,   0,   1,   0,   0,   0,   1,   0,   0],
        [0,   0,   0,   1,   1,   1,   1,   1,   1,  0],
        [0,   0,   0,   1,   0,   0,   0,   0,   1,   0],
        [0,   0,   0,   1,   0,   0,   0,   0,   1,   0],
        [0,   0,   0,   1,   0,   0,   0,   0,   1,   0],
        [0,   0,   0,   1,   0,   0,   0,   0,   1,   0],
        [0,   0,   0,   1,   0,   0,   0,   0,   1,   .5]]
        self.discoveredMaze = [([0]*10) for i in range(10)]
        self.clock = pygame.time.Clock()
        self.reset()

    def reset(self):
        # init game state
        self.direction = Direction.RIGHT
        self.head = Point(0, 0)
        self.score = 0
        self.frame_iteration = 0

    
    def play_step(self,action):
        # 1. collect user input
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                quit()

        
        # 2. move
        self.possibleMoves = [0,0,0,0]#self._options()
        self._move(action) # update the head
       
        
        # 3. check if game over
        if (self.head.column==END_C  and self.head.row == END_R):
            game_over = True 
            self.score=1
            print("Solved")
        else:
            game_over = False
            

        
        # 5. update ui and clock
        self._update_ui()
        self.clock.tick(SPEED)
        # 6. return game over and score
        return game_over, self.score
    
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
        clock_wise = [Direction.RIGHT, Direction.DOWN, Direction.LEFT, Direction.UP]
        idx = clock_wise.index(self.direction)
        print(idx)
        if(self.maze[int(row_/20)][int(column_/20)+1] == 1): #right open
            op[0] = 1
        if(self.maze[int(row_/20)][int(column_/20)-1] == 1): #left open
            op[2] = 1
        if(self.maze[int(row_/20)+1][int(column_/20)] == 1):   #down open
            op[1] = 1
        if(self.maze[int(row_/20)-1][int(column_/20)] == 1):   # up open
            op[3] = 1
        #opt =[0, 0, 0, 0]
        #for i in range(4):
        #    opt= op((idx+i)%4)
        return op

    def _move(self, action):
        row_ = self.head.row
        column_ = self.head.column
        print(row_)
        clock_wise = [Direction.RIGHT, Direction.DOWN, Direction.LEFT, Direction.UP]
        idx = clock_wise.index(self.direction)
        op = self._option()
        pause = 0
        if np.array_equal(action, [1, 0, 0, 0]) and op[0]==1:
            new_dir = clock_wise[idx] # no change
        elif np.array_equal(action, [0, 1, 0 , 0]) and op[1]==1:
            next_idx = (idx + 1) % 4
            new_dir = clock_wise[next_idx] # right turn r -> d -> l -> u
        elif np.array_equal(action, [0, 0, 1, 0]) and op[2]==1:
            next_idx = (idx -1 ) % 4
            new_dir = clock_wise[next_idx] # left turn r -> d -> l -> u
        elif np.array_equal(action, [0, 0, 0, 1]) and op[3]==1: # [0, 0, 1]
            next_idx = (idx - 2) % 4
            new_dir = clock_wise[next_idx] # turn around r -> u -> l -> d
        else: # pause
            print('should pause')
            pause =1
            next_idx = (idx) % 4
            new_dir = clock_wise[next_idx] #Pause	

        self.direction = new_dir	
        if self.direction == Direction.RIGHT and pause==0:
            self.maze[int(row_/20)][int(column_/20)+1] = .75
            self.maze[int(row_/20)][int(column_/20)] = default_maze[int(row_/20)][int(column_/20)]
            column_ += BLOCK_SIZE		
            print('RIGHT')
        elif self.direction == Direction.LEFT and pause==0:
            self.maze[int(row_/20)][int(column_/20)-1] = .75
            self.maze[int(row_/20)][int(column_/20)] = default_maze[int(row_/20)][int(column_/20)]
            column_ -= BLOCK_SIZE
            print('LEFT')
        elif self.direction == Direction.DOWN and pause==0:
            self.maze[int(row_/20)+1][int(column_/20)] = .75
            self.maze[int(row_/20)][int(column_/20)] = default_maze[int(row_/20)][int(column_/20)]
            row_ += BLOCK_SIZE
            print('down')
        elif self.direction == Direction.UP and pause==0:
            self.maze[int(row_/20)-1][int(column_/20)] = .75
            self.maze[int(row_/20)][int(column_/20)] = default_maze[int(row_/20)][int(column_/20)]
            row_ -= BLOCK_SIZE
            print('up')
        
        self.head = Point(row_, column_)
        #maze = defalt_maze
        #print(maze)
        #maze[int(x/20)][int(y/20)] = .75
        #print(maze)
        print(self.head)

if __name__ == '__main__':
    game = mazeAI()
    
    # game loop
    while True:
    #for i in range(10):
        action = [0, 0 , 0 ,0]
        action[random.randint(0,3)] = 1
        print(action)
        game_over, score = game.play_step(action)
        
        if game_over == True:
            break   
    print('Solved?', score)
        
        
    pygame.quit()