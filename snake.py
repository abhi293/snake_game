import pygame
import random
import time
import sys
from search_algorithms import *

# Parse command-line arguments
if len(sys.argv) != 3:
    print("Usage: python snake.py <level> <search_algorithm>")
    sys.exit(1)

level = sys.argv[1].lower()  # Convert to lowercase
search_algorithm = sys.argv[2].lower()

# Validate level
LEVELS = {"level0": 0, "level1": 5, "level2": 10, "level3": 15}
if level not in LEVELS:
    print("Invalid level! Choose from: level0, level1, level2, level3")
    sys.exit(1)

# Validate search algorithm
ALGORITHMS = {"bfs": bfs, "dfs": dfs, "ucs": ucs, "ids": ids, "a*": astar, "random": random_move, "greedy_bfs": greedy_bfs}
if search_algorithm not in ALGORITHMS:
    print("Invalid search algorithm! Choose from: bfs, dfs, ucs, ids, a*, random, greedy_bfs")
    sys.exit(1)

# Initialize pygame
pygame.init()

# Constants
WIDTH, HEIGHT = 500, 500
CELL_SIZE = 20
WHITE, BLACK, GREEN, RED, GRAY = (255, 255, 255), (0, 0, 0), (0, 255, 0), (255, 0, 0), (128, 128, 128)
FONT = pygame.font.Font(None, 36)

# Grid settings
ROWS, COLS = WIDTH // CELL_SIZE, HEIGHT // CELL_SIZE

# AI Snake starting position
snake_pos = [ROWS // 2, COLS // 2]

# Food position
food_pos = [random.randint(0, ROWS - 1), random.randint(0, COLS - 1)]

# Timer settings
TIME_LIMIT = 30
start_time = time.time()

# Setup Pygame window
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption(f"AI Snake Game ({search_algorithm.upper()} - {level.upper()})")

clock = pygame.time.Clock()

# Score
score = 0

# Generate obstacles based on level
obstacles = set()
OBSTACLE_COUNT = (ROWS * COLS * LEVELS[level]+5) // 100  # % of total grid size
while len(obstacles) < OBSTACLE_COUNT:
    obstacle = (random.randint(0, ROWS - 1), random.randint(0, COLS - 1))
    if obstacle != tuple(snake_pos) and obstacle != tuple(food_pos):
        obstacles.add(obstacle)

# Directions (Up, Down, Left, Right)
DIRECTIONS = [(-1, 0), (1, 0), (0, -1), (0, 1)]

# Game Over function
def game_over():
    my_font = pygame.font.SysFont('times new roman', 50)
    game_over_surface = my_font.render(
        'Your Score is : ' + str(score), True, RED)
    game_over_rect = game_over_surface.get_rect()
    game_over_rect.midtop = (WIDTH/2, HEIGHT/4)
    screen.blit(game_over_surface, game_over_rect)
    pygame.display.flip()

    print("Score: ", score)
    time.sleep(1)
    pygame.quit()
    quit()

def regenerate_food():
    max_distance = 6 # Food will generate within 5 steps of the snake
    while True:
        # Generate food within a limited distance from the snake
        row_offset = random.randint(-max_distance, max_distance)
        col_offset = random.randint(-max_distance, max_distance)
        new_food_pos = (snake_pos[0] + row_offset, snake_pos[1] + col_offset)

        # Ensure the food is within bounds and not on an obstacle or the snake
        if (
            0 <= new_food_pos[0] < ROWS and
            0 <= new_food_pos[1] < COLS and
            new_food_pos not in obstacles and
            new_food_pos != tuple(snake_pos)
        ):
            return new_food_pos


running = True
path = []
circling_steps = 0  # Count how many steps the snake has been circling
max_circling_steps = 20  # Limit for circling steps before forcing food regeneration

while running:
    screen.fill(BLACK)

    # Timer logic
    elapsed_time = time.time() - start_time
    time_left = max(0, TIME_LIMIT - int(elapsed_time))

    # If time runs out, end game
    if time_left == 0:
        running = False
        game_over()

    # Check if snake has consumed food
    if tuple(snake_pos) == tuple(food_pos):
        score += 1
        food_pos = regenerate_food()
        path = []  # Reset path to calculate a new one
        circling_steps = 0  # Reset circling steps

    # If no path exists, attempt to regenerate or move randomly
    if not path:
        path = ALGORITHMS[search_algorithm](tuple(snake_pos), tuple(food_pos), obstacles, ROWS, COLS)
        if not path:
            # If no valid path is found, move randomly
            random.shuffle(DIRECTIONS)
            for direction in DIRECTIONS:
                new_row = snake_pos[0] + direction[0]
                new_col = snake_pos[1] + direction[1]
                if (
                    0 <= new_row < ROWS and 0 <= new_col < COLS and
                    (new_row, new_col) not in obstacles
                ):
                    path = [direction]
                    break

    # Move AI Snake
    if path:
        move = path.pop(0)
        snake_pos[0] += move[0]
        snake_pos[1] += move[1]

    # Check if AI hits wall or obstacle (Game Over)
    if snake_pos[0] < 0 or snake_pos[0] >= ROWS or snake_pos[1] < 0 or snake_pos[1] >= COLS or tuple(snake_pos) in obstacles:
        running = False
        game_over()

    # Draw Snake
    pygame.draw.rect(screen, GREEN, (snake_pos[1] * CELL_SIZE, snake_pos[0] * CELL_SIZE, CELL_SIZE, CELL_SIZE))

    # Draw Food
    pygame.draw.rect(screen, RED, (food_pos[1] * CELL_SIZE, food_pos[0] * CELL_SIZE, CELL_SIZE, CELL_SIZE))

    # Draw Obstacles
    for obs in obstacles:
        pygame.draw.rect(screen, GRAY, (obs[1] * CELL_SIZE, obs[0] * CELL_SIZE, CELL_SIZE, CELL_SIZE))

    # Display Timer
    timer_text = FONT.render(f"Time Left: {time_left}s", True, WHITE)
    screen.blit(timer_text, (20, 20))

    # Display Score
    score_text = FONT.render(f"Score: {score}", True, WHITE)
    screen.blit(score_text, (20, 50))

    pygame.display.update()
    clock.tick(5)  # Control AI speed

    # Handle events
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
            game_over()

pygame.quit()

print("Score: ", score)
