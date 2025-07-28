# Nothing to implement here
# This is a python script to help you visualise your outputs

import pygame
import csv
import os

WINDOW_SIZE = 500
BG_COLOR = (255, 255, 255)
OBSTACLE_COLOR = (0, 100, 200)
OBSTACLE_ALPHA = 180
NODE_COLOR = (0, 180, 0)
NODE_RADIUS = 1
EDGE_COLOR = (120, 120, 120)
EDGE_WIDTH = 2
PATH_COLOR = (220, 0, 0)
PATH_WIDTH = 4

def load_obstacles(csv_path):
    obstacles = []
    with open(csv_path, newline='') as csvfile:
        reader = csv.reader(csvfile)
        for row in reader:
            if not row or row[0].strip().startswith('#'):
                continue
            vals = [float(x.strip()) for x in row]
            if len(vals) == 3:
                obstacles.append({'x': vals[0], 'y': vals[1], 'diameter': vals[2]})
    return obstacles

def load_nodes(csv_path):
    nodes = {}
    with open(csv_path, newline='') as csvfile:
        reader = csv.reader(csvfile)
        for row in reader:
            if not row or row[0].strip().startswith('#'):
                continue
            if len(row) >= 3:
                try:
                    node_id = int(row[0].strip())
                    x = float(row[1].strip())
                    y = float(row[2].strip())
                    nodes[node_id] = {'x': x, 'y': y}
                except Exception:
                    continue
    return nodes

def load_edges(csv_path):
    edges = []
    with open(csv_path, newline='') as csvfile:
        reader = csv.reader(csvfile)
        for row in reader:
            if not row or row[0].strip().startswith('#'):
                continue
            if len(row) >= 2:
                try:
                    n1 = int(row[0].strip())
                    n2 = int(row[1].strip())
                    edges.append((n1, n2))
                except Exception:
                    continue
    return edges

def load_path(csv_path):
    with open(csv_path, newline='') as csvfile:
        reader = csv.reader(csvfile)
        try:
            row = next(reader)
            path = [int(x.strip()) for x in row if x.strip().isdigit()]
        except StopIteration:
            path = []
    return path

def world_to_screen(x, y, scale, offset):
    sx = int(WINDOW_SIZE // 2 + x * scale + offset[0])
    sy = int(WINDOW_SIZE // 2 - y * scale + offset[1])
    return sx, sy

def main():
    pygame.init()
    screen = pygame.display.set_mode((WINDOW_SIZE, WINDOW_SIZE))

    obstacles = load_obstacles(os.path.join("CSV", "obstacles.csv"))
    nodes = load_nodes(os.path.join("CSV", "nodes.csv"))
    edges = load_edges(os.path.join("CSV", "edges.csv"))
    path = load_path(os.path.join("CSV", "path.csv"))

    # Find scale to fit all obstacles and nodes
    margin = 0.1
    all_r = [o['diameter']/2 for o in obstacles] if obstacles else [0]
    min_x = min([x - r for x, r in zip([o['x'] for o in obstacles], all_r)] + [n['x'] for n in nodes.values()]) - margin
    max_x = max([x + r for x, r in zip([o['x'] for o in obstacles], all_r)] + [n['x'] for n in nodes.values()]) + margin
    min_y = min([y - r for y, r in zip([o['y'] for o in obstacles], all_r)] + [n['y'] for n in nodes.values()]) - margin
    max_y = max([y + r for y, r in zip([o['y'] for o in obstacles], all_r)] + [n['y'] for n in nodes.values()]) + margin
    world_w = max_x - min_x
    world_h = max_y - min_y
    scale = int(0.9 * WINDOW_SIZE / max(world_w, world_h))
    offset = [int(-((min_x + max_x) / 2) * scale), int(((min_y + max_y) / 2) * scale)]

    running = True
    clock = pygame.time.Clock()
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        screen.fill(BG_COLOR)

        top_left = world_to_screen(-0.55, 0.55, scale, offset)
        top_right = world_to_screen(0.55, 0.55, scale, offset)
        bottom_right = world_to_screen(0.55, -0.55, scale, offset)
        bottom_left = world_to_screen(-0.55, -0.55, scale, offset)
        pygame.draw.lines(
            screen, (0, 0, 0), True,
            [top_left, top_right, bottom_right, bottom_left], 2
        )
        for n1, n2 in edges:
            if n1 in nodes and n2 in nodes:
                x1, y1 = nodes[n1]['x'], nodes[n1]['y']
                x2, y2 = nodes[n2]['x'], nodes[n2]['y']
                sx1, sy1 = world_to_screen(x1, y1, scale, offset)
                sx2, sy2 = world_to_screen(x2, y2, scale, offset)
                pygame.draw.line(screen, EDGE_COLOR, (sx1, sy1), (sx2, sy2), EDGE_WIDTH)

        
        if len(path) > 1:
            for i in range(len(path)-1):
                n1, n2 = path[i], path[i+1]
                if n1 in nodes and n2 in nodes:
                    x1, y1 = nodes[n1]['x'], nodes[n1]['y']
                    x2, y2 = nodes[n2]['x'], nodes[n2]['y']
                    sx1, sy1 = world_to_screen(x1, y1, scale, offset)
                    sx2, sy2 = world_to_screen(x2, y2, scale, offset)
                    pygame.draw.line(screen, PATH_COLOR, (sx1, sy1), (sx2, sy2), PATH_WIDTH)

        
        for obs in obstacles:
            x, y, d = obs['x'], obs['y'], obs['diameter']
            sx, sy = world_to_screen(x, y, scale, offset)
            radius = int((d / 2) * scale)
            surface = pygame.Surface((radius*2, radius*2), pygame.SRCALPHA)
            pygame.draw.circle(surface, OBSTACLE_COLOR + (OBSTACLE_ALPHA,), (radius, radius), radius)
            screen.blit(surface, (sx - radius, sy - radius))

       
        for idx, node in nodes.items():
            sx, sy = world_to_screen(node['x'], node['y'], scale, offset)
            pygame.draw.circle(screen, NODE_COLOR, (sx, sy), NODE_RADIUS)

        pygame.display.flip()
        clock.tick(20)

    pygame.quit()

main()