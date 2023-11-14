from mesa.model import Model
from mesa.agent import Agent
from mesa.space import MultiGrid
from mesa.time import SimultaneousActivation
from mesa.datacollection import DataCollector

import numpy as np

class Cell():
    import sys
    FLT_MAX = sys.float_info.max
    def __init__(self, g=None, h=None, f=None, x=-1, y=-1, parent_x=None, parent_y=None, n=0):
        # Como se tienen varios destinos se ocupa una lista para cada atributo
        if g is None: self.g = [self.FLT_MAX for i in range(n)]
        else: self.g = g
        
        if h is None: self.h = [self.FLT_MAX for i in range(n)]
        else: self.h = h
        
        if f is None: self.f = [self.FLT_MAX for i in range(n)]
        else: self.f = f
        
        if parent_x is None: self.parent_x = [-1 for i in range(n)]
        else: self.parent_x = parent_x
        
        if parent_y is None: self.parent_y = [-1 for i in range(n)]
        else: self.parent_y = parent_y
        self.x = x
        self.y = y

    def __lt__(self, other):
        return self.f < other.f

class Charger(Agent):
    def __init(self,unique_id,model, agente: Agent = None):
        super().__init(unique_id, model)
        self.agente = Agent

class Celda(Agent):
    def __init__(self, unique_id, model, suciedad: bool = False):
        super().__init__(unique_id, model)
        self.sucia = suciedad


class Mueble(Agent):
    def __init__(self, unique_id, model):
        super().__init__(unique_id, model)


class RobotLimpieza(Agent):
    import sys
    FLT_MAX = sys.float_info.max
    def __init__(self, unique_id, model):
        super().__init__(unique_id, model)
        self.isCharging = False
        self.sig_pos = None
        self.movimientos = 0
        self.carga = 100
        self.camino_carga = []

    def limpiar_una_celda(self, lista_de_celdas_sucias):
        celda_a_limpiar = self.random.choice(lista_de_celdas_sucias)
        celda_a_limpiar.sucia = False
        self.sig_pos = celda_a_limpiar.pos

    def seleccionar_nueva_pos(self, lista_de_vecinos):
        self.sig_pos = self.random.choice(lista_de_vecinos).pos

    @staticmethod
    def buscar_celdas_sucia(lista_de_vecinos):
        
        celdas_sucias = list()
        for vecino in lista_de_vecinos:
            if isinstance(vecino, Celda) and vecino.sucia:
                celdas_sucias.append(vecino)
        return celdas_sucias

    def step(self):
        vecinos = self.model.grid.get_neighbors(

            self.pos, moore=True, include_center=False)

        vecinos_disponibles = list()
        for vecino in vecinos:
            if not isinstance(vecino, (Mueble, RobotLimpieza)):
                vecinos_disponibles.append(vecino)

        celdas_sucias = self.buscar_celdas_sucia(vecinos_disponibles)
        
        # Calcular ruta a cargador más cercano
        if (len(self.camino_carga) == 0) and self.carga < 30 and not self.isCharging :
            self.camino_carga = self.aStar()
        
        if(self.isCharging and not isinstance(self.model.grid.__getitem__((self.pos[0], self.pos[1]))[0], Charger)):
            self.isCharging = False

        # Elegir ruta
        if (len(self.camino_carga) > 0 and not self.isCharging):
            estimated_sig_pos = self.camino_carga[-1]       
            if not (isinstance(self.model.grid.__getitem__((estimated_sig_pos[0], estimated_sig_pos[1]))[0], RobotLimpieza) and isinstance(self.model.grid.__getitem__((estimated_sig_pos[0], estimated_sig_pos[1]))[0], Charger)):  
                sig_pos = self.camino_carga.pop()
                self.sig_pos = sig_pos
            if len(self.camino_carga) == 0 and isinstance(self.model.grid.__getitem__((self.sig_pos[0], self.sig_pos[1]))[0], Charger):
                self.isCharging = True
        elif len(celdas_sucias) == 0 and not self.isCharging:
            self.seleccionar_nueva_pos(vecinos_disponibles)
        else:
            if not self.isCharging:
                self.limpiar_una_celda(celdas_sucias)

        if self.carga >= 100:
            self.carga = 100
            self.isCharging = False

    def advance(self):
        robots = get_sig_positions(self.model)
        
        celdas_no_disponibles = list()
        cambio = False
        for robot in robots:
            if self.sig_pos == robot["sig_pos"] and self.unique_id != robot["unique_id"]:
                celdas_no_disponibles.append(robot["sig_pos"])
                cambio = True
        
        vecinos = self.model.grid.get_neighbors(
                        self.pos, moore=True, include_center=False)
        vecinos_disponibles = list()
        
        for vecino in vecinos:
            if not isinstance(vecino, Mueble) or (isinstance(vecino, RobotLimpieza) and vecino.sig_pos not in celdas_no_disponibles):
                vecinos_disponibles.append(vecino)
        
        if cambio:
            self.seleccionar_nueva_pos(vecinos_disponibles)

        if self.pos != self.sig_pos:
            self.movimientos += 1

        if self.carga > 0:
            if self.isCharging:
                if isinstance(self.model.grid.__getitem__((self.sig_pos[0], self.sig_pos[1]))[0], Charger): 
                    self.carga += 25
                    self.carga = min(self.carga, 100)
            else:
                self.carga -= 1
            self.model.grid.move_agent(self, self.sig_pos)

    def aStar(self):
        start = self.sig_pos
        dest = self.model.pos_cargadores
        n = len(dest)

        # Priority queue - Procesar nodo con menor f (Cell)
        open_list = list()
        # Almacena el path en orden inverso
        path = list()
        # Almacena los nodos visitados para saber si ya se visitaron en O(1)
        visited_nodes = set()
        # Almacena los detalles de cada nodo y se modifica en cada iteracion para reordenar adecuadamente la priority_queue
        cell_details = list()
        for i in range(self.model.grid.width):
            cell_details.append(list())
            for j in range(self.model.grid.height):
                cell_details[i].append(Cell(x=i, y=j, n=n))
        
        # Cambiar datos de nodos destinos en cell_details
        destination_cells = list()
        for x, y in dest:
            cell_details[x][y].g = [0] * n
            cell_details[x][y].h = [0] * n
            cell_details[x][y].f = [0] * n
            destination_cells.append(cell_details[x][y])

        # Cambiar datos de nodo inicial en cell_details
        start_cell = cell_details[start[0]][start[1]]
        start_cell.g = [0] * n
        start_cell.h = self.heuristic(start_cell, destination_cells)
        start_cell.f = [start_cell.g[i] + start_cell.h[i] for i in range(n)]
        start_cell.parent_x = [start[0]] * n
        start_cell.parent_y = [start[1]] * n
        
        # Para cada destino (cargador) se calcula el path y se elige el más corto
        for i in range(n):
            visited_nodes.add((start[0], start[1]))
            open_list.append(start_cell)

            path.append(self.aStarHelper(cell_details, open_list, destination_cells, visited_nodes, i))
            open_list.clear()
            visited_nodes.clear()
        
        return min(path, key=len)

    def aStarHelper(self, cell_details, open_list, destination_cells, visited_nodes, i):
        import heapq
        path = list()
        
        # 8 movimientos posibles
        moves = [(1, 0), (0, 1), (-1, 0), (0, -1), (1, 1),(-1, -1), (-1, 1), (1, -1)]
        # moves = [(1, 0), (0, 1), (-1, 0), (0, -1)]
        while len(open_list) != 0:
            heapq.heapify(open_list)
            current_cell = heapq.heappop(open_list)
            x = current_cell.x
            y = current_cell.y

            for move in moves:
                new_x = x + move[0]
                new_y = y + move[1]

                if not self.isValid(new_x, new_y):
                    continue
                
                # Recorrer para todos los destinos
                if (new_x, new_y) == (destination_cells[i].x, destination_cells[i].y):
                        cell_details[new_x][new_y].parent_x[i] = x
                        cell_details[new_x][new_y].parent_y[i] = y
                        
                        row = destination_cells[i].x
                        col = destination_cells[i].y

                        while not (cell_details[row][col].parent_x[i] == row and 
                                   cell_details[row][col].parent_y[i] == col):
                            path.append((row, col))
                            temp_row = cell_details[row][col].parent_x[i]
                            temp_col = cell_details[row][col].parent_y[i]
                            row = temp_row
                            col = temp_col
                        return path
                # Checar que no sea un obstaculo y no haya sido visitado
                elif (not isinstance(self.model.grid.__getitem__((new_x, new_y))[0], Mueble) and
                      (new_x, new_y) not in visited_nodes
                      ):
                    new_G = current_cell.g[i] + 1
                    new_H = self.heuristic(cell_details[new_x][new_y], [destination_cells[i]])
                    new_F = new_G + new_H[0]

                    if (cell_details[new_x][new_y].f[i] == self.FLT_MAX or 
                        cell_details[new_x][new_y].f[i] > new_F):
                        open_list.append(cell_details[new_x][new_y])
                        cell_details[new_x][new_y].f[i] = new_F
                        cell_details[new_x][new_y].g[i] = new_G
                        cell_details[new_x][new_y].h[i] = new_H
                        cell_details[new_x][new_y].parent_x[i] = x
                        cell_details[new_x][new_y].parent_y[i] = y
             
    def heuristic(self, src:Cell, dest:[]):
        import math
        # Distancia Euclidiana
        res = list()
        for cell in dest:
            res.append(math.sqrt((src.x - cell.x) ** 2 + (src.y - cell.y) ** 2))

        return res
    
    def isValid(self, row, col):
        return (row >= 0 and 
                row < self.model.grid.width and 
                col >= 0 and col < self.model.grid.height)


class Habitacion(Model):
    def __init__(self, M: int, N: int,
                 num_agentes: int = 5,
                 num_baterias: int = 4, 
                 porc_celdas_sucias: float = 0.6,
                 porc_muebles: float = 0.1,
                 modo_pos_inicial: str = 'Fija',
                 ):

        self.num_baterias = num_baterias
        self.num_agentes = num_agentes
        self.porc_celdas_sucias = porc_celdas_sucias
        self.porc_muebles = porc_muebles

        self.grid = MultiGrid(M, N, False)
        self.schedule = SimultaneousActivation(self)

        posiciones_disponibles = [pos for _, pos in self.grid.coord_iter()]

        # Posicionamiento de muebles
        num_muebles = int(M * N * porc_muebles)
        posiciones_muebles = self.random.sample(posiciones_disponibles, k=num_muebles)

        for id, pos in enumerate(posiciones_muebles):
            mueble = Mueble(int(f"{num_agentes}0{id}") + 1, self)
            self.grid.place_agent(mueble, pos)
            posiciones_disponibles.remove(pos)

        # Posicionamiento de cargadores
        pos_cargadores = self.random.sample(posiciones_disponibles, k=num_baterias)
        self.pos_cargadores = pos_cargadores

        for id in range(num_baterias):
            cargador = Charger(id + 77, self)
            self.grid.place_agent(cargador,pos_cargadores[id])
            posiciones_disponibles.remove(pos_cargadores[id])


        # Posicionamiento de celdas sucias
        self.num_celdas_sucias = int(M * N * porc_celdas_sucias)
        posiciones_celdas_sucias = self.random.sample(
            posiciones_disponibles, k=self.num_celdas_sucias)

        for id, pos in enumerate(posiciones_disponibles):
            suciedad = pos in posiciones_celdas_sucias
            celda = Celda(int(f"{num_agentes}{id}") + 1, self, suciedad)
            self.grid.place_agent(celda, pos)

        # Posicionamiento de agentes robot
        if modo_pos_inicial == 'Aleatoria':
            pos_inicial_robots = self.random.sample(posiciones_disponibles, k=num_agentes)
        else:  # 'Fija'
            pos_inicial_robots = [(1, 1)] * num_agentes

        for id in range(num_agentes):
            robot = RobotLimpieza(id, self)
            self.grid.place_agent(robot, pos_inicial_robots[id])
            self.schedule.add(robot)

        self.datacollector = DataCollector(
            model_reporters={"Grid": get_grid, "Cargas": get_cargas,
                             "CeldasSucias": get_sucias, "Movimientos": get_cant_movimientos},
        )

    def step(self):
        self.datacollector.collect(self)

        self.schedule.step()

    def todoLimpio(self):
        for (content, x, y) in self.grid.coord_iter():
            for obj in content:
                if isinstance(obj, Celda) and obj.sucia:
                    return False
        return True


def get_grid(model: Model) -> np.ndarray:
    """
    Método para la obtención de la grid y representarla en un notebook
    :param model: Modelo (entorno)
    :return: grid
    """
    grid = np.zeros((model.grid.width, model.grid.height))
    for cell in model.grid.coord_iter():
        cell_content, pos = cell
        x, y = pos
        for obj in cell_content:
            if isinstance(obj, RobotLimpieza):
                grid[x][y] = 2
            elif isinstance(obj, Celda):
                grid[x][y] = int(obj.sucia)
    return grid


def get_cargas(model: Model):
    return [(agent.unique_id, agent.carga) for agent in model.schedule.agents]


def get_sig_positions(model: Model):
    return [{"unique_id": agent.unique_id, "sig_pos" : agent.sig_pos} for agent in model.schedule.agents]


def get_positions(model: Model):
    return [{"unique_id": agent.unique_id, "pos" : agent.pos} for agent in model.schedule.agents]


def get_robots(model: Model):
    return [agent for agent in model.schedule.agents]

def get_sucias(model: Model) -> int:
    """
    Método para determinar el número total de celdas sucias
    :param model: Modelo Mesa
    :return: número de celdas sucias
    """
    sum_sucias = 0
    for cell in model.grid.coord_iter():
        cell_content, pos = cell
        for obj in cell_content:
            if isinstance(obj, Celda) and obj.sucia:
                sum_sucias += 1
    return sum_sucias / model.num_celdas_sucias

def get_cant_movimientos(model: Model) -> int:
    """
    Método para determinar el número de movimientos de cada robot
    """
    movimientos = 0
    for agent in model.schedule.agents:
        if isinstance(agent, RobotLimpieza):
            movimientos += agent.movimientos
    return movimientos

def get_movimientos(agent: Agent) -> dict:
    if isinstance(agent, RobotLimpieza):
        return {agent.unique_id: agent.movimientos}
    # else:
    #    return 0
