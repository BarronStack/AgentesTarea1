# Algoritmos de Búsqueda en Pacman
### Seminario II - Agentes Autónomos

Autor    : Juan Pablo Sánchez - 201630002-3 <br>
Profesor : Mauricio Araya 

El objetivo de esta tarea involucra la implementación de algoritmos de búsquedad sobre un agente Pacman desarrollado para el curso [CS188 de la Universidad de Berkeley](https://inst.eecs.berkeley.edu/~cs188/sp21/project1/). 

# Tabla de Contenidos
1. [Objetivos](#id0)
2. [Getting started](#id1)
3. [Problema : Encontrar comida en un punto fijo del mapa](#id2)
    * [Depth-First Search](#id2.1) 
    * [Breath-First Search](#id2.2)
    * [A* Search](#id2.3)
4. [Problema : Encontrar las esquinas del mapa](#id3)
5. [Problema : Heuristica para encontrar las esquinas mapa](#id4)

# Objetivos <div id="id0"></div>

- Desarrolle o adapte una solución de los 3 primeros ejercicios.
- Desarrolle uno o toos los ejercicios del 4 al 7.
- Proponga una modificación:
    - Al ambiente u objetivo del agente.
    - A la heuristica utilizada para la búsqueda.
    - A los algoritmos propuestos.
- Comente su código adecuadamente.
- Genere un video explicativo de 5 minutos explicando su modificación y mostrando los resultados.
- Suba su código y reporte a [aula](aula.usm.cl). Debe incluir un link donde pueda visualizarse su video.

# Getting Started <div id="id1"></div>

El proyecto utiliza Python en una versión superior a 3.0. Esta implementación en particular fue desarrollada en Python 3.8.10.

1. Clonar el repositorio
~~~~
git clone git@github.com:BarronStack/AgentesTarea1.git
~~~~

2. Iniciar un juego interactivo Pacman. Utiliza las teclas WASD o las flechas el teclado para controlar Pacman.
~~~~
python3 pacman.py
~~~~
# Problema : Encontrar comida en un punto fijo del mapa <div id="id2"></div>

A continuación se presentan las implementaciones de los algoritmos de búsqueda *DFS*, *BFS* y*A* * realizadas en el archivo `search.py` el cuál almacena los algoritmos a ser utilizados por los agentes de búsqueda en el archivo `searchAgents.py`. 

El objetivo es desarrollar los algoritmos de tal forma que permitan al agente encontrar una ruta oṕtima al punto donde se encuentra la comida dentro del laberinto. Estos algoritmos serán utilizados posteriormente por los agentes de búsqueda a implementar en la sección del problema de la búsqueda de las esquinas del laberinto.

Los algoritmos en general reciben como entrada el problema, el cual a su vez almacena:
- El estado inicial.
- El estado objetivo.
- La función para expandir el camino (determinarndo posibles rutas, acciones y costos).

Puntualmente, el algoritmo `aStarSearch()` tiene como entrada la heuristica a utilizar dentro del cálculo de los costos.

Los algoritmos retornan como salida la serie de `acciones` o `movimientos` calculados que debe realizar el agente para llegar al punto objetivo. 

# Depth-First Search <div id="id2.1"></div>

El algoritmo DFS (Búsqueda en profundidad) es un tipo de algoritmo de búsqueda no informada utilizado para recorrer los posibles nodos de un grafo o árbol. Funciona expandiendo todos los caminos o ramas que va encontrando. Una vez finaliza su exploración en cierto camino, regresa a un nodo anterior y repite el mismo proceso por otro camino no explorado. 

Si bien, este algoritmo explora completamente los caminos posibles no implica ser el oṕtimo en términos de costos o tiempo de ejecución. 
 

```python
def depthFirstSearch(problem):    
    print("Start:", problem.getStartState())
    print("Is the start a goal?", problem.isGoalState(problem.getStartState()))

    # Se define el nodo inicial desde donde se comenzará a explorar el laberinto.
    # Este nodo incluye el estado actual (coordenadas x,y) y las acciones a realizar.
    # Al no ser expandido todavía, se desconocen las acciones a realizar.
    node = {"state":problem.getStartState(),"actions":[]}
    
    # Se utiliza un Stack para aplicar una politica LIFO al ingresar y remover nodos.
    frontier = util.Stack()
    frontier.push(node)

    explored = []

    # Se repite el proceso mientras existan posibles caminos a explorar.
    while not frontier.isEmpty():
        
        # Se adquiere el primer nodo en la frontera y se comprueba si no ha sido explorado/expandido.
        # De no ser explorado todavía se expande chequeando primero si corresponde la punto objetivo.
        
        node = frontier.pop()
        currentState = node["state"]
        actionsState = node["actions"]

        if currentState not in explored:
            explored.append(currentState)
        
            if problem.isGoalState(currentState):
                return actionsState

            else:
                # Se expande el nodo con sus posibles caminos, definiendo los estados y acciones para llegar a cada uno de ellos agregandolos a nuestra frontera.
                # Una vez hecho esto, se irá explorando en orden a partir del primer nodo que se encuentre repitiendo el proceso hasta que no pueda expandirse más.

                successors = problem.expand(currentState)
                for successor in successors:
                    newActionState = actionsState + [successor[1]]

                    # Se almacena las coordenadas del nuevo estado y las acciones para llegar al este.
                    newNode = {"state":successor[0],"actions":newActionState}
                    frontier.push(newNode)
    

    # Se retorna la serie de acciones que debe realizar el agente para arribar al objetivo por una ruta. No necesariamente será la ruta oṕtima. 
    return actionsState
```

### Ejecución del algoritmo.

Para comprobar la implementación del algoritmo DFS en el problema, se pueden usar los siguientes comandos para ejecutar Pacman con un `searchAgent` que utilice el algoritmo en 3 casos de laberintos predefinidos.

~~~~
python3 pacman.py -l tinyMaze -p SearchAgent -a fn=dfs
~~~~

~~~~
python3 pacman.py -l mediumMaze -p SearchAgent -a fn=dfs
~~~~

~~~~
python3 pacman.py -l bigMaze -z .5 -p SearchAgent -a fn=dfs
~~~~

# Breath-First Search <div id="id2.2"></div>

El algoritmo BFS (Búsqueda en anchura) es un tipo de algoritmo de búsqueda no informada utilizado para recorrer los posibles nodos de un grafo o árbol. Funciona expandiendo y explorando los nodos adyacentes a la posición de forma recursiva así recorriendo todos los vecinos sucesivamente hasta completar la búsqueda. 

En este algoritmo, cada búsqueda tendrá un costo asociado con el que se dará prioridad a que nodos vecinos ir explorando. Si bien este algoritmo puede entregar una solución oṕtima, la velocidad con la que obtenga la solución dependerá de la complejidad y los costos de la estructura a explorar.

```Python
def breadthFirstSearch(problem):
    print("Start:", problem.getStartState())
    print("Is the start a goal?", problem.isGoalState(problem.getStartState()))    
    
    # Se define el nodo inicial desde donde se comenzará a explorar el laberinto.
    # Este nodo incluye el estado actual (coordenadas x,y),las acciones a realizar y se le asigna un costo.
    # Al no ser expandido todavía, se desconocen las acciones a realizar.
    node = {"state":problem.getStartState(),"actions":[],"cost":0}
    
    # Se utiliza un Queue para aplicar una politica FIFO al ingresar y remover nodos.
    frontier = util.Queue()
    frontier.push(node)

    explored = [] 

    # Se repite el proceso mientras existan posibles caminos a explorar.
    while not frontier.isEmpty():
        
        # Se adquiere el primer nodo en la frontera y se comprueba si no ha sido explorado/expandido.
        # De no ser explorado todavía se expande chequeando primero si corresponde la punto objetivo.
        
        node = frontier.pop()
        currentState = node["state"]
        actionsState = node["actions"]
        costState    = node["cost"]

        if currentState not in explored:
            explored.append(currentState)
        
            if problem.isGoalState(currentState):
                return actionsState

            else:
                # Se expande el nodo con sus posibles caminos, definiendo los estados, acciones y costos para llegar a cada uno de ellos agregandolos a nuestra frontera.
                # Se incorpora un costo asociado a la acción que debo realizar para llegar al nuevo nodo.
                # Teniendo los costos asociados, se puede expandir primero la ruta adyancente que tenga un menor costo de exploración. 
                successors = problem.expand(currentState)
                for successor in successors:
                    newActionState = actionsState + [successor[1]]
                    newCostState   = costState + successor[2]
                    
                    # Se almacena las coordenadas del nuevo estado, las acciones y el costo asociado a las acciones para llegar a este.
                    newNode = {"state":successor[0],"actions":newActionState,"cost":newCostState}
                    frontier.push(newNode)  
    
    # Se retorna la serie de acciones que debe realizar el agente para arribar al objetivo por una ruta.
    return actionsState
```
### Ejecución del algoritmo.

Para comprobar la implementación del algoritmo BFS en el problema,  se pueden usar los siguientes comandos para ejecutar Pacman con un `searchAgent` que utilice el algoritmo en 3 casos de laberintos predefinidos.

~~~~
python3 pacman.py -l tinyMaze -p SearchAgent -a fn=bfs
~~~~

~~~~
python3 pacman.py -l mediumMaze -p SearchAgent -a fn=bfs
~~~~

~~~~
python3 pacman.py -l bigMaze -z .5 -p SearchAgent -a fn=bfs
~~~~

# A* Search <div id="id2.3"></div>

```Python
def aStarSearch(problem, heuristic=nullHeuristic):  
    print("Start:", problem.getStartState())
    print("Is the start a goal?", problem.isGoalState(problem.getStartState()))

    # Se define el nodo inicial desde donde se comenzará a explorar el laberinto.
    # Este nodo incluye el estado actual (coordenadas x,y),las acciones a realizar y se le asigna un costo.
    # Al no ser expandido todavía, se desconocen las acciones a realizar.
    node = {"state":problem.getStartState(),"actions":[],"cost":0}

    # Se utiliza una PriorityQueue para aplicar una politica de búsqueda según prioridad definida por el cálculo de la heuristica.
    frontier = util.PriorityQueue()
    frontier.push(node, 0)

    explored = [] 

    # Se repite el proceso mientras existan posibles caminos a explorar.
    while not frontier.isEmpty():
        
        # Se adquiere el primer nodo en la frontera y se comprueba si no ha sido explorado/expandido.
        # De no ser explorado todavía se expande chequeando primero si corresponde la punto objetivo.
        

        node = frontier.pop()
        currentState = node["state"]
        actionsState = node["actions"]
        costState    = node["cost"]

        explored.append((currentState, costState))

        if problem.isGoalState(currentState):
            return actionsState

        else:
            # Se expande el nodo con sus posibles caminos, definiendo los estados, acciones y costos para llegar a cada uno de ellos agregandolos a nuestra frontera.

            successors = problem.expand(currentState)
            for successor in successors:
                newCurrentState = successor[0]
                newActionState  = actionsState + [successor[1]]
                newCostState    = problem.getCostOfActionSequence(newActionState)

                # En este instante se define el nuevo nodo a explorar con el costo asociado a la ruta pero está pendiente comprobar si ya fue explorado y definir la prioridad que tendrá según el cálculo de la heuristica.

                newNode = {"state":newCurrentState,"actions":newActionState,"cost":newCostState}

                # Se comprueba si el siguiente nodo futuro ya ha sido explorado anteriormente o no 
                visited = False
                for exploredNode in explored:
                    exploredState, exploredCost = exploredNode

                    if (exploredState == newCurrentState) and (exploredCost <= newCostState):
                        visited = True

                # En caso de no ser explorado se puede considerar en la ruta futura. Por lo que se almacena su información más una prioridad h() definida el cálculo de la heuristica que define cuanto es el costo faltante para llegar al destino. Así el algoritmo puede identificar y explorar los nodos más cercanos al objetivo. 
                if not visited:
                    newPriority = newCostState + heuristic(newCurrentState,problem)

                    frontier.push(newNode,newPriority)
                    explored.append((newCurrentState,newCostState))

    # Se retorna la serie de acciones que debe realizar el agente para arribar al objetivo por una ruta.
    return actionsState
```

### Ejecución del algoritmo.

Para comprobar la implementación del algoritmo A* en el problema, se puede utilizar el siguiente comando para ejecutar Pacman con un `searchAgent` que utilice el algoritmo y haga uso de una heuristica trivial para determinar los costos.

~~~~
python3 pacman.py -l bigMaze -z .5 -p SearchAgent -a fn=astar
~~~~

Para la solución oṕtima, se pueden utilizar los siguientes comandos para ejecutar Pacman con un `searchAgent` que utilice el algoritmo y haga uso de una heuristica a base de la distancia de manhattan para calcular los costos en 3 casos de laberintos predefinidos.

~~~~
python3 pacman.py -l tinyMaze -p SearchAgent -a fn=astar,heuristic=manhattanHeuristic
~~~~

~~~~
python3 pacman.py -l mediumMaze -p SearchAgent -a fn=astar,heuristic=manhattanHeuristic
~~~~

~~~~
python pacman.py -l bigMaze -z .5 -p SearchAgent -a fn=astar,heuristic=manhattanHeuristic
~~~~

# Problema : Encontrar las esquinas del mapa <div id="id3"></div>
# Problema : Heuristica para encontrar las esquinas mapa <div id="id2.4"></div> 
