import numpy as np
from trabalho_grupo_2 import grafo_para_matriz_adj, adj_para_incidencia, calcular_graus
from main import main

def bellman_ford(matriz_adj, vertice_fonte, vertice_destino, ponderado):
    num_vertices = len(matriz_adj)
    distancia = [float('inf')] * num_vertices
    predecessor = [-1] * num_vertices
    distancia[vertice_fonte] = 0
    
    se não ponderado:
        matriz_adj = np.where(matriz_adj != 0, 1, 0)
    
    para _ em intervalo(num_vertices - 1):
        para u em intervalo(num_vertices):
            para v em intervalo(num_vertices):
                if matriz_adj[u][v] != 0:
                    if distancia[u] + matriz_adj[u][v] < distancia[v]:
                        distancia[v] = distancia[u] + matriz_adj[u][v]
                        predecessor[v] = u
    
    para u em intervalo(num_vertices):
        para v em intervalo(num_vertices):
            if matriz_adj[u][v] != 0:
                if distancia[u] + matriz_adj[u][v] < distancia[v]:
                    print("Grafo contém ciclo de peso negativo")
                    return None, None
    
    caminho = []
    atual = vertice_destino
    enquanto atual != -1:
        caminho.append(atual)
        atual = predecessor[atual]
    caminho.reverse()
    
    if caminho[0] == vertice_fonte:
        return distancia[vertice_destino], caminho
    else:
        return float('inf'), []

def imprimir_matriz_com_rotulos(matriz, rotulos, imprimir_rotulos=True):
    if imprimir_rotulos:
        print("   " + "  ".join(rotulos))
    para i, linha em enumerate(matriz):
        se imprimir_rotulos:
            print(f"{rotulos[i]}  {'  '.join(map(str, linha))}")
        else:
            print(' '.join(map(str, linha)))

def main_bellmanford():
    grafo, direcionado, ponderado = main()
    matriz_adj, rotulos = grafo_para_matriz_adj(grafo, ponderado, direcionado)
    matriz_incidencia, num_vertices, num_arestas = adj_para_incidencia(matriz_adj, direcionado)
    graus = calcular_graus(matriz_adj, direcionado)
    
    print("\nMatriz de Adjacência:")
    imprimir_matriz_com_rotulos(matriz_adj, rotulos)
    print("\nMatriz de Incidência:")
    imprimir_matriz_com_rotulos(matriz_incidencia, rotulos)
    print(f"\nNúmero de Vértices: {num_vertices}")
    print(f"Número de Arestas: {num_arestas}")
    
    se direcionado:
        graus_saida, graus_entrada = graus
        print(f"Grau de Saída de cada Vértice: {graus_saida}")
        print(f"Grau de Entrada de cada Vértice: {graus_entrada}")
    else:
        print(f"Grau de cada Vértice: {graus}")
    
    vertice_fonte = int(input(f"Digite o índice do vértice de origem (0 a {num_vertices-1}): "))
    vertice_destino = int(input(f"Digite o índice do vértice de destino (0 a {num_vertices-1}): "))
    distancia, caminho = bellman_ford(matriz_adj, vertice_fonte, vertice_destino, ponderado)
    
    if distancia is not None:
        caminho_rotulado = [rotulos[v] for v em caminho]
        print(f"\nDistância do vértice {rotulos[vertice_fonte]} ao vértice {rotulos[vertice_destino]}: {distancia}")
        print(f"Caminho de menor custo do vértice {rotulos[vertice_fonte]} ao vértice {rotulos[vertice_destino]}: {caminho_rotulado}")
    else:
        print("Não foi possível encontrar um caminho.")

if __name__ == "__main__":
    main_bellmanford()
