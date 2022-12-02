import numpy as np

class RosaNautica:
    def __init__(self):
        #Primer posicion alineado con x positiva
        self.obstaculos = [[1, 1, 1, 1, 1, 1, 1], 
                                [0, 0, 0, 0, 0, 0, 1], 
                                [1, 0, 1, 1, 1, 0, 0], 
                                [1, 0, 1, 1, 1, 0, 0], 
                                [1, 0, 1, 1, 1, 0, 0], 
                                [1, 0, 1, 1, 1, 0, 1],
                                [1, 0, 1, 1, 1, 0, 1], 
                                [1, 0, 0, 0, 0, 0, 0]]


    def trayectoria_movimiento(self, x0, y0, xf,yf):
        #Transformar matriz
        obstaculos_lista = []

        for x in range(len(self.obstaculos)):
            for y in range(len(self.obstaculos[0])): 
                if self.obstaculos[x][y]==1:
                    obstaculos_lista.append([x,y])

        #Crear campos potenciales
        mundo_potencial = np.array([[0 for i in range(7)] for i in range(8)])

        #For que agrega la capa atractiva
        capa_atractiva = np.array([[0 for i in range(7)] for i in range(8)])
        for x in range(len(mundo_potencial)): 
            for y in range(len(mundo_potencial[0])):
                capa_atractiva[x][y] = 8*np.linalg.norm(np.array([x, y])-np.array([xf, yf]))
        mundo_potencial = mundo_potencial + capa_atractiva
        #print(capa_atractiva)

        #For que agrega la capa repulsiva
        for xi, yi in obstaculos_lista:
            capa_repulsiva = np.array([[0 for i in range(7)] for i in range(8)])
            for x in range(len(mundo_potencial)): 
                for y in range(len(mundo_potencial[0])):
                    if np.linalg.norm(np.array([x, y])-np.array([xi, yi])) == 0: 
                        capa_repulsiva[x][y] = 100
                    else:
                        capa_repulsiva[x][y] = 1/np.linalg.norm(np.array([x, y])-np.array([xi, yi]))
            mundo_potencial = mundo_potencial + capa_repulsiva
            #print(capa_repulsiva)


        i = 0
        q = [[x0, y0]]
        mundo_potencial = mundo_potencial-np.ndarray.min(mundo_potencial)
        print(mundo_potencial)
        minx=x0
        miny=y0
        xi=q[i][0]
        yi=q[i][1]
        pasos=[]
        while not (xi == xf and yi == yf): 
            minimo=mundo_potencial[xi][yi]
            brujula=''
            #Izquierda
            if yi-1>=0: 
                if mundo_potencial[xi][yi-1] < minimo:  
                    minimo=mundo_potencial[xi][yi-1] 
                    miny=yi-1 
                    brujula='I'
            #Derecha 
            if yi+1<len(mundo_potencial[0]): 
                if mundo_potencial[xi][yi+1] < minimo:  
                    minimo=mundo_potencial[xi][yi+1] 
                    miny=yi+1 
                    brujula='D'
            #Norte
            if xi-1>=0: 
                if mundo_potencial[xi-1][yi] < minimo:  
                    minimo=mundo_potencial[xi-1][yi]  
                    minx=xi-1 
                    brujula='N'
            #Sur
            if xi+1<len(mundo_potencial): 
                if mundo_potencial[xi+1][yi] < minimo:  
                    minimo=mundo_potencial[xi+1][yi] 
                    minx=xi+1 
                    brujula='S'
            q.append([minx, miny])
            pasos.append(brujula)
            i=i+1
            xi=q[i][0]
            yi=q[i][1]
        return pasos
