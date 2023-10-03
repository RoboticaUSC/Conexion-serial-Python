import serial


class Serial_Connection:
    def Lectura_Serial(self):
        while True:
            Var1=self.arduino.readline().decode().strip()#lee la primera linea del serial y lo guarda en una variable
            #si se quiere leer mas lineas del serial seria descomentar la siguiente linea de codigo
            #self.Var2=self.arduino.readline().decode().strip()
            print(Var1)
            
    def __init__(self):
        
        self.parametro=[]
        self.arduino =serial.Serial('COM13', 9600)#Definimos el puerto COM al que se conecta y los baudios 
        self.Lectura_Serial()
            
serial=Serial_Connection()               
serial.Lectura_Serial()         
        
            
    
    
    
    
    
    