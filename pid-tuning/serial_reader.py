import serial
import time

# --- Configuración ---
PORT = 'COM3' # En Windows. En Linux/Mac puede ser '/dev/ttyUSB0' o '/dev/tty.SLAB_USBtoUART'
BAUDRATE = 115200
FILENAME = 'log_robot_1.csv'
# ---------------------

print(f"Conectando a {PORT} a {BAUDRATE} baudios...")
print(f"Guardando datos en {FILENAME}")
print("Presiona Ctrl+C para detener la captura.")

try:
    # Abrimos el puerto serie
    with serial.Serial(PORT, BAUDRATE, timeout=1) as ser:
        ser.flushInput() # Limpiamos cualquier dato viejo
        
        # Abrimos el archivo donde guardaremos los datos
        with open(FILENAME, 'w', encoding='utf-8') as f:
            print("Conexión establecida. Capturando datos...")
            
            while True:
                try:
                    # Leemos una línea completa desde el ESP32
                    line = ser.readline().decode('utf-8').strip()
                    
                    if line:
                        print(line) # Mostramos en consola lo que recibimos
                        f.write(line + '\n') # Escribimos la línea en el archivo
                        
                except UnicodeDecodeError:
                    print("Error de decodificación. Omitiendo línea.")

except serial.SerialException as e:
    print(f"Error: No se pudo abrir el puerto {PORT}. ¿Está conectado?")
    print(e)
except KeyboardInterrupt:
    print("\nCaptura detenida por el usuario. Archivo guardado.")
except Exception as e:
    print(f"Ocurrió un error inesperado: {e}")