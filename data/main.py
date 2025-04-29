import pandas as pd
import matplotlib.pyplot as plt

def generate_plot():
    try:
        with open('movements.txt', 'r') as file:
            lines = [line.strip() for line in file if line.strip()]
        
        if not lines:
            print("El archivo movements.txt está vacío.")
            return

        # Procesar movimientos del carrito
        movements = [line for line in lines if line in ['INICIO', 'ADELANTE', 'DERECHA', 'IZQUIERDA']]
        movement_counts = pd.Series(movements).value_counts()

        # Procesar datos de sensores
        sensor_data = [line for line in lines if line.startswith('TEMP:')]
        if sensor_data:
            sensor_df = pd.DataFrame([dict(zip(['TEMP', 'HUM', 'ACCX', 'ACCY', 'ACCZ', 'GYRX', 'GYRY', 'GYRZ'],
                                              [float(x.split(':')[1]) for x in line.split(',')]))
                                     for line in sensor_data])

        # Gráfico de movimientos
        plt.figure(figsize=(8, 6))
        movement_counts.plot(kind='bar', color=['#4CAF50', '#2196F3', '#F44336', '#FF9800'])
        plt.title('Frecuencia de Movimientos del Carrito')
        plt.xlabel('Movimiento')
        plt.ylabel('Frecuencia')
        plt.grid(True, axis='y')
        plt.savefig('movement_plot.png')
        plt.close()

        # Gráficos de sensores
        if sensor_data:
            # Temperatura y humedad
            plt.figure(figsize=(10, 6))
            plt.plot(sensor_df['TEMP'], label='Temperatura (°C)', color='#FF5722')
            plt.plot(sensor_df['HUM'], label='Humedad (%)', color='#2196F3')
            plt.title('Temperatura y Humedad a lo largo del Tiempo')
            plt.xlabel('Muestra')
            plt.ylabel('Valor')
            plt.legend()
            plt.grid(True)
            plt.savefig('temp_humidity_plot.png')
            plt.close()

            # Aceleración
            plt.figure(figsize=(10, 6))
            plt.plot(sensor_df['ACCX'], label='Aceleración X (m/s²)', color='#4CAF50')
            plt.plot(sensor_df['ACCY'], label='Aceleración Y (m/s²)', color='#2196F3')
            plt.plot(sensor_df['ACCZ'], label='Aceleración Z (m/s²)', color='#F44336')
            plt.title('Aceleración a lo largo del Tiempo')
            plt.xlabel('Muestra')
            plt.ylabel('Valor (m/s²)')
            plt.legend()
            plt.grid(True)
            plt.savefig('accel_plot.png')
            plt.close()

            # Giro
            plt.figure(figsize=(10, 6))
            plt.plot(sensor_df['GYRX'], label='Giro X (rad/s)', color='#4CAF50')
            plt.plot(sensor_df['GYRY'], label='Giro Y (rad/s)', color='#2196F3')
            plt.plot(sensor_df['GYRZ'], label='Giro Z (rad/s)', color='#F44336')
            plt.title('Giro a lo largo del Tiempo')
            plt.xlabel('Muestra')
            plt.ylabel('Valor (rad/s)')
            plt.legend()
            plt.grid(True)
            plt.savefig('gyro_plot.png')
            plt.close()

        print("Gráficos generados: movement_plot.png, temp_humidity_plot.png, accel_plot.png, gyro_plot.png")
    except Exception as e:
        print(f"Error al generar los gráficos: {e}")

if __name__ == "__main__":
    generate_plot()