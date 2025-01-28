import pandas as pd

# Leer el archivo CSV
df = pd.read_csv('robot-base.csv', skiprows=6)  # Usamos skiprows para ignorar las primeras filas no relevantes

# Filtrar las columnas necesarias
df_filtered = df[['Time (Seconds)', 'qX', 'qY', 'qZ', 'qW', 'X', 'Y', 'Z']]

# Renombrar las columnas para mayor claridad (opcional)
df_filtered.columns = ['Time', 'qX', 'qY', 'qZ', 'qW', 'PosX', 'PosY', 'PosZ']

# Crear el archivo de salida y escribir el encabezado
with open('archivo_salida.txt', 'w') as f:
    # Escribir el encabezado
    f.write("# Time qX qY qZ qW PosX PosY PosZ\n")
    
    # Escribir los datos
    for index, row in df_filtered.iterrows():
        # Crear la línea de texto con el formato deseado
        linea = f"{row['Time']} {row['qX']} {row['qY']} {row['qZ']} {row['qW']} {row['PosX']} {row['PosY']} {row['PosZ']}\n"
        f.write(linea)
