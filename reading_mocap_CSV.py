import pandas as pd
from datetime import datetime
import csv

# Cargar el archivo 
file_name = 'short_dataset.csv'

# --- We extract header information ----
frame_rate = None
capture_start_time = None
total_exported_frames = None

# Open and read the CSV file
with open(file_name, 'r') as file:
    csv_reader = csv.reader(file)
    for row in csv_reader:
        # Extract data from the first row
        if row[0] == "Format Version":
            # Parse required values
            frame_rate = row[row.index("Capture Frame Rate") + 1]
            capture_start_time = row[row.index("Capture Start Time") + 1]
            total_exported_frames = row[row.index("Total Exported Frames") + 1]
            break

# Convert to a datetime object
time_obj = datetime.strptime(f"{capture_start_time}", "%Y-%m-%d %I.%M.%S.%f %p")

# Convert to Unix timestamp
zero_unix_time_sec = int(time_obj.timestamp())

# Print extracted values
print(f"Frame Rate: {frame_rate}")
print(f"Capture Start Time: {capture_start_time}")
print(f"Capture Start Time (Unix): {zero_unix_time_sec:.9f}")
print(f"Total Exported Frames: {total_exported_frames}")

data = pd.read_csv(file_name)

# Eliminar filas irrelevantes y establecer encabezados correctos
clean_data = data.iloc[4:].reset_index(drop=True)  # Tomar filas relevantes
clean_data.columns = data.iloc[4].values          # Usar la fila como encabezado
clean_data = clean_data.iloc[1:]                  # Omitir la fila redundante

# Seleccionar las columnas importantes
columns_of_interest = [
    "Time (Seconds)",       # Tiempo
    "qX", "qY", "qZ", "qW",     # Rotación (cuaternión)
    "X", "Y", "Z"  # Posición
]
filtered_data = clean_data[columns_of_interest].dropna()  # Eliminar NaN

# Renombrar columnas para facilitar el formato
filtered_data.columns = ["Time", "Quat_X", "Quat_Y", "Quat_Z", "Quat_W", "Pos_X", "Pos_Y", "Pos_Z"]

# Asegurarse de que Time sea numérico antes de sumar
filtered_data["Time"] = pd.to_numeric(filtered_data["Time"], errors="coerce")

# here we add the beginning time 
filtered_data["Time"] += zero_unix_time_sec

# Formatear la columna Time para que tenga 9 decimales
filtered_data["Time"] = filtered_data["Time"].apply(lambda x: f"{x:.9f}")


# Convertir a texto con encabezado
output_text = "#timestamp qx qy qz qw px py pz\n"
output_text += "\n".join(
    filtered_data.apply(lambda row: " ".join(map(str, row)), axis=1)
)

# Guardar como archivo de texto

name_output_txt = file_name.split('.')[0]  # Split by '.' and take the first part
output_file_name = name_output_txt + '_GT' + '.txt'

with open(output_file_name, "w") as file:
    file.write(output_text)

output_file_name


