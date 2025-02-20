import pandas as pd
import numpy as np
from scipy.spatial.transform import Slerp, Rotation

def read_csv(filepath):
    # Leer el CSV omitiendo las primeras 6 líneas
    df = pd.read_csv(filepath, skiprows=6)
    return df

def detect_errors(df, error_frames):
    """
    Devuelve las filas con errores especificadas por el usuario basándose en la columna 'Frame'.
    """
    return df[df['Frame'].isin(error_frames)]

def interpolate_quaternions(df, error_frames):
    """
    Corrige los cuaterniones en las filas especificadas usando interpolación SLERP.
    """
    valid_df = df.dropna(subset=['qX', 'qY', 'qZ', 'qW'])
    valid_frames = valid_df['Frame'].values
    
    if len(valid_frames) < 2:
        raise ValueError("No hay suficientes valores válidos para interpolar.")
    
    # Convertir cuaterniones válidos a Rotations
    rotations = Rotation.from_quat(valid_df[['qX', 'qY', 'qZ', 'qW']].values)
    slerp = Slerp(valid_frames, rotations)
    
    # Interpolar valores
    interpolated_frames = []
    for frame in error_frames:
        if frame in valid_frames:
            continue  # Si el frame ya es válido, no es necesario interpolar
        
        interp_rotation = slerp([frame])  # Interpolar el cuaternión
        quat_values = interp_rotation.as_quat().flatten()
        
        # Asignar los valores interpolados a la fila correspondiente
        df.loc[df['Frame'] == frame, ['qX', 'qY', 'qZ', 'qW']] = quat_values
        interpolated_frames.append(df[df['Frame'] == frame])
    
    return pd.concat(interpolated_frames) if interpolated_frames else pd.DataFrame()

def process_csv(filepath, error_frames, modify_position):
    df = read_csv(filepath)
    errors = detect_errors(df, error_frames)
    
    if errors.empty:
        print("No se encontraron errores en las filas especificadas.")
        return
    
    corrected_data = interpolate_quaternions(df, error_frames)
    
    if modify_position:
        corrected_data[['X', 'Y', 'Z']] = corrected_data[['X', 'Y', 'Z']].interpolate()
    
    print(corrected_data)  # Mostrar los datos corregidos
    return corrected_data.to_csv(index=False)

# Ejemplo de uso
error_frames = list(range(1344, 1366))
process_csv('2-cold-tt-data.csv', error_frames, modify_position=False)
print("datos corregidos")
