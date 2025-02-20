import pandas as pd
import argparse
import os

def check_data_integrity(csv_file):
    # Cargar el archivo CSV
    try:
        data = pd.read_csv(csv_file)
    except Exception as e:
        print(f"Error al leer el archivo CSV: {e}")
        return
    
    # Verificar si hay valores nulos
    null_values = data.isnull().sum()
    if null_values.any():
        print(f"Advertencia: El archivo contiene valores nulos en las siguientes columnas:\n{null_values}")
    else:
        print("No se encontraron valores nulos.")

    # Verificar el formato de las columnas
    print("Verificación de tipos de datos:")
    print(data.dtypes)

def main():
    parser = argparse.ArgumentParser(description='Verificar integridad de datos del CSV.')
    parser.add_argument('--input', type=str, required=True, help='Nombre de la carpeta del dataset')
    args = parser.parse_args()

    # Construir la ruta del archivo CSV
    dataset_folder = args.input
    csv_file = os.path.join('/home/samuel/dev/environment_modeling/ROSBAGS', dataset_folder, f'{dataset_folder}.csv')

    check_data_integrity(csv_file)

if __name__ == "__main__":
    main()
