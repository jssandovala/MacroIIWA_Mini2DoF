import pandas as pd
import matplotlib.pyplot as plt

# Remplacer ce chemin par le chemin de ton fichier CSV
file_path = "/home/amir/ros2_ws/src/mini_2r/data/trajectory_data_2025-11-04_16-26-16.csv"

# Lecture du fichier CSV
df = pd.read_csv(file_path)

# Affichage des 5 premières lignes (optionnel)
print("Colonnes disponibles :", df.columns.tolist())
print(df.head())

time = df['Time'].to_numpy()
p_x = df['p_x'].to_numpy()
p_y = df['p_y'].to_numpy()
d_x = df['d_x'].to_numpy()
d_y = df['d_y'].to_numpy()

# Tracer pos_x et d_pos_x en fonction du temps
if 'Time' in df.columns and 'p_x' in df.columns and 'd_x' in df.columns:
    # Tracer pos_x et d_pos_x
    plt.figure(figsize=(10, 6))
    #plt.plot(time,p_x ,label='Position actuelle  X', color='blue')
    #plt.plot(time,d_x, label='Position desire X ', color='red', linestyle='--')

    #plt.plot(time,p_y ,label='Position actuelle Y ', color='green')
    #plt.plot(time,d_y, label='Position desire y ', color='orange', linestyle='--')

    plt.plot(p_y,p_x ,label='Position actuelle ', color='blue')
    plt.plot(d_y,d_x, label='Position desire ', color='red')


    plt.title('Suivi de trajectoire')
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.show()
else:
    print("Les colonnes 'time', 'pos_x' ou 'd_pos_x' ne sont pas présentes dans le CSV.")