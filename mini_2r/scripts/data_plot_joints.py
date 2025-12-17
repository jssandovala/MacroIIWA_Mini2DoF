import pandas as pd
import matplotlib.pyplot as plt

# Remplacer ce chemin par le chemin de ton fichier CSV
file_path = "/home/amir/ros2_ws/src/mini_2r/data/joints_Oct_29_2025_15_56_15.csv"

# Lecture du fichier CSV
df = pd.read_csv(file_path)

# Affichage des 5 premières lignes (optionnel)
print("Colonnes disponibles :", df.columns.tolist())
print(df.head())

time = df['time'].to_numpy()
q_0 = df['q_0'].to_numpy()
q_1= df['q_1'].to_numpy()
d__q0 = df['d_q0'].to_numpy()
d_q1 = df['d_q1'].to_numpy()

# Tracer pos_x et d_pos_x en fonction du temps
if 'time' in df.columns and 'q_0' in df.columns and 'd_q0' in df.columns:
    # Tracer pos_x et d_pos_x
    plt.figure(figsize=(10, 6))
    plt.plot( time,q_0 ,label='Position actuelle', color='blue')
    #plt.plot(time,d__q0, label='Position desire ', color='red')
    plt.title('Suivi de trajectoire')
    plt.xlabel('time (s)')
    plt.ylabel('joint 0 (rad)')
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.show()
else:
    print("Les colonnes 'time', 'pos_x' ou 'd_pos_x' ne sont pas présentes dans le CSV.")