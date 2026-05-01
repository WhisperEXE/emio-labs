import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

# ---------------------------------------------------------
# 1. READ THE CSV FILES
# ---------------------------------------------------------
df_beam = pd.read_csv('beam_log.csv')
df_cos = pd.read_csv('cosserat_log.csv')
df_real = pd.read_csv('1d_log.csv') 

if 'Interval' not in df_real.columns:
    df_real['Interval'] = range(len(df_real))

# ---------------------------------------------------------
# 2. VISUALIZATION (Updated Settings)
# ---------------------------------------------------------
GENERAL_FONT_SIZE = 18

# Reduced height to 7 to compress vertical space
fig = plt.figure(figsize=(20, 7)) 
fig.suptitle('3D Trajectory Comparison (Beam vs Cosserat vs Real-World)', fontsize=22)

groups = [
    (1, 'GM_y', 'GM_z', 'GM'),
    (2, 'M1_y', 'M1_z', 'M1'),
    (3, 'M2_y', 'M2_z', 'M2')
]

color_beam = '#1f77b4' # Blue
color_cos = '#ff7f0e'  # Orange
color_real = '#2ca02c' # Green

for pos, y_col, z_col, label in groups:
    ax = fig.add_subplot(1, 3, pos, projection='3d')
    
    # Ground Truth
    ax.plot(df_real['Interval'].to_numpy(), df_real[y_col].to_numpy(), df_real[z_col].to_numpy(), 
            label='Real-World', color=color_real, linestyle='-', linewidth=2.5, marker='s', markersize=5, alpha=0.9)

    # Beam Model
    ax.plot(df_beam['Interval'].to_numpy(), df_beam[y_col].to_numpy(), df_beam[z_col].to_numpy(), 
            label='Beam', color=color_beam, linestyle=':', linewidth=2, marker='o', markersize=4, alpha=0.7)
    
    # Cosserat Model
    ax.plot(df_cos['Interval'].to_numpy(), df_cos[y_col].to_numpy(), df_cos[z_col].to_numpy(), 
            label='Cosserat', color=color_cos, linestyle=':', linewidth=2, marker='x', markersize=4, alpha=0.9)

    # Labels and Ticks
    ax.set_xlabel('Interval (Frame)', fontsize=GENERAL_FONT_SIZE, labelpad=10)
    ax.set_ylabel('Y Coordinate (mm)', fontsize=GENERAL_FONT_SIZE, labelpad=10)
    ax.set_zlabel('Z Coordinate (mm)', fontsize=GENERAL_FONT_SIZE, labelpad=10)
    ax.tick_params(axis='both', which='major', labelsize=GENERAL_FONT_SIZE - 2)
    
    # Move title to bottom and reduce vertical gap
    ax.set_title(f'{label} Trajectory', fontsize=GENERAL_FONT_SIZE + 4, y=-0.18)
    ax.legend(fontsize=GENERAL_FONT_SIZE - 2)

# Manual Spacing Adjustment (No tight_layout)
plt.subplots_adjust(
    left=0.05,    
    right=0.95,   
    top=0.92,     # High top to reduce gap with main title
    bottom=0.15,  # Room for bottom titles
    wspace=0.05   # Tight horizontal gap
)

plt.savefig('beam_visualization.png', dpi=300, bbox_inches='tight')
print("✅ Visualization saved to 'beam_visualization.png'")

# ---------------------------------------------------------
# 3. ERROR CALCULATION (MAE & MSE)
# ---------------------------------------------------------
# (The rest of your error calculation remains exactly the same)
print("\n--- ERROR METRICS (Compared to Real-World Ground Truth) ---")

min_frames = min(len(df_beam), len(df_cos), len(df_real))
eval_cols = ['GM_y', 'GM_z', 'M1_y', 'M1_z', 'M2_y', 'M2_z']

real_data = df_real[eval_cols].iloc[:min_frames]
beam_data = df_beam[eval_cols].iloc[:min_frames]
cos_data  = df_cos[eval_cols].iloc[:min_frames]

beam_diff = real_data - beam_data
cos_diff  = real_data - cos_data

print("OVERALL MODEL PERFORMANCE (Averaged across all points M1, M2, GM):")
print(f"Beam Model     -> MAE: {beam_diff.abs().mean().mean():.3f} | MSE: {(beam_diff**2).mean().mean():.3f}")
print(f"Cosserat Model -> MAE: {cos_diff.abs().mean().mean():.3f} | MSE: {(cos_diff**2).mean().mean():.3f}")

print("\nDETAILED BREAKDOWN (MAE per coordinate):")
for col in eval_cols:
    beam_mae_col = beam_diff[col].abs().mean()
    cos_mae_col  = cos_diff[col].abs().mean()
    winner = "Beam" if beam_mae_col < cos_mae_col else "Cosserat"
    print(f"{col:4s} - Beam Error: {beam_mae_col:6.2f} | Cosserat Error: {cos_mae_col:6.2f}  (More accurate: {winner})")