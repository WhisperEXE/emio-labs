import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

# ---------------------------------------------------------
# 1. READ THE CSV FILES
# ---------------------------------------------------------
df_hyper = pd.read_csv('hyper_log.csv')
df_nonuniform = pd.read_csv('nonuniform_log.csv')
df_tetra_linear = pd.read_csv('tetra_linear_log.csv')
df_tetra = pd.read_csv('tetra_log.csv')

# Load the ground truth data
df_real = pd.read_csv('3d_log.csv')

# Ensure the synthetic log has an 'Interval' column for the X-axis of the plot
if 'Interval' not in df_real.columns:
    df_real['Interval'] = range(len(df_real))

# ---------------------------------------------------------
# 2. VISUALIZATION (Corrected and Complete)
# ---------------------------------------------------------
GENERAL_FONT_SIZE = 18

fig = plt.figure(figsize=(20, 8)) 
fig.suptitle('3D Trajectory Comparison (4 Models vs Real-World)', fontsize=22)

# --- RE-ADDED DEFINITION OF GROUPS ---
groups = [
    (1, 'GM_y', 'GM_z', 'GM'),
    (2, 'M1_y', 'M1_z', 'M1'),
    (3, 'M2_y', 'M2_z', 'M2')
]

# (Ensure models is also defined)
models = [
    ('Hyper', df_hyper, '#1f77b4', 'o'),               
    ('Nonuniform', df_nonuniform, '#ff7f0e', 'x'),     
    ('Tetra Linear', df_tetra_linear, '#2ca02c', '^'), 
    ('Tetra', df_tetra, '#d62728', 's')                
]

for pos, y_col, z_col, label in groups:
    ax = fig.add_subplot(1, 3, pos, projection='3d')
    
    # Plot Ground Truth
    ax.plot(df_real['Interval'].to_numpy(), df_real[y_col].to_numpy(), df_real[z_col].to_numpy(), 
            label='Real-World', color='black', linestyle='-', linewidth=2.5, marker='*', markersize=6, alpha=0.9)

    # Plot Models
    for model_name, df, color, marker in models:
        ax.plot(df['Interval'].to_numpy(), df[y_col].to_numpy(), df[z_col].to_numpy(), 
                label=model_name, color=color, linestyle=':', linewidth=2, marker=marker, markersize=4, alpha=0.7)

    ax.set_xlabel('Interval (Frame)', fontsize=GENERAL_FONT_SIZE, labelpad=10)
    ax.set_ylabel('Y Coordinate (mm)', fontsize=GENERAL_FONT_SIZE, labelpad=10)
    ax.set_zlabel('Z Coordinate (mm)', fontsize=GENERAL_FONT_SIZE, labelpad=10)
    
    # Bottom title
    ax.set_title(f'{label} Trajectory', fontsize=GENERAL_FONT_SIZE + 4, y=-0.18)
    ax.legend(fontsize=GENERAL_FONT_SIZE - 2)
    ax.tick_params(axis='both', which='major', labelsize=GENERAL_FONT_SIZE - 2)

# Manual adjustment for tight spacing
plt.subplots_adjust(
    left=0.01,    
    right=0.99,   
    bottom=0.15,   
    top=0.92,     
    wspace=0.05   # Reduced gap
)

plt.savefig('volume_visualization.png', dpi=300, bbox_inches='tight')
print("✅ Visualization saved successfully.")

# ---------------------------------------------------------
# 3. ERROR CALCULATION (MAE & MSE)
# ---------------------------------------------------------
print("\n--- ERROR METRICS (Compared to Real-World Ground Truth) ---")

# Ensure we only compare overlapping frames to prevent index errors
min_frames = min(len(df_hyper), len(df_nonuniform), len(df_tetra_linear), len(df_tetra), len(df_real))

eval_cols = ['GM_y', 'GM_z', 'M1_y', 'M1_z', 'M2_y', 'M2_z']

# Slice DataFrames to the minimum length
real_data = df_real[eval_cols].iloc[:min_frames]

model_dfs = {
    'Hyper': df_hyper[eval_cols].iloc[:min_frames],
    'Nonuniform': df_nonuniform[eval_cols].iloc[:min_frames],
    'Tetra Linear': df_tetra_linear[eval_cols].iloc[:min_frames],
    'Tetra': df_tetra[eval_cols].iloc[:min_frames]
}

# --- Overall Model Averages ---
print("OVERALL MODEL PERFORMANCE (Averaged across all points M1, M2, GM):")
model_errors = {}
for name, m_df in model_dfs.items():
    diff = real_data - m_df
    mae = diff.abs().mean().mean()
    mse = (diff**2).mean().mean()
    
    # Store differences to use in the detailed breakdown below
    model_errors[name] = {'diff': diff, 'mae': mae, 'mse': mse}
    print(f"{name:15s} -> MAE: {mae:6.3f} | MSE: {mse:8.3f}")

# --- Detailed Breakdown Per Point ---
print("\nDETAILED BREAKDOWN (MAE per coordinate):")
for col in eval_cols:
    print(f"\n--- {col} ---")
    min_mae = float('inf')
    best_model = None
    
    # Print the error for each model on this specific coordinate
    for name, err_dict in model_errors.items():
        col_mae = err_dict['diff'][col].abs().mean()
        print(f"  {name:15s}: {col_mae:6.2f}")
        
        # Track the winner
        if col_mae < min_mae:
            min_mae = col_mae
            best_model = name
            
    print(f"  🏆 Best Model for {col}: {best_model}")