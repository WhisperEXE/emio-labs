import pandas as pd
import matplotlib.pyplot as plt

# Load the benchmark data
df1 = pd.read_csv('benchmark_form1.csv')
df2 = pd.read_csv('benchmark_form2.csv')
df3 = pd.read_csv('benchmark_form3.csv')

# Create a figure with two stacked subplots
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(14, 10))

# --- Subplot 1: Tracking Error ---
# Added .to_numpy() to prevent the Multi-dimensional indexing ValueError
ax1.plot(df1['Step'].to_numpy(), df1['Error_Norm'].to_numpy(), label='Formulation I: Integrated (Native)', color='#3fb950', linewidth=2.5)
ax1.plot(df2['Step'].to_numpy(), df2['Error_Norm'].to_numpy(), label='Formulation II: Reduced QP', color='#79c0ff', linewidth=2, linestyle='--')
ax1.plot(df3['Step'].to_numpy(), df3['Error_Norm'].to_numpy(), label='Formulation III: OIM-Inspired', color='#d2a8ff', linewidth=2, linestyle='-.')

ax1.set_title('End-Effector Tracking Error Over Time', fontsize=16, fontweight='bold')
ax1.set_xlabel('Simulation Step', fontsize=14)
ax1.set_ylabel('Error Norm (mm)', fontsize=14)
ax1.legend(fontsize=12)
ax1.grid(True, linestyle=':', alpha=0.6)
ax1.set_xlim(0, 400)

# --- Subplot 2: Motor Angle (Chattering check) ---
# Added .to_numpy() here as well
ax2.plot(df1['Step'].to_numpy(), df1['Motor0_rad'].to_numpy(), label='Formulation I: Integrated (Native)', color='#3fb950', linewidth=2.5)
ax2.plot(df2['Step'].to_numpy(), df2['Motor0_rad'].to_numpy(), label='Formulation II: Reduced QP', color='#79c0ff', linewidth=1.5, alpha=0.9)
ax2.plot(df3['Step'].to_numpy(), df3['Motor0_rad'].to_numpy(), label='Formulation III: OIM-Inspired', color='#d2a8ff', linewidth=1.5, alpha=0.8)

ax2.set_title('Motor 0 Command Angle (Observing Oscillatory Dynamics)', fontsize=16, fontweight='bold')
ax2.set_xlabel('Simulation Step', fontsize=14)
ax2.set_ylabel('Motor 0 Angle (rad)', fontsize=14)
ax2.legend(fontsize=12)
ax2.grid(True, linestyle=':', alpha=0.6)
ax2.set_xlim(0, 400)

# Polish and save
plt.tight_layout()
plt.savefig('ik_benchmark_visualization.png', dpi=300, bbox_inches='tight')
print("Successfully generated ik_benchmark_visualization.png")