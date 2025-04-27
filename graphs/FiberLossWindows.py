import numpy as np
import matplotlib.pyplot as plt


wavelength_um = np.linspace(0.7, 1.8, 500)


A_rayleigh = 0.78
rayleigh_loss = A_rayleigh / (wavelength_um**4)


ir_start_wl = 1.62
ir_factor = 0.10
ir_steepness = 13
ir_loss_refined = ir_factor * np.exp(ir_steepness * (wavelength_um - ir_start_wl)) * (wavelength_um > ir_start_wl)


oh_centers = [0.95, 1.24, 1.383]
oh_heights = [0.25, 0.45, 2.2]
oh_widths = [0.015, 0.020, 0.028]

oh_loss = np.zeros_like(wavelength_um)
for center, height, width in zip(oh_centers, oh_heights, oh_widths):
    oh_loss += height * np.exp(-0.5 * ((wavelength_um - center) / width)**2)


base_loss = 0.01


total_attenuation = rayleigh_loss + ir_loss_refined + oh_loss + base_loss
min_theoretical_loss = 0.16
total_attenuation = np.maximum(min_theoretical_loss, total_attenuation)


plt.style.use('seaborn-v0_8-whitegrid')
fig, ax = plt.subplots(figsize=(8, 5))


ax.plot(wavelength_um, total_attenuation, color='black', linewidth=1.5, label='Atenuacija')


mask1 = (wavelength_um > 0.8) & (wavelength_um < 0.9)
min_loss_850_idx_local = np.argmin(total_attenuation[mask1])
min_loss_850_idx_global = np.where(mask1)[0][min_loss_850_idx_local]
min_loss_850_wl = wavelength_um[min_loss_850_idx_global]

mask2 = (wavelength_um > 1.27) & (wavelength_um < 1.35)
min_loss_1310_idx_local = np.argmin(total_attenuation[mask2])
min_loss_1310_idx_global = np.where(mask2)[0][min_loss_1310_idx_local]
min_loss_1310_wl = wavelength_um[min_loss_1310_idx_global]

mask3 = (wavelength_um > 1.50) & (wavelength_um < 1.60)
min_loss_1550_idx_local = np.argmin(total_attenuation[mask3])
min_loss_1550_idx_global = np.where(mask3)[0][min_loss_1550_idx_local]
min_loss_1550_wl = wavelength_um[min_loss_1550_idx_global]

win1_bounds = (min_loss_850_wl - 0.03, min_loss_850_wl + 0.03)
win2_bounds = (1.28, 1.34)
win3_bounds = (1.525, 1.570)


window_color = '0.85'
ax.axvspan(win1_bounds[0], win1_bounds[1], facecolor=window_color, alpha=1.0, zorder=0)
ax.axvspan(win2_bounds[0], win2_bounds[1], facecolor=window_color, alpha=1.0, zorder=0)
ax.axvspan(win3_bounds[0], win3_bounds[1], facecolor=window_color, alpha=1.0, zorder=0)


label_y_pos = max(4.1, np.max(total_attenuation) * 0.9)
label_rot = 90


ax.text((win1_bounds[0] + win1_bounds[1]) / 2, label_y_pos, '850-nm prozor',
        rotation=label_rot, va='top', ha='center', fontsize=10)
ax.text((win2_bounds[0] + win2_bounds[1]) / 2, label_y_pos, '1.3-µm prozor\n(O-Band)',
        rotation=label_rot, va='top', ha='center', fontsize=10)
ax.text((win3_bounds[0] + win3_bounds[1]) / 2, label_y_pos, '1.55-µm prozor\n(C-Band)',
        rotation=label_rot, va='top', ha='center', fontsize=10)


peak_1383_idx = np.argmin(np.abs(wavelength_um - oh_centers[2]))
peak_1383_wl = wavelength_um[peak_1383_idx]
peak_1383_val = total_attenuation[peak_1383_idx]

peak_1240_idx = np.argmin(np.abs(wavelength_um - oh_centers[1]))
peak_1240_wl = wavelength_um[peak_1240_idx]
peak_1240_val = total_attenuation[peak_1240_idx]

peak_0950_idx = np.argmin(np.abs(wavelength_um - oh_centers[0]))
peak_0950_wl = wavelength_um[peak_0950_idx]
peak_0950_val = total_attenuation[peak_0950_idx]


rayleigh_label_wl = 0.78
rayleigh_label_idx = np.argmin(np.abs(wavelength_um - rayleigh_label_wl))

rayleigh_text_y = total_attenuation[rayleigh_label_idx] + 0.5
ax.text(rayleigh_label_wl + 0.08 , rayleigh_text_y, 'Rayleighovo\n raspršenje',
        ha='right', va='bottom', fontsize=10)


ir_label_wl = 1.70
ir_label_idx = np.argmin(np.abs(wavelength_um - ir_label_wl))

ir_text_y = total_attenuation[ir_label_idx] + 0.4
ax.text(ir_label_wl - 0.08, ir_text_y, 'Molekularna\napsorpcija',
        ha='center', va='top', fontsize=10) #mozda makni relativni pomak


oh_label_x = 1.05
oh_label_y = peak_1383_val * 0.85 + 0.2
line_y_at_oh_label_x = np.interp(oh_label_x, wavelength_um, total_attenuation)
oh_label_y = max(oh_label_y, line_y_at_oh_label_x + 0.2)

ax.text(oh_label_x, oh_label_y, 'OH- apsorpcija', ha='right', va='bottom', fontsize=10)

arrow_props = dict(arrowstyle='->', color='black', shrinkB=5, connectionstyle="arc3,rad=-0.1")

ax.annotate('', xy=(peak_1383_wl, peak_1383_val), xytext=(oh_label_x, oh_label_y), textcoords='data', arrowprops=arrow_props)
arrow_target_1240 = max(peak_1240_val, total_attenuation[min_loss_1310_idx_global] + 0.1)
ax.annotate('', xy=(peak_1240_wl, arrow_target_1240), xytext=(oh_label_x, oh_label_y), textcoords='data', arrowprops=arrow_props)
arrow_target_0950 = max(peak_0950_val, total_attenuation[min_loss_850_idx_global] + 0.1)
ax.annotate('', xy=(peak_0950_wl, arrow_target_0950), xytext=(oh_label_x, oh_label_y), textcoords='data', arrowprops=arrow_props)


ax.set_xlabel("Valna duljina, λ (µm)", fontsize=11)
ax.set_ylabel("Atenuacija, α (dB·km⁻¹)", fontsize=11)
ax.set_title("Atenuacija single-mode optičkog vlakna", fontsize=12)

ax.set_xlim(0.7, 1.8)
ax.set_ylim(0, max(4.1, np.max(total_attenuation)*1.05))
ax.set_xticks(np.arange(0.7, 1.9, 0.1))
ax.set_yticks(np.arange(0, np.ceil(ax.get_ylim()[1]), 1))

ax.grid(False)
ax.spines['top'].set_visible(False)
ax.spines['right'].set_visible(False)
ax.spines['left'].set_linewidth(1.2)
ax.spines['left'].set_color('black')
ax.spines['bottom'].set_linewidth(1.2)
ax.spines['bottom'].set_color('black')

ax.tick_params(axis='both', which='major', labelsize=10, direction='out', length=6, width=1, colors='black')

plt.tight_layout(rect=[0, 0, 1, 0.96])


plt.savefig('atenuacija_vlakna_300dpi.png', dpi=300)

plt.show()


print(f"Gubitci u optičkom vlaknu:")
print(f"  At {min_loss_850_wl:.3f} µm (1st window min): {total_attenuation[min_loss_850_idx_global]:.2f} dB/km")
print(f"  At {min_loss_1310_wl:.3f} µm (2nd window min): {total_attenuation[min_loss_1310_idx_global]:.2f} dB/km")
print(f"  At {peak_1383_wl:.3f} µm (Water Peak): {peak_1383_val:.2f} dB/km")
print(f"  At {min_loss_1550_wl:.3f} µm (3rd window min): {total_attenuation[min_loss_1550_idx_global]:.2f} dB/km")