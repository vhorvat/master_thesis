import numpy as np
import matplotlib.pyplot as plt


num_channels = 5
amplitude = 1.0
sigma = 0.4
center_freq_start = 1.0

center_freq_spacing_fdm = 2.5 * sigma * 2

center_freq_spacing_ofdm = 1.0 * sigma * 2


freq_min = 0
freq_max_fdm = center_freq_start + (num_channels) * center_freq_spacing_fdm
freq_max_ofdm = center_freq_start + (num_channels - 1) * center_freq_spacing_ofdm + 3 * sigma
freq_max = max(freq_max_fdm, freq_max_ofdm) + 1
frequency = np.linspace(freq_min, freq_max, 800)


def channel_shape(freq, center_freq, sigma, amp):
  """Generates a Gaussian-like shape for a channel."""
  return amp * np.exp(-np.power(freq - center_freq, 2.) / (2 * np.power(sigma, 2.)))


line_thickness = 2.0
axis_line_thickness = 1.5
annot_line_thickness = 1.8
title_fontsize = 16
label_fontsize = 14
ch_label_fontsize = 11
annotation_fontsize = 12
figure_resolution_dpi = 300


fig, axes = plt.subplots(2, 1, figsize=(10, 7.5))
ax1, ax2 = axes


ax1.set_title("Tradicionalni FDM", y=-0.25, fontsize=title_fontsize)
ax1.set_ylabel("Snaga", fontsize=label_fontsize)

ax1.axhline(0, color='black', linewidth=axis_line_thickness)

centers_fdm = []
for i in range(num_channels):
  center_freq = center_freq_start + i * center_freq_spacing_fdm
  centers_fdm.append(center_freq)
  power = channel_shape(frequency, center_freq, sigma, amplitude)

  color = 'darkgrey' if i >= 2 else 'black'
  ax1.plot(frequency, power, color=color, linewidth=line_thickness)
  ax1.text(center_freq, amplitude * 1.05, f'Ch{i+1}', ha='center', va='bottom', fontsize=ch_label_fontsize)


ax2.set_title("OFDM", y=-0.35, fontsize=title_fontsize)
ax2.set_ylabel("Snaga", fontsize=label_fontsize)
ax2.set_xlabel("Frekvencija", fontsize=label_fontsize)
ax2.axhline(0, color='black', linewidth=axis_line_thickness)

centers_ofdm = []
for i in range(num_channels):
  center_freq = center_freq_start + i * center_freq_spacing_ofdm
  centers_ofdm.append(center_freq)
  power = channel_shape(frequency, center_freq, sigma, amplitude)

  color = 'darkgrey' if i >= 2 else 'black'
  ax2.plot(frequency, power, color=color, linewidth=line_thickness)
  ax2.text(center_freq, amplitude * 1.05, f'Ch{i+1}', ha='center', va='bottom', fontsize=ch_label_fontsize)



ofdm_bw_end = centers_ofdm[-1] + 2 * sigma
fdm_bw_end = centers_fdm[-1] + 2 * sigma


y_pos = amplitude * 0.6
y_arrow = amplitude * 0.5
y_text = amplitude * 0.55


ax2.vlines([ofdm_bw_end, fdm_bw_end], 0, y_pos, color='black', linestyles='solid', linewidth=annot_line_thickness)


ax2.annotate('', xy=(fdm_bw_end, y_arrow), xytext=(ofdm_bw_end, y_arrow),
             arrowprops=dict(arrowstyle='<->', color='black', lw=annot_line_thickness))


ax2.text((ofdm_bw_end + fdm_bw_end) / 2, y_text, 'Ušteda frekvencijskog pojasa',
         ha='center', va='bottom', fontsize=annotation_fontsize)



for ax in axes:
    ax.set_yticks([])

    ax.spines['top'].set_visible(False)
    ax.spines['right'].set_visible(False)
    ax.spines['left'].set_visible(False)
    ax.set_ylim(-0.1, amplitude * 1.3)
    ax.set_xlim(freq_min, freq_max)

ax1.set_xticklabels([])
ax1.tick_params(axis='x', which='both', bottom=False)


plt.tight_layout(rect=[0, 0.05, 1, 0.98])


plt.savefig("fdm_vs_ofdm_comparison.png", dpi=figure_resolution_dpi, bbox_inches='tight')


plt.show()