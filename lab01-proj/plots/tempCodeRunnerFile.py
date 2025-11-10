import csv
import matplotlib.pyplot as plt
import seaborn as sns

INPUT_CSV = "tprop_raw.csv"
OUTPUT_CSV = "efficiency_results_tprop.csv"
OUTPUT_PNG = "efficiency_results_tprop.png"
OUTPUT_PDF = "efficiency_results_tprop.pdf"

sns.set_theme(style="white")
plt.rcParams.update({
    "font.size": 11,
    "axes.labelsize": 13,
    "axes.titlesize": 14,
    "legend.fontsize": 10,
})

# --- Read raw data ---
rows = []
with open(INPUT_CSV, newline="") as f:
    reader = csv.DictReader(f)
    for r in reader:
        t_prop_us = float(r["t_prop_us"])              # requested propagation delay (us)
        t_prop_actual_us = float(r["t_prop_actual_us"])# actual propagation delay set (us)
        baud = float(r["baud_bps"])
        time_s = float(r["time_s"])
        file_size = float(r["file_size_bytes"])
        delivered = r["delivered"].strip().lower()

        payload_bits = file_size * 8.0                  # application payload bits sent once
        R_bps = payload_bits / time_s                   # goodput in bit/s
        S = R_bps / baud                                # efficiency

        rows.append({
            "t_prop_us": t_prop_us,
            "t_prop_actual_us": t_prop_actual_us,
            "baud_bps": baud,
            "time_s": time_s,
            "payload_bits": payload_bits,
            "goodput_bps": R_bps,
            "efficiency_S": S,
            "delivered": delivered,
        })

# --- Write processed CSV for report appendix ---
with open(OUTPUT_CSV, "w", newline="") as f:
    writer = csv.DictWriter(f, fieldnames=rows[0].keys())
    writer.writeheader()
    writer.writerows(rows)
print(f"✅ Results saved to '{OUTPUT_CSV}'")

# --- Prep arrays for plotting ---
# We'll use the *requested* propagation delay for x (t_prop_us),
# but we’ll also annotate the exact time at each point like the other plots.
tprop_ms = [r["t_prop_us"] / 1000.0 for r in rows]   # convert us -> ms for x-axis readability
effs     = [r["efficiency_S"] for r in rows]
times    = [r["time_s"] for r in rows]

# --- Plot ---
fig, ax1 = plt.subplots(figsize=(7.5, 4.8))

color1 = "#00cc7a"
color2 = "#c47ceb"

# Efficiency curve (left axis)
ax1.plot(tprop_ms, effs, marker="o", linewidth=2, color=color1, label="Efficiency S = R/C")
ax1.set_xlabel("One-way propagation delay (ms)")
ax1.set_ylabel("Efficiency S", color=color1)
ax1.tick_params(axis="y", labelcolor=color1)
ax1.set_ylim(0, 1.0)

# Time curve (right axis)
ax2 = ax1.twinx()
ax2.plot(tprop_ms, times, marker="s", linewidth=2, color=color2, label="Transfer Time (s)")
ax2.set_ylabel("Transfer Time (s)", color=color2)
ax2.tick_params(axis="y", labelcolor=color2)

# --- Annotate each point like the other polished plots ---

# Above efficiency point: numeric S
for x, y in zip(tprop_ms, effs):
    ax1.text(
        x,
        y + 0.025,
        f"{y:.2f}",
        ha="center",
        va="bottom",
        fontsize=9,
        color=color1,
        fontweight="bold",
    )

# Above time point: "Xs"
for x, y in zip(tprop_ms, times):
    ax2.text(
        x,
        y + 0.025,
        f"{y:.1f}s",
        ha="center",
        va="bottom",
        fontsize=9,
        color=color2,
        fontweight="bold",
    )

# --- Style tweaks to match the previous figures ---
plt.title("Efficiency and Transfer Time vs Propagation Delay (C = 9600 bit/s)")
ax1.grid(False)
ax2.grid(False)

# Legend combining both axes
lines1, labels1 = ax1.get_legend_handles_labels()
lines2, labels2 = ax2.get_legend_handles_labels()
ax2.legend(lines1 + lines2, labels1 + labels2, loc="best")

plt.tight_layout()
plt.savefig(OUTPUT_PNG, dpi=300)
plt.savefig(OUTPUT_PDF)
plt.show()

print(f"✅ Plot saved as '{OUTPUT_PNG}' and '{OUTPUT_PDF}'")
