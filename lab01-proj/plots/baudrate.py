import csv
import matplotlib.pyplot as plt
import seaborn as sns

# === CONFIG ===
INPUT_CSV = "baudrate.csv"
OUTPUT_CSV = "efficiency_results_baudrate.csv"
PNG_OUT = "baudrate_dualaxis_final.png"
PDF_OUT = "baudrate_dualaxis_final.pdf"

sns.set_theme(style="white")
plt.rcParams.update({
    "font.size": 11,
    "axes.labelsize": 13,
    "axes.titlesize": 14,
    "legend.fontsize": 10,
})

# === READ DATA ===
records = []
with open(INPUT_CSV, "r", newline="") as f:
    reader = csv.DictReader(f)
    for row in reader:
        baud = float(row["baud_bps"])
        time_s = float(row["time_s"])
        file_size_bytes = float(row["file_size_bytes"])
        delivered = row["delivered"].strip().lower()

        payload_bits = file_size_bytes * 8.0
        goodput_bps = payload_bits / time_s if delivered == "yes" else 0.0
        efficiency = goodput_bps / baud if delivered == "yes" else 0.0

        records.append({
            "baud_bps": baud,
            "time_s": time_s,
            "payload_bits": payload_bits,
            "goodput_bps": goodput_bps,
            "efficiency_S": efficiency,
            "delivered": delivered
        })

# === WRITE INTERMEDIATE RESULTS CSV ===
with open(OUTPUT_CSV, "w", newline="") as f:
    writer = csv.DictWriter(f, fieldnames=records[0].keys())
    writer.writeheader()
    writer.writerows(records)

print(f"✅ Results saved to '{OUTPUT_CSV}'")

# === PREPARE PLOT DATA (only successful transfers) ===
bauds = [r["baud_bps"] for r in records if r["delivered"] == "yes"]
effs = [r["efficiency_S"] for r in records if r["delivered"] == "yes"]
times = [r["time_s"] for r in records if r["delivered"] == "yes"]

# === PLOT ===
fig, ax1 = plt.subplots(figsize=(7.5, 4.8))

color1 = "#00cc7a"   # efficiency color
color2 = "#c47ceb"   # time color

# Efficiency (left axis)
ax1.plot(bauds, effs, marker="o", linewidth=2, color=color1, label="Efficiency S = R/C")
ax1.set_xlabel("Link Capacity C [bit/s]")
ax1.set_ylabel("Efficiency S", color=color1)
ax1.tick_params(axis="y", labelcolor=color1)
ax1.set_ylim(0, 1.0)

# Transfer time (right axis)
ax2 = ax1.twinx()
ax2.plot(bauds, times, marker="s", linewidth=2, color=color2, label="Transfer Time (s)")
ax2.set_ylabel("Transfer Time (s)", color=color2)
ax2.tick_params(axis="y", labelcolor=color2)

# X axis formatting (log scale with your exact baud ticks)
ax1.set_xscale("log")
ax1.set_xticks(bauds)
ax1.get_xaxis().set_major_formatter(plt.ScalarFormatter())
ax1.tick_params(axis='x', rotation=20)

# --- Annotations (same style/offset for both curves) ---

# Efficiency above green markers
for x, y in zip(bauds, effs):
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

# Transfer time above purple markers
for x, y in zip(bauds, times):
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

# Title / legend / cleanup
plt.title("Efficiency and Transfer Time vs Link Capacity (Baud Rate)")
ax1.grid(False)
ax2.grid(False)

lines1, labels1 = ax1.get_legend_handles_labels()
lines2, labels2 = ax2.get_legend_handles_labels()
ax2.legend(lines1 + lines2, labels1 + labels2, loc="best")

plt.tight_layout()

# === EXPORTS ===
fig.savefig(PNG_OUT, dpi=300, bbox_inches="tight")
fig.savefig(PDF_OUT, bbox_inches="tight")

print(f"📊 Plot saved as '{PNG_OUT}' (PNG, 300 dpi)")
print(f"📄 Plot saved as '{PDF_OUT}' (PDF, vector)")

plt.show()
