import csv
import statistics
import matplotlib.pyplot as plt
import seaborn as sns

# === CONFIG ===
INPUT_CSV = "ber_raw.csv"
OUTPUT_CSV = "efficiency_results_ber.csv"
OUTPUT_PNG = "efficiency_results_ber.png"
OUTPUT_PDF = "efficiency_results_ber.pdf"

sns.set_theme(style="white")
plt.rcParams.update({
    "font.size": 11,
    "axes.labelsize": 13,
    "axes.titlesize": 14,
    "legend.fontsize": 10,
})

# --- Read raw data ---
raw = []
with open(INPUT_CSV, newline="") as f:
    reader = csv.DictReader(f)
    for row in reader:
        raw.append({
            "ber": float(row["ber"]),
            "baud": float(row["baud_bps"]),
            "time": float(row["time_s"]),
            "file_size_bytes": float(row["file_size_bytes"]),
            "delivered": row["delivered"].strip().lower()
        })

# --- Group by BER ---
grouped = {}
for r in raw:
    b = r["ber"]
    grouped.setdefault(b, {"times": [], "baud": r["baud"], "file_size_bytes": r["file_size_bytes"], "delivered_flags": []})
    grouped[b]["times"].append(r["time"])
    grouped[b]["delivered_flags"].append(r["delivered"])

# --- Compute averages ---
results = []
for b, vals in grouped.items():
    baud = vals["baud"]
    file_size = vals["file_size_bytes"]
    times = [t for t, d in zip(vals["times"], vals["delivered_flags"]) if d == "yes"]
    delivered = "yes" if times else "no"

    if times:
        avg_t = statistics.mean(times)
        std_t = statistics.stdev(times) if len(times) > 1 else 0.0
        payload_bits = file_size * 8
        R = payload_bits / avg_t
        S = R / baud
    else:
        avg_t, std_t, S = None, None, 0.0

    results.append({
        "ber": b,
        "baud_bps": baud,
        "avg_time_s": avg_t if avg_t else 0.0,
        "std_time_s": std_t if std_t else 0.0,
        "efficiency_S": S,
        "delivered": delivered
    })

# --- Write processed results ---
with open(OUTPUT_CSV, "w", newline="") as f:
    writer = csv.DictWriter(f, fieldnames=results[0].keys())
    writer.writeheader()
    writer.writerows(results)
print(f"✅ Results saved to '{OUTPUT_CSV}'")

# --- Separate successful vs failed ---
success = [r for r in results if r["delivered"] == "yes"]
failed = [r for r in results if r["delivered"] == "no"]

bers_s = [r["ber"] for r in success]
effs_s = [r["efficiency_S"] for r in success]
times_s = [r["avg_time_s"] for r in success]
stds_s = [r["std_time_s"] for r in success]

bers_f = [r["ber"] for r in failed]
effs_f = [r["efficiency_S"] for r in failed]

# --- Plot ---
fig, ax1 = plt.subplots(figsize=(7.5, 4.8))
color1 = "#00cc7a"
color2 = "#c47ceb"

# Efficiency (left)
ax1.plot(bers_s, effs_s, marker="o", linewidth=2, color=color1, label="Efficiency S = R/C")
ax1.scatter(bers_f, effs_f, color="red", marker="x", s=80, label="Transfer failed")
ax1.set_xlabel("Bit Error Rate (BER)")
ax1.set_ylabel("Efficiency S", color=color1)
ax1.tick_params(axis="y", labelcolor=color1)
ax1.set_ylim(-0.05, 1.0)  # 👈 lowered min slightly to show red ✗ markers

# Secondary axis: transfer time
ax2 = ax1.twinx()
ax2.plot(bers_s, times_s, marker="s", linewidth=2, color=color2, label="Transfer Time (s)")
ax2.errorbar(bers_s, times_s, yerr=stds_s, fmt="none", ecolor="black", capsize=4)
ax2.set_ylabel("Transfer Time (s)", color=color2)
ax2.tick_params(axis="y", labelcolor=color2)

# X-axis: linear + expanded limits
ax1.set_xticks(bers_s + bers_f)
ax1.set_xticklabels([f"{b:.5f}" for b in bers_s + bers_f], rotation=20)
ax1.set_xlim(-0.000005, 0.00009)

# Labels above each point
for x, y in zip(bers_s, effs_s):
    ax1.text(x, y + 0.025, f"{y:.2f}", ha="center", va="bottom", fontsize=9, color=color1, fontweight="bold")
for x, y in zip(bers_s, times_s):
    ax2.text(x, y + 0.025, f"{y:.1f}s", ha="center", va="bottom", fontsize=9, color=color2, fontweight="bold")

# --- Style ---
plt.title("Efficiency and Transfer Time vs Bit Error Rate (9600 bit/s)")
ax1.grid(False)
ax2.grid(False)

# Combine legends
lines, labels = ax1.get_legend_handles_labels()
lines2, labels2 = ax2.get_legend_handles_labels()
ax2.legend(lines + lines2, labels + labels2, loc="best")

plt.tight_layout()
plt.savefig(OUTPUT_PNG, dpi=300)
plt.savefig(OUTPUT_PDF)
plt.show()

print(f"✅ Plot saved as '{OUTPUT_PNG}' and '{OUTPUT_PDF}'")
