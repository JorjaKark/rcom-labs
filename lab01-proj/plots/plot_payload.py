import csv
import statistics
import matplotlib.pyplot as plt
import seaborn as sns

# === CONFIG ===
INPUT_CSV = "payload_raw.csv"
OUTPUT_CSV = "efficiency_results_payload.csv"
OUTPUT_PNG = "efficiency_results_payload.png"
OUTPUT_PDF = "efficiency_results_payload.pdf"

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
            "payload": int(row["payload_bytes"]),
            "baud": float(row["baud_bps"]),
            "time": float(row["time_s"]),
            "file_size_bytes": float(row["file_size_bytes"]),
            "delivered": row["delivered"].strip().lower()
        })

# --- Group (in case of repeats, though you have 1 per payload) ---
grouped = {}
for r in raw:
    p = r["payload"]
    grouped.setdefault(p, {"times": [], "baud": r["baud"], "file_size_bytes": r["file_size_bytes"], "delivered_flags": []})
    grouped[p]["times"].append(r["time"])
    grouped[p]["delivered_flags"].append(r["delivered"])

# --- Compute averages ---
results = []
for p, vals in grouped.items():
    baud = vals["baud"]
    file_size = vals["file_size_bytes"]
    times = [t for t, d in zip(vals["times"], vals["delivered_flags"]) if d == "yes"]
    delivered = "yes" if times else "no"

    if times:
        avg_t = statistics.mean(times)
        payload_bits = file_size * 8
        R = payload_bits / avg_t
        S = R / baud
    else:
        avg_t, S = None, 0.0

    results.append({
        "payload_bytes": p,
        "baud_bps": baud,
        "avg_time_s": avg_t if avg_t else 0.0,
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

p_s = [r["payload_bytes"] for r in success]
effs_s = [r["efficiency_S"] for r in success]
times_s = [r["avg_time_s"] for r in success]

# --- Plot ---
fig, ax1 = plt.subplots(figsize=(7.5, 4.8))
color1 = "#00cc7a"
color2 = "#c47ceb"

# Efficiency (left axis)
ax1.plot(p_s, effs_s, marker="o", linewidth=2, color=color1, label="Efficiency S = R/C")
ax1.set_xlabel("Maximum Payload Size (bytes)")
ax1.set_ylabel("Efficiency S", color=color1)
ax1.tick_params(axis="y", labelcolor=color1)
ax1.set_ylim(0.0, 1.0)

# Transfer time (right axis)
ax2 = ax1.twinx()
ax2.plot(p_s, times_s, marker="s", linewidth=2, color=color2, label="Transfer Time (s)")
ax2.set_ylabel("Transfer Time (s)", color=color2)
ax2.tick_params(axis="y", labelcolor=color2)

# X-axis: logarithmic scale for readability
ax1.set_xscale("log")
ax1.set_xticks(p_s)
ax1.get_xaxis().set_major_formatter(plt.ScalarFormatter())
ax1.set_xlim(min(p_s) * 0.8, max(p_s) * 1.2)

# Labels above points
for x, y in zip(p_s, effs_s):
    ax1.text(x, y + 0.02, f"{y:.2f}", ha="center", va="bottom", fontsize=9, color=color1, fontweight="bold")
for x, y in zip(p_s, times_s):
    ax2.text(x, y + 0.02, f"{y:.1f}s", ha="center", va="bottom", fontsize=9, color=color2, fontweight="bold")

# --- Style ---
plt.title("Efficiency and Transfer Time vs Payload Size")
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
