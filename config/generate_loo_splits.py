import csv
import os

input_csv = "TotalStation_to_LIO_3D.csv"
output_dir = "loo_splits"
os.makedirs(output_dir, exist_ok=True)

# Read all points from input CSV
with open(input_csv, newline="") as f:
    reader = list(csv.DictReader(f))

N = len(reader)

for i in range(N):
    held_out_name = reader[i]["Point Name"]
    filename = os.path.join(output_dir, f"loo_{i+1:02d}_holdout_{held_out_name}.csv")

    with open(filename, "w", newline="") as f:
        fieldnames = list(reader[0].keys()) + ["split"]
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()

        for j, row in enumerate(reader):
            writer.writerow({**row, "split": "validation" if j == i else "calibration"})

    print(f"[{i+1}/{N}] {filename}")

print(f"\nDone. {N} files written to '{output_dir}/'")

