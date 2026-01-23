import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from itertools import combinations

# Read the ground truth landmark locations
df = pd.read_csv('ground_truth_all.csv')

print(f"Loaded {len(df)} landmarks")

# Calculate pairwise distances and find constraints
constraints = []
constraint_id = 0

# Use combinations to ensure we only get unique pairs (no duplicates like 1-2 and 2-1)
for (idx1, row1), (idx2, row2) in combinations(df.iterrows(), 2):
    id1 = int(row1['id'])
    id2 = int(row2['id'])
    
    # Calculate Euclidean distance
    distance = np.sqrt((row1['x'] - row2['x'])**2 + (row1['y'] - row2['y'])**2)
    
    # If distance is less than 30cm (0.3m), add as constraint
    if distance < 0.45:
        constraints.append({
            'constraint_id': constraint_id,
            'tag_id_1': id1,
            'tag_id_2': id2,
            'distance': distance
        })
        constraint_id += 1

# Convert to DataFrame
constraints_df = pd.DataFrame(constraints)

# Save to CSV
output_file = 'pole_constraints.csv'
constraints_df.to_csv(output_file, index=False)

print(f"\nFound {len(constraints_df)} pole constraints (distance < 30cm)")
print(f"Saved constraints to: {output_file}")

# Print first few constraints
print("\nFirst 10 constraints:")
print(constraints_df.head(10))

# Statistics
print(f"\nConstraint statistics:")
print(f"Mean distance: {constraints_df['distance'].mean():.4f} m")
print(f"Min distance: {constraints_df['distance'].min():.4f} m")
print(f"Max distance: {constraints_df['distance'].max():.4f} m")

# Find unique tags involved in constraints
unique_tags = set(constraints_df['tag_id_1'].tolist() + constraints_df['tag_id_2'].tolist())
print(f"Number of unique tags with constraints: {len(unique_tags)}")

# Visualization
fig, ax = plt.subplots(figsize=(16, 10))

# Plot all landmarks
ax.scatter(df['x'], df['y'], c='blue', s=30, alpha=0.6, label='Landmarks', zorder=2)

# Plot constraints as lines
for _, constraint in constraints_df.iterrows():
    tag1 = df[df['id'] == constraint['tag_id_1']].iloc[0]
    tag2 = df[df['id'] == constraint['tag_id_2']].iloc[0]
    
    ax.plot([tag1['x'], tag2['x']], [tag1['y'], tag2['y']], 
            'r-', alpha=0.3, linewidth=1, zorder=1)

# Add first constraint as example to legend
if len(constraints_df) > 0:
    tag1 = df[df['id'] == constraints_df.iloc[0]['tag_id_1']].iloc[0]
    tag2 = df[df['id'] == constraints_df.iloc[0]['tag_id_2']].iloc[0]
    ax.plot([tag1['x'], tag2['x']], [tag1['y'], tag2['y']], 
            'r-', alpha=0.5, linewidth=2, label='Pole Constraints', zorder=1)

ax.set_xlabel('X (m)', fontsize=12)
ax.set_ylabel('Y (m)', fontsize=12)
ax.set_title(f'Landmark Locations with Pole Constraints\n({len(constraints_df)} constraints, distance < 30cm)', 
             fontsize=14)
ax.legend(fontsize=10)
ax.grid(True, alpha=0.3)
ax.axis('equal')

plt.tight_layout()
plt.savefig('pole_constraints_visualization.png', dpi=150)
print("\nVisualization saved to: pole_constraints_visualization.png")
plt.show()

# Additional analysis: Group tags by poles
print("\n" + "="*60)
print("POLE GROUPING ANALYSIS")
print("="*60)

# Create a graph structure to find connected components (poles)
from collections import defaultdict

def find_poles(constraints_df):
    """Find groups of tags on the same pole using Union-Find"""
    parent = {}
    
    def find(x):
        if x not in parent:
            parent[x] = x
        if parent[x] != x:
            parent[x] = find(parent[x])
        return parent[x]
    
    def union(x, y):
        px, py = find(x), find(y)
        if px != py:
            parent[px] = py
    
    # Build the union-find structure
    for _, row in constraints_df.iterrows():
        union(row['tag_id_1'], row['tag_id_2'])
    
    # Group tags by their root parent
    poles = defaultdict(list)
    for tag in parent.keys():
        poles[find(tag)].append(tag)
    
    return list(poles.values())

poles = find_poles(constraints_df)
poles.sort(key=lambda x: min(x))  # Sort by minimum tag ID in each pole

print(f"\nFound {len(poles)} distinct poles:")
for i, pole_tags in enumerate(poles):
    pole_tags_sorted = sorted(pole_tags)
    print(f"Pole {i}: {len(pole_tags_sorted)} tags - {pole_tags_sorted}")

# Save pole grouping to file
with open('pole_groups.txt', 'w') as f:
    f.write(f"Total poles: {len(poles)}\n\n")
    for i, pole_tags in enumerate(poles):
        pole_tags_sorted = sorted(pole_tags)
        f.write(f"Pole {i}: {pole_tags_sorted}\n")

print("\nPole grouping saved to: pole_groups.txt")
