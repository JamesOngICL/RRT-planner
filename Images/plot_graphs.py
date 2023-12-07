import numpy as np
import matplotlib.pyplot as plt 

# Creating the dataset
data = {'100':30.31, '200':31.41, '300':32.41, '400':30.98, '500':32.55}
iterations = list(data.keys())
num_reached = list(data.values())

# Create a figure and axis
fig, ax = plt.figure(figsize=(10, 5)), plt.gca()

# Create a bar plot
ax.bar(iterations, num_reached, color='blue', width=0.4, label='Goal Reached')

# Create a line plot on the same axis
ax.plot(iterations, num_reached, color='red', marker='o', label='Trend Line')

# Adding labels and title
plt.xlabel("Number of Iterations")
plt.ylabel("Average Path Length")
plt.title("Iterations Against Path Length Given Reached")
# plt.ylim(0,500)
# Adding gridlines for better readability
plt.grid(True, which='both', linestyle='--', linewidth=0.5)

# Add a legend to distinguish the plots
plt.legend()

# Show the plot
plt.show()
