import csv
import matplotlib.pyplot as plt
import numpy as np

# Read CSV and remove duplicates
time_data = {}
with open('plot.csv', 'r') as file:
    reader = csv.reader(file)
    next(reader)  # Skip the header
    for row in reader:
        time = float(row[0])  # Assuming time values are in the first column
        data = float(row[1])  # And data values are in the second column
        if time not in time_data:
            time_data[time] = data

# Sort the data
times = np.array(sorted(time_data.keys()))
data = np.array([time_data[t] for t in times])

# Calculate the derivative
derivative_v = np.gradient(data)
derivative_a = np.gradient(derivative_v)
derivative_j = np.gradient(derivative_a)

# Plot the data
# # Plot the data
plt.figure(figsize=(10, 5))
plt.subplot(1, 4, 1)
plt.plot(times, data, label='Data')
plt.title('Data')
plt.legend()

plt.subplot(1, 4, 2)
plt.plot(times, derivative_v, label='Derivative')
plt.title('Derivative')
plt.legend()

plt.subplot(1, 4, 3)
plt.plot(times, derivative_a, label='Derivative')
plt.title('Derivative')
plt.legend()
plt.subplot(1, 4, 4)
plt.plot(times, derivative_j , label='Derivative')
plt.title('Derivative')
plt.legend()

plt.tight_layout()
plt.show()