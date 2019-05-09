import matplotlib.pyplot as plt
import sys


if len(sys.argv) < 2:
    sys.exit("usage: python3 vis.py <dat_1> <dat_2> ... <dat_n>\n" +\
             "cwd must contain a time.dat");


def dataset_load(src):
    """
    Dumps a dat file with newline delimiters into a float array.

    src: str
        name of source file
    """
    set = []
    for line in open(src, "r").readlines():
        set.append(float(line.strip()))
    return set


time = dataset_load("time.dat")
fig, ax = plt.subplots(1)

# Generate a plot of each specified dat
for i in range(1, len(sys.argv)):
    set = dataset_load(sys.argv[i])

    while len(set) < len(time):
        set.append(0)

    ax.plot(time, set)

fig.show()
ax.set_title(", ".join(sys.argv[1:]))
axes = plt.gca()
# axes.set_ylim([0, 4500])
plt.show()
