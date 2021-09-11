import argparse

import matplotlib.pyplot as plt
from matplotlib.patches import Arc, Rectangle

parser = argparse.ArgumentParser(
    description="Generate SVG image of cart pole physics model."
)

parser.add_argument("path", type=str, help="resulted svg path")
parser.add_argument("--tex", action="store_true")

args = parser.parse_args()


if args.tex:
    plt.rcParams.update(
        {
            "text.usetex": True,
            "font.family": "sans-serif",
            "font.sans-serif": ["Helvetica"],
        }
    )

plt.figure(figsize=[12, 6])
ax = plt.gca()

plt.xlim(-20, 20)
plt.ylim(-10, 10)

plt.axhline(0, c="green", linewidth=2, linestyle="--", zorder=-1, alpha=0.5)
plt.axvline(0, c="green", linewidth=2, linestyle="--", zorder=-1, alpha=0.5)

width, height = 6, 4
rect = Rectangle(
    (-width / 2, -height / 2), width, height, color="red", linewidth=4, fill=False
)
ax.add_patch(rect)

theta_arc = Arc(
    (0, 0),
    width=10,
    height=10,
    theta1=-90,
    theta2=-45,
    alpha=0.5,
    linewidth=2,
    linestyle="--",
    color="green",
)
ax.add_patch(theta_arc)

plt.plot([0, 9], [0, -9], color="orange", linewidth=8)

plt.arrow(0, 0, -8, 0, color="blue", width=0.1, zorder=2)
plt.arrow(0, 0, -8, 0, color="blue", width=0.1, zorder=2)
plt.arrow(-4, -4, 0, -5, color="blue", width=0.1, zorder=2)

plt.annotate(
    "",
    xy=(0.5, 0.5),
    xytext=(9.5, -8.5),
    arrowprops={"arrowstyle": "<->", "color": "green"},
)

ax.annotate(r"$\theta$", (1, -4), size=20, color="green")
ax.annotate(r"$m_{c}$", (-2, 0.5), size=20, color="red")
ax.annotate(r"$m_{p}$", (3, -6), size=20, color="orange")
ax.annotate(r"$l$", (5.5, -3.5), size=20, color="green")
ax.annotate(r"$x$", (7, 0.5), size=20, color="green")
ax.annotate(r"$g$", (-3.5, -7), size=20, color="blue")
ax.annotate(r"$f$", (-5, 0.5), size=20, color="blue")
ax.annotate(r"$\theta$", (1, -4), size=20, color="green")

plt.axis("off")
plt.savefig(args.path)
