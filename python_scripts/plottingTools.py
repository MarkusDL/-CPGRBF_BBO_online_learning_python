import numpy as np
import matplotlib.pyplot as plt


def moving_average(a, n=3):
    ret = np.cumsum(a, dtype=float)
    ret[n:] = ret[n:] - ret[:-n]
    return ret[n - 1:] / n


def plot_barplots_with_error_bars(ax, means, stds, titel=" ", ylabel=" "):
    if not ax:
        return
    assert (len(means) == len(stds)), f"length of means and stds not equal in plot: {len(means)} != {len(stds)}"
    means_arr, stds_arr = np.array(means), np.array(stds)

    x_pos = np.arange(means_arr.shape[1])
    for i in range(means_arr.shape[0]):
        ax.bar(x_pos, means_arr[i], yerr=stds_arr[i], alpha=0.5, ecolor='black', capsize=10)
    ax.set_ylabel(ylabel)
    ax.set_xticks(x_pos)
    ax.set_title(titel)
    ax.yaxis.grid(True)
    plt.tight_layout()


def plot_barplots(ax, means, titel=" ", ylabel=" "):
    if not ax:
        return
    x_pos = np.arange(len(means))
    ax.bar(x_pos, means, align='center', alpha=0.5, capsize=10)
    ax.set_ylabel(ylabel)
    ax.set_xticks(x_pos)
    ax.set_title(titel)
    ax.yaxis.grid(True)
    plt.tight_layout()


def plot_singel_timeseries(ax, variable, titel=" ", xlabel=" ", ylabel=" ", color="blue", legend=""):
    if not ax:
        return

    ax.plot(variable, color=color, label=legend)
    ax.set_ylabel(ylabel)
    ax.set_xlabel(xlabel)
    ax.set_title(titel)
    ax.legend()
    ax.yaxis.grid(True)
