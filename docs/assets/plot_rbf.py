"""Generate relaxed barrier function plots for docs/rbf.md."""

from pathlib import Path
import logging
import warnings

import numpy as np

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib import font_manager
from matplotlib import ticker

warnings.filterwarnings(
    "ignore",
    message="Font 'default' does not have a glyph for.*",
    category=UserWarning,
)
logging.getLogger("matplotlib").setLevel(logging.ERROR)


SCRIPT_DIR = Path(__file__).resolve().parent
CMU_SERIF_FONT = Path.home() / "Library/Fonts/cmu.serif-roman.ttf"

RBF_DELTA = 0.075
RBF_MU = 1.0

RBF_PARAMS_DELTA_SWEEP = [
    {"delta": 0.75, "mu": 1.0, "color": "tab:blue", "linestyle": ":"},
    {"delta": 0.10, "mu": 1.0, "color": "tab:green", "linestyle": "--"},
    {"delta": 0.01, "mu": 1.0, "color": "tab:purple", "linestyle": "-."},
]
RBF_PARAMS_MU_SWEEP = [
    {"delta": 0.10, "mu": 0.01, "color": "tab:blue", "linestyle": ":"},
    {"delta": 0.10, "mu": 0.10, "color": "tab:green", "linestyle": "--"},
    {"delta": 0.10, "mu": 1.00, "color": "tab:purple", "linestyle": "-."},
]

BLACK_LW = 0.75
RED = "tab:red"
BLUE = "tab:blue"


def configure_matplotlib():
    if CMU_SERIF_FONT.exists():
        font_manager.fontManager.addfont(str(CMU_SERIF_FONT))
        font_family = "CMU Serif"
    else:
        font_family = "serif"

    plt.rcParams.update(
        {
            "font.size": 10.5,
            "font.family": font_family,
            "mathtext.fontset": "cm",
            "axes.grid": True,
            "grid.linestyle": ":",
            "grid.alpha": 0.55,
            "axes.unicode_minus": False,
        }
    )


def beta(h, delta, mu):
    h = np.asarray(h, dtype=float)
    y = np.empty_like(h)
    mask = h > delta
    y[mask] = -mu * np.log(h[mask])
    dh = (h[~mask] - 2.0 * delta) / delta
    y[~mask] = mu * (-np.log(delta) + 0.5 * dh * dh - 0.5)
    return y


def beta_prime(h, delta, mu):
    h = np.asarray(h, dtype=float)
    y = np.empty_like(h)
    mask = h > delta
    y[mask] = -mu / h[mask]
    y[~mask] = mu * (h[~mask] - 2.0 * delta) / (delta * delta)
    return y


def beta_second(h, delta, mu):
    h = np.asarray(h, dtype=float)
    y = np.empty_like(h)
    mask = h > delta
    y[mask] = mu / (h[mask] * h[mask])
    y[~mask] = mu / (delta * delta)
    return y


def draw_left_refs(ax, delta, include_zero_y=True):
    if include_zero_y:
        ax.axhline(0.0, color="black", lw=BLACK_LW, zorder=0)
    ax.axvline(0.0, color="black", lw=BLACK_LW, zorder=0)
    ax.axvline(delta, color=BLUE, lw=0.9, ls="--", alpha=0.75, zorder=0)


def draw_right_refs(ax, lower, upper, center, delta, include_zero_y=True):
    if include_zero_y:
        ax.axhline(0.0, color="black", lw=BLACK_LW, zorder=0)
    for x in (lower, center, upper):
        ax.axvline(x, color="black", lw=BLACK_LW, zorder=0)
    ax.axvline(lower + delta, color=BLUE, lw=0.9, ls="--", alpha=0.75, zorder=0)
    ax.axvline(upper - delta, color=BLUE, lw=0.9, ls="--", alpha=0.75, zorder=0)


def mask_two_sided_quadratic_regions(z, values, lower, upper, delta):
    mask = (z <= lower + delta) | (z >= upper - delta)
    masked_values = np.full_like(values, np.nan)
    masked_values[mask] = values[mask]
    return masked_values


def format_log_y_axis(ax):
    major_ticks = [1e-1, 1e0, 1e1, 1e2, 1e3]

    def formatter(value, _position):
        exponent = int(np.round(np.log10(value)))
        return rf"$10^{{{exponent}}}$"

    ax.yaxis.set_major_locator(ticker.FixedLocator(major_ticks))
    ax.yaxis.set_major_formatter(ticker.FuncFormatter(formatter))
    ax.yaxis.set_minor_locator(ticker.LogLocator(base=10.0, subs=np.arange(2, 10) * 0.1))
    ax.yaxis.set_minor_formatter(ticker.NullFormatter())


def generate_rbf_plot():
    fig, axes = plt.subplots(3, 2, figsize=(11, 9.4), constrained_layout=True)

    log_near_zero = 5.0e-3
    h_log = np.linspace(log_near_zero, 2.6, 2400)
    h = np.linspace(-0.75, RBF_DELTA, 2400)
    z = np.linspace(-1.35, 2.35, 2400)
    lower, upper = -1.0, 2.0
    center = 0.5
    inside = (z > lower + log_near_zero) & (z < upper - log_near_zero)
    main_label = rf"$\delta={RBF_DELTA:g},\ \mu={RBF_MU:g}$"

    log_penalty = np.empty((3, 2), dtype=object)
    quadratic_penalty = np.empty((3, 2), dtype=object)

    # Subplot [0, 0]: One-sided relaxed barrier
    ax = axes[0, 0]
    draw_left_refs(ax, RBF_DELTA)
    
    log_penalty[0, 0] = ax.plot(h_log, -RBF_MU * np.log(h_log), color=RED, lw=2.2, label=r"$p_{\mu,\delta}(h) = - \mu \log(h)$", zorder=2)
    quadratic_penalty[0, 0] = ax.plot(h, beta(h, RBF_DELTA, RBF_MU), color=BLUE, lw=2.0, ls="--", label=main_label, zorder=3)
    
    ax.annotate(r"$h=0$", xy=(0.0, -1.65), xytext=(-0.06, -1.65), fontsize=10.5, ha="right")
    ax.annotate(r"$p=0$", xy=(2.25, 0.0), xytext=(2.25, 0.25), fontsize=10.5)
    ax.annotate(r"$\delta$", xy=(RBF_DELTA, 0.2), xytext=(RBF_DELTA + 0.1, 0.2), fontsize=10.5, ha="right", color=BLUE)
    ax.set_xlim(-0.75, 2.6)
    ax.set_ylim(-1.8, 10.0)
    ax.set_ylabel(r"$p_{\mu,\delta}(h)$")
    ax.set_title("One-sided relaxed barrier")
    ax.legend(frameon=False, loc="upper right", fontsize=9)

    # Subplot [0, 1]: Two-sided relaxed barrier
    ax = axes[0, 1]
    draw_right_refs(ax, lower, upper, center, RBF_DELTA)
    solid = np.full_like(z, np.nan)
    solid[inside] = -np.log(z[inside] - lower) - np.log(upper - z[inside])
    solid -= -np.log(center - lower) - np.log(upper - center)
    relaxed = beta(z - lower, RBF_DELTA, RBF_MU) + beta(upper - z, RBF_DELTA, RBF_MU)
    relaxed -= beta(center - lower, RBF_DELTA, RBF_MU) + beta(upper - center, RBF_DELTA, RBF_MU)
    relaxed_quadratic = mask_two_sided_quadratic_regions(z, relaxed, lower, upper, RBF_DELTA)
    
    log_penalty[0, 1] = ax.plot(z, solid, color=RED, lw=2.2, label="$p_{\mu,\delta}(h)$", zorder=2)
    quadratic_penalty[0, 1] = ax.plot(z, relaxed_quadratic, color=BLUE, lw=2.0, ls="--", label=main_label, zorder=3)
    
    ax.annotate(r"$h=0.5$", xy=(center, -0.48), xytext=(center + 0.08, -0.48), fontsize=10.5)
    ax.annotate(r"$p=0$", xy=(1.98, 0.0), xytext=(2.02, 0.25), fontsize=10.5)
    ax.annotate(r"$h_{\min} + \delta$", xy=(lower + RBF_DELTA, 0.2), xytext=(lower + RBF_DELTA - 0.05, 1.0), fontsize=10.5, ha="right", color=BLUE)
    ax.annotate(r"$h_{\max} - \delta$", xy=(upper - RBF_DELTA, 0.2), xytext=(upper - RBF_DELTA + 0.05, 1.0), fontsize=10.5, ha="left", color=BLUE)
    ax.set_xlim(-1.35, 2.35)
    ax.set_ylim(-0.6, 10.0)
    ax.set_ylabel(r"$p_{\mu,\delta}(h)$")
    ax.set_title(r"Two-sided interval $-1 \leq h \leq 2$")
    ax.legend(frameon=False, loc="upper center", fontsize=9)

    # Subplot [1, 0]: One-sided relaxed barrier 1st derivative
    ax = axes[1, 0]
    draw_left_refs(ax, RBF_DELTA)
    
    log_penalty[1, 0] = ax.plot(h_log, -RBF_MU / h_log, color=RED, lw=2.0, label=r"$p'_{\mu,\delta}(h) = - \mu / h$", zorder=2)
    quadratic_penalty[1, 0] = ax.plot(h, beta_prime(h, RBF_DELTA, RBF_MU), color=BLUE, lw=2.0, ls="--", label=main_label, zorder=3)
    
    ax.set_xlim(-0.75, 2.6)
    ax.set_ylim(-55.0, 8.0)
    ax.set_ylabel(r"$p'_{\mu,\delta}(h)$")
    ax.legend(frameon=False, loc="lower right", fontsize=9)

    # Subplot [1, 1]: Two-sided relaxed barrier 1st derivative
    ax = axes[1, 1]
    draw_right_refs(ax, lower, upper, center, RBF_DELTA)
    blog_prime = np.full_like(z, np.nan)
    blog_prime[inside] = -1.0 / (z[inside] - lower) + 1.0 / (upper - z[inside])
    relaxed_prime = beta_prime(z - lower, RBF_DELTA, RBF_MU) - beta_prime(upper - z, RBF_DELTA, RBF_MU)
    relaxed_prime_quadratic = mask_two_sided_quadratic_regions(z, relaxed_prime, lower, upper, RBF_DELTA)
    
    log_penalty[1, 1] = ax.plot(z, blog_prime, color=RED, lw=2.0, label=r"$p_{\log}^{\prime}(h)$", zorder=2)
    quadratic_penalty[1, 1] = ax.plot(z, relaxed_prime_quadratic, color=BLUE, lw=2.0, ls="--", label=main_label, zorder=3)
    
    ax.set_xlim(-1.35, 2.35)
    ax.set_ylim(-55.0, 55.0)
    ax.set_ylabel(r"$p'_{\mu,\delta}(h)$")
    ax.legend(frameon=False, loc="upper center", fontsize=9)

    # Subplot [2, 0]: One-sided relaxed barrier 2nd derivative
    ax = axes[2, 0]
    draw_left_refs(ax, RBF_DELTA, include_zero_y=False)
    
    log_penalty[2, 0] = ax.plot(h_log, RBF_MU / (h_log * h_log), color=RED, lw=2.0, label=r"$p''_{\mu,\delta}(h) = \mu / h^2$", zorder=2)
    quadratic_penalty[2, 0] = ax.plot(h, beta_second(h, RBF_DELTA, RBF_MU), color=BLUE, lw=2.0, ls="--", label=main_label, zorder=3)
    
    ax.set_yscale("log")
    ax.set_xlim(-0.75, 2.6)
    ax.set_ylim(1e-1, 3e3)
    format_log_y_axis(ax)
    ax.set_xlabel(r"constraint margin $h$")
    ax.set_ylabel(r"$p''_{\mu,\delta}(h)$")
    ax.legend(frameon=False, loc="upper right", fontsize=9)

    # Subplot [2, 1]: Two-sided relaxed barrier 2nd derivative
    ax = axes[2, 1]
    draw_right_refs(ax, lower, upper, center, RBF_DELTA, include_zero_y=False)
    blog_second = np.full_like(z, np.nan)
    blog_second[inside] = RBF_MU / ((z[inside] - lower) ** 2) + RBF_MU / ((upper - z[inside]) ** 2)
    relaxed_second = beta_second(z - lower, RBF_DELTA, RBF_MU) + beta_second(upper - z, RBF_DELTA, RBF_MU)
    relaxed_second_quadratic = mask_two_sided_quadratic_regions(z, relaxed_second, lower, upper, RBF_DELTA)
    
    log_penalty[2, 1] = ax.plot(z, blog_second, color=RED, lw=2.0, label=r"$p_{\log}^{\prime\prime}(h)$", zorder=2)
    quadratic_penalty[2, 1] = ax.plot(z, relaxed_second_quadratic, color=BLUE, lw=2.0, ls="--", label=main_label, zorder=3)
    
    ax.set_yscale("log")
    ax.set_xlim(-1.35, 2.35)
    ax.set_ylim(1e-1, 3e3)
    format_log_y_axis(ax)
    ax.set_xlabel(r"constraint coordinate $h$")
    ax.set_ylabel(r"$p''_{\mu,\delta}(h)$")
    ax.legend(frameon=False, loc="upper center", fontsize=9)

    fig.suptitle("Relaxed logarithmic barrier, derivative, and curvature", fontsize=14)
    
    # Save figure
    fig.savefig(SCRIPT_DIR / "rbf_penalty.png", dpi=220, bbox_inches="tight")
    plt.close(fig)


def draw_one_sided_tuning_axis(ax, curves, title):
    h = np.linspace(-0.08, 1.5, 2200)

    ax.axhline(0.0, color="black", lw=BLACK_LW, zorder=0)
    ax.axvline(0.0, color="black", lw=BLACK_LW, zorder=0)
    for curve in curves:
        d, m = curve["delta"], curve["mu"]
        ax.axvline(d, color=curve["color"], lw=0.9, ls=curve["linestyle"], alpha=0.65, zorder=0)
        ax.plot(
            h,
            beta(h, d, m),
            color=curve["color"],
            lw=2.0,
            ls=curve["linestyle"],
            label=rf"$\delta={d:g},\ \mu={m:g}$",
            zorder=2,
        )

    ax.annotate(r"$h=0$", xy=(0.0, -0.48), xytext=(-0.012, -0.48), fontsize=10.5, ha="right")
    ax.annotate(r"$p=0$", xy=(1.28, 0.0), xytext=(1.28, 0.18), fontsize=10.5)
    ax.set_xlim(-0.08, 1.5)
    ax.set_ylim(-0.55, 7.0)
    ax.set_xlabel(r"constraint margin $h$")
    ax.set_ylabel(r"$p_{\mu,\delta}(h)$")
    ax.set_title(title)
    ax.legend(frameon=False, loc="upper right", fontsize=9)


def generate_rbf_tuning_plot():
    fig, axes = plt.subplots(1, 2, figsize=(11, 4.2), constrained_layout=True)
    
    # Subplot [0, 0]: One-sided relaxed barrier; fix mu, sweep delta
    draw_one_sided_tuning_axis(
        axes[0],
        RBF_PARAMS_DELTA_SWEEP,
        r"$\delta$ sweep with $\mu=1$",
    )
    
    # Subplot [0, 1]: One-sided relaxed barrier; fix delta, sweep mu
    draw_one_sided_tuning_axis(
        axes[1],
        RBF_PARAMS_MU_SWEEP,
        r"$\mu$ sweep with $\delta=0.1$",
    )
    fig.suptitle("One-sided relaxed barrier tuning sweeps", fontsize=14)

    # Save figure
    fig.savefig(SCRIPT_DIR / "rbf_penalty_tuning_pairs.png", dpi=220, bbox_inches="tight")
    plt.close(fig)


def main():
    configure_matplotlib()
    generate_rbf_plot()
    generate_rbf_tuning_plot()


if __name__ == "__main__":
    main()
