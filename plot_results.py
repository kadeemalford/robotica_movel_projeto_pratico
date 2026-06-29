#!/usr/bin/env python3
"""Gera graficos e metricas a partir do arquivo .npz do experimento."""

from __future__ import annotations

import argparse
import csv
from datetime import datetime
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


REQUIRED_KEYS = {
    "time",
    "formation_state",
    "formation_reference",
    "limo_pose",
    "drone_pose",
    "limo_command",
    "drone_command",
    "obstacle_distance",
}


def load_logs(path: str) -> dict[str, np.ndarray]:
    data = np.load(path)
    logs = {key: data[key] for key in data.files}
    missing = sorted(REQUIRED_KEYS.difference(logs))
    if missing:
        raise ValueError(f"Arquivo de log incompleto. Variaveis ausentes: {', '.join(missing)}")
    return logs


def flatten_time(time_array: np.ndarray) -> np.ndarray:
    return np.asarray(time_array).reshape(-1)


def wrap_to_pi(angle: np.ndarray) -> np.ndarray:
    return np.arctan2(np.sin(angle), np.cos(angle))


def compute_error_series(logs: dict[str, np.ndarray]) -> dict[str, np.ndarray]:
    formation_state = logs["formation_state"]
    formation_reference = logs["formation_reference"]
    drone_pose = logs["drone_pose"]

    formation_error = formation_reference - formation_state
    formation_error[:, 4] = wrap_to_pi(formation_error[:, 4])
    formation_error[:, 5] = wrap_to_pi(formation_error[:, 5])

    return {
        "xy_error": np.linalg.norm(formation_error[:, :2], axis=1),
        "rho_error": np.abs(formation_error[:, 3]),
        "alpha_error_deg": np.rad2deg(np.abs(formation_error[:, 4])),
        "beta_error_deg": np.rad2deg(np.abs(formation_error[:, 5])),
        "altitude_error": np.abs(drone_pose[:, 2] - formation_reference[:, 3]),
        "formation_error": formation_error,
    }


def compute_metrics(logs: dict[str, np.ndarray]) -> dict[str, float]:
    errors = compute_error_series(logs)
    obstacle_distance = np.asarray(logs["obstacle_distance"]).reshape(-1)
    time_values = flatten_time(logs["time"])

    return {
        "duration_s": float(time_values[-1] - time_values[0]) if len(time_values) > 1 else 0.0,
        "mean_xy_error_m": float(np.mean(errors["xy_error"])),
        "max_xy_error_m": float(np.max(errors["xy_error"])),
        "mean_rho_error_m": float(np.mean(errors["rho_error"])),
        "max_rho_error_m": float(np.max(errors["rho_error"])),
        "mean_alpha_error_deg": float(np.mean(errors["alpha_error_deg"])),
        "max_alpha_error_deg": float(np.max(errors["alpha_error_deg"])),
        "mean_beta_error_deg": float(np.mean(errors["beta_error_deg"])),
        "max_beta_error_deg": float(np.max(errors["beta_error_deg"])),
        "mean_altitude_error_m": float(np.mean(errors["altitude_error"])),
        "max_altitude_error_m": float(np.max(errors["altitude_error"])),
        "min_obstacle_distance_m": float(np.min(obstacle_distance)),
    }


def print_metrics(metrics: dict[str, float]) -> None:
    print("Metricas do experimento")
    for key, value in metrics.items():
        print(f"- {key}: {value:.6f}")


def save_metrics_csv(metrics: dict[str, float], output_dir: Path) -> Path:
    csv_path = output_dir / "metrics.csv"
    with csv_path.open("w", newline="", encoding="utf-8") as csv_file:
        writer = csv.writer(csv_file)
        writer.writerow(["metric", "value"])
        for key, value in metrics.items():
            writer.writerow([key, f"{value:.10f}"])
    return csv_path


def create_output_dir(logfile: Path, results_root: Path) -> Path:
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    output_dir = results_root / f"{logfile.stem}_{timestamp}"
    output_dir.mkdir(parents=True, exist_ok=False)
    return output_dir


def save_figure(fig: plt.Figure, path: Path, show: bool) -> None:
    fig.tight_layout()
    fig.savefig(path, dpi=200, bbox_inches="tight")
    print(f"Grafico salvo em {path}")
    if show:
        fig.show()
    plt.close(fig)


def plot_trajectory_xy(logs: dict[str, np.ndarray], output_dir: Path, show: bool) -> None:
    formation_state = logs["formation_state"]
    formation_reference = logs["formation_reference"]
    drone_pose = logs["drone_pose"]

    fig, ax = plt.subplots(figsize=(7, 6))
    ax.plot(formation_reference[:, 0], formation_reference[:, 1], "r--", label="referencia")
    ax.plot(formation_state[:, 0], formation_state[:, 1], "b", label="LIMO controle")
    ax.plot(drone_pose[:, 0], drone_pose[:, 1], "g", label="drone")
    ax.set_title("Plano XY")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.axis("equal")
    ax.legend()
    save_figure(fig, output_dir / "01_trajetoria_xy.png", show)


def plot_altitude(logs: dict[str, np.ndarray], output_dir: Path, show: bool) -> None:
    t = flatten_time(logs["time"])
    drone_pose = logs["drone_pose"]
    formation_reference = logs["formation_reference"]

    fig, ax = plt.subplots(figsize=(7, 4))
    ax.plot(t, drone_pose[:, 2], label="z drone")
    ax.plot(t, formation_reference[:, 3], "r--", label="z ref")
    ax.set_title("Altitude do drone")
    ax.set_xlabel("tempo [s]")
    ax.set_ylabel("z [m]")
    ax.legend()
    save_figure(fig, output_dir / "02_altitude_drone.png", show)


def plot_formation_variables(logs: dict[str, np.ndarray], output_dir: Path, show: bool) -> None:
    t = flatten_time(logs["time"])
    formation_state = logs["formation_state"]
    formation_reference = logs["formation_reference"]

    fig, ax = plt.subplots(figsize=(8, 5))
    ax.plot(t, formation_state[:, 3], label="rho")
    ax.plot(t, formation_reference[:, 3], "--", label="rho ref")
    ax.plot(t, np.rad2deg(formation_state[:, 4]), label="alpha [deg]")
    ax.plot(t, np.rad2deg(formation_reference[:, 4]), "--", label="alpha ref [deg]")
    ax.plot(t, np.rad2deg(formation_state[:, 5]), label="beta [deg]")
    ax.plot(t, np.rad2deg(formation_reference[:, 5]), "--", label="beta ref [deg]")
    ax.set_title("Variaveis de formacao")
    ax.set_xlabel("tempo [s]")
    ax.legend()
    save_figure(fig, output_dir / "03_variaveis_formacao.png", show)


def plot_formation_errors(logs: dict[str, np.ndarray], output_dir: Path, show: bool) -> None:
    t = flatten_time(logs["time"])
    errors = compute_error_series(logs)
    formation_error = errors["formation_error"]

    fig, ax = plt.subplots(figsize=(8, 5))
    ax.plot(t, formation_error[:, 0], label="erro x_f [m]")
    ax.plot(t, formation_error[:, 1], label="erro y_f [m]")
    ax.plot(t, errors["rho_error"], label="|erro rho| [m]")
    ax.plot(t, errors["alpha_error_deg"], label="|erro alpha| [deg]")
    ax.plot(t, errors["beta_error_deg"], label="|erro beta| [deg]")
    ax.set_title("Erros da formacao")
    ax.set_xlabel("tempo [s]")
    ax.legend()
    save_figure(fig, output_dir / "04_erros_formacao.png", show)


def plot_limo_commands(logs: dict[str, np.ndarray], output_dir: Path, show: bool) -> None:
    t = flatten_time(logs["time"])
    limo_command = logs["limo_command"]

    fig, ax = plt.subplots(figsize=(7, 4))
    ax.plot(t, limo_command[:, 0], label="u_r")
    ax.plot(t, limo_command[:, 1], label="omega_r")
    ax.set_title("Comandos do LIMO")
    ax.set_xlabel("tempo [s]")
    ax.legend()
    save_figure(fig, output_dir / "05_comandos_limo.png", show)


def plot_drone_commands(logs: dict[str, np.ndarray], output_dir: Path, show: bool) -> None:
    t = flatten_time(logs["time"])
    drone_command = logs["drone_command"]

    fig, ax = plt.subplots(figsize=(8, 4))
    ax.plot(t, drone_command[:, 0], label="vx")
    ax.plot(t, drone_command[:, 1], label="vy")
    ax.plot(t, drone_command[:, 2], label="vz")
    ax.plot(t, drone_command[:, 3], label="r")
    ax.set_title("Comandos do drone")
    ax.set_xlabel("tempo [s]")
    ax.legend()
    save_figure(fig, output_dir / "06_comandos_drone.png", show)


def plot_obstacle_distance(logs: dict[str, np.ndarray], output_dir: Path, show: bool) -> None:
    t = flatten_time(logs["time"])
    obstacle_distance = np.asarray(logs["obstacle_distance"]).reshape(-1)

    fig, ax = plt.subplots(figsize=(7, 4))
    ax.plot(t, obstacle_distance, label="distancia ao obstaculo")
    ax.axhline(0.50, color="orange", linestyle="--", label="zona influencia")
    ax.axhline(0.15, color="red", linestyle=":", label="raio obstaculo")
    ax.set_title("Distancia ao obstaculo")
    ax.set_xlabel("tempo [s]")
    ax.set_ylabel("distancia [m]")
    ax.legend()
    save_figure(fig, output_dir / "07_distancia_obstaculo.png", show)


def export_results(logs: dict[str, np.ndarray], output_dir: Path, show: bool) -> None:
    plot_trajectory_xy(logs, output_dir, show)
    plot_altitude(logs, output_dir, show)
    plot_formation_variables(logs, output_dir, show)
    plot_formation_errors(logs, output_dir, show)
    plot_limo_commands(logs, output_dir, show)
    plot_drone_commands(logs, output_dir, show)
    plot_obstacle_distance(logs, output_dir, show)


def analyze_log(logfile: str | Path, output_dir: str | Path | None = None, show: bool = False) -> tuple[Path, Path]:
    logfile_path = Path(logfile).resolve()
    logs = load_logs(str(logfile_path))
    if output_dir is None:
        output_dir_path = create_output_dir(logfile_path, Path("resultados").resolve())
    else:
        output_dir_path = Path(output_dir).resolve()
        output_dir_path.mkdir(parents=True, exist_ok=True)

    metrics = compute_metrics(logs)
    print_metrics(metrics)
    metrics_path = save_metrics_csv(metrics, output_dir_path)
    export_results(logs, output_dir_path, show=show)
    return output_dir_path, metrics_path


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Gera graficos e metricas do arquivo .npz do experimento")
    parser.add_argument("logfile", help="arquivo .npz gerado por formation_controller.py")
    parser.add_argument(
        "--results-dir",
        default="resultados",
        help="diretorio raiz onde sera criada uma nova subpasta de resultados a cada execucao",
    )
    parser.add_argument("--show", action="store_true", help="abre as janelas dos graficos alem de salvá-los")
    return parser


def main() -> None:
    args = build_parser().parse_args()
    logfile = Path(args.logfile).resolve()
    results_root = Path(args.results_dir).resolve()
    output_dir = create_output_dir(logfile, results_root)
    output_dir, metrics_path = analyze_log(logfile, output_dir=output_dir, show=args.show)

    print(f"Pasta de resultados criada em {output_dir}")
    print(f"Metricas salvas em {metrics_path}")


if __name__ == "__main__":
    main()
