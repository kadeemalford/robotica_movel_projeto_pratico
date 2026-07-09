#!/usr/bin/env python3
"""Controlador de estrutura virtual para uma formacao LIMO + Bebop 2.

O codigo segue a mesma ideia do projeto em ``robotica_movel``:
- laço externo cinemático;
- laço interno dinâmico;
- modo simulacao por padrao;
- interface ROS1 opcional para experimento real.

Assumicoes relevantes:
- a pose do LIMO recebida pelo OptiTrack representa o centro de gravidade;
- o ponto de controle do LIMO esta a ``a = 0.10 m`` a frente do centro, no eixo X do robo;
- a dinamica identificada do LIMO segue o modelo exibido nas aulas da disciplina,
  com estados de velocidade ``[u, omega]`` e entradas ``[u_r, omega_r]``.
"""

from __future__ import annotations

import argparse
import math
import time
from datetime import datetime
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import numpy as np
from plot_results import analyze_log

try:
    import rospy
    from geometry_msgs.msg import PoseStamped, Twist
    from std_msgs.msg import Empty
except ImportError:  # pragma: no cover - ROS nao costuma existir no ambiente de teste
    rospy = None
    PoseStamped = Twist = Empty = None


def wrap_to_pi(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def smooth_term(gains: np.ndarray, limits: np.ndarray, error: np.ndarray) -> np.ndarray:
    return limits * np.tanh(gains * error)


def rotation_world_from_body(yaw: float) -> np.ndarray:
    return np.array(
        [
            [math.cos(yaw), -math.sin(yaw), 0.0, 0.0],
            [math.sin(yaw), math.cos(yaw), 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ],
        dtype=float,
    )


def create_experiment_output_dir(base_name: str | None = None) -> Path:
    results_dir = Path("resultados").resolve()
    results_dir.mkdir(parents=True, exist_ok=True)
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    stem = base_name if base_name else "resultado_formacao"
    output_dir = results_dir / f"{stem}_{timestamp}"
    output_dir.mkdir(parents=True, exist_ok=False)
    return output_dir


def resolve_output_paths(save_argument: str | None) -> tuple[Path, Path]:
    if save_argument:
        save_path = Path(save_argument).expanduser().resolve()
        output_dir = save_path.parent
        output_dir.mkdir(parents=True, exist_ok=True)
        return output_dir, save_path

    output_dir = create_experiment_output_dir()
    return output_dir, output_dir / "resultado_formacao.npz"


@dataclass
class LimoParams:
    a: float = 0.10
    alpha: float = 0.0
    theta: np.ndarray = field(
        default_factory=lambda: np.array([0.1521, 0.0953, 0.0031, 0.9840, -0.0451, 1.6422], dtype=float)
    )
    kinematic_gains: np.ndarray = field(default_factory=lambda: np.array([1.4, 1.4], dtype=float))
    kinematic_limits: np.ndarray = field(default_factory=lambda: np.array([0.45, 0.45], dtype=float))
    dynamic_gains: np.ndarray = field(default_factory=lambda: np.diag([2.2, 2.0]).astype(float))
    command_limits: np.ndarray = field(default_factory=lambda: np.array([1.2, 2.5], dtype=float))

    @property
    def theta1(self) -> float:
        return float(self.theta[0])

    @property
    def theta2(self) -> float:
        return float(self.theta[1])

    @property
    def theta3(self) -> float:
        return float(self.theta[2])

    @property
    def theta4(self) -> float:
        return float(self.theta[3])

    @property
    def theta5(self) -> float:
        return float(self.theta[4])

    @property
    def theta6(self) -> float:
        return float(self.theta[5])

    @property
    def input_matrix(self) -> np.ndarray:
        return np.diag([1.0 / self.theta1, 1.0 / self.theta2])

    @property
    def input_matrix_inverse(self) -> np.ndarray:
        return np.diag([self.theta1, self.theta2])

    def drift(self, velocity: np.ndarray) -> np.ndarray:
        u, omega = velocity
        return np.array(
            [
                (self.theta3 / self.theta1) * (omega ** 2) - (self.theta4 / self.theta1) * u,
                -(self.theta5 / self.theta2) * u * omega - (self.theta6 / self.theta2) * omega,
            ],
            dtype=float,
        )


@dataclass
class DroneParams:
    ku: np.ndarray = field(default_factory=lambda: np.diag([0.8417, 0.8354, 3.9660, 9.8524]).astype(float))
    kv: np.ndarray = field(default_factory=lambda: np.diag([0.18227, 0.17095, 4.0010, 4.7295]).astype(float))
    kinematic_gains: np.ndarray = field(default_factory=lambda: np.array([1.0, 1.0, 1.4, 1.0], dtype=float))
    kinematic_limits: np.ndarray = field(default_factory=lambda: np.array([0.7, 0.7, 0.6, 0.7], dtype=float))
    dynamic_gains: np.ndarray = field(default_factory=lambda: np.diag([1.0, 1.0, 1.0, 1.0]).astype(float))
    command_limits: np.ndarray = field(default_factory=lambda: np.array([1.0, 1.0, 1.0, 1.0], dtype=float))
    yaw_reference: float = 0.0

    @property
    def ku_inverse(self) -> np.ndarray:
        return np.linalg.inv(self.ku)


@dataclass
class FormationParams:
    rate_hz: float = 30.0
    task_duration: float = 40.0
    control_gains: np.ndarray = field(default_factory=lambda: np.array([1.4, 1.4, 1.0, 0.0, 0.0, 0.0], dtype=float))
    control_limits: np.ndarray = field(default_factory=lambda: np.array([0.55, 0.55, 0.10, 0.00, 0.00, 0.00], dtype=float))
    obstacle_center: np.ndarray = field(default_factory=lambda: np.array([-0.2, 0.425], dtype=float))
    obstacle_radius: float = 0.15
    obstacle_influence_radius: float = 0.50
    obstacle_gain: float = 1.6
    obstacle_speed_limit: float = 0.5


@dataclass
class LimoState:
    pose: np.ndarray
    velocity: np.ndarray


@dataclass
class DroneState:
    pose: np.ndarray
    body_velocity: np.ndarray


class FormationController:
    def __init__(
        self,
        simulation: bool = True,
        real_time: bool = False,
        ros_namespace_limo: str = "L1",
        ros_namespace_drone: str = "B1",
    ) -> None:
        self.simulation = simulation or rospy is None
        self.real_time = real_time
        self.limo_params = LimoParams()
        self.drone_params = DroneParams()
        self.formation_params = FormationParams()
        self.dt = 1.0 / self.formation_params.rate_hz

        self.prev_limo_velocity_reference: Optional[np.ndarray] = None
        self.prev_drone_velocity_reference: Optional[np.ndarray] = None

        self.logs: Dict[str, List[np.ndarray]] = {
            "time": [],
            "formation_state": [],
            "formation_reference": [],
            "limo_pose": [],
            "drone_pose": [],
            "limo_command": [],
            "drone_command": [],
            "obstacle_distance": [],
        }

        self.limo_state = LimoState(
            pose=np.array([0.4, -0.25, 0.0], dtype=float),
            velocity=np.zeros(2, dtype=float),
        )
        self.drone_state = DroneState(
            pose=np.array([0.4, 0.05, 1.5, 0.0], dtype=float),
            body_velocity=np.zeros(4, dtype=float),
        )

        self.ros_namespace_limo = ros_namespace_limo.strip("/")
        self.ros_namespace_drone = ros_namespace_drone.strip("/")
        self.last_ros_limo_pose: Optional[np.ndarray] = None
        self.last_ros_drone_pose: Optional[np.ndarray] = None
        self.last_ros_limo_stamp: Optional[float] = None
        self.last_ros_drone_stamp: Optional[float] = None

        self.pose_sub_limo = None
        self.pose_sub_drone = None
        self.cmd_pub_limo = None
        self.cmd_pub_drone = None
        self.takeoff_pub = None
        self.land_pub = None

        if not self.simulation:
            if rospy is None:
                raise RuntimeError("rospy nao esta disponivel, mas o modo ROS foi solicitado.")
            self._setup_ros()

    def _setup_ros(self) -> None:
        rospy.init_node("formation_virtual_controller", anonymous=True)
        self.pose_sub_limo = rospy.Subscriber(
            f"/vrpn_client_node/{self.ros_namespace_limo}/pose",
            PoseStamped,
            self._limo_pose_callback,
            queue_size=1,
        )
        self.pose_sub_drone = rospy.Subscriber(
            f"/vrpn_client_node/{self.ros_namespace_drone}/pose",
            PoseStamped,
            self._drone_pose_callback,
            queue_size=1,
        )
        self.cmd_pub_limo = rospy.Publisher(f"/{self.ros_namespace_limo}/cmd_vel", Twist, queue_size=10)
        self.cmd_pub_drone = rospy.Publisher(f"/{self.ros_namespace_drone}/cmd_vel", Twist, queue_size=10)
        self.takeoff_pub = rospy.Publisher(f"/{self.ros_namespace_drone}/takeoff", Empty, queue_size=1)
        self.land_pub = rospy.Publisher(f"/{self.ros_namespace_drone}/land", Empty, queue_size=1)

    def _yaw_from_quaternion(self, z: float, w: float) -> float:
        return 2.0 * math.atan2(z, w)

    def _limo_pose_callback(self, msg: PoseStamped) -> None:
        now = time.time()
        pose = np.array(
            [
                msg.pose.position.x,
                msg.pose.position.y,
                self._yaw_from_quaternion(msg.pose.orientation.z, msg.pose.orientation.w),
            ],
            dtype=float,
        )
        if self.last_ros_limo_pose is not None and self.last_ros_limo_stamp is not None:
            dt = max(now - self.last_ros_limo_stamp, 1e-3)
            cp_now = self.limo_control_point_from_pose(pose)
            cp_prev = self.limo_control_point_from_pose(self.last_ros_limo_pose)
            cp_vel = (cp_now - cp_prev) / dt
            u = cp_vel[0] * math.cos(pose[2]) + cp_vel[1] * math.sin(pose[2])
            omega = (-cp_vel[0] * math.sin(pose[2]) + cp_vel[1] * math.cos(pose[2])) / self.limo_params.a
            self.limo_state.velocity = np.array([u, omega], dtype=float)
        self.limo_state.pose = pose
        self.last_ros_limo_pose = pose.copy()
        self.last_ros_limo_stamp = now

    def _drone_pose_callback(self, msg: PoseStamped) -> None:
        now = time.time()
        pose = np.array(
            [
                msg.pose.position.x,
                msg.pose.position.y,
                msg.pose.position.z,
                self._yaw_from_quaternion(msg.pose.orientation.z, msg.pose.orientation.w),
            ],
            dtype=float,
        )
        if self.last_ros_drone_pose is not None and self.last_ros_drone_stamp is not None:
            dt = max(now - self.last_ros_drone_stamp, 1e-3)
            world_velocity = (pose - self.last_ros_drone_pose) / dt
            self.drone_state.body_velocity = np.linalg.inv(rotation_world_from_body(pose[3])) @ world_velocity
        self.drone_state.pose = pose
        self.last_ros_drone_pose = pose.copy()
        self.last_ros_drone_stamp = now

    def limo_control_point_from_pose(self, pose: np.ndarray) -> np.ndarray:
        return np.array(
            [
                pose[0] + self.limo_params.a * math.cos(pose[2]),
                pose[1] + self.limo_params.a * math.sin(pose[2]),
                0.0,
            ],
            dtype=float,
        )

    def formation_reference(self, t: float) -> Tuple[np.ndarray, np.ndarray]:
        omega = 2.0 * math.pi / 40.0
        xf = 0.75 * math.sin(omega * t)
        yf = 0.75 * math.sin(2.0 * omega * t)
        xfd = 0.75 * omega * math.cos(omega * t)
        yfd = 0.75 * 2.0 * omega * math.cos(2.0 * omega * t)
        qd = np.array([xf, yf, 0.0, 1.5, 0.0, math.pi / 2.0], dtype=float)
        qd_dot = np.array([xfd, yfd, 0.0, 0.0, 0.0, 0.0], dtype=float)
        return qd, qd_dot

    def current_formation_state(self) -> np.ndarray:
        control_point = self.limo_control_point_from_pose(self.limo_state.pose)
        rel = self.drone_state.pose[:3] - control_point
        rho = float(np.linalg.norm(rel))
        planar = float(np.linalg.norm(rel[:2]))
        alpha = math.atan2(rel[1], rel[0]) if planar > 1e-6 else 0.0
        beta = math.atan2(rel[2], planar) if rho > 1e-6 else math.pi / 2.0
        return np.array([control_point[0], control_point[1], 0.0, rho, alpha, beta], dtype=float)

    def drone_desired_position(self, q: np.ndarray) -> np.ndarray:
        xf, yf, zf, rho, alpha, beta = q
        planar = rho * math.cos(beta)
        return np.array(
            [
                xf + planar * math.cos(alpha),
                yf + planar * math.sin(alpha),
                zf + rho * math.sin(beta),
            ],
            dtype=float,
        )

    def drone_formation_jacobian(self, q: np.ndarray) -> np.ndarray:
        _, _, _, rho, alpha, beta = q
        cos_alpha = math.cos(alpha)
        sin_alpha = math.sin(alpha)
        cos_beta = math.cos(beta)
        sin_beta = math.sin(beta)
        return np.array(
            [
                [1.0, 0.0, 0.0, cos_beta * cos_alpha, -rho * cos_beta * sin_alpha, -rho * sin_beta * cos_alpha],
                [0.0, 1.0, 0.0, cos_beta * sin_alpha, rho * cos_beta * cos_alpha, -rho * sin_beta * sin_alpha],
                [0.0, 0.0, 1.0, sin_beta, 0.0, rho * cos_beta],
            ],
            dtype=float,
        )

    def formation_error(self, qd: np.ndarray, q: np.ndarray) -> np.ndarray:
        error = qd - q
        error[4] = wrap_to_pi(error[4])
        error[5] = wrap_to_pi(error[5])
        return error

    def formation_outer_loop(self, qd: np.ndarray, qd_dot: np.ndarray, q: np.ndarray) -> Tuple[np.ndarray, float]:
        error = self.formation_error(qd, q)
        secondary = qd_dot + smooth_term(self.formation_params.control_gains, self.formation_params.control_limits, error)

        delta = q[:2] - self.formation_params.obstacle_center
        distance = float(np.linalg.norm(delta))
        influence = self.formation_params.obstacle_influence_radius
        if distance < 1e-6 or distance >= influence:
            return secondary, distance

        # Subtarefa de desvio de obstaculo (prioridade sobre a formacao), com
        # ativacao suave para evitar chattering na fronteira da zona de influencia.
        # h vai de 0 (na fronteira) a 1 (no raio fisico do obstaculo).
        radius = self.formation_params.obstacle_radius
        activation = (influence - distance) / max(influence - radius, 1e-6)
        activation = float(np.clip(activation, 0.0, 1.0))
        # Suavizacao (smoothstep) para transicao continua e derivada nula nas bordas.
        activation = activation * activation * (3.0 - 2.0 * activation)

        normal = delta / distance
        j_obs = np.zeros((1, 6), dtype=float)
        j_obs[0, 0] = normal[0]
        j_obs[0, 1] = normal[1]

        scalar_speed = min(
            self.formation_params.obstacle_speed_limit,
            self.formation_params.obstacle_gain * (influence - distance),
        )
        j_norm_sq = float((j_obs @ j_obs.T)[0, 0])
        j_pinv = j_obs.T / j_norm_sq  # (6, 1)
        projector = np.eye(6) - j_pinv @ j_obs
        primary = j_pinv.reshape(-1) * scalar_speed

        # Combinacao NSB suave: fora da zona (activation=0) segue a formacao;
        # ao se aproximar (activation->1) prioriza o desvio no espaco nulo.
        obstacle_task = primary + projector @ secondary
        combined = (1.0 - activation) * secondary + activation * obstacle_task
        return combined, distance

    def limo_velocity_reference(self, world_velocity: np.ndarray, yaw: float) -> np.ndarray:
        nu_x, nu_y = world_velocity
        u_d = nu_x * math.cos(yaw) + nu_y * math.sin(yaw)
        omega_d = (-nu_x * math.sin(yaw) + nu_y * math.cos(yaw)) / self.limo_params.a
        return np.array([u_d, omega_d], dtype=float)

    def limo_dynamic_controller(self, velocity_reference: np.ndarray, current_velocity: np.ndarray) -> np.ndarray:
        if self.prev_limo_velocity_reference is None:
            derivative = np.zeros_like(velocity_reference)
        else:
            derivative = (velocity_reference - self.prev_limo_velocity_reference) / self.dt
        self.prev_limo_velocity_reference = velocity_reference.copy()
        drift = self.limo_params.drift(current_velocity)
        command = self.limo_params.input_matrix_inverse @ (
            derivative + self.limo_params.dynamic_gains @ (velocity_reference - current_velocity) - drift
        )
        return np.clip(command, -self.limo_params.command_limits, self.limo_params.command_limits)

    def drone_dynamic_controller(self, velocity_reference: np.ndarray) -> np.ndarray:
        if self.prev_drone_velocity_reference is None:
            derivative = np.zeros_like(velocity_reference)
        else:
            derivative = (velocity_reference - self.prev_drone_velocity_reference) / self.dt
        self.prev_drone_velocity_reference = velocity_reference.copy()
        current_velocity = self.drone_state.body_velocity
        command = self.drone_params.ku_inverse @ (
            derivative + self.drone_params.dynamic_gains @ (velocity_reference - current_velocity) + self.drone_params.kv @ current_velocity
        )
        return np.clip(command, -self.drone_params.command_limits, self.drone_params.command_limits)

    def compute_commands(self, t: float) -> Dict[str, np.ndarray]:
        q = self.current_formation_state()
        qd, qd_dot = self.formation_reference(t)
        qdot = self.formation_outer_loop(qd, qd_dot, q)
        formation_velocity, obstacle_distance = qdot

        limo_reference = self.limo_velocity_reference(formation_velocity[:2], self.limo_state.pose[2])
        limo_command = self.limo_dynamic_controller(limo_reference, self.limo_state.velocity)

        drone_target_position = self.drone_desired_position(qd)
        drone_position_error = drone_target_position - self.drone_state.pose[:3]
        drone_world_velocity = self.drone_formation_jacobian(q) @ formation_velocity
        yaw_error = wrap_to_pi(self.drone_params.yaw_reference - self.drone_state.pose[3])
        drone_aux_world = np.array(
            [
                drone_world_velocity[0],
                drone_world_velocity[1],
                drone_world_velocity[2],
                0.0,
            ],
            dtype=float,
        )
        drone_aux_world[:3] += smooth_term(
            self.drone_params.kinematic_gains[:3],
            self.drone_params.kinematic_limits[:3],
            drone_position_error,
        )
        drone_aux_world[3] = smooth_term(
            np.array([self.drone_params.kinematic_gains[3]], dtype=float),
            np.array([self.drone_params.kinematic_limits[3]], dtype=float),
            np.array([yaw_error], dtype=float),
        )[0]
        drone_velocity_reference = np.linalg.inv(rotation_world_from_body(self.drone_state.pose[3])) @ drone_aux_world
        drone_command = self.drone_dynamic_controller(drone_velocity_reference)

        return {
            "q": q,
            "qd": qd,
            "obstacle_distance": np.array([obstacle_distance], dtype=float),
            "limo_reference": limo_reference,
            "limo_command": limo_command,
            "drone_reference": drone_velocity_reference,
            "drone_command": drone_command,
        }

    def step_simulation(self, commands: Dict[str, np.ndarray]) -> None:
        limo_acc = self.limo_params.drift(self.limo_state.velocity) + self.limo_params.input_matrix @ commands["limo_command"]
        self.limo_state.velocity = self.limo_state.velocity + self.dt * limo_acc
        u, omega = self.limo_state.velocity
        x, y, psi = self.limo_state.pose
        self.limo_state.pose = np.array(
            [
                x + self.dt * u * math.cos(psi),
                y + self.dt * u * math.sin(psi),
                wrap_to_pi(psi + self.dt * omega),
            ],
            dtype=float,
        )

        body_acc = self.drone_params.ku @ commands["drone_command"] - self.drone_params.kv @ self.drone_state.body_velocity
        self.drone_state.body_velocity = self.drone_state.body_velocity + self.dt * body_acc
        world_velocity = rotation_world_from_body(self.drone_state.pose[3]) @ self.drone_state.body_velocity
        self.drone_state.pose = np.array(
            [
                self.drone_state.pose[0] + self.dt * world_velocity[0],
                self.drone_state.pose[1] + self.dt * world_velocity[1],
                self.drone_state.pose[2] + self.dt * world_velocity[2],
                wrap_to_pi(self.drone_state.pose[3] + self.dt * world_velocity[3]),
            ],
            dtype=float,
        )

    def publish_ros_commands(self, commands: Dict[str, np.ndarray]) -> None:
        limo_msg = Twist()
        limo_msg.linear.x = float(commands["limo_command"][0])
        limo_msg.angular.z = float(commands["limo_command"][1])
        self.cmd_pub_limo.publish(limo_msg)

        drone_msg = Twist()
        drone_msg.linear.x = float(commands["drone_command"][0])
        drone_msg.linear.y = float(commands["drone_command"][1])
        drone_msg.linear.z = float(commands["drone_command"][2])
        drone_msg.angular.z = float(commands["drone_command"][3])
        self.cmd_pub_drone.publish(drone_msg)

    def publish_zero_commands(self) -> None:
        if self.simulation:
            return
        self.cmd_pub_limo.publish(Twist())
        self.cmd_pub_drone.publish(Twist())

    def log_step(self, t: float, commands: Dict[str, np.ndarray]) -> None:
        self.logs["time"].append(np.array([t], dtype=float))
        self.logs["formation_state"].append(commands["q"].copy())
        self.logs["formation_reference"].append(commands["qd"].copy())
        self.logs["limo_pose"].append(self.limo_state.pose.copy())
        self.logs["drone_pose"].append(self.drone_state.pose.copy())
        self.logs["limo_command"].append(commands["limo_command"].copy())
        self.logs["drone_command"].append(commands["drone_command"].copy())
        self.logs["obstacle_distance"].append(commands["obstacle_distance"].copy())

    def save_logs(self, path: str | Path) -> None:
        data = {key: np.array(value) for key, value in self.logs.items()}
        output_path = Path(path)
        output_path.parent.mkdir(parents=True, exist_ok=True)
        np.savez(output_path, **data)

    def plot_logs(self) -> None:
        try:
            import matplotlib.pyplot as plt
        except ImportError:
            print("matplotlib nao esta disponivel; pulando graficos.")
            return

        time_values = np.array(self.logs["time"]).reshape(-1)
        formation_state = np.array(self.logs["formation_state"])
        formation_reference = np.array(self.logs["formation_reference"])
        limo_pose = np.array(self.logs["limo_pose"])
        drone_pose = np.array(self.logs["drone_pose"])
        obstacle_distance = np.array(self.logs["obstacle_distance"]).reshape(-1)

        fig = plt.figure(figsize=(11, 9))
        ax_xy = fig.add_subplot(221)
        ax_xy.plot(formation_reference[:, 0], formation_reference[:, 1], "r--", label="referencia da formacao")
        ax_xy.plot(formation_state[:, 0], formation_state[:, 1], "b", label="ponto de controle LIMO")
        ax_xy.plot(drone_pose[:, 0], drone_pose[:, 1], "g", label="drone")
        obstacle = plt.Circle(self.formation_params.obstacle_center, self.formation_params.obstacle_radius, color="k", alpha=0.25)
        influence = plt.Circle(self.formation_params.obstacle_center, self.formation_params.obstacle_influence_radius, color="orange", alpha=0.12)
        ax_xy.add_patch(obstacle)
        ax_xy.add_patch(influence)
        ax_xy.set_xlabel("x [m]")
        ax_xy.set_ylabel("y [m]")
        ax_xy.set_title("Plano XY")
        ax_xy.axis("equal")
        ax_xy.legend()

        ax_z = fig.add_subplot(222)
        ax_z.plot(time_values, drone_pose[:, 2], label="z drone")
        ax_z.plot(time_values, np.full_like(time_values, 1.5), "r--", label="z referencia")
        ax_z.set_xlabel("tempo [s]")
        ax_z.set_ylabel("z [m]")
        ax_z.set_title("Altitude do drone")
        ax_z.legend()

        ax_cmd = fig.add_subplot(223)
        limo_cmd = np.array(self.logs["limo_command"])
        drone_cmd = np.array(self.logs["drone_command"])
        ax_cmd.plot(time_values, limo_cmd[:, 0], label="u_r LIMO")
        ax_cmd.plot(time_values, limo_cmd[:, 1], label="omega_r LIMO")
        ax_cmd.plot(time_values, drone_cmd[:, 2], label="vz drone")
        ax_cmd.plot(time_values, drone_cmd[:, 3], label="r drone")
        ax_cmd.set_xlabel("tempo [s]")
        ax_cmd.set_title("Comandos")
        ax_cmd.legend()

        ax_obs = fig.add_subplot(224)
        ax_obs.plot(time_values, obstacle_distance, label="distancia ao obstaculo")
        ax_obs.axhline(self.formation_params.obstacle_influence_radius, color="orange", linestyle="--", label="zona de influencia")
        ax_obs.axhline(self.formation_params.obstacle_radius, color="red", linestyle=":", label="raio do obstaculo")
        ax_obs.set_xlabel("tempo [s]")
        ax_obs.set_ylabel("distancia [m]")
        ax_obs.set_title("Aproximacao do obstaculo")
        ax_obs.legend()

        fig.tight_layout()
        plt.show()

    def run(self, duration: Optional[float] = None, takeoff: bool = False) -> None:
        duration = self.formation_params.task_duration if duration is None else duration
        if not self.simulation and takeoff:
            self.takeoff_pub.publish(Empty())
            time.sleep(3.0)

        steps = int(duration * self.formation_params.rate_hz)
        ros_rate = rospy.Rate(self.formation_params.rate_hz) if not self.simulation else None

        for step in range(steps):
            t = step * self.dt
            if not self.simulation:
                if self.last_ros_limo_pose is None or self.last_ros_drone_pose is None:
                    ros_rate.sleep()
                    continue

            commands = self.compute_commands(t)
            if self.simulation:
                self.step_simulation(commands)
            else:
                self.publish_ros_commands(commands)
            self.log_step(t, commands)

            if self.real_time:
                time.sleep(self.dt)
            elif ros_rate is not None:
                ros_rate.sleep()

        if not self.simulation:
            self.publish_zero_commands()


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Controlador de estrutura virtual LIMO + Bebop 2")
    parser.add_argument("--ros", action="store_true", help="usa ROS1 ao inves de simulacao")
    parser.add_argument("--real-time", action="store_true", help="espera o periodo de amostragem tambem em simulacao")
    parser.add_argument("--duration", type=float, default=40.0, help="tempo total da execucao em segundos")
    parser.add_argument(
        "--save",
        type=str,
        default=None,
        help="arquivo .npz de saida; se omitido, cria uma pasta unica em resultados/ com o .npz e a analise",
    )
    parser.add_argument("--no-save", action="store_true", help="nao salva o log")
    parser.add_argument("--plot", action="store_true", help="mostra graficos ao final")
    parser.add_argument("--no-analyze", action="store_true", help="nao gera metrics.csv nem os graficos automaticamente")
    parser.add_argument("--takeoff", action="store_true", help="envia takeoff ao drone no modo ROS")
    parser.add_argument("--limo-ns", type=str, default="L1", help="namespace ROS do LIMO")
    parser.add_argument("--drone-ns", type=str, default="B1", help="namespace ROS do drone")
    return parser


def main() -> None:
    args = build_parser().parse_args()
    controller = FormationController(
        simulation=not args.ros,
        real_time=args.real_time,
        ros_namespace_limo=args.limo_ns,
        ros_namespace_drone=args.drone_ns,
    )
    controller.run(duration=args.duration, takeoff=args.takeoff)
    if not args.no_save:
        output_dir, log_path = resolve_output_paths(args.save)
        controller.save_logs(log_path)
        print(f"Log salvo em {log_path}")
        if not args.no_analyze:
            analyzed_dir, metrics_path = analyze_log(log_path, output_dir=output_dir, show=False)
            print(f"Analise salva em {analyzed_dir}")
            print(f"Metricas salvas em {metrics_path}")
    if args.plot:
        controller.plot_logs()


if __name__ == "__main__":
    main()
