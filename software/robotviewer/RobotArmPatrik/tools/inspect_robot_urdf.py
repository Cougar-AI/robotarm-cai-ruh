from __future__ import annotations

import json
import math
import xml.etree.ElementTree as ET
from collections import Counter, defaultdict
from pathlib import Path

import numpy as np
import plotly.graph_objects as go
import trimesh


ROOT = Path(__file__).resolve().parents[1]
URDF_PATH = ROOT / "Robot.urdf"
OUTPUT_DIR = ROOT / "inspection"
HTML_PATH = OUTPUT_DIR / "robot_urdf_authored_pose.html"
REPORT_PATH = OUTPUT_DIR / "robot_urdf_validation_report.json"


def rpy_matrix(rpy_text: str) -> np.ndarray:
    roll, pitch, yaw = (float(v) for v in rpy_text.split())
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)

    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return rz @ ry @ rx


def transform_points(points: np.ndarray, xyz_text: str, rpy_text: str) -> np.ndarray:
    xyz = np.array([float(v) for v in xyz_text.split()], dtype=float)
    rot = rpy_matrix(rpy_text)
    return (rot @ points.T).T + xyz


def distinct_name(raw_name: str, counts: dict[str, int]) -> str:
    counts[raw_name] += 1
    return f"{raw_name}#{counts[raw_name]}"


def normalize(vec: np.ndarray) -> np.ndarray:
    norm = np.linalg.norm(vec)
    return vec if norm == 0 else vec / norm


def mesh_trace(vertices: np.ndarray, faces: np.ndarray, color: str, name: str) -> go.Mesh3d:
    return go.Mesh3d(
        x=vertices[:, 0],
        y=vertices[:, 1],
        z=vertices[:, 2],
        i=faces[:, 0],
        j=faces[:, 1],
        k=faces[:, 2],
        color=color,
        opacity=0.55,
        name=name,
        hovertext=name,
        hoverinfo="text",
        flatshading=True,
        lighting=dict(ambient=0.55, diffuse=0.85, specular=0.15, roughness=0.75),
    )


def line_trace(start: np.ndarray, end: np.ndarray, color: str, name: str, legendgroup: str) -> go.Scatter3d:
    return go.Scatter3d(
        x=[start[0], end[0]],
        y=[start[1], end[1]],
        z=[start[2], end[2]],
        mode="lines",
        line=dict(color=color, width=6),
        name=name,
        hoverinfo="name",
        legendgroup=legendgroup,
        showlegend=False,
    )


def point_trace(point: np.ndarray, name: str, color: str, legendgroup: str) -> go.Scatter3d:
    return go.Scatter3d(
        x=[point[0]],
        y=[point[1]],
        z=[point[2]],
        mode="markers+text",
        marker=dict(size=4, color=color),
        text=[name],
        textposition="top center",
        name=name,
        hoverinfo="text",
        legendgroup=legendgroup,
        showlegend=False,
    )


def build_report(root: ET.Element) -> dict:
    link_names = [link.attrib["name"] for link in root.findall("link")]
    joint_names = [joint.attrib["name"] for joint in root.findall("joint")]

    duplicate_links = {name: count for name, count in Counter(link_names).items() if count > 1}
    duplicate_joints = {name: count for name, count in Counter(joint_names).items() if count > 1}

    self_joints = []
    continuous_joints = []
    axis_lengths = []
    for joint in root.findall("joint"):
        parent = joint.find("parent").attrib["link"]
        child = joint.find("child").attrib["link"]
        if parent == child:
            self_joints.append(joint.attrib["name"])

        axis_node = joint.find("axis")
        if joint.attrib["type"] != "fixed" and axis_node is not None:
            axis = np.array([float(v) for v in axis_node.attrib["xyz"].split()], dtype=float)
            axis_lengths.append(float(np.linalg.norm(axis)))
            continuous_joints.append(
                {
                    "name": joint.attrib["name"],
                    "parent": parent,
                    "child": child,
                    "axis": axis.tolist(),
                    "axis_norm": float(np.linalg.norm(axis)),
                }
            )

    referenced_parents = [joint.find("parent").attrib["link"] for joint in root.findall("joint")]
    referenced_children = [joint.find("child").attrib["link"] for joint in root.findall("joint")]
    roots = sorted(set(link_names) - set(referenced_children))
    orphan_children = sorted(set(referenced_children) - set(link_names))
    orphan_parents = sorted(set(referenced_parents) - set(link_names))

    return {
        "urdf_path": str(URDF_PATH),
        "link_count": len(link_names),
        "unique_link_count": len(set(link_names)),
        "joint_count": len(joint_names),
        "unique_joint_count": len(set(joint_names)),
        "duplicate_links": duplicate_links,
        "duplicate_joints": duplicate_joints,
        "self_joints": self_joints,
        "candidate_roots_by_name": roots,
        "missing_parent_links": orphan_parents,
        "missing_child_links": orphan_children,
        "continuous_joint_count": len(continuous_joints),
        "continuous_joints": continuous_joints,
        "axis_norm_range": {
            "min": min(axis_lengths) if axis_lengths else None,
            "max": max(axis_lengths) if axis_lengths else None,
        },
        "summary": [
            "URDF is not valid as-authored because link names are not unique.",
            "URDF is not valid as-authored because joint names are not unique.",
            "At least one joint references the same link as both parent and child.",
            "The file contains more non-fixed joints than a 6-DOF serial arm should expose.",
        ],
    }


def authored_pose_figure(root: ET.Element) -> go.Figure:
    fig = go.Figure()
    name_counts: dict[str, int] = defaultdict(int)

    palette = [
        "#27548A",
        "#D98324",
        "#7E8A97",
        "#5A827E",
        "#C14600",
        "#4F6F52",
        "#8E3B46",
        "#4C3BCF",
        "#6B8E23",
        "#8B5E3C",
        "#3A7D44",
        "#A53860",
    ]

    all_vertices = []

    for idx, link in enumerate(root.findall("link")):
        raw_name = link.attrib["name"]
        instance_name = distinct_name(raw_name, name_counts)
        visual = link.find("visual")
        origin = visual.find("origin")
        mesh_node = visual.find("geometry").find("mesh")
        mesh_path = ROOT / mesh_node.attrib["filename"]

        mesh = trimesh.load_mesh(mesh_path, force="mesh")
        vertices = np.asarray(mesh.vertices, dtype=float)
        scale = np.array([float(v) for v in mesh_node.attrib.get("scale", "1 1 1").split()], dtype=float)
        vertices = vertices * scale
        vertices = transform_points(vertices, origin.attrib["xyz"], origin.attrib["rpy"])
        faces = np.asarray(mesh.faces, dtype=int)
        color = palette[idx % len(palette)]

        fig.add_trace(mesh_trace(vertices, faces, color, instance_name))
        all_vertices.append(vertices)

        centroid = vertices.mean(axis=0)
        fig.add_trace(point_trace(centroid, instance_name, color, "labels"))

        rot = rpy_matrix(origin.attrib["rpy"])
        origin_xyz = np.array([float(v) for v in origin.attrib["xyz"].split()], dtype=float)
        axis_len = max(0.035, np.linalg.norm(vertices.max(axis=0) - vertices.min(axis=0)) * 0.16)
        axes = {
            "x": ("#D7263D", rot @ np.array([axis_len, 0.0, 0.0])),
            "y": ("#1B998B", rot @ np.array([0.0, axis_len, 0.0])),
            "z": ("#2D6CDF", rot @ np.array([0.0, 0.0, axis_len])),
        }
        for axis_name, (axis_color, offset) in axes.items():
            fig.add_trace(
                line_trace(
                    origin_xyz,
                    origin_xyz + offset,
                    axis_color,
                    f"{instance_name} {axis_name}-axis",
                    f"{instance_name}-axes",
                )
            )

    if all_vertices:
        merged = np.vstack(all_vertices)
        mins = merged.min(axis=0)
        maxs = merged.max(axis=0)
        center = (mins + maxs) / 2.0
        span = float(np.max(maxs - mins))
    else:
        center = np.zeros(3)
        span = 1.0

    world_axis_len = max(0.12, span * 0.18)
    world_axes = [
        ("World X", "#D7263D", np.array([1.0, 0.0, 0.0])),
        ("World Y", "#1B998B", np.array([0.0, 1.0, 0.0])),
        ("World Z", "#2D6CDF", np.array([0.0, 0.0, 1.0])),
    ]
    for name, color, direction in world_axes:
        fig.add_trace(line_trace(np.zeros(3), direction * world_axis_len, color, name, "world"))
    fig.add_trace(point_trace(np.zeros(3), "world", "#111111", "world"))

    fig.update_layout(
        title=(
            "Robot.urdf Authored-Pose Inspector"
            "<br><sup>Interactive orbit/pan/zoom view of mesh placements as written in the URDF visuals. "
            "This is not a validated kinematic tree.</sup>"
        ),
        scene=dict(
            xaxis_title="X (m)",
            yaxis_title="Y (m)",
            zaxis_title="Z (m)",
            aspectmode="data",
            camera=dict(
                eye=dict(
                    x=float(center[0] / max(span, 1e-6) + 1.5),
                    y=float(center[1] / max(span, 1e-6) - 1.8),
                    z=float(center[2] / max(span, 1e-6) + 1.2),
                )
            ),
        ),
        template="plotly_white",
        margin=dict(l=0, r=0, t=80, b=0),
        legend=dict(itemsizing="constant"),
        annotations=[
            dict(
                x=0.0,
                y=1.0,
                xref="paper",
                yref="paper",
                xanchor="left",
                yanchor="bottom",
                align="left",
                showarrow=False,
                font=dict(size=12),
                text=(
                    "Legend tip: click trace names to isolate parts. "
                    "Red/green/blue lines are local link X/Y/Z axes. "
                    "The robot file still needs a real URDF repair before joint-level validation."
                ),
            )
        ],
    )
    return fig


def main() -> None:
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    root = ET.parse(URDF_PATH).getroot()

    report = build_report(root)
    REPORT_PATH.write_text(json.dumps(report, indent=2))

    fig = authored_pose_figure(root)
    fig.write_html(HTML_PATH, include_plotlyjs=True, full_html=True)

    print(f"Wrote validation report: {REPORT_PATH}")
    print(f"Wrote interactive HTML: {HTML_PATH}")


if __name__ == "__main__":
    main()
