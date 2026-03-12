#!/usr/bin/env python3
"""
inspect_RobotDescription_structure.py

Lightweight interactive robot structure inspector for:
- MJCF / MuJoCo XML
- URDF
- Xacro (best-effort, via xacro if installed)

Main design goal:
Avoid Safari / Plotly freezes by defaulting to LIGHTWEIGHT mesh previews.

What this generates:
1) JSON structural report
2) Self-contained HTML viewer with:
   - kinematic tree
   - 3D link frames / joint axes
   - mesh previews (default: decimated point cloud, not full mesh)
   - cross references between tree / tables / 3D objects
   - XML source viewer at the bottom

Important:
- Default mesh mode is "pointcloud" for performance.
- Full triangle mesh mode is optional via --mesh-mode full, but may freeze Safari.
"""

from __future__ import annotations

import argparse
import html
import json
import math
import os
import re
import shutil
import subprocess
import sys
import uuid
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field, asdict
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import numpy as np

try:
    import trimesh
except Exception:
    trimesh = None


# ============================================================
# Math helpers
# ============================================================

def normalize(v: np.ndarray, eps: float = 1e-12) -> np.ndarray:
    n = np.linalg.norm(v)
    if n < eps:
        return v.copy()
    return v / n


def rpy_to_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return rz @ ry @ rx


def axis_angle_to_matrix(axis: np.ndarray, angle: float) -> np.ndarray:
    axis = normalize(axis)
    x, y, z = axis
    c = math.cos(angle)
    s = math.sin(angle)
    C = 1 - c
    return np.array([
        [x * x * C + c,     x * y * C - z * s, x * z * C + y * s],
        [y * x * C + z * s, y * y * C + c,     y * z * C - x * s],
        [z * x * C - y * s, z * y * C + x * s, z * z * C + c],
    ])


def quat_wxyz_to_matrix(q: List[float]) -> np.ndarray:
    w, x, y, z = q
    n = math.sqrt(w*w + x*x + y*y + z*z)
    if n == 0:
        return np.eye(3)
    w, x, y, z = w/n, x/n, y/n, z/n
    return np.array([
        [1 - 2*(y*y + z*z),     2*(x*y - z*w),     2*(x*z + y*w)],
        [2*(x*y + z*w),         1 - 2*(x*x + z*z), 2*(y*z - x*w)],
        [2*(x*z - y*w),         2*(y*z + x*w),     1 - 2*(x*x + y*y)],
    ])


def matrix_to_quat_wxyz(R: np.ndarray) -> List[float]:
    m = R
    t = np.trace(m)
    if t > 0:
        S = math.sqrt(t + 1.0) * 2
        w = 0.25 * S
        x = (m[2, 1] - m[1, 2]) / S
        y = (m[0, 2] - m[2, 0]) / S
        z = (m[1, 0] - m[0, 1]) / S
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        S = math.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2
        w = (m[2, 1] - m[1, 2]) / S
        x = 0.25 * S
        y = (m[0, 1] + m[1, 0]) / S
        z = (m[0, 2] + m[2, 0]) / S
    elif m[1, 1] > m[2, 2]:
        S = math.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2
        w = (m[0, 2] - m[2, 0]) / S
        x = (m[0, 1] + m[1, 0]) / S
        y = 0.25 * S
        z = (m[1, 2] + m[2, 1]) / S
    else:
        S = math.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2
        w = (m[1, 0] - m[0, 1]) / S
        x = (m[0, 2] + m[2, 0]) / S
        y = (m[1, 2] + m[2, 1]) / S
        z = 0.25 * S
    return [float(w), float(x), float(y), float(z)]


def make_transform(R: Optional[np.ndarray] = None, t: Optional[np.ndarray] = None) -> np.ndarray:
    T = np.eye(4)
    if R is not None:
        T[:3, :3] = R
    if t is not None:
        T[:3, 3] = t
    return T


def transform_points(T: np.ndarray, pts: np.ndarray) -> np.ndarray:
    if pts.size == 0:
        return pts
    hom = np.hstack([pts, np.ones((pts.shape[0], 1))])
    out = (T @ hom.T).T
    return out[:, :3]


def parse_floats(s: Optional[str], n: Optional[int] = None, default: Optional[List[float]] = None) -> List[float]:
    if s is None:
        return (default or []).copy()
    vals = [float(x) for x in s.replace(",", " ").split()]
    if n is not None and len(vals) != n:
        if len(vals) == 0 and default is not None:
            return default.copy()
        raise ValueError(f"Expected {n} floats, got {len(vals)} from {s!r}")
    return vals


def safe_name(name: str) -> str:
    return re.sub(r"[^a-zA-Z0-9_.-]", "_", name)


def deterministic_color(name: str) -> str:
    import hashlib
    h = hashlib.md5(name.encode("utf-8")).hexdigest()
    r = int(h[0:2], 16)
    g = int(h[2:4], 16)
    b = int(h[4:6], 16)
    # brighten a bit
    r = (r + 160) // 2
    g = (g + 160) // 2
    b = (b + 160) // 2
    return f"rgb({r},{g},{b})"


# ============================================================
# Data model
# ============================================================

@dataclass
class MeshRef:
    mesh_name: str
    mesh_file: Optional[str]
    visual_or_collision: str
    geom_type: str
    local_pos: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    local_quat_wxyz: List[float] = field(default_factory=lambda: [1.0, 0.0, 0.0, 0.0])
    world_pos: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    world_quat_wxyz: List[float] = field(default_factory=lambda: [1.0, 0.0, 0.0, 0.0])
    material: Optional[str] = None


@dataclass
class JointInfo:
    name: str
    joint_type: str
    parent: Optional[str]
    child: Optional[str]
    axis_local: List[float] = field(default_factory=lambda: [0.0, 0.0, 1.0])
    axis_world: List[float] = field(default_factory=lambda: [0.0, 0.0, 1.0])
    origin_world: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    origin_local: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    rpy_local: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    quat_world_wxyz: List[float] = field(default_factory=lambda: [1.0, 0.0, 0.0, 0.0])
    limits: Dict[str, Any] = field(default_factory=dict)
    extra: Dict[str, Any] = field(default_factory=dict)


@dataclass
class LinkInfo:
    name: str
    parent: Optional[str]
    world_pos: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    world_quat_wxyz: List[float] = field(default_factory=lambda: [1.0, 0.0, 0.0, 0.0])
    meshes: List[MeshRef] = field(default_factory=list)
    joints_out: List[str] = field(default_factory=list)
    child_links: List[str] = field(default_factory=list)
    inertial: Dict[str, Any] = field(default_factory=dict)
    extra: Dict[str, Any] = field(default_factory=dict)


# ============================================================
# Inspector
# ============================================================

class RobotStructureInspector:
    def __init__(
        self,
        xml_path: Path,
        mesh_mode: str = "pointcloud",
        max_points_per_mesh: int = 1500,
        max_faces_per_mesh: int = 4000,
    ) -> None:
        self.xml_path = xml_path.resolve()
        self.base_dir = self.xml_path.parent
        self.mesh_mode = mesh_mode
        self.max_points_per_mesh = max_points_per_mesh
        self.max_faces_per_mesh = max_faces_per_mesh

        self.kind = "unknown"
        self.model_name = self.xml_path.stem
        self.links: Dict[str, LinkInfo] = {}
        self.joints: Dict[str, JointInfo] = {}
        self.assets_meshes: Dict[str, str] = {}
        self.warnings: List[str] = []
        self.notes: List[str] = []

    def load_xml_root(self) -> ET.Element:
        if self.xml_path.suffix.lower() == ".xacro":
            return self._load_xacro()
        return ET.parse(self.xml_path).getroot()

    def _load_xacro(self) -> ET.Element:
        xacro_bin = shutil.which("xacro")
        if not xacro_bin:
            self.warnings.append("xacro file provided but xacro executable not found. Parsing raw XML.")
            return ET.parse(self.xml_path).getroot()
        try:
            proc = subprocess.run(
                [xacro_bin, str(self.xml_path)],
                capture_output=True,
                text=True,
                check=True,
            )
            return ET.fromstring(proc.stdout)
        except subprocess.CalledProcessError as exc:
            self.warnings.append(f"xacro expansion failed. Parsing raw XML. stderr={exc.stderr[:300]}")
            return ET.parse(self.xml_path).getroot()

    def inspect(self) -> Dict[str, Any]:
        root = self.load_xml_root()
        self.model_name = root.attrib.get("name", self.model_name)

        if root.tag == "mujoco":
            self.kind = "mjcf"
            self._parse_mjcf(root)
        elif root.tag == "robot":
            self.kind = "urdf"
            self._parse_urdf(root)
        else:
            self.kind = f"xml:{root.tag}"
            self.warnings.append(f"Unknown root tag {root.tag}, trying URDF-like parse.")
            self._parse_urdf(root)

        mesh_render_items = self._build_mesh_render_items()

        report = {
            "source_file": str(self.xml_path),
            "model_name": self.model_name,
            "kind": self.kind,
            "link_count": len(self.links),
            "joint_count": len(self.joints),
            "mesh_asset_count": len(self.assets_meshes),
            "links": {k: asdict(v) for k, v in self.links.items()},
            "joints": {k: asdict(v) for k, v in self.joints.items()},
            "mesh_assets": self.assets_meshes,
            "mesh_render_items": mesh_render_items,
            "warnings": self.warnings,
            "notes": self.notes,
            "xml_source": self.xml_path.read_text(encoding="utf-8", errors="replace"),
            "viewer_settings": {
                "mesh_mode": self.mesh_mode,
                "max_points_per_mesh": self.max_points_per_mesh,
                "max_faces_per_mesh": self.max_faces_per_mesh,
            },
        }
        report["notes"].extend(summarize_industry_patterns(report))
        return report

    # --------------------------------------------------------
    # MJCF parsing
    # --------------------------------------------------------

    def _parse_mjcf(self, root: ET.Element) -> None:
        compiler = root.find("compiler")
        meshdir = compiler.attrib.get("meshdir", "") if compiler is not None else ""
        self.meshdir = (self.base_dir / meshdir).resolve() if meshdir else self.base_dir

        asset = root.find("asset")
        if asset is not None:
            for mesh in asset.findall("mesh"):
                file_attr = mesh.attrib.get("file")
                if not file_attr:
                    continue
                mesh_name = mesh.attrib.get("name") or Path(file_attr).stem
                self.assets_meshes[mesh_name] = str((self.meshdir / file_attr).resolve())

        worldbody = root.find("worldbody")
        if worldbody is None:
            self.warnings.append("MJCF has no worldbody.")
            return

        for body in worldbody.findall("body"):
            self._walk_mjcf_body(body, parent_link=None, parent_T=np.eye(4))

    def _walk_mjcf_body(self, body: ET.Element, parent_link: Optional[str], parent_T: np.ndarray) -> None:
        name = body.attrib.get("name", f"unnamed_body_{uuid.uuid4().hex[:8]}")
        pos = np.array(parse_floats(body.attrib.get("pos"), default=[0.0, 0.0, 0.0]), dtype=float)

        if "quat" in body.attrib:
            R = quat_wxyz_to_matrix(parse_floats(body.attrib["quat"], 4))
        elif "euler" in body.attrib:
            eul = parse_floats(body.attrib["euler"], 3)
            R = rpy_to_matrix(eul[0], eul[1], eul[2])
        elif "axisangle" in body.attrib:
            aa = parse_floats(body.attrib["axisangle"], 4)
            R = axis_angle_to_matrix(np.array(aa[:3], dtype=float), aa[3])
        else:
            R = np.eye(3)

        T_local = make_transform(R, pos)
        T_world = parent_T @ T_local

        link = LinkInfo(
            name=name,
            parent=parent_link,
            world_pos=T_world[:3, 3].tolist(),
            world_quat_wxyz=matrix_to_quat_wxyz(T_world[:3, :3]),
        )

        inertial = body.find("inertial")
        if inertial is not None:
            link.inertial = dict(inertial.attrib)

        for geom in body.findall("geom"):
            mesh_name = geom.attrib.get("mesh")
            geom_type = geom.attrib.get("type", "mesh" if mesh_name else "unknown")
            role = self._mjcf_geom_role(geom)

            local_pos = np.array(parse_floats(geom.attrib.get("pos"), default=[0.0, 0.0, 0.0]), dtype=float)

            if "quat" in geom.attrib:
                gR = quat_wxyz_to_matrix(parse_floats(geom.attrib["quat"], 4))
            elif "euler" in geom.attrib:
                eul = parse_floats(geom.attrib["euler"], 3)
                gR = rpy_to_matrix(eul[0], eul[1], eul[2])
            elif "axisangle" in geom.attrib:
                aa = parse_floats(geom.attrib["axisangle"], 4)
                gR = axis_angle_to_matrix(np.array(aa[:3], dtype=float), aa[3])
            else:
                gR = np.eye(3)

            Tg = T_world @ make_transform(gR, local_pos)
            mesh_file = self.assets_meshes.get(mesh_name) if mesh_name else None

            link.meshes.append(
                MeshRef(
                    mesh_name=mesh_name or geom.attrib.get("name", geom_type),
                    mesh_file=mesh_file,
                    visual_or_collision=role,
                    geom_type=geom_type,
                    local_pos=local_pos.tolist(),
                    local_quat_wxyz=matrix_to_quat_wxyz(gR),
                    world_pos=Tg[:3, 3].tolist(),
                    world_quat_wxyz=matrix_to_quat_wxyz(Tg[:3, :3]),
                    material=geom.attrib.get("material"),
                )
            )

        body_joints = body.findall("joint")

        if parent_link is not None:
            if len(body_joints) == 0:
                joint_name = f"{parent_link}__to__{name}__fixed"
                self.joints[joint_name] = JointInfo(
                    name=joint_name,
                    joint_type="fixed",
                    parent=parent_link,
                    child=name,
                    origin_local=pos.tolist(),
                    origin_world=T_world[:3, 3].tolist(),
                    quat_world_wxyz=matrix_to_quat_wxyz(T_world[:3, :3]),
                )
                link.extra["implicit_fixed_joint"] = True
            elif len(body_joints) == 1:
                joint = body_joints[0]
                axis_local = np.array(parse_floats(joint.attrib.get("axis"), default=[0.0, 0.0, 1.0]), dtype=float)
                axis_world = T_world[:3, :3] @ normalize(axis_local)
                joint_name = joint.attrib.get("name", f"{parent_link}__to__{name}")
                self.joints[joint_name] = JointInfo(
                    name=joint_name,
                    joint_type=joint.attrib.get("type", self._infer_mjcf_joint_type(joint)),
                    parent=parent_link,
                    child=name,
                    axis_local=axis_local.tolist(),
                    axis_world=axis_world.tolist(),
                    origin_local=pos.tolist(),
                    origin_world=T_world[:3, 3].tolist(),
                    quat_world_wxyz=matrix_to_quat_wxyz(T_world[:3, :3]),
                    limits={k: joint.attrib[k] for k in ["range", "actuatorfrcrange", "frictionloss"] if k in joint.attrib},
                    extra={k: v for k, v in joint.attrib.items() if k not in {"name", "axis", "type", "range"}},
                )
            else:
                self.warnings.append(
                    f"Body {name} has {len(body_joints)} joints. Valid MJCF, but not one-link/one-joint URDF style."
                )
                for idx, joint in enumerate(body_joints):
                    axis_local = np.array(parse_floats(joint.attrib.get("axis"), default=[0.0, 0.0, 1.0]), dtype=float)
                    axis_world = T_world[:3, :3] @ normalize(axis_local)
                    joint_name = joint.attrib.get("name", f"{parent_link}__to__{name}__{idx}")
                    self.joints[joint_name] = JointInfo(
                        name=joint_name,
                        joint_type=joint.attrib.get("type", self._infer_mjcf_joint_type(joint)),
                        parent=parent_link,
                        child=name,
                        axis_local=axis_local.tolist(),
                        axis_world=axis_world.tolist(),
                        origin_local=pos.tolist(),
                        origin_world=T_world[:3, 3].tolist(),
                        quat_world_wxyz=matrix_to_quat_wxyz(T_world[:3, :3]),
                        limits={k: joint.attrib[k] for k in ["range", "actuatorfrcrange", "frictionloss"] if k in joint.attrib},
                        extra={k: v for k, v in joint.attrib.items() if k not in {"name", "axis", "type", "range"}},
                    )
        else:
            if body.find("freejoint") is not None:
                link.extra["has_freejoint"] = True

        self.links[name] = link

        if parent_link is not None and parent_link in self.links:
            self.links[parent_link].child_links.append(name)
            for joint_name, joint in self.joints.items():
                if joint.child == name and joint.parent == parent_link:
                    self.links[parent_link].joints_out.append(joint_name)

        for child in body.findall("body"):
            self._walk_mjcf_body(child, parent_link=name, parent_T=T_world)

    def _mjcf_geom_role(self, geom: ET.Element) -> str:
        geom_class = geom.attrib.get("class", "")
        group = geom.attrib.get("group")
        contype = geom.attrib.get("contype")
        conaffinity = geom.attrib.get("conaffinity")
        if geom_class == "visual":
            return "visual"
        if geom_class == "collision":
            return "collision"
        if group == "2" or (contype == "0" and conaffinity == "0"):
            return "visual"
        return "collision"

    def _infer_mjcf_joint_type(self, joint: ET.Element) -> str:
        return joint.attrib.get("type", "hinge")

    # --------------------------------------------------------
    # URDF parsing
    # --------------------------------------------------------

    def _parse_urdf(self, root: ET.Element) -> None:
        for link_el in root.findall("link"):
            name = link_el.attrib.get("name", f"unnamed_link_{uuid.uuid4().hex[:8]}")
            link = LinkInfo(name=name, parent=None)

            inertial = link_el.find("inertial")
            if inertial is not None:
                link.inertial = self._collect_urdf_inertial(inertial)

            for tag, role in [("visual", "visual"), ("collision", "collision")]:
                for ge in link_el.findall(tag):
                    origin_xyz, origin_rpy, origin_quat = self._urdf_origin(ge.find("origin"))
                    geom = ge.find("geometry")

                    mesh_name = None
                    mesh_file = None
                    geom_type = "unknown"

                    if geom is not None:
                        mesh = geom.find("mesh")
                        if mesh is not None:
                            filename = mesh.attrib.get("filename") or mesh.attrib.get("url")
                            mesh_file = self._resolve_mesh_uri(filename) if filename else None
                            mesh_name = Path(filename).stem if filename else f"{name}_{role}_mesh"
                            geom_type = "mesh"
                        else:
                            for prim in ["box", "cylinder", "sphere"]:
                                if geom.find(prim) is not None:
                                    geom_type = prim
                                    mesh_name = f"{name}_{role}_{prim}"
                                    break

                    link.meshes.append(
                        MeshRef(
                            mesh_name=mesh_name or f"{name}_{role}",
                            mesh_file=mesh_file,
                            visual_or_collision=role,
                            geom_type=geom_type,
                            local_pos=origin_xyz,
                            local_quat_wxyz=origin_quat,
                        )
                    )

            self.links[name] = link

        for joint_el in root.findall("joint"):
            jname = joint_el.attrib.get("name", f"unnamed_joint_{uuid.uuid4().hex[:8]}")
            jtype = joint_el.attrib.get("type", "fixed")

            parent_el = joint_el.find("parent")
            child_el = joint_el.find("child")

            parent_name = parent_el.attrib.get("link") if parent_el is not None else None
            child_name = child_el.attrib.get("link") if child_el is not None else None

            xyz, rpy, quat = self._urdf_origin(joint_el.find("origin"))
            axis = parse_floats(
                joint_el.find("axis").attrib.get("xyz") if joint_el.find("axis") is not None else None,
                default=[0.0, 0.0, 1.0],
            )

            limits = {}
            lim = joint_el.find("limit")
            if lim is not None:
                limits = dict(lim.attrib)

            self.joints[jname] = JointInfo(
                name=jname,
                joint_type=jtype,
                parent=parent_name,
                child=child_name,
                axis_local=axis,
                origin_local=xyz,
                rpy_local=rpy,
                quat_world_wxyz=quat,
                limits=limits,
            )

            if child_name in self.links:
                self.links[child_name].parent = parent_name
            if parent_name in self.links and child_name in self.links:
                self.links[parent_name].child_links.append(child_name)
                self.links[parent_name].joints_out.append(jname)

        roots = [name for name, link in self.links.items() if link.parent is None]
        if not roots:
            self.warnings.append("No URDF root link detected.")
            return

        for root_name in roots:
            self._propagate_urdf(root_name, np.eye(4))

    def _collect_urdf_inertial(self, inertial_el: ET.Element) -> Dict[str, Any]:
        out: Dict[str, Any] = {}
        origin = inertial_el.find("origin")
        if origin is not None:
            xyz, rpy, quat = self._urdf_origin(origin)
            out["origin_xyz"] = xyz
            out["origin_rpy"] = rpy
            out["origin_quat_wxyz"] = quat

        mass = inertial_el.find("mass")
        inertia = inertial_el.find("inertia")
        if mass is not None:
            out["mass"] = mass.attrib.get("value")
        if inertia is not None:
            out["inertia"] = dict(inertia.attrib)
        return out

    def _urdf_origin(self, origin_el: Optional[ET.Element]) -> Tuple[List[float], List[float], List[float]]:
        if origin_el is None:
            return [0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]
        xyz = parse_floats(origin_el.attrib.get("xyz"), default=[0.0, 0.0, 0.0])
        rpy = parse_floats(origin_el.attrib.get("rpy"), default=[0.0, 0.0, 0.0])
        quat = matrix_to_quat_wxyz(rpy_to_matrix(*rpy))
        return xyz, rpy, quat

    def _resolve_mesh_uri(self, uri: str) -> Optional[str]:
        if not uri:
            return None
        if uri.startswith("package://"):
            rel = uri.split("package://", 1)[1]
            parts = rel.split("/", 1)
            rel_path = parts[1] if len(parts) > 1 else parts[0]
            return str((self.base_dir / rel_path).resolve())
        if uri.startswith("file://"):
            return str(Path(uri[7:]).resolve())
        return str((self.base_dir / uri).resolve())

    def _propagate_urdf(self, link_name: str, parent_T: np.ndarray) -> None:
        link = self.links[link_name]
        link.world_pos = parent_T[:3, 3].tolist()
        link.world_quat_wxyz = matrix_to_quat_wxyz(parent_T[:3, :3])

        for mesh in link.meshes:
            R = quat_wxyz_to_matrix(mesh.local_quat_wxyz)
            Tm = parent_T @ make_transform(R, np.array(mesh.local_pos))
            mesh.world_pos = Tm[:3, 3].tolist()
            mesh.world_quat_wxyz = matrix_to_quat_wxyz(Tm[:3, :3])

        for joint_name in link.joints_out:
            joint = self.joints[joint_name]
            child_name = joint.child
            if child_name is None or child_name not in self.links:
                continue

            R = rpy_to_matrix(*joint.rpy_local)
            Tj = parent_T @ make_transform(R, np.array(joint.origin_local))
            joint.origin_world = Tj[:3, 3].tolist()
            joint.quat_world_wxyz = matrix_to_quat_wxyz(Tj[:3, :3])
            joint.axis_world = (Tj[:3, :3] @ normalize(np.array(joint.axis_local, dtype=float))).tolist()

            self._propagate_urdf(child_name, Tj)

    # --------------------------------------------------------
    # Mesh preview generation
    # --------------------------------------------------------

    def _build_mesh_render_items(self) -> List[Dict[str, Any]]:
        items: List[Dict[str, Any]] = []

        if trimesh is None:
            self.warnings.append("trimesh is not installed; 3D mesh previews disabled.")
            return items

        for link_name, link in self.links.items():
            for mesh in link.meshes:
                if mesh.geom_type != "mesh" or not mesh.mesh_file:
                    continue

                p = Path(mesh.mesh_file)
                if not p.exists():
                    self.warnings.append(f"Missing mesh file: {p}")
                    continue

                try:
                    raw = trimesh.load_mesh(str(p), force="mesh")
                    if raw is None:
                        continue

                    if isinstance(raw, trimesh.Scene):
                        if not raw.geometry:
                            continue
                        raw = trimesh.util.concatenate(tuple(raw.geometry.values()))

                    if not isinstance(raw, trimesh.Trimesh):
                        continue

                    if raw.vertices is None or len(raw.vertices) == 0:
                        continue

                    local_R = quat_wxyz_to_matrix(mesh.local_quat_wxyz)
                    local_T = make_transform(local_R, np.array(mesh.local_pos))
                    verts = transform_points(local_T, np.asarray(raw.vertices))

                    color = deterministic_color(f"{link_name}:{mesh.mesh_name}:{mesh.visual_or_collision}")

                    item = {
                        "id": f"{safe_name(link_name)}::{safe_name(mesh.mesh_name)}::{mesh.visual_or_collision}",
                        "link_name": link_name,
                        "mesh_name": mesh.mesh_name,
                        "mesh_file": mesh.mesh_file,
                        "role": mesh.visual_or_collision,
                        "mode": self.mesh_mode,
                        "color": color,
                    }

                    if self.mesh_mode == "pointcloud":
                        pts = self._sample_points_from_mesh(raw, self.max_points_per_mesh)
                        pts = transform_points(local_T, pts)
                        item["points"] = pts.tolist()
                    elif self.mesh_mode == "wireframe":
                        pts, segs = self._extract_wireframe(raw, self.max_faces_per_mesh)
                        pts = transform_points(local_T, pts)
                        item["points"] = pts.tolist()
                        item["segments"] = segs
                    else:
                        verts2, faces2 = self._extract_mesh_faces(raw, self.max_faces_per_mesh)
                        verts2 = transform_points(local_T, verts2)
                        item["vertices"] = verts2.tolist()
                        item["faces"] = faces2.tolist()

                    items.append(item)

                except Exception as exc:
                    self.warnings.append(f"Failed mesh preview for {p.name}: {exc}")

        if self.mesh_mode == "full":
            self.warnings.append(
                "mesh_mode=full uses triangle meshes and may freeze Safari on heavy models. pointcloud is safer."
            )

        return items

    def _sample_points_from_mesh(self, mesh: "trimesh.Trimesh", max_points: int) -> np.ndarray:
        n_faces = len(mesh.faces) if mesh.faces is not None else 0
        if n_faces == 0:
            verts = np.asarray(mesh.vertices)
            if len(verts) <= max_points:
                return verts
            idx = np.linspace(0, len(verts) - 1, max_points).astype(int)
            return verts[idx]

        n = min(max_points, max(300, n_faces // 4))
        try:
            pts, _ = trimesh.sample.sample_surface(mesh, n)
            return pts
        except Exception:
            verts = np.asarray(mesh.vertices)
            if len(verts) <= max_points:
                return verts
            idx = np.linspace(0, len(verts) - 1, max_points).astype(int)
            return verts[idx]

    def _extract_wireframe(self, mesh: "trimesh.Trimesh", max_faces: int) -> Tuple[np.ndarray, List[List[int]]]:
        verts = np.asarray(mesh.vertices)
        faces = np.asarray(mesh.faces)

        if len(faces) > max_faces:
            idx = np.linspace(0, len(faces) - 1, max_faces).astype(int)
            faces = faces[idx]

        # unique undirected edges from chosen faces
        edge_set = set()
        for f in faces:
            a, b, c = int(f[0]), int(f[1]), int(f[2])
            for u, v in [(a, b), (b, c), (c, a)]:
                edge_set.add(tuple(sorted((u, v))))

        segments = [[u, v] for (u, v) in edge_set]
        return verts, segments

    def _extract_mesh_faces(self, mesh: "trimesh.Trimesh", max_faces: int) -> Tuple[np.ndarray, np.ndarray]:
        verts = np.asarray(mesh.vertices)
        faces = np.asarray(mesh.faces)

        if len(faces) > max_faces:
            idx = np.linspace(0, len(faces) - 1, max_faces).astype(int)
            faces = faces[idx]

        return verts, faces


# ============================================================
# Industry notes
# ============================================================

def summarize_industry_patterns(report: Dict[str, Any]) -> List[str]:
    notes: List[str] = []
    model_name = report.get("model_name", "").lower()
    links = report.get("links", {})
    joints = report.get("joints", {})

    if "ur5" in model_name:
        notes.append(
            "UR5e spherical wrist is split as wrist_1_link -> wrist_2_link -> wrist_3_link. Industry-standard split follows kinematic stages, not cosmetic shell boundaries."
        )
        notes.append(
            "UR5e also shows one rigid link can own multiple visual meshes. Mesh count does not equal joint count."
        )

    if "g1" in model_name:
        notes.append(
            "Unitree G1 ankle is split by motion stage: ankle_pitch_link -> ankle_roll_link. Foot contact points are attached to the final ankle-roll stage."
        )
        notes.append(
            "This is a strong precedent for splitting a unified-looking ankle/heel adapter into serial single-DoF links."
        )

    max_meshes = 0
    max_link = None
    for lname, l in links.items():
        n = len(l.get("meshes", []))
        if n > max_meshes:
            max_meshes = n
            max_link = lname
    if max_link and max_meshes > 1:
        notes.append(f"Link '{max_link}' owns {max_meshes} mesh refs, reinforcing that one link may carry several visual submeshes.")

    nonfixed = [j for j in joints.values() if j.get("joint_type") not in ["fixed"]]
    if len(nonfixed) >= 3:
        notes.append("Compound mechanisms are usually represented as serial 1-DoF stages rather than one multi-axis monolithic link.")

    notes.append("For your multi-motor ankle adapter, a strong first-pass split is: shank -> ankle_pitch_link -> ankle_roll_link -> foot_link.")
    return notes


# ============================================================
# HTML viewer
# ============================================================

def build_tree_positions(links: Dict[str, Any]) -> Dict[str, Tuple[float, float]]:
    roots = [name for name, link in links.items() if link.get("parent") is None]
    children = {name: link.get("child_links", []) for name, link in links.items()}

    depth_map: Dict[str, int] = {}
    order: List[str] = []

    def dfs(name: str, depth: int) -> None:
        depth_map[name] = depth
        order.append(name)
        for c in children.get(name, []):
            dfs(c, depth + 1)

    for r in roots:
        dfs(r, 0)

    layers: Dict[int, List[str]] = {}
    for name, depth in depth_map.items():
        layers.setdefault(depth, []).append(name)

    pos: Dict[str, Tuple[float, float]] = {}
    for depth, names in sorted(layers.items()):
        names = sorted(names)
        n = len(names)
        for i, name in enumerate(names):
            pos[name] = (i - (n - 1) / 2.0, -depth)
    return pos


def make_html_report(report: Dict[str, Any], html_path: Path) -> None:
    links = report["links"]
    joints = report["joints"]
    mesh_items = report["mesh_render_items"]
    tree_pos = build_tree_positions(links)

    escaped_xml = html.escape(report["xml_source"])

    # Minimal XML highlighting
    escaped_xml = re.sub(r'(&lt;/?)([A-Za-z0-9_:\-\.]+)', r'\1<span class="xml-tag">\2</span>', escaped_xml)
    escaped_xml = re.sub(r'([A-Za-z_:][A-Za-z0-9_:\-\.]*)(=)(&quot;[^&]*?&quot;)', r'<span class="xml-attr">\1</span>\2<span class="xml-str">\3</span>', escaped_xml)

    html_text = f"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8" />
<title>Robot Structure Inspector</title>
<script src="https://cdn.plot.ly/plotly-2.35.2.min.js"></script>
<style>
:root {{
  --bg: #eef2f7;
  --panel: #ffffff;
  --text: #1f2937;
  --muted: #5b6472;
  --line: #d6dbe4;
  --accent: #0f2a5f;
  --hl: #ffcc00;
}}
* {{ box-sizing: border-box; }}
body {{
  margin: 0;
  font-family: -apple-system,BlinkMacSystemFont,Segoe UI,Roboto,sans-serif;
  background: var(--bg);
  color: var(--text);
}}
header {{
  padding: 16px 20px;
  background: #08193d;
  color: white;
}}
header .sub {{
  font-size: 14px;
  opacity: 0.95;
  margin-top: 6px;
}}
.layout {{
  display: grid;
  grid-template-columns: 300px 420px 1fr;
  gap: 12px;
  padding: 12px;
  min-height: 78vh;
}}
.panel {{
  background: var(--panel);
  border: 1px solid var(--line);
  border-radius: 14px;
  padding: 12px;
  overflow: hidden;
}}
.leftcol {{
  display: flex;
  flex-direction: column;
  gap: 12px;
}}
.card-title {{
  font-size: 13px;
  font-weight: 700;
  margin-bottom: 8px;
}}
.controls button {{
  margin: 0 6px 8px 0;
  padding: 8px 10px;
  border-radius: 10px;
  border: 1px solid #b9c4d4;
  background: #f8fafc;
  cursor: pointer;
}}
.controls button:hover {{ background: #edf2f8; }}
.small {{
  font-size: 12px;
  color: var(--muted);
}}
#treePlot, #scenePlot {{
  width: 100%;
  height: 760px;
}}
.tablewrap {{
  max-height: 260px;
  overflow: auto;
  border: 1px solid var(--line);
  border-radius: 10px;
}}
table {{
  width: 100%;
  border-collapse: collapse;
  font-size: 12px;
}}
th, td {{
  padding: 7px 8px;
  border-bottom: 1px solid #e8ecf3;
  text-align: left;
  vertical-align: top;
}}
th {{
  position: sticky;
  top: 0;
  background: #f7f9fc;
  z-index: 1;
}}
tr.data-row {{
  cursor: pointer;
}}
tr.data-row:hover {{
  background: #f4f8ff;
}}
tr.selected-row {{
  background: #fff3bf !important;
}}
.selection {{
  font-size: 14px;
  line-height: 1.45;
}}
.footer {{
  padding: 0 12px 12px 12px;
}}
.xml-panel {{
  background: var(--panel);
  border: 1px solid var(--line);
  border-radius: 14px;
  padding: 12px;
}}
.xml-box {{
  font-family: ui-monospace,SFMono-Regular,Menlo,Consolas,monospace;
  font-size: 12px;
  line-height: 1.45;
  max-height: 360px;
  overflow: auto;
  white-space: pre-wrap;
  word-break: break-word;
  background: #0f172a;
  color: #d7e3f4;
  border-radius: 10px;
  padding: 12px;
}}
.xml-tag {{ color: #7dd3fc; font-weight: 700; }}
.xml-attr {{ color: #fbbf24; }}
.xml-str {{ color: #86efac; }}
.note-list {{
  margin: 0;
  padding-left: 18px;
  font-size: 13px;
}}
@media (max-width: 1400px) {{
  .layout {{
    grid-template-columns: 300px 1fr;
    grid-template-rows: auto auto;
  }}
  .scenePanel {{
    grid-column: 1 / span 2;
  }}
}}
</style>
</head>
<body>
<header>
  <div style="font-size: 18px; font-weight: 700;">Robot Structure Inspector</div>
  <div class="sub">
    <b>Model:</b> {html.escape(report["model_name"])} &nbsp; | &nbsp;
    <b>Kind:</b> {html.escape(report["kind"])} &nbsp; | &nbsp;
    <b>Source:</b> {html.escape(report["source_file"])}
  </div>
</header>

<div class="layout">
  <div class="leftcol">
    <div class="panel controls">
      <div class="card-title">Controls</div>
      <button onclick="showAllMeshes()">Show All Meshes</button>
      <button onclick="hideAllMeshes()">Hide All Meshes</button><br/>
      <button onclick="showVisualOnly()">Visual Only</button>
      <button onclick="showCollisionOnly()">Collision Only</button>
      <button onclick="resetHighlight()">Reset Highlight</button><br/>
      <button onclick="toggleFrames()">Toggle Frames</button>
      <button onclick="toggleJoints()">Toggle Joints</button>
      <div class="small" style="margin-top:8px;">
        Hover or click:
        <ul>
          <li>a tree node to highlight all meshes for that link</li>
          <li>a mesh row to highlight just that mesh</li>
          <li>a joint row to highlight parent/child links</li>
          <li>a 3D mesh directly to cross-reference its link</li>
        </ul>
      </div>
    </div>

    <div class="panel">
      <div class="card-title">Current Selection</div>
      <div id="selectionBox" class="selection">Nothing selected yet.</div>
    </div>

    <div class="panel">
      <div class="card-title">Links ({len(links)})</div>
      <div class="tablewrap">
        <table id="linksTable">
          <thead>
            <tr><th>Link</th><th>Parent</th><th># Meshes</th></tr>
          </thead>
          <tbody></tbody>
        </table>
      </div>
    </div>

    <div class="panel">
      <div class="card-title">Meshes ({len(mesh_items)})</div>
      <div class="tablewrap">
        <table id="meshesTable">
          <thead>
            <tr><th>Mesh</th><th>Link</th><th>Role</th><th>File</th></tr>
          </thead>
          <tbody></tbody>
        </table>
      </div>
    </div>

    <div class="panel">
      <div class="card-title">Joints ({len(joints)})</div>
      <div class="tablewrap">
        <table id="jointsTable">
          <thead>
            <tr><th>Joint</th><th>Type</th><th>Parent</th><th>Child</th></tr>
          </thead>
          <tbody></tbody>
        </table>
      </div>
    </div>

    <div class="panel">
      <div class="card-title">Warnings</div>
      <ul class="note-list">
        {''.join(f'<li>{html.escape(w)}</li>' for w in report["warnings"]) if report["warnings"] else '<li>None</li>'}
      </ul>
    </div>
  </div>

  <div class="panel">
    <div class="card-title" style="font-size: 28px; margin-bottom: 4px;">Kinematic Tree</div>
    <div id="treePlot"></div>
  </div>

  <div class="panel scenePanel">
    <div class="card-title" style="font-size: 28px; margin-bottom: 4px;">3D Mesh / Frame / Joint Inspector</div>
    <div id="scenePlot"></div>
  </div>
</div>

<div class="footer">
  <div class="xml-panel">
    <div class="card-title">Source XML / URDF / MJCF / Xacro</div>
    <div class="small" style="margin-bottom:8px;">
      This is a read-only source viewer so you can compare the structural tree against the authored file.
    </div>
    <div class="xml-box">{escaped_xml}</div>
  </div>
</div>

<script>
const REPORT = {json.dumps(report)};
const LINKS = REPORT.links;
const JOINTS = REPORT.joints;
const MESH_ITEMS = REPORT.mesh_render_items;
const TREE_POS = {json.dumps(tree_pos)};

let framesVisible = true;
let jointsVisible = true;

const sceneDiv = document.getElementById("scenePlot");
const treeDiv = document.getElementById("treePlot");
const selectionBox = document.getElementById("selectionBox");

const linkRowMap = new Map();
const meshRowMap = new Map();
const jointRowMap = new Map();

const traceMeta = {{
  meshIndices: [],
  frameIndices: [],
  jointIndices: [],
  linkNodeTraceIndex: null
}};

function setSelectionHtml(htmlStr) {{
  selectionBox.innerHTML = htmlStr;
}}

function buildTables() {{
  const linksBody = document.querySelector("#linksTable tbody");
  const meshesBody = document.querySelector("#meshesTable tbody");
  const jointsBody = document.querySelector("#jointsTable tbody");

  Object.entries(LINKS).sort((a,b)=>a[0].localeCompare(b[0])).forEach(([name, link]) => {{
    const tr = document.createElement("tr");
    tr.className = "data-row";
    tr.innerHTML = `
      <td>${{name}}</td>
      <td>${{link.parent ?? "-"}}</td>
      <td>${{(link.meshes || []).length}}</td>
    `;
    tr.onclick = () => highlightLink(name);
    linksBody.appendChild(tr);
    linkRowMap.set(name, tr);
  }});

  MESH_ITEMS.forEach((m, idx) => {{
    const tr = document.createElement("tr");
    tr.className = "data-row";
    const fileBase = m.mesh_file ? m.mesh_file.split("/").pop() : "-";
    tr.innerHTML = `
      <td>${{m.mesh_name}}</td>
      <td>${{m.link_name}}</td>
      <td>${{m.role}}</td>
      <td>${{fileBase}}</td>
    `;
    tr.onclick = () => highlightMeshById(m.id);
    meshesBody.appendChild(tr);
    meshRowMap.set(m.id, tr);
  }});

  Object.entries(JOINTS).sort((a,b)=>a[0].localeCompare(b[0])).forEach(([name, j]) => {{
    const tr = document.createElement("tr");
    tr.className = "data-row";
    tr.innerHTML = `
      <td>${{name}}</td>
      <td>${{j.joint_type}}</td>
      <td>${{j.parent ?? "-"}}</td>
      <td>${{j.child ?? "-"}}</td>
    `;
    tr.onclick = () => highlightJoint(name);
    jointsBody.appendChild(tr);
    jointRowMap.set(name, tr);
  }});
}}

function clearSelectedRows() {{
  document.querySelectorAll(".selected-row").forEach(el => el.classList.remove("selected-row"));
}}

function buildTreeFigure() {{
  const edgeX = [];
  const edgeY = [];
  Object.entries(JOINTS).forEach(([jname, j]) => {{
    if (TREE_POS[j.parent] && TREE_POS[j.child]) {{
      edgeX.push(TREE_POS[j.parent][0], TREE_POS[j.child][0], null);
      edgeY.push(TREE_POS[j.parent][1], TREE_POS[j.child][1], null);
    }}
  }});

  const nodeX = [];
  const nodeY = [];
  const nodeText = [];
  const nodeCustom = [];
  const nodeColor = [];

  Object.entries(LINKS).sort((a,b)=>a[0].localeCompare(b[0])).forEach(([name, link]) => {{
    const p = TREE_POS[name] || [0,0];
    nodeX.push(p[0]);
    nodeY.push(p[1]);
    nodeText.push(name);
    nodeCustom.push(name);
    nodeColor.push(deterministicColorJS(name));
  }});

  const data = [
    {{
      x: edgeX,
      y: edgeY,
      mode: "lines",
      type: "scatter",
      line: {{color: "#9aa4b2", width: 2}},
      hoverinfo: "skip",
      showlegend: false
    }},
    {{
      x: nodeX,
      y: nodeY,
      mode: "markers+text",
      type: "scatter",
      text: nodeText,
      textposition: "bottom center",
      customdata: nodeCustom,
      marker: {{
        size: 16,
        color: nodeColor,
        line: {{color: "#1f2937", width: 1}}
      }},
      hovertemplate: "<b>%{{customdata}}</b><extra></extra>",
      showlegend: false
    }}
  ];

  const layout = {{
    margin: {{l: 10, r: 10, t: 10, b: 10}},
    xaxis: {{visible: false}},
    yaxis: {{visible: false}},
    paper_bgcolor: "white",
    plot_bgcolor: "white"
  }};

  Plotly.newPlot(treeDiv, data, layout, {{responsive: true, displaylogo: false}});
  traceMeta.linkNodeTraceIndex = 1;

  treeDiv.on("plotly_click", ev => {{
    if (!ev.points || !ev.points.length) return;
    const linkName = ev.points[0].customdata;
    highlightLink(linkName);
  }});
}}

function makeLineTrace3D(x, y, z, color, width, name, legendgroup, visible=true, hovertext="") {{
  return {{
    type: "scatter3d",
    mode: "lines",
    x, y, z,
    line: {{color, width}},
    hovertemplate: hovertext ? hovertext + "<extra></extra>" : "<extra></extra>",
    name,
    legendgroup,
    showlegend: false,
    visible
  }};
}}

function makeMarkerTrace3D(x, y, z, color, size, name, legendgroup, visible=true, hovertext="") {{
  return {{
    type: "scatter3d",
    mode: "markers",
    x, y, z,
    marker: {{color, size}},
    hovertemplate: hovertext ? hovertext + "<extra></extra>" : "<extra></extra>",
    name,
    legendgroup,
    showlegend: false,
    visible
  }};
}}

function buildSceneFigure() {{
  const data = [];

  // Mesh previews
  MESH_ITEMS.forEach((item, idx) => {{
    const traceIndex = data.length;
    const color = item.color;
    const opacity = item.role === "visual" ? 0.62 : 0.22;

    if (item.mode === "pointcloud") {{
      const xs = item.points.map(p=>p[0]);
      const ys = item.points.map(p=>p[1]);
      const zs = item.points.map(p=>p[2]);

      data.push({{
        type: "scatter3d",
        mode: "markers",
        x: xs,
        y: ys,
        z: zs,
        marker: {{
          size: 2,
          color: color,
          opacity: opacity
        }},
        customdata: Array(xs.length).fill(item.id),
        hovertemplate: `<b>${{item.mesh_name}}</b><br>link=${{item.link_name}}<br>role=${{item.role}}<extra></extra>`,
        name: item.mesh_name,
        showlegend: false,
        legendgroup: item.link_name
      }});
    }} else if (item.mode === "wireframe") {{
      const pts = item.points;
      const segs = item.segments || [];
      const xs = [];
      const ys = [];
      const zs = [];
      segs.forEach(([a,b]) => {{
        xs.push(pts[a][0], pts[b][0], null);
        ys.push(pts[a][1], pts[b][1], null);
        zs.push(pts[a][2], pts[b][2], null);
      }});
      data.push({{
        type: "scatter3d",
        mode: "lines",
        x: xs,
        y: ys,
        z: zs,
        line: {{color: color, width: 1}},
        customdata: Array(xs.length).fill(item.id),
        hovertemplate: `<b>${{item.mesh_name}}</b><br>link=${{item.link_name}}<br>role=${{item.role}}<extra></extra>`,
        name: item.mesh_name,
        showlegend: false,
        legendgroup: item.link_name,
        opacity: opacity
      }});
    }} else {{
      const verts = item.vertices;
      const faces = item.faces;
      data.push({{
        type: "mesh3d",
        x: verts.map(v=>v[0]),
        y: verts.map(v=>v[1]),
        z: verts.map(v=>v[2]),
        i: faces.map(f=>f[0]),
        j: faces.map(f=>f[1]),
        k: faces.map(f=>f[2]),
        color: color,
        opacity: opacity,
        flatshading: true,
        hovertemplate: `<b>${{item.mesh_name}}</b><br>link=${{item.link_name}}<br>role=${{item.role}}<extra></extra>`,
        name: item.mesh_name,
        showlegend: false,
        legendgroup: item.link_name
      }});
    }}

    traceMeta.meshIndices.push(traceIndex);
  }});

  // Link frames
  const frameLen = 0.045;
  Object.entries(LINKS).forEach(([name, link]) => {{
    const p = link.world_pos;
    const q = link.world_quat_wxyz;
    const R = quatToMatJS(q);

    const axes = [
      {{vec:[1,0,0], color:"red", axis:"X"}},
      {{vec:[0,1,0], color:"green", axis:"Y"}},
      {{vec:[0,0,1], color:"blue", axis:"Z"}}
    ];

    axes.forEach(a => {{
      const t = mulMatVecJS(R, a.vec);
      const q2 = [p[0]+t[0]*frameLen, p[1]+t[1]*frameLen, p[2]+t[2]*frameLen];
      data.push(makeLineTrace3D(
        [p[0], q2[0]],
        [p[1], q2[1]],
        [p[2], q2[2]],
        a.color,
        5,
        `${{name}}_${{a.axis}}`,
        `frame_${{name}}`,
        true,
        `<b>Frame</b><br>link=${{name}}<br>axis=${{a.axis}}`
      ));
      traceMeta.frameIndices.push(data.length - 1);
    }});

    data.push(makeMarkerTrace3D(
      [p[0]],[p[1]],[p[2]],
      "black", 3.2,
      `frame_${{name}}_origin`,
      `frame_${{name}}`,
      true,
      `<b>Link frame</b><br>${{name}}`
    ));
    traceMeta.frameIndices.push(data.length - 1);
  }});

  // Joint axes
  const jointLen = 0.07;
  Object.entries(JOINTS).forEach(([jname, j]) => {{
    const p = j.origin_world || [0,0,0];
    const a = normalizeJS(j.axis_world || [0,0,1]);
    const q2 = [p[0]+a[0]*jointLen, p[1]+a[1]*jointLen, p[2]+a[2]*jointLen];
    data.push({{
      type: "scatter3d",
      mode: "lines+markers",
      x: [p[0], q2[0]],
      y: [p[1], q2[1]],
      z: [p[2], q2[2]],
      line: {{color: "#f59e0b", width: 7}},
      marker: {{color: "#f59e0b", size: 2.5}},
      hovertemplate: `<b>${{jname}}</b><br>type=${{j.joint_type}}<br>parent=${{j.parent}}<br>child=${{j.child}}<extra></extra>`,
      name: jname,
      showlegend: false,
      legendgroup: "joint_axes"
    }});
    traceMeta.jointIndices.push(data.length - 1);
  }});

  const layout = {{
    margin: {{l: 0, r: 0, t: 0, b: 0}},
    scene: {{
      aspectmode: "data",
      xaxis: {{title: "X"}},
      yaxis: {{title: "Y"}},
      zaxis: {{title: "Z"}},
      camera: {{
        up: {{x:0, y:0, z:1}},
        center: {{x:0, y:0, z:0}},
        eye: {{x:1.5, y:1.2, z:0.9}}
      }}
    }},
    paper_bgcolor: "white",
    plot_bgcolor: "white",
    uirevision: "robot-scene-stable"
  }};

  // Turn off heavy hover compare behavior
  Plotly.newPlot(sceneDiv, data, layout, {{
    responsive: true,
    displaylogo: false,
    scrollZoom: true
  }});

  sceneDiv.on("plotly_click", ev => {{
    if (!ev.points || !ev.points.length) return;
    const pt = ev.points[0];
    if (pt.customdata) {{
      highlightMeshById(pt.customdata);
    }}
  }});
}}

function deterministicColorJS(name) {{
  let h = 0;
  for (let i = 0; i < name.length; i++) {{
    h = ((h << 5) - h) + name.charCodeAt(i);
    h |= 0;
  }}
  const r = Math.floor(((Math.abs(h * 17) % 256) + 160) / 2);
  const g = Math.floor(((Math.abs(h * 31) % 256) + 160) / 2);
  const b = Math.floor(((Math.abs(h * 47) % 256) + 160) / 2);
  return `rgb(${{r}},${{g}},${{b}})`;
}}

function quatToMatJS(q) {{
  const [w,x,y,z] = q;
  const n = Math.sqrt(w*w+x*x+y*y+z*z) || 1.0;
  const W=w/n, X=x/n, Y=y/n, Z=z/n;
  return [
    [1 - 2*(Y*Y + Z*Z), 2*(X*Y - Z*W),     2*(X*Z + Y*W)],
    [2*(X*Y + Z*W),     1 - 2*(X*X + Z*Z), 2*(Y*Z - X*W)],
    [2*(X*Z - Y*W),     2*(Y*Z + X*W),     1 - 2*(X*X + Y*Y)]
  ];
}}

function mulMatVecJS(R, v) {{
  return [
    R[0][0]*v[0] + R[0][1]*v[1] + R[0][2]*v[2],
    R[1][0]*v[0] + R[1][1]*v[1] + R[1][2]*v[2],
    R[2][0]*v[0] + R[2][1]*v[1] + R[2][2]*v[2]
  ];
}}

function normalizeJS(v) {{
  const n = Math.hypot(v[0], v[1], v[2]) || 1.0;
  return [v[0]/n, v[1]/n, v[2]/n];
}}

function setMeshVisibility(filterFn) {{
  const vis = [];
  const current = sceneDiv.data;
  for (let i = 0; i < current.length; i++) {{
    if (traceMeta.meshIndices.includes(i)) {{
      const item = MESH_ITEMS[traceMeta.meshIndices.indexOf(i)];
      vis.push(filterFn(item));
    }} else {{
      vis.push(current[i].visible === false ? false : true);
    }}
  }}
  Plotly.restyle(sceneDiv, {{visible: vis}});
}}

function showAllMeshes() {{
  const vis = sceneDiv.data.map((tr, i) => traceMeta.meshIndices.includes(i) ? true : tr.visible);
  Plotly.restyle(sceneDiv, {{visible: vis}});
}}

function hideAllMeshes() {{
  const vis = sceneDiv.data.map((tr, i) => traceMeta.meshIndices.includes(i) ? false : tr.visible);
  Plotly.restyle(sceneDiv, {{visible: vis}});
}}

function showVisualOnly() {{
  const vis = sceneDiv.data.map((tr, i) => {{
    if (!traceMeta.meshIndices.includes(i)) return tr.visible;
    const item = MESH_ITEMS[traceMeta.meshIndices.indexOf(i)];
    return item.role === "visual";
  }});
  Plotly.restyle(sceneDiv, {{visible: vis}});
}}

function showCollisionOnly() {{
  const vis = sceneDiv.data.map((tr, i) => {{
    if (!traceMeta.meshIndices.includes(i)) return tr.visible;
    const item = MESH_ITEMS[traceMeta.meshIndices.indexOf(i)];
    return item.role === "collision";
  }});
  Plotly.restyle(sceneDiv, {{visible: vis}});
}}

function toggleFrames() {{
  framesVisible = !framesVisible;
  const inds = traceMeta.frameIndices;
  const vals = inds.map(() => framesVisible);
  Plotly.restyle(sceneDiv, {{visible: vals}}, inds);
}}

function toggleJoints() {{
  jointsVisible = !jointsVisible;
  const inds = traceMeta.jointIndices;
  const vals = inds.map(() => jointsVisible);
  Plotly.restyle(sceneDiv, {{visible: vals}}, inds);
}}

function resetHighlight() {{
  clearSelectedRows();
  setSelectionHtml("Nothing selected yet.");

  // Restore moderate opacity and width
  sceneDiv.data.forEach((tr, i) => {{
    if (traceMeta.meshIndices.includes(i)) {{
      const idx = traceMeta.meshIndices.indexOf(i);
      const item = MESH_ITEMS[idx];
      if (tr.type === "scatter3d" && tr.mode === "markers") {{
        Plotly.restyle(sceneDiv, {{
          "marker.color": [item.color],
          "marker.size": [2],
          "marker.opacity": [item.role === "visual" ? 0.62 : 0.22]
        }}, [i]);
      }} else if (tr.type === "scatter3d" && tr.mode === "lines") {{
        Plotly.restyle(sceneDiv, {{
          "line.color": [item.color],
          "line.width": [1],
          "opacity": [item.role === "visual" ? 0.62 : 0.22]
        }}, [i]);
      }} else if (tr.type === "mesh3d") {{
        Plotly.restyle(sceneDiv, {{
          "color": [item.color],
          "opacity": [item.role === "visual" ? 0.62 : 0.22]
        }}, [i]);
      }}
    }}
  }});
}}

function highlightMeshById(meshId) {{
  clearSelectedRows();
  const row = meshRowMap.get(meshId);
  if (row) row.classList.add("selected-row");

  const item = MESH_ITEMS.find(m => m.id === meshId);
  if (!item) return;

  const linkRow = linkRowMap.get(item.link_name);
  if (linkRow) linkRow.classList.add("selected-row");

  setSelectionHtml(`
    <b>Mesh:</b> ${{item.mesh_name}}<br/>
    <b>Link:</b> ${{item.link_name}}<br/>
    <b>Role:</b> ${{item.role}}<br/>
    <b>File:</b> ${{item.mesh_file || "-"}}
  `);

  sceneDiv.data.forEach((tr, i) => {{
    if (!traceMeta.meshIndices.includes(i)) return;
    const idx = traceMeta.meshIndices.indexOf(i);
    const m = MESH_ITEMS[idx];
    const selected = m.id === meshId;

    if (tr.type === "scatter3d" && tr.mode === "markers") {{
      Plotly.restyle(sceneDiv, {{
        "marker.color": [selected ? "rgb(255,215,0)" : m.color],
        "marker.size": [selected ? 4.5 : 1.5],
        "marker.opacity": [selected ? 1.0 : 0.08]
      }}, [i]);
    }} else if (tr.type === "scatter3d" && tr.mode === "lines") {{
      Plotly.restyle(sceneDiv, {{
        "line.color": [selected ? "rgb(255,215,0)" : m.color],
        "line.width": [selected ? 3 : 1],
        "opacity": [selected ? 1.0 : 0.08]
      }}, [i]);
    }} else if (tr.type === "mesh3d") {{
      Plotly.restyle(sceneDiv, {{
        "color": [selected ? "rgb(255,215,0)" : m.color],
        "opacity": [selected ? 1.0 : 0.05]
      }}, [i]);
    }}
  }});
}}

function highlightLink(linkName) {{
  clearSelectedRows();
  const row = linkRowMap.get(linkName);
  if (row) row.classList.add("selected-row");

  const link = LINKS[linkName];
  if (!link) return;

  setSelectionHtml(`
    <b>Link:</b> ${{linkName}}<br/>
    <b>Parent:</b> ${{link.parent ?? "-"}}<br/>
    <b>Children:</b> ${{(link.child_links || []).join(", ") || "-"}}<br/>
    <b># Meshes:</b> ${{(link.meshes || []).length}}
  `);

  const selectedMeshIds = new Set(MESH_ITEMS.filter(m => m.link_name === linkName).map(m => m.id));
  MESH_ITEMS.forEach(m => {{
    if (m.link_name === linkName) {{
      const mr = meshRowMap.get(m.id);
      if (mr) mr.classList.add("selected-row");
    }}
  }});

  sceneDiv.data.forEach((tr, i) => {{
    if (!traceMeta.meshIndices.includes(i)) return;
    const idx = traceMeta.meshIndices.indexOf(i);
    const m = MESH_ITEMS[idx];
    const selected = selectedMeshIds.has(m.id);

    if (tr.type === "scatter3d" && tr.mode === "markers") {{
      Plotly.restyle(sceneDiv, {{
        "marker.color": [selected ? "rgb(255,215,0)" : m.color],
        "marker.size": [selected ? 4 : 1.5],
        "marker.opacity": [selected ? 0.95 : 0.07]
      }}, [i]);
    }} else if (tr.type === "scatter3d" && tr.mode === "lines") {{
      Plotly.restyle(sceneDiv, {{
        "line.color": [selected ? "rgb(255,215,0)" : m.color],
        "line.width": [selected ? 2.5 : 1],
        "opacity": [selected ? 0.95 : 0.07]
      }}, [i]);
    }} else if (tr.type === "mesh3d") {{
      Plotly.restyle(sceneDiv, {{
        "color": [selected ? "rgb(255,215,0)" : m.color],
        "opacity": [selected ? 0.95 : 0.05]
      }}, [i]);
    }}
  }});
}}

function highlightJoint(jointName) {{
  clearSelectedRows();
  const row = jointRowMap.get(jointName);
  if (row) row.classList.add("selected-row");

  const j = JOINTS[jointName];
  if (!j) return;

  if (linkRowMap.get(j.parent)) linkRowMap.get(j.parent).classList.add("selected-row");
  if (linkRowMap.get(j.child)) linkRowMap.get(j.child).classList.add("selected-row");

  setSelectionHtml(`
    <b>Joint:</b> ${{jointName}}<br/>
    <b>Type:</b> ${{j.joint_type}}<br/>
    <b>Parent:</b> ${{j.parent ?? "-"}}<br/>
    <b>Child:</b> ${{j.child ?? "-"}}<br/>
    <b>Axis (world):</b> ${{JSON.stringify(j.axis_world || [])}}
  `);

  const selectedLinks = new Set([j.parent, j.child].filter(Boolean));

  sceneDiv.data.forEach((tr, i) => {{
    if (!traceMeta.meshIndices.includes(i)) return;
    const idx = traceMeta.meshIndices.indexOf(i);
    const m = MESH_ITEMS[idx];
    const selected = selectedLinks.has(m.link_name);

    if (tr.type === "scatter3d" && tr.mode === "markers") {{
      Plotly.restyle(sceneDiv, {{
        "marker.color": [selected ? "rgb(255,215,0)" : m.color],
        "marker.size": [selected ? 4 : 1.5],
        "marker.opacity": [selected ? 0.95 : 0.06]
      }}, [i]);
    }} else if (tr.type === "scatter3d" && tr.mode === "lines") {{
      Plotly.restyle(sceneDiv, {{
        "line.color": [selected ? "rgb(255,215,0)" : m.color],
        "line.width": [selected ? 2.5 : 1],
        "opacity": [selected ? 0.95 : 0.06]
      }}, [i]);
    }} else if (tr.type === "mesh3d") {{
      Plotly.restyle(sceneDiv, {{
        "color": [selected ? "rgb(255,215,0)" : m.color],
        "opacity": [selected ? 0.95 : 0.05]
      }}, [i]);
    }}
  }});
}}

buildTables();
buildTreeFigure();
buildSceneFigure();
</script>
</body>
</html>
"""
    html_path.write_text(html_text, encoding="utf-8")


# ============================================================
# CLI
# ============================================================

def main() -> None:
    ap = argparse.ArgumentParser(description="Inspect MJCF / URDF / Xacro robot structure and generate a performant HTML viewer.")
    ap.add_argument("xml_path", type=Path, help="Path to MJCF XML, URDF, or Xacro")
    ap.add_argument("--outdir", type=Path, default=None, help="Output directory")
    ap.add_argument(
        "--mesh-mode",
        choices=["pointcloud", "wireframe", "full"],
        default="pointcloud",
        help="3D preview mode. pointcloud is fastest and recommended.",
    )
    ap.add_argument("--max-points-per-mesh", type=int, default=1500, help="Point sample cap for pointcloud mode")
    ap.add_argument("--max-faces-per-mesh", type=int, default=4000, help="Face cap for wireframe/full modes")
    ap.add_argument("--print-summary", action="store_true", help="Print summary to terminal")
    args = ap.parse_args()

    xml_path = args.xml_path.resolve()
    if not xml_path.exists():
        raise FileNotFoundError(f"Input file not found: {xml_path}")

    outdir = args.outdir or (xml_path.parent / "inspection")
    outdir.mkdir(parents=True, exist_ok=True)

    inspector = RobotStructureInspector(
        xml_path=xml_path,
        mesh_mode=args.mesh_mode,
        max_points_per_mesh=args.max_points_per_mesh,
        max_faces_per_mesh=args.max_faces_per_mesh,
    )
    report = inspector.inspect()

    stem = safe_name(xml_path.stem)
    json_path = outdir / f"{stem}_structure_report.json"
    html_path = outdir / f"{stem}_structure_report.html"

    json_path.write_text(json.dumps(report, indent=2), encoding="utf-8")
    make_html_report(report, html_path)

    print(f"Wrote JSON report: {json_path}")
    print(f"Wrote interactive HTML: {html_path}")

    if args.print_summary:
        print("\\n=== Summary ===")
        print(f"Model: {report['model_name']} ({report['kind']})")
        print(f"Links: {report['link_count']}")
        print(f"Joints: {report['joint_count']}")
        print(f"Mesh assets: {report['mesh_asset_count']}")
        print(f"Renderable mesh items: {len(report['mesh_render_items'])}")
        print(f"Mesh mode: {report['viewer_settings']['mesh_mode']}")
        if report["warnings"]:
            print("Warnings:")
            for w in report["warnings"]:
                print(f"  * {w}")


if __name__ == "__main__":
    main()
