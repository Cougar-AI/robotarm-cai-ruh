#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import math
import os
import re
import shutil
import subprocess
import sys
import time
import uuid
import xml.etree.ElementTree as ET
from collections import defaultdict
from dataclasses import asdict, dataclass, field
from pathlib import Path
from types import SimpleNamespace
from typing import Any, Dict, List, Optional, Tuple

import numpy as np
import trimesh
from PySide6 import QtCore, QtGui, QtWidgets
from vispy import scene


def normalize(v: np.ndarray, eps: float = 1e-12) -> np.ndarray:
    n = np.linalg.norm(v)
    if n < eps:
        return v.copy()
    return v / n


def distance_point_to_segment_2d(point: np.ndarray, a: np.ndarray, b: np.ndarray) -> float:
    ab = b - a
    denom = float(np.dot(ab, ab))
    if denom <= 1e-12:
        return float(np.linalg.norm(point - a))
    t = float(np.dot(point - a, ab) / denom)
    t = max(0.0, min(1.0, t))
    closest = a + t * ab
    return float(np.linalg.norm(point - closest))


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
    return np.array(
        [
            [x * x * C + c, x * y * C - z * s, x * z * C + y * s],
            [y * x * C + z * s, y * y * C + c, y * z * C - x * s],
            [z * x * C - y * s, z * y * C + x * s, z * z * C + c],
        ]
    )


def quat_wxyz_to_matrix(q: List[float]) -> np.ndarray:
    w, x, y, z = q
    n = math.sqrt(w * w + x * x + y * y + z * z)
    if n == 0:
        return np.eye(3)
    w, x, y, z = w / n, x / n, y / n, z / n
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


def matrix_to_quat_wxyz(R: np.ndarray) -> List[float]:
    m = R
    t = np.trace(m)
    if t > 0:
        s = math.sqrt(t + 1.0) * 2
        w = 0.25 * s
        x = (m[2, 1] - m[1, 2]) / s
        y = (m[0, 2] - m[2, 0]) / s
        z = (m[1, 0] - m[0, 1]) / s
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = math.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2
        w = (m[2, 1] - m[1, 2]) / s
        x = 0.25 * s
        y = (m[0, 1] + m[1, 0]) / s
        z = (m[0, 2] + m[2, 0]) / s
    elif m[1, 1] > m[2, 2]:
        s = math.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2
        w = (m[0, 2] - m[2, 0]) / s
        x = (m[0, 1] + m[1, 0]) / s
        y = 0.25 * s
        z = (m[1, 2] + m[2, 1]) / s
    else:
        s = math.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2
        w = (m[1, 0] - m[0, 1]) / s
        x = (m[0, 2] + m[2, 0]) / s
        y = (m[1, 2] + m[2, 1]) / s
        z = 0.25 * s
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
    return (T @ hom.T).T[:, :3]


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


def deterministic_rgb(name: str) -> Tuple[float, float, float]:
    import hashlib

    h = hashlib.md5(name.encode("utf-8")).hexdigest()
    r = (int(h[0:2], 16) + 150) / 510.0
    g = (int(h[2:4], 16) + 150) / 510.0
    b = (int(h[4:6], 16) + 150) / 510.0
    return (r, g, b)


@dataclass
class MeshRef:
    mesh_name: str
    mesh_file: Optional[str]
    visual_or_collision: str
    geom_type: str
    mesh_scale: List[float] = field(default_factory=lambda: [1.0, 1.0, 1.0])
    local_pos: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    local_quat_wxyz: List[float] = field(default_factory=lambda: [1.0, 0.0, 0.0, 0.0])
    world_pos: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    world_quat_wxyz: List[float] = field(default_factory=lambda: [1.0, 0.0, 0.0, 0.0])
    material: Optional[str] = None
    extra: Dict[str, Any] = field(default_factory=dict)


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


@dataclass
class RenderMeshItem:
    item_id: str
    link_name: str
    mesh_name: str
    role: str
    mode: str
    color: Tuple[float, float, float, float]
    vertices: np.ndarray
    faces: Optional[np.ndarray] = None
    segments: Optional[np.ndarray] = None


class RobotStructureInspector:
    def __init__(
        self,
        xml_path: Path,
        mesh_mode: str = "full",
        max_faces_per_mesh: int = 50000,
        max_points_per_mesh: int = 4000,
        xml_text: Optional[str] = None,
    ) -> None:
        self.xml_path = xml_path.resolve()
        self.base_dir = self.xml_path.parent
        self.mesh_mode = mesh_mode
        self.max_faces_per_mesh = max_faces_per_mesh
        self.max_points_per_mesh = max_points_per_mesh
        self.xml_text_override = xml_text

        self.kind = "unknown"
        self.model_name = self.xml_path.stem
        self.links: Dict[str, LinkInfo] = {}
        self.joints: Dict[str, JointInfo] = {}
        self.assets_meshes: Dict[str, str] = {}
        self.assets_mesh_scales: Dict[str, List[float]] = {}
        self.warnings: List[str] = []
        self.notes: List[str] = []
        self.xml_source = ""
        self._mesh_cache: Dict[Tuple[str, str], RenderMeshItem] = {}
        self._geometry_metrics_cache: Dict[str, Dict[str, Any]] = {}

    def load_xml_root(self) -> ET.Element:
        self.xml_source = self.xml_text_override if self.xml_text_override is not None else self.xml_path.read_text(encoding="utf-8", errors="replace")
        if self.xml_path.suffix.lower() == ".xacro":
            xacro_bin = shutil.which("xacro")
            if not xacro_bin:
                self.warnings.append("xacro file provided but xacro executable not found. Parsing raw XML.")
                return ET.fromstring(self.xml_source)
            proc = subprocess.run([xacro_bin, str(self.xml_path)], capture_output=True, text=True)
            if proc.returncode != 0:
                self.warnings.append(f"xacro expansion failed. Parsing raw XML. stderr={proc.stderr[:300]}")
                return ET.fromstring(self.xml_source)
            self.xml_source = proc.stdout
            return ET.fromstring(proc.stdout)
        return ET.fromstring(self.xml_source)

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
            self.warnings.append(f"Unknown root tag {root.tag}, attempting URDF-like parse.")
            self._parse_urdf(root)

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
            "mesh_asset_scales": self.assets_mesh_scales,
            "mesh_metrics": self.collect_mesh_metrics(),
            "warnings": self.warnings,
            "notes": self.notes,
            "xml_source": self.xml_source,
            "viewer_settings": {
                "mesh_mode": self.mesh_mode,
                "max_faces_per_mesh": self.max_faces_per_mesh,
                "max_points_per_mesh": self.max_points_per_mesh,
            },
        }
        report["notes"].extend(self._summarize_patterns())
        return report

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
                self.assets_mesh_scales[mesh_name] = parse_floats(mesh.attrib.get("scale"), 3, [1.0, 1.0, 1.0])

        worldbody = root.find("worldbody")
        if worldbody is None:
            self.warnings.append("MJCF has no worldbody.")
            return

        for body in worldbody.findall("body"):
            self._walk_mjcf_body(body, parent_link=None, parent_T=np.eye(4))

    def _walk_mjcf_body(self, body: ET.Element, parent_link: Optional[str], parent_T: np.ndarray) -> None:
        name = body.attrib.get("name", f"unnamed_body_{uuid.uuid4().hex[:8]}")
        pos = np.array(parse_floats(body.attrib.get("pos"), 3, [0.0, 0.0, 0.0]), dtype=float)

        if "quat" in body.attrib:
            R = quat_wxyz_to_matrix(parse_floats(body.attrib["quat"], 4))
        elif "euler" in body.attrib:
            R = rpy_to_matrix(*parse_floats(body.attrib["euler"], 3))
        elif "axisangle" in body.attrib:
            aa = parse_floats(body.attrib["axisangle"], 4)
            R = axis_angle_to_matrix(np.array(aa[:3], dtype=float), aa[3])
        else:
            R = np.eye(3)

        T_world = parent_T @ make_transform(R, pos)

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
            geom_name = geom.attrib.get("name", geom.attrib.get("mesh", geom.attrib.get("type", "geom")))
            role = self._mjcf_geom_role(geom)
            local_pos = np.array(parse_floats(geom.attrib.get("pos"), 3, [0.0, 0.0, 0.0]), dtype=float)
            if "quat" in geom.attrib:
                gR = quat_wxyz_to_matrix(parse_floats(geom.attrib["quat"], 4))
            elif "euler" in geom.attrib:
                gR = rpy_to_matrix(*parse_floats(geom.attrib["euler"], 3))
            elif "axisangle" in geom.attrib:
                aa = parse_floats(geom.attrib["axisangle"], 4)
                gR = axis_angle_to_matrix(np.array(aa[:3], dtype=float), aa[3])
            else:
                gR = np.eye(3)

            Tg = T_world @ make_transform(gR, local_pos)
            mesh_name = geom.attrib.get("mesh")
            mesh_file = self.assets_meshes.get(mesh_name) if mesh_name else None
            mesh_scale = self.assets_mesh_scales.get(mesh_name, [1.0, 1.0, 1.0])

            link.meshes.append(
                MeshRef(
                    mesh_name=mesh_name or geom_name,
                    mesh_file=mesh_file,
                    visual_or_collision=role,
                    geom_type=geom.attrib.get("type", "mesh" if mesh_name else "unknown"),
                    mesh_scale=mesh_scale,
                    local_pos=local_pos.tolist(),
                    local_quat_wxyz=matrix_to_quat_wxyz(gR),
                    world_pos=Tg[:3, 3].tolist(),
                    world_quat_wxyz=matrix_to_quat_wxyz(Tg[:3, :3]),
                    material=geom.attrib.get("material"),
                    extra=dict(geom.attrib),
                )
            )

        self.links[name] = link

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
            else:
                for idx, joint in enumerate(body_joints):
                    axis_local = np.array(parse_floats(joint.attrib.get("axis"), 3, [0.0, 0.0, 1.0]), dtype=float)
                    axis_world = T_world[:3, :3] @ normalize(axis_local)
                    joint_name = joint.attrib.get("name", f"{parent_link}__to__{name}__{idx}")
                    self.joints[joint_name] = JointInfo(
                        name=joint_name,
                        joint_type=joint.attrib.get("type", "hinge"),
                        parent=parent_link,
                        child=name,
                        axis_local=axis_local.tolist(),
                        axis_world=axis_world.tolist(),
                        origin_local=pos.tolist(),
                        origin_world=T_world[:3, 3].tolist(),
                        quat_world_wxyz=matrix_to_quat_wxyz(T_world[:3, :3]),
                        limits={k: joint.attrib[k] for k in ["range", "forcerange", "frictionloss"] if k in joint.attrib},
                        extra={k: v for k, v in joint.attrib.items() if k not in {"name", "axis", "type", "range"}},
                    )

        if parent_link is not None and parent_link in self.links:
            self.links[parent_link].child_links.append(name)
            for joint_name, joint in self.joints.items():
                if joint.parent == parent_link and joint.child == name:
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

    def _parse_urdf(self, root: ET.Element) -> None:
        for link_el in root.findall("link"):
            name = link_el.attrib.get("name", f"unnamed_link_{uuid.uuid4().hex[:8]}")
            link = LinkInfo(name=name, parent=None)
            inertial = link_el.find("inertial")
            if inertial is not None:
                link.inertial = dict(inertial.attrib)

            for tag, role in [("visual", "visual"), ("collision", "collision")]:
                for ge in link_el.findall(tag):
                    xyz, rpy, quat = self._urdf_origin(ge.find("origin"))
                    geom = ge.find("geometry")
                    mesh_name = None
                    mesh_file = None
                    geom_type = "unknown"
                    mesh_scale = [1.0, 1.0, 1.0]
                    extra: Dict[str, Any] = {}

                    if geom is not None:
                        mesh = geom.find("mesh")
                        if mesh is not None:
                            filename = mesh.attrib.get("filename") or mesh.attrib.get("url")
                            mesh_file = self._resolve_mesh_uri(filename) if filename else None
                            mesh_name = Path(filename).stem if filename else f"{name}_{role}_mesh"
                            geom_type = "mesh"
                            mesh_scale = parse_floats(mesh.attrib.get("scale"), 3, [1.0, 1.0, 1.0])
                            extra = dict(mesh.attrib)
                        else:
                            for prim in ["box", "cylinder", "sphere"]:
                                node = geom.find(prim)
                                if node is not None:
                                    geom_type = prim
                                    mesh_name = f"{name}_{role}_{prim}"
                                    extra = dict(node.attrib)
                                    break

                    link.meshes.append(
                        MeshRef(
                            mesh_name=mesh_name or f"{name}_{role}",
                            mesh_file=mesh_file,
                            visual_or_collision=role,
                            geom_type=geom_type,
                            mesh_scale=mesh_scale,
                            local_pos=xyz,
                            local_quat_wxyz=quat,
                            extra=extra,
                        )
                    )

            self.links[name] = link

        for joint_el in root.findall("joint"):
            jname = joint_el.attrib.get("name", f"unnamed_joint_{uuid.uuid4().hex[:8]}")
            jtype = joint_el.attrib.get("type", "fixed")
            parent_name = joint_el.find("parent").attrib.get("link") if joint_el.find("parent") is not None else None
            child_name = joint_el.find("child").attrib.get("link") if joint_el.find("child") is not None else None
            xyz, rpy, quat = self._urdf_origin(joint_el.find("origin"))
            axis = parse_floats(
                joint_el.find("axis").attrib.get("xyz") if joint_el.find("axis") is not None else None,
                3,
                [0.0, 0.0, 1.0],
            )
            limits = dict(joint_el.find("limit").attrib) if joint_el.find("limit") is not None else {}

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

    def _propagate_urdf(self, link_name: str, parent_T: np.ndarray) -> None:
        link = self.links[link_name]
        link.world_pos = parent_T[:3, 3].tolist()
        link.world_quat_wxyz = matrix_to_quat_wxyz(parent_T[:3, :3])

        for mesh in link.meshes:
            R = quat_wxyz_to_matrix(mesh.local_quat_wxyz)
            Tm = parent_T @ make_transform(R, np.array(mesh.local_pos, dtype=float))
            mesh.world_pos = Tm[:3, 3].tolist()
            mesh.world_quat_wxyz = matrix_to_quat_wxyz(Tm[:3, :3])

        for joint_name in link.joints_out:
            joint = self.joints[joint_name]
            if not joint.child or joint.child not in self.links:
                continue
            R = rpy_to_matrix(*joint.rpy_local)
            Tj = parent_T @ make_transform(R, np.array(joint.origin_local, dtype=float))
            joint.origin_world = Tj[:3, 3].tolist()
            joint.quat_world_wxyz = matrix_to_quat_wxyz(Tj[:3, :3])
            joint.axis_world = (Tj[:3, :3] @ normalize(np.array(joint.axis_local, dtype=float))).tolist()
            self._propagate_urdf(joint.child, Tj)

    def _urdf_origin(self, origin_el: Optional[ET.Element]) -> Tuple[List[float], List[float], List[float]]:
        if origin_el is None:
            return [0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]
        xyz = parse_floats(origin_el.attrib.get("xyz"), 3, [0.0, 0.0, 0.0])
        rpy = parse_floats(origin_el.attrib.get("rpy"), 3, [0.0, 0.0, 0.0])
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

    def _summarize_patterns(self) -> List[str]:
        notes: List[str] = []
        model_name = self.model_name.lower()
        if "ur5" in model_name:
            notes.append("UR5e uses a standard base -> shoulder -> upper arm -> forearm -> wrist_1 -> wrist_2 -> wrist_3 serial chain.")
            notes.append("One kinematic link can own multiple visual meshes; mesh count is not expected to match joint count.")
        nonfixed = [j for j in self.joints.values() if j.joint_type != "fixed"]
        if len(nonfixed) >= 3:
            notes.append("Serial articulated robots are usually modeled as one mechanical DoF per non-fixed joint stage.")
        return notes

    def build_render_items(self, include_collision: bool = False) -> List[RenderMeshItem]:
        items: List[RenderMeshItem] = []
        for link_name, link in self.links.items():
            for mesh in link.meshes:
                if mesh.visual_or_collision == "collision" and not include_collision:
                    continue
                try:
                    item = self._make_render_item(link_name, mesh)
                    if item is not None:
                        items.append(item)
                except Exception as exc:
                    self.warnings.append(f"Failed render build for {mesh.mesh_name}: {exc}")
        return items

    def _make_render_item(self, link_name: str, mesh: MeshRef) -> Optional[RenderMeshItem]:
        cache_key = (link_name, mesh.mesh_name + mesh.visual_or_collision + self.mesh_mode)
        if cache_key in self._mesh_cache:
            return self._mesh_cache[cache_key]

        color_rgb = deterministic_rgb(f"{link_name}:{mesh.mesh_name}:{mesh.visual_or_collision}")
        alpha = 0.9 if mesh.visual_or_collision == "visual" else 0.24
        color = (color_rgb[0], color_rgb[1], color_rgb[2], alpha)
        world_T = make_transform(quat_wxyz_to_matrix(mesh.world_quat_wxyz), np.array(mesh.world_pos, dtype=float))

        loaded = self._load_local_geometry(mesh)
        if loaded is None:
            return None
        verts, faces = loaded

        verts = transform_points(world_T, verts)
        if self.mesh_mode == "points":
            pts = self._sample_points(verts, faces, self.max_points_per_mesh)
            item = RenderMeshItem(
                item_id=f"{safe_name(link_name)}::{safe_name(mesh.mesh_name)}::{mesh.visual_or_collision}",
                link_name=link_name,
                mesh_name=mesh.mesh_name,
                role=mesh.visual_or_collision,
                mode="points",
                color=color,
                vertices=pts,
            )
        elif self.mesh_mode == "wireframe":
            segs = self._build_wire_segments(verts, faces, self.max_faces_per_mesh)
            item = RenderMeshItem(
                item_id=f"{safe_name(link_name)}::{safe_name(mesh.mesh_name)}::{mesh.visual_or_collision}",
                link_name=link_name,
                mesh_name=mesh.mesh_name,
                role=mesh.visual_or_collision,
                mode="wireframe",
                color=color,
                vertices=verts,
                segments=segs,
            )
        else:
            if faces is not None and len(faces) > self.max_faces_per_mesh:
                idx = np.linspace(0, len(faces) - 1, self.max_faces_per_mesh).astype(int)
                faces = faces[idx]
            item = RenderMeshItem(
                item_id=f"{safe_name(link_name)}::{safe_name(mesh.mesh_name)}::{mesh.visual_or_collision}",
                link_name=link_name,
                mesh_name=mesh.mesh_name,
                role=mesh.visual_or_collision,
                mode="full",
                color=color,
                vertices=verts,
                faces=faces,
            )
        self._mesh_cache[cache_key] = item
        return item

    def _load_local_geometry(self, mesh: MeshRef) -> Optional[Tuple[np.ndarray, Optional[np.ndarray]]]:
        if mesh.geom_type == "mesh" and mesh.mesh_file:
            p = Path(mesh.mesh_file)
            if not p.exists():
                self.warnings.append(f"Missing mesh file: {p}")
                return None
            raw = trimesh.load_mesh(str(p), force="mesh")
            if isinstance(raw, trimesh.Scene):
                raw = trimesh.util.concatenate(tuple(raw.geometry.values()))
            if not isinstance(raw, trimesh.Trimesh):
                return None
            verts = np.asarray(raw.vertices, dtype=float) * np.array(mesh.mesh_scale, dtype=float)
            faces = np.asarray(raw.faces, dtype=int) if raw.faces is not None else None
            return verts, faces
        return self._build_primitive_mesh(mesh)

    def geometry_metrics_for_mesh(self, mesh: MeshRef) -> Optional[Dict[str, Any]]:
        key = json.dumps(
            {
                "file": mesh.mesh_file,
                "geom_type": mesh.geom_type,
                "scale": mesh.mesh_scale,
                "extra": mesh.extra,
            },
            sort_keys=True,
            default=str,
        )
        if key in self._geometry_metrics_cache:
            return self._geometry_metrics_cache[key]

        loaded = self._load_local_geometry(mesh)
        if loaded is None:
            return None
        verts, faces = loaded
        mins = verts.min(axis=0)
        maxs = verts.max(axis=0)
        extents = maxs - mins
        bbox_center = (mins + maxs) / 2.0
        metrics: Dict[str, Any] = {
            "mesh_name": mesh.mesh_name,
            "mesh_file": mesh.mesh_file,
            "geom_type": mesh.geom_type,
            "scale": list(mesh.mesh_scale),
            "bounds_min": mins.tolist(),
            "bounds_max": maxs.tolist(),
            "extents": extents.tolist(),
            "bbox_center": bbox_center.tolist(),
            "vertex_count": int(len(verts)),
            "face_count": int(len(faces)) if faces is not None else 0,
        }

        if faces is not None and len(faces) > 0:
            tri = trimesh.Trimesh(vertices=verts, faces=faces, process=False)
            metrics["surface_centroid"] = np.asarray(tri.centroid, dtype=float).tolist()
            center_mass = np.asarray(tri.center_mass, dtype=float)
            metrics["center_mass"] = center_mass.tolist()
            metrics["is_watertight"] = bool(tri.is_watertight)
            metrics["volume"] = float(tri.volume) if np.isfinite(tri.volume) else None
        else:
            metrics["surface_centroid"] = np.asarray(verts.mean(axis=0), dtype=float).tolist()
            metrics["center_mass"] = metrics["surface_centroid"]
            metrics["is_watertight"] = False
            metrics["volume"] = None

        recommended = metrics["center_mass"] if metrics.get("is_watertight") and metrics.get("volume") not in (None, 0.0) else metrics["surface_centroid"]
        metrics["recommended_frame_origin"] = recommended
        self._geometry_metrics_cache[key] = metrics
        return metrics

    def collect_mesh_metrics(self) -> Dict[str, Any]:
        out: Dict[str, Any] = {}
        for link in self.links.values():
            for mesh in link.meshes:
                metrics = self.geometry_metrics_for_mesh(mesh)
                if metrics is not None:
                    out[f"{link.name}:{mesh.mesh_name}:{mesh.visual_or_collision}"] = metrics
        return out

    def _build_primitive_mesh(self, mesh: MeshRef) -> Optional[Tuple[np.ndarray, np.ndarray]]:
        extra = mesh.extra
        geom_type = mesh.geom_type
        if geom_type == "box":
            size = parse_floats(extra.get("size"), 3, [0.1, 0.1, 0.1])
            prim = trimesh.creation.box(extents=size)
        elif geom_type == "sphere":
            radius = float(extra.get("radius", 0.05))
            prim = trimesh.creation.icosphere(radius=radius, subdivisions=2)
        elif geom_type == "cylinder":
            radius = float(extra.get("radius", 0.05))
            length = float(extra.get("length", 0.1))
            prim = trimesh.creation.cylinder(radius=radius, height=length, sections=24)
        elif geom_type == "capsule":
            size = parse_floats(extra.get("size"), default=[0.04, 0.1])
            radius = float(size[0]) if size else 0.04
            height = float(size[1] * 2.0) if len(size) > 1 else 0.1
            prim = trimesh.creation.capsule(radius=radius, height=height, count=[12, 12])
        else:
            return None
        return np.asarray(prim.vertices, dtype=float), np.asarray(prim.faces, dtype=int)

    def _sample_points(self, verts: np.ndarray, faces: Optional[np.ndarray], max_points: int) -> np.ndarray:
        if faces is None or len(faces) == 0 or len(verts) <= max_points:
            idx = np.linspace(0, len(verts) - 1, min(len(verts), max_points)).astype(int)
            return verts[idx]
        raw = trimesh.Trimesh(vertices=verts, faces=faces, process=False)
        pts, _ = trimesh.sample.sample_surface(raw, min(max_points, max(600, len(faces) // 6)))
        return pts

    def _build_wire_segments(self, verts: np.ndarray, faces: Optional[np.ndarray], max_faces: int) -> np.ndarray:
        if faces is None or len(faces) == 0:
            return np.zeros((0, 3), dtype=float)
        if len(faces) > max_faces:
            idx = np.linspace(0, len(faces) - 1, max_faces).astype(int)
            faces = faces[idx]
        edge_set = set()
        for face in faces:
            a, b, c = int(face[0]), int(face[1]), int(face[2])
            edge_set.add(tuple(sorted((a, b))))
            edge_set.add(tuple(sorted((b, c))))
            edge_set.add(tuple(sorted((c, a))))
        segments = []
        for a, b in edge_set:
            segments.append(verts[a])
            segments.append(verts[b])
        return np.asarray(segments, dtype=float)


class XmlSyntaxHighlighter(QtGui.QSyntaxHighlighter):
    def __init__(self, parent: QtGui.QTextDocument) -> None:
        super().__init__(parent)
        self.rules: List[Tuple[QtCore.QRegularExpression, QtGui.QTextCharFormat]] = []

        tag_fmt = QtGui.QTextCharFormat()
        tag_fmt.setForeground(QtGui.QColor("#0f2a5f"))
        tag_fmt.setFontWeight(QtGui.QFont.Weight.Bold)

        attr_fmt = QtGui.QTextCharFormat()
        attr_fmt.setForeground(QtGui.QColor("#8b2f1c"))

        val_fmt = QtGui.QTextCharFormat()
        val_fmt.setForeground(QtGui.QColor("#2d6a4f"))

        comment_fmt = QtGui.QTextCharFormat()
        comment_fmt.setForeground(QtGui.QColor("#6b7280"))
        comment_fmt.setFontItalic(True)

        self.rules.append((QtCore.QRegularExpression(r"</?[A-Za-z0-9_:\-\.]+"), tag_fmt))
        self.rules.append((QtCore.QRegularExpression(r"\b[A-Za-z_:][A-Za-z0-9_:\-\.]*(?=\=)"), attr_fmt))
        self.rules.append((QtCore.QRegularExpression(r"\"[^\"]*\""), val_fmt))
        self.rules.append((QtCore.QRegularExpression(r"<!--[^>]*-->"), comment_fmt))

    def highlightBlock(self, text: str) -> None:
        for pattern, fmt in self.rules:
            it = pattern.globalMatch(text)
            while it.hasNext():
                match = it.next()
                self.setFormat(match.capturedStart(), match.capturedLength(), fmt)


class SourceEditor(QtWidgets.QPlainTextEdit):
    def __init__(self, text: str) -> None:
        super().__init__()
        self.setLineWrapMode(QtWidgets.QPlainTextEdit.LineWrapMode.NoWrap)
        self.setFont(QtGui.QFontDatabase.systemFont(QtGui.QFontDatabase.SystemFont.FixedFont))
        self.setPlainText(text)
        self.highlighter = XmlSyntaxHighlighter(self.document())

    def jump_to_token(self, token: str) -> None:
        if not token:
            return
        doc = self.document()
        cursor = doc.find(token)
        if cursor.isNull():
            return
        self.setTextCursor(cursor)
        self.centerCursor()

    def load_text(self, text: str) -> None:
        self.setPlainText(text)
        self.document().setModified(False)


class CameraPoseWidget(QtWidgets.QWidget):
    applyRequested = QtCore.Signal(dict)
    readRequested = QtCore.Signal()
    orbitRequested = QtCore.Signal()
    stopOrbitRequested = QtCore.Signal()

    def __init__(self) -> None:
        super().__init__()
        form = QtWidgets.QGridLayout(self)
        form.setContentsMargins(0, 0, 0, 0)
        form.setHorizontalSpacing(6)
        form.setVerticalSpacing(4)

        self.az_edit = QtWidgets.QLineEdit("35")
        self.el_edit = QtWidgets.QLineEdit("20")
        self.dist_edit = QtWidgets.QLineEdit("2.0")
        self.roll_edit = QtWidgets.QLineEdit("0")
        self.fov_edit = QtWidgets.QLineEdit("50")
        self.cx_edit = QtWidgets.QLineEdit("0")
        self.cy_edit = QtWidgets.QLineEdit("0")
        self.cz_edit = QtWidgets.QLineEdit("0")

        labels = [
            ("Az", self.az_edit),
            ("El", self.el_edit),
            ("Dist", self.dist_edit),
            ("Roll", self.roll_edit),
            ("FOV", self.fov_edit),
            ("Cx", self.cx_edit),
            ("Cy", self.cy_edit),
            ("Cz", self.cz_edit),
        ]
        for idx, (label, widget) in enumerate(labels):
            form.addWidget(QtWidgets.QLabel(label), 0 if idx < 5 else 1, (idx % 5) * 2)
            form.addWidget(widget, 0 if idx < 5 else 1, (idx % 5) * 2 + 1)
            widget.setMaximumWidth(76)

        self.apply_button = QtWidgets.QPushButton("Apply Camera")
        self.read_button = QtWidgets.QPushButton("Read")
        self.orbit_button = QtWidgets.QPushButton("Demo Orbit")
        self.stop_orbit_button = QtWidgets.QPushButton("Stop")
        form.addWidget(self.apply_button, 0, 10)
        form.addWidget(self.read_button, 0, 11)
        form.addWidget(self.orbit_button, 1, 10)
        form.addWidget(self.stop_orbit_button, 1, 11)

        self.apply_button.clicked.connect(self._emit_apply)
        self.read_button.clicked.connect(self.readRequested.emit)
        self.orbit_button.clicked.connect(self.orbitRequested.emit)
        self.stop_orbit_button.clicked.connect(self.stopOrbitRequested.emit)

    def _emit_apply(self) -> None:
        try:
            payload = {
                "azimuth": float(self.az_edit.text()),
                "elevation": float(self.el_edit.text()),
                "distance": float(self.dist_edit.text()),
                "roll": float(self.roll_edit.text()),
                "fov": float(self.fov_edit.text()),
                "center": [
                    float(self.cx_edit.text()),
                    float(self.cy_edit.text()),
                    float(self.cz_edit.text()),
                ],
            }
        except ValueError:
            return
        self.applyRequested.emit(payload)

    def set_pose(self, pose: Dict[str, Any]) -> None:
        self.az_edit.setText(f"{pose['azimuth']:.3f}")
        self.el_edit.setText(f"{pose['elevation']:.3f}")
        self.dist_edit.setText(f"{pose['distance']:.3f}")
        self.roll_edit.setText(f"{pose['roll']:.3f}")
        self.fov_edit.setText(f"{pose['fov']:.3f}")
        center = pose["center"]
        self.cx_edit.setText(f"{center[0]:.4f}")
        self.cy_edit.setText(f"{center[1]:.4f}")
        self.cz_edit.setText(f"{center[2]:.4f}")


class BuilderFormWidget(QtWidgets.QWidget):
    insertLinkRequested = QtCore.Signal()
    insertJointRequested = QtCore.Signal()
    insertInertialRequested = QtCore.Signal()
    insertAssemblyRequested = QtCore.Signal()

    def __init__(self) -> None:
        super().__init__()
        layout = QtWidgets.QVBoxLayout(self)
        form = QtWidgets.QFormLayout()

        self.asset_path_label = QtWidgets.QLabel("-")
        self.asset_path_label.setTextInteractionFlags(QtCore.Qt.TextInteractionFlag.TextSelectableByMouse)
        self.link_name_edit = QtWidgets.QLineEdit("new_link")
        self.parent_link_combo = QtWidgets.QComboBox()
        self.joint_name_edit = QtWidgets.QLineEdit("new_joint")
        self.joint_type_combo = QtWidgets.QComboBox()
        self.joint_type_combo.addItems(["fixed", "revolute", "continuous", "prismatic"])
        self.axis_edit = QtWidgets.QLineEdit("0 0 1")
        self.scale_edit = QtWidgets.QLineEdit("1 1 1")
        self.density_edit = QtWidgets.QLineEdit("1000")
        self.origin_mode_combo = QtWidgets.QComboBox()
        self.origin_mode_combo.addItems(["recommended_frame_origin", "center_mass", "surface_centroid", "bbox_center"])
        self.include_inertial_check = QtWidgets.QCheckBox()
        self.include_inertial_check.setChecked(True)

        form.addRow("Asset", self.asset_path_label)
        form.addRow("Link Name", self.link_name_edit)
        form.addRow("Parent Link", self.parent_link_combo)
        form.addRow("Joint Name", self.joint_name_edit)
        form.addRow("Joint Type", self.joint_type_combo)
        form.addRow("Axis", self.axis_edit)
        form.addRow("Scale", self.scale_edit)
        form.addRow("Density", self.density_edit)
        form.addRow("Center Mode", self.origin_mode_combo)
        form.addRow("Add Inertial", self.include_inertial_check)

        button_row = QtWidgets.QHBoxLayout()
        self.insert_link_button = QtWidgets.QPushButton("Insert Link")
        self.insert_joint_button = QtWidgets.QPushButton("Insert Joint")
        self.insert_inertial_button = QtWidgets.QPushButton("Insert Inertial")
        self.insert_assembly_button = QtWidgets.QPushButton("Insert Assembly")
        for btn in [self.insert_link_button, self.insert_joint_button, self.insert_inertial_button, self.insert_assembly_button]:
            button_row.addWidget(btn)

        layout.addLayout(form)
        layout.addLayout(button_row)
        layout.addStretch(1)

        self.insert_link_button.clicked.connect(self.insertLinkRequested.emit)
        self.insert_joint_button.clicked.connect(self.insertJointRequested.emit)
        self.insert_inertial_button.clicked.connect(self.insertInertialRequested.emit)
        self.insert_assembly_button.clicked.connect(self.insertAssemblyRequested.emit)

    def set_parent_links(self, links: List[str]) -> None:
        current = self.parent_link_combo.currentText()
        self.parent_link_combo.blockSignals(True)
        self.parent_link_combo.clear()
        self.parent_link_combo.addItems(links if links else ["base_link"])
        idx = self.parent_link_combo.findText(current)
        if idx >= 0:
            self.parent_link_combo.setCurrentIndex(idx)
        self.parent_link_combo.blockSignals(False)

    def set_asset(self, rel_path: str, link_name: str, joint_name: str, scale: List[float]) -> None:
        self.asset_path_label.setText(rel_path)
        self.link_name_edit.setText(link_name)
        self.joint_name_edit.setText(joint_name)
        self.scale_edit.setText(format_xyz(scale))

    def values(self) -> Dict[str, Any]:
        return {
            "asset_path": self.asset_path_label.text(),
            "link_name": self.link_name_edit.text().strip(),
            "parent_link": self.parent_link_combo.currentText().strip() or "base_link",
            "joint_name": self.joint_name_edit.text().strip(),
            "joint_type": self.joint_type_combo.currentText(),
            "axis": parse_floats(self.axis_edit.text(), 3, [0.0, 0.0, 1.0]),
            "scale": parse_floats(self.scale_edit.text(), 3, [1.0, 1.0, 1.0]),
            "density": float(self.density_edit.text()),
            "origin_mode": self.origin_mode_combo.currentText(),
            "include_inertial": self.include_inertial_check.isChecked(),
        }

class VispyRobotCanvas(QtWidgets.QWidget):
    linkPicked = QtCore.Signal(str)
    jointPicked = QtCore.Signal(str)
    healthEvent = QtCore.Signal(str)
    nudgeCommitted = QtCore.Signal(str, str, list)

    def __init__(self, inspector: RobotStructureInspector, include_collision: bool = True) -> None:
        super().__init__()
        self.inspector = inspector
        self.include_collision = include_collision
        self.show_frames = True
        self.show_joints = True
        self.show_centers = True
        self.show_collisions = False
        self.nudge_enabled = False
        self.nudge_plane = "xy"
        self.gizmo_enabled = True
        self.nudge_target_kind: Optional[str] = None
        self.nudge_target_name: Optional[str] = None
        self._dragging_nudge = False
        self._dragging_pan = False
        self._drag_mode: Optional[str] = None
        self._active_gizmo_axis: Optional[str] = None
        self._last_drag_pos: Optional[np.ndarray] = None
        self._pan_last_pos: Optional[np.ndarray] = None
        self._drag_total = np.zeros(3, dtype=float)
        self._drag_axis_world = np.zeros(3, dtype=float)
        self._drag_axis_screen = np.zeros(2, dtype=float)
        self.frame_visuals: Dict[str, List[Any]] = defaultdict(list)
        self.link_visuals: Dict[str, List[Any]] = defaultdict(list)
        self.center_visuals: Dict[str, List[Any]] = defaultdict(list)
        self.center_visual_data: Dict[str, List[np.ndarray]] = defaultdict(list)
        self.item_visuals: Dict[str, List[Any]] = defaultdict(list)
        self.item_visual_data: Dict[str, Dict[str, Any]] = {}
        self.item_meta: Dict[str, RenderMeshItem] = {}
        self.link_bounds: Dict[str, np.ndarray] = {}
        self.item_centers: Dict[str, np.ndarray] = {}
        self.item_visibility: Dict[str, bool] = {}
        self.joint_visuals: Dict[str, List[Any]] = defaultdict(list)
        self.joint_visual_data: Dict[str, Dict[str, Any]] = {}
        self.orbit_timer = QtCore.QTimer(self)
        self.orbit_timer.setInterval(33)
        self.orbit_timer.timeout.connect(self._orbit_step)
        self._orbit_step_deg = 1.5

        self.canvas = scene.SceneCanvas(keys="interactive", bgcolor="#eef2f7", size=(1200, 800), show=False)
        self.view = self.canvas.central_widget.add_view()
        self.view.camera = scene.cameras.TurntableCamera(fov=50, azimuth=35, elevation=20, distance=2.0)
        self.view.camera.up = "+z"
        self.grid = self.view.add_grid()
        self.canvas.native.setFocusPolicy(QtCore.Qt.FocusPolicy.StrongFocus)
        self.canvas.native.installEventFilter(self)

        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.addWidget(self.canvas.native)

        self.selection_box = scene.visuals.Line(pos=np.zeros((0, 3)), color="#ffcc00", width=3, parent=self.view.scene)
        self.gizmo_visuals: Dict[str, Dict[str, Any]] = {}
        self.gizmo_data: Dict[str, Dict[str, Any]] = {}
        self._init_gizmo_visuals()
        self.canvas.events.mouse_press.connect(self._on_mouse_press)
        self.canvas.events.mouse_move.connect(self._on_mouse_move)
        self.canvas.events.mouse_release.connect(self._on_mouse_release)
        self.reload_scene(inspector)

    def reload_scene(self, inspector: RobotStructureInspector) -> None:
        self.inspector = inspector
        self.orbit_timer.stop()
        self._clear_visuals()
        self._build_scene()
        self._update_gizmo_visuals()
        self.fit_view()

    def _clear_visuals(self) -> None:
        for bucket in [self.frame_visuals, self.link_visuals, self.center_visuals, self.item_visuals, self.joint_visuals]:
            for visuals in bucket.values():
                for vis in visuals:
                    vis.parent = None
        self.frame_visuals.clear()
        self.link_visuals.clear()
        self.center_visuals.clear()
        self.center_visual_data.clear()
        self.item_visuals.clear()
        self.item_visual_data.clear()
        self.item_meta.clear()
        self.link_bounds.clear()
        self.item_centers.clear()
        self.item_visibility.clear()
        self.joint_visuals.clear()
        self.joint_visual_data.clear()
        self.selection_box.set_data(pos=np.zeros((0, 3)))
        self._dragging_nudge = False
        self._dragging_pan = False
        self._drag_mode = None
        self._active_gizmo_axis = None
        self._pan_last_pos = None
        self._hide_gizmo_visuals()

    def _build_scene(self) -> None:
        t0 = time.perf_counter()
        all_pts = []
        for item in self.inspector.build_render_items(include_collision=self.include_collision):
            self.item_meta[item.item_id] = item
            self.item_centers[item.item_id] = item.vertices.mean(axis=0)
            visible = item.role != "collision"
            self.item_visibility[item.item_id] = visible
            if item.link_name not in self.link_bounds:
                self.link_bounds[item.link_name] = item.vertices.copy()
            else:
                self.link_bounds[item.link_name] = np.vstack([self.link_bounds[item.link_name], item.vertices])
            all_pts.append(item.vertices)

            if item.mode == "points":
                visual = scene.visuals.Markers(parent=self.view.scene)
                visual.set_data(item.vertices, face_color=item.color, edge_width=0, size=4)
            elif item.mode == "wireframe":
                visual = scene.visuals.Line(pos=item.segments, color=item.color, width=1.0, connect="segments", parent=self.view.scene)
            else:
                visual = scene.visuals.Mesh(vertices=item.vertices, faces=item.faces, color=item.color, shading="smooth", parent=self.view.scene)
            visual.visible = visible
            self.item_visuals[item.item_id].append(visual)
            self.link_visuals[item.link_name].append(visual)
            self.item_visual_data[item.item_id] = {
                "mode": item.mode,
                "visual": visual,
                "vertices": item.vertices.copy(),
                "faces": None if item.faces is None else item.faces.copy(),
                "segments": None if item.segments is None else item.segments.copy(),
                "color": item.color,
            }

            mesh_ref = next(
                (
                    m
                    for m in self.inspector.links[item.link_name].meshes
                    if safe_name(m.mesh_name) == safe_name(item.mesh_name) and m.visual_or_collision == item.role
                ),
                None,
            )
            if mesh_ref is not None:
                metrics = self.inspector.geometry_metrics_for_mesh(mesh_ref)
                if metrics is not None:
                    local_center = np.array(metrics["recommended_frame_origin"], dtype=float).reshape(1, 3)
                    world_T = make_transform(quat_wxyz_to_matrix(mesh_ref.world_quat_wxyz), np.array(mesh_ref.world_pos, dtype=float))
                    world_center = transform_points(world_T, local_center)
                    marker = scene.visuals.Markers(parent=self.view.scene)
                    marker.set_data(world_center, face_color="#ff9f1c", edge_width=0, size=7)
                    self.center_visuals[item.link_name].append(marker)
                    self.center_visual_data[item.link_name].append(world_center.copy())

        axis_span = 0.12
        for link_name, link in self.inspector.links.items():
            pos = np.array(link.world_pos, dtype=float)
            R = quat_wxyz_to_matrix(link.world_quat_wxyz)
            for color, a, b in [
                ("#d7263d", pos, pos + R @ np.array([axis_span, 0, 0])),
                ("#1b998b", pos, pos + R @ np.array([0, axis_span, 0])),
                ("#2d6cdf", pos, pos + R @ np.array([0, 0, axis_span])),
            ]:
                vis = scene.visuals.Line(pos=np.vstack([a, b]), color=color, width=2, parent=self.view.scene)
                self.frame_visuals[link_name].append(vis)

        for joint_name, joint in self.inspector.joints.items():
            pos = np.array(joint.origin_world, dtype=float)
            axis = normalize(np.array(joint.axis_world, dtype=float))
            if np.linalg.norm(axis) < 1e-9:
                axis = np.array([0.0, 0.0, 1.0])
            a = pos - axis * 0.08
            b = pos + axis * 0.08
            color = "#6f42c1" if joint.joint_type != "fixed" else "#6b7280"
            line = scene.visuals.Line(pos=np.vstack([a, b]), color=color, width=3, parent=self.view.scene)
            mark = scene.visuals.Markers(parent=self.view.scene)
            mark.set_data(pos.reshape(1, 3), face_color=color, edge_width=0, size=6)
            self.joint_visuals[joint_name].extend([line, mark])
            self.joint_visual_data[joint_name] = {
                "line": line,
                "line_pos": np.vstack([a, b]),
                "marker": mark,
                "marker_pos": pos.reshape(1, 3),
                "color": color,
            }

        if all_pts:
            merged = np.vstack(all_pts)
            self.global_center = merged.mean(axis=0)
            self.global_radius = max(float(np.linalg.norm(merged.max(axis=0) - merged.min(axis=0))), 0.5)
        else:
            self.global_center = np.zeros(3)
            self.global_radius = 1.0

        self.toggle_frames(self.show_frames)
        self.toggle_joints(self.show_joints)
        self.toggle_centers(self.show_centers)
        self.healthEvent.emit(f"scene built in {time.perf_counter() - t0:.3f}s with {len(self.item_meta)} mesh items")

    def fit_view(self) -> None:
        self.view.camera.center = self.global_center
        self.view.camera.scale_factor = self.global_radius
        self._update_gizmo_visuals()

    def focus_world_point(self, point: np.ndarray, keep_distance: bool = True) -> None:
        if not keep_distance:
            self.view.camera.scale_factor = max(float(self.view.camera.scale_factor), 0.15)
        self.view.camera.center = np.array(point, dtype=float)
        self._update_gizmo_visuals()

    def pan_by_pixels(self, dx: float, dy: float) -> None:
        scale = max(float(self.view.camera.scale_factor), 0.1) * 0.0022
        right, up = self._camera_screen_axes()
        delta = (-dx * right + dy * up) * scale
        self.view.camera.center = np.array(self.view.camera.center, dtype=float) + delta
        self._update_gizmo_visuals()
        self.healthEvent.emit(f"camera pan dx={dx:.1f} dy={dy:.1f} center={json.dumps(self.camera_pose()['center'])}")

    def center_orbit_on_canvas_pos(self, pos_xy: Tuple[float, float], threshold_px: float = 84.0) -> Optional[Dict[str, Any]]:
        picked = self._pick_world_point_at_canvas_pos(pos_xy, threshold_px=threshold_px)
        if picked is None:
            return None
        point, link_name = picked
        self.focus_world_point(point)
        self.healthEvent.emit(
            f"orbit center set from click link={link_name} point={json.dumps([float(v) for v in point.tolist()])}"
        )
        return {"point": [float(v) for v in point.tolist()], "link": link_name}

    def focus_link(self, link_name: str) -> None:
        pts = self.link_bounds.get(link_name)
        if pts is None or len(pts) == 0:
            return
        mins = pts.min(axis=0)
        maxs = pts.max(axis=0)
        self.view.camera.center = (mins + maxs) / 2.0
        self.view.camera.scale_factor = max(float(np.linalg.norm(maxs - mins)), 0.15)
        self.selection_box.set_data(pos=self._bbox_lines(mins, maxs))
        self._update_gizmo_visuals()

    def focus_joint(self, joint_name: str) -> None:
        joint = self.inspector.joints.get(joint_name)
        if joint is None:
            return
        pos = np.array(joint.origin_world, dtype=float)
        self.view.camera.center = pos
        self.view.camera.scale_factor = 0.25
        size = np.array([0.03, 0.03, 0.03], dtype=float)
        self.selection_box.set_data(pos=self._bbox_lines(pos - size, pos + size))
        self._update_gizmo_visuals()

    def toggle_frames(self, visible: bool) -> None:
        self.show_frames = visible
        for visuals in self.frame_visuals.values():
            for vis in visuals:
                vis.visible = visible

    def toggle_joints(self, visible: bool) -> None:
        self.show_joints = visible
        for visuals in self.joint_visuals.values():
            for vis in visuals:
                vis.visible = visible

    def toggle_centers(self, visible: bool) -> None:
        self.show_centers = visible
        for visuals in self.center_visuals.values():
            for vis in visuals:
                vis.visible = visible

    def toggle_collision_meshes(self, visible: bool) -> None:
        self.show_collisions = visible
        for item_id, item in self.item_meta.items():
            if item.role == "collision":
                self.set_item_visible(item_id, visible)

    def set_item_visible(self, item_id: str, visible: bool) -> None:
        self.item_visibility[item_id] = visible
        for vis in self.item_visuals.get(item_id, []):
            vis.visible = visible

    def visible_items(self) -> Dict[str, bool]:
        return dict(self.item_visibility)

    def camera_pose(self) -> Dict[str, Any]:
        center = self.view.camera.center
        return {
            "azimuth": float(self.view.camera.azimuth),
            "elevation": float(self.view.camera.elevation),
            "distance": float(self.view.camera.scale_factor),
            "roll": float(getattr(self.view.camera, "roll", 0.0)),
            "fov": float(self.view.camera.fov),
            "center": [float(center[0]), float(center[1]), float(center[2])],
        }

    def apply_camera_pose(self, pose: Dict[str, Any]) -> None:
        self.view.camera.azimuth = pose["azimuth"]
        self.view.camera.elevation = pose["elevation"]
        self.view.camera.scale_factor = max(pose["distance"], 1e-3)
        self.view.camera.roll = pose.get("roll", 0.0)
        self.view.camera.fov = pose.get("fov", self.view.camera.fov)
        self.view.camera.center = np.array(pose["center"], dtype=float)
        self._update_gizmo_visuals()
        self.healthEvent.emit(f"camera pose applied: {json.dumps(self.camera_pose())}")

    def set_nudge_mode(self, enabled: bool, plane: str) -> None:
        self.nudge_enabled = enabled
        self.nudge_plane = plane
        self._update_gizmo_visuals()

    def set_gizmo_enabled(self, enabled: bool) -> None:
        self.gizmo_enabled = enabled
        self._update_gizmo_visuals()

    def set_nudge_target(self, kind: Optional[str], name: Optional[str]) -> None:
        self.nudge_target_kind = kind
        self.nudge_target_name = name
        self._active_gizmo_axis = None
        self._update_gizmo_visuals()

    def start_demo_orbit(self, step_deg: float = 1.5) -> None:
        self._orbit_step_deg = step_deg
        self.orbit_timer.start()
        self.healthEvent.emit("demo orbit started")

    def stop_demo_orbit(self) -> None:
        if self.orbit_timer.isActive():
            self.orbit_timer.stop()
            self.healthEvent.emit("demo orbit stopped")

    def render_health_snapshot(self) -> Dict[str, Any]:
        t0 = time.perf_counter()
        img = self.canvas.render()
        dt = time.perf_counter() - t0
        info = {
            "render_seconds": dt,
            "shape": list(img.shape),
            "dtype": str(img.dtype),
            "mean_pixel": float(np.mean(img)),
            "std_pixel": float(np.std(img)),
        }
        self.healthEvent.emit(f"render snapshot: {json.dumps(info)}")
        return info

    def pick_link_at_canvas_pos(self, pos_xy: Tuple[float, float], threshold_px: float = 48.0) -> Optional[str]:
        item_id = self._pick_item_at_canvas_pos(pos_xy, threshold_px)
        if item_id is None:
            return None
        return self.item_meta[item_id].link_name

    def _pick_item_at_canvas_pos(self, pos_xy: Tuple[float, float], threshold_px: float) -> Optional[str]:
        projected = self._project_item_centers()
        if not projected:
            return None
        click = np.array(pos_xy, dtype=float)
        best_item = None
        best_dist = threshold_px
        for item_id, screen_pt in projected.items():
            if not self.item_visibility.get(item_id, False):
                continue
            dist = float(np.linalg.norm(screen_pt - click))
            if dist < best_dist:
                best_item = item_id
                best_dist = dist
        return best_item

    def _project_item_centers(self) -> Dict[str, np.ndarray]:
        transform = self._scene_to_canvas_transform()
        if transform is None:
            return {}
        out: Dict[str, np.ndarray] = {}
        for item_id, center in self.item_centers.items():
            mapped = transform.map(np.array([center]))[0]
            out[item_id] = np.array([mapped[0], mapped[1]], dtype=float)
        return out

    def _on_mouse_press(self, event: Any) -> None:
        if self.nudge_enabled and self.nudge_target_kind and self.nudge_target_name and event.button == 1:
            axis = self._pick_gizmo_axis_at_canvas_pos(tuple(event.pos[:2]), threshold_px=18.0)
            if axis is not None and self._begin_axis_drag(axis, np.array(event.pos[:2], dtype=float)):
                return
            self._begin_plane_drag(np.array(event.pos[:2], dtype=float))
            return
        if event.button != 1:
            return
        item_id = self._pick_item_at_canvas_pos(tuple(event.pos[:2]), threshold_px=52.0)
        if item_id is None:
            return
        link_name = self.item_meta[item_id].link_name
        self.linkPicked.emit(link_name)
        self.healthEvent.emit(f"canvas click picked link {link_name}")

    def _on_mouse_move(self, event: Any) -> None:
        if not self._dragging_nudge or self._last_drag_pos is None or self.nudge_target_kind is None or self.nudge_target_name is None:
            return
        pos = np.array(event.pos[:2], dtype=float)
        delta_px = pos - self._last_drag_pos
        self._last_drag_pos = pos
        if self._drag_mode == "axis":
            screen_norm = float(np.linalg.norm(self._drag_axis_screen))
            if screen_norm < 1e-6:
                return
            axis_step = float(np.dot(delta_px, self._drag_axis_screen / screen_norm))
            delta = self._drag_axis_world * (axis_step * (self._gizmo_handle_length() / screen_norm))
        else:
            world_scale = max(float(self.view.camera.scale_factor), 0.1) * 0.0015
            if self.nudge_plane == "xy":
                delta = np.array([delta_px[0] * world_scale, -delta_px[1] * world_scale, 0.0], dtype=float)
            elif self.nudge_plane == "xz":
                delta = np.array([delta_px[0] * world_scale, 0.0, -delta_px[1] * world_scale], dtype=float)
            else:
                delta = np.array([0.0, delta_px[0] * world_scale, -delta_px[1] * world_scale], dtype=float)
        self._drag_total += delta
        self.apply_preview_nudge(self.nudge_target_kind, self.nudge_target_name, delta)

    def _on_mouse_release(self, event: Any) -> None:
        if not self._dragging_nudge:
            return
        self._dragging_nudge = False
        if self.nudge_target_kind and self.nudge_target_name and float(np.linalg.norm(self._drag_total)) > 0:
            self.healthEvent.emit(
                f"nudge commit {self.nudge_target_kind} {self.nudge_target_name} delta={self._drag_total.tolist()}"
            )
            self.nudgeCommitted.emit(self.nudge_target_kind, self.nudge_target_name, self._drag_total.tolist())
        self._last_drag_pos = None
        self._drag_total = np.zeros(3, dtype=float)
        self._drag_mode = None
        self._active_gizmo_axis = None
        self._update_gizmo_visuals()

    def _orbit_step(self) -> None:
        self.view.camera.azimuth += self._orbit_step_deg

    def apply_preview_nudge(self, kind: str, name: str, delta: np.ndarray) -> None:
        if kind == "link":
            for item_id, meta in self.item_meta.items():
                if meta.link_name != name:
                    continue
                data = self.item_visual_data[item_id]
                if data["mode"] == "points":
                    data["vertices"] = data["vertices"] + delta
                    data["visual"].set_data(data["vertices"], face_color=data["color"], edge_width=0, size=4)
                elif data["mode"] == "wireframe":
                    data["segments"] = data["segments"] + delta
                    data["visual"].set_data(pos=data["segments"], color=data["color"], width=1.0, connect="segments")
                else:
                    data["vertices"] = data["vertices"] + delta
                    data["visual"].set_data(vertices=data["vertices"], faces=data["faces"], color=data["color"])
                self.item_centers[item_id] = self.item_centers[item_id] + delta
            if name in self.link_bounds:
                self.link_bounds[name] = self.link_bounds[name] + delta
            markers = self.center_visuals.get(name, [])
            centers = self.center_visual_data.get(name, [])
            for idx, marker in enumerate(markers):
                centers[idx] = centers[idx] + delta
                marker.set_data(centers[idx], face_color="#ff9f1c", edge_width=0, size=7)
            self.focus_link(name)
        elif kind == "joint":
            data = self.joint_visual_data.get(name)
            if not data:
                return
            data["line_pos"] = data["line_pos"] + delta
            data["marker_pos"] = data["marker_pos"] + delta
            data["line"].set_data(pos=data["line_pos"], color=data["color"], width=3)
            data["marker"].set_data(data["marker_pos"], face_color=data["color"], edge_width=0, size=6)
            self.focus_joint(name)
        self._update_gizmo_visuals()

    def project_gizmo_handles(self) -> Dict[str, Dict[str, np.ndarray]]:
        out: Dict[str, Dict[str, np.ndarray]] = {}
        transform = self._scene_to_canvas_transform()
        if transform is None:
            return out
        for axis, data in self.gizmo_data.items():
            mapped = transform.map(np.vstack([data["origin"], data["end"]]))
            out[axis] = {
                "origin": np.array(mapped[0][:2], dtype=float),
                "end": np.array(mapped[1][:2], dtype=float),
            }
        return out

    def _init_gizmo_visuals(self) -> None:
        for axis, color in [("x", "#d7263d"), ("y", "#1b998b"), ("z", "#2d6cdf")]:
            line = scene.visuals.Line(pos=np.zeros((0, 3)), color=color, width=5, parent=self.view.scene)
            marker = scene.visuals.Markers(parent=self.view.scene)
            marker.set_data(np.zeros((0, 3)), face_color=color, edge_width=0, size=12)
            line.visible = False
            marker.visible = False
            self.gizmo_visuals[axis] = {"line": line, "marker": marker, "color": color}

    def _hide_gizmo_visuals(self) -> None:
        self.gizmo_data.clear()
        for visuals in self.gizmo_visuals.values():
            visuals["line"].visible = False
            visuals["marker"].visible = False

    def _scene_to_canvas_transform(self) -> Optional[Any]:
        try:
            return self.view.scene.node_transform(self.canvas.scene)
        except Exception:
            return None

    def _camera_screen_axes(self) -> Tuple[np.ndarray, np.ndarray]:
        az = math.radians(float(self.view.camera.azimuth))
        el = math.radians(float(self.view.camera.elevation))
        world_up = np.array([0.0, 0.0, 1.0], dtype=float)
        forward = normalize(
            np.array(
                [
                    -math.cos(el) * math.cos(az),
                    -math.cos(el) * math.sin(az),
                    -math.sin(el),
                ],
                dtype=float,
            )
        )
        right = normalize(np.cross(forward, world_up))
        if float(np.linalg.norm(right)) < 1e-9:
            right = np.array([1.0, 0.0, 0.0], dtype=float)
        up = normalize(np.cross(right, forward))
        return right, up

    def _pick_world_point_at_canvas_pos(self, pos_xy: Tuple[float, float], threshold_px: float) -> Optional[Tuple[np.ndarray, str]]:
        transform = self._scene_to_canvas_transform()
        if transform is None:
            return None
        click = np.array(pos_xy, dtype=float)
        best_point: Optional[np.ndarray] = None
        best_link: Optional[str] = None
        best_dist = threshold_px
        for item_id, data in self.item_visual_data.items():
            if not self.item_visibility.get(item_id, False):
                continue
            vertices = data["vertices"]
            if vertices is None or len(vertices) == 0:
                continue
            step = max(1, int(math.ceil(len(vertices) / 500)))
            sampled = vertices[::step]
            mapped = transform.map(sampled)
            screen = np.asarray(mapped[:, :2], dtype=float)
            dists = np.linalg.norm(screen - click.reshape(1, 2), axis=1)
            idx = int(np.argmin(dists))
            if float(dists[idx]) < best_dist:
                best_dist = float(dists[idx])
                best_point = np.array(sampled[idx], dtype=float)
                best_link = self.item_meta[item_id].link_name
        if best_point is not None and best_link is not None:
            return best_point, best_link
        item_id = self._pick_item_at_canvas_pos(pos_xy, threshold_px=threshold_px)
        if item_id is None:
            return None
        return np.array(self.item_centers[item_id], dtype=float), self.item_meta[item_id].link_name

    def _nudge_target_pose(self) -> Optional[Tuple[np.ndarray, np.ndarray]]:
        if self.nudge_target_kind == "link" and self.nudge_target_name in self.inspector.links:
            link = self.inspector.links[self.nudge_target_name]
            return np.array(link.world_pos, dtype=float), quat_wxyz_to_matrix(link.world_quat_wxyz)
        if self.nudge_target_kind == "joint" and self.nudge_target_name in self.inspector.joints:
            joint = self.inspector.joints[self.nudge_target_name]
            return np.array(joint.origin_world, dtype=float), quat_wxyz_to_matrix(joint.quat_world_wxyz)
        return None

    def _gizmo_handle_length(self) -> float:
        return max(float(self.view.camera.scale_factor) * 0.18, 0.06)

    def _update_gizmo_visuals(self) -> None:
        pose = self._nudge_target_pose()
        if not self.gizmo_enabled or pose is None:
            self._hide_gizmo_visuals()
            return
        origin, R = pose
        handle_length = self._gizmo_handle_length()
        self.gizmo_data.clear()
        for axis, unit in [("x", np.array([1.0, 0.0, 0.0])), ("y", np.array([0.0, 1.0, 0.0])), ("z", np.array([0.0, 0.0, 1.0]))]:
            axis_world = normalize(R @ unit)
            end = origin + axis_world * handle_length
            visuals = self.gizmo_visuals[axis]
            color = visuals["color"]
            width = 6 if axis == self._active_gizmo_axis else 4
            size = 14 if axis == self._active_gizmo_axis else 11
            visuals["line"].set_data(pos=np.vstack([origin, end]), color=color, width=width)
            visuals["marker"].set_data(end.reshape(1, 3), face_color=color, edge_width=0, size=size)
            visuals["line"].visible = True
            visuals["marker"].visible = True
            self.gizmo_data[axis] = {
                "origin": origin.copy(),
                "end": end.copy(),
                "axis_world": axis_world.copy(),
            }

    def _pick_gizmo_axis_at_canvas_pos(self, pos_xy: Tuple[float, float], threshold_px: float) -> Optional[str]:
        projected = self.project_gizmo_handles()
        if not projected:
            return None
        click = np.array(pos_xy, dtype=float)
        best_axis = None
        best_dist = threshold_px
        for axis, data in projected.items():
            dist = distance_point_to_segment_2d(click, data["origin"], data["end"])
            end_dist = float(np.linalg.norm(click - data["end"]))
            dist = min(dist, end_dist)
            if dist < best_dist:
                best_axis = axis
                best_dist = dist
        return best_axis

    def _begin_plane_drag(self, pos: np.ndarray) -> None:
        self._dragging_nudge = True
        self._drag_mode = "plane"
        self._active_gizmo_axis = None
        self._last_drag_pos = pos
        self._drag_total = np.zeros(3, dtype=float)
        self._update_gizmo_visuals()
        self.healthEvent.emit(f"nudge start {self.nudge_target_kind} {self.nudge_target_name} plane={self.nudge_plane}")

    def _begin_axis_drag(self, axis: str, pos: np.ndarray) -> bool:
        data = self.gizmo_data.get(axis)
        projected = self.project_gizmo_handles().get(axis)
        if data is None or projected is None:
            return False
        screen_vec = projected["end"] - projected["origin"]
        if float(np.linalg.norm(screen_vec)) < 1e-6:
            return False
        self._dragging_nudge = True
        self._drag_mode = "axis"
        self._active_gizmo_axis = axis
        self._last_drag_pos = pos
        self._drag_total = np.zeros(3, dtype=float)
        self._drag_axis_world = np.array(data["axis_world"], dtype=float)
        self._drag_axis_screen = np.array(screen_vec, dtype=float)
        self._update_gizmo_visuals()
        self.healthEvent.emit(f"gizmo drag start axis={axis} target={self.nudge_target_kind}:{self.nudge_target_name}")
        return True

    def eventFilter(self, obj: Any, event: Any) -> bool:
        if obj is self.canvas.native:
            event_type = event.type()
            if event_type == QtCore.QEvent.Type.MouseButtonPress:
                return self._qt_mouse_press(event)
            if event_type == QtCore.QEvent.Type.MouseMove:
                return self._qt_mouse_move(event)
            if event_type == QtCore.QEvent.Type.MouseButtonRelease:
                return self._qt_mouse_release(event)
            if event_type == QtCore.QEvent.Type.MouseButtonDblClick:
                return self._qt_mouse_double_click(event)
        return super().eventFilter(obj, event)

    def _qt_mouse_press(self, event: QtGui.QMouseEvent) -> bool:
        pos = np.array([event.position().x(), event.position().y()], dtype=float)
        modifiers = event.modifiers()
        is_pan_press = (
            event.button() in (QtCore.Qt.MouseButton.RightButton, QtCore.Qt.MouseButton.MiddleButton)
            or (
                event.button() == QtCore.Qt.MouseButton.LeftButton
                and modifiers & (QtCore.Qt.KeyboardModifier.ShiftModifier | QtCore.Qt.KeyboardModifier.AltModifier)
            )
        )
        if is_pan_press:
            self._dragging_pan = True
            self._pan_last_pos = pos
            self.healthEvent.emit("camera pan start")
            return True
        return False

    def _qt_mouse_move(self, event: QtGui.QMouseEvent) -> bool:
        if not self._dragging_pan or self._pan_last_pos is None:
            return False
        pos = np.array([event.position().x(), event.position().y()], dtype=float)
        delta = pos - self._pan_last_pos
        self._pan_last_pos = pos
        self.pan_by_pixels(float(delta[0]), float(delta[1]))
        return True

    def _qt_mouse_release(self, event: QtGui.QMouseEvent) -> bool:
        if not self._dragging_pan:
            return False
        self._dragging_pan = False
        self._pan_last_pos = None
        self.healthEvent.emit("camera pan stop")
        return True

    def _qt_mouse_double_click(self, event: QtGui.QMouseEvent) -> bool:
        if event.button() != QtCore.Qt.MouseButton.LeftButton:
            return False
        result = self.center_orbit_on_canvas_pos((float(event.position().x()), float(event.position().y())))
        if result is None:
            return False
        if result["link"]:
            self.linkPicked.emit(result["link"])
        return True

    def _bbox_lines(self, mins: np.ndarray, maxs: np.ndarray) -> np.ndarray:
        x0, y0, z0 = mins
        x1, y1, z1 = maxs
        corners = np.array(
            [
                [x0, y0, z0],
                [x1, y0, z0],
                [x1, y1, z0],
                [x0, y1, z0],
                [x0, y0, z1],
                [x1, y0, z1],
                [x1, y1, z1],
                [x0, y1, z1],
            ],
            dtype=float,
        )
        edges = [(0, 1), (1, 2), (2, 3), (3, 0), (4, 5), (5, 6), (6, 7), (7, 4), (0, 4), (1, 5), (2, 6), (3, 7)]
        out = []
        for a, b in edges:
            out.append(corners[a])
            out.append(corners[b])
        return np.asarray(out, dtype=float)


class ViewerSelfTest(QtCore.QObject):
    def __init__(
        self,
        window: "InspectorMainWindow",
        timeout_seconds: float,
        live_edit: bool = False,
        builder_insert: bool = False,
        nudge_test: bool = False,
        gizmo_test: bool = False,
        camera_controls_test: bool = False,
    ) -> None:
        super().__init__(window)
        self.window = window
        self.timeout_seconds = timeout_seconds
        self.live_edit = live_edit
        self.builder_insert = builder_insert
        self.nudge_test = nudge_test
        self.gizmo_test = gizmo_test
        self.camera_controls_test = camera_controls_test
        self.started_at = time.perf_counter()
        self.done = False
        self.timeout_timer = QtCore.QTimer(self)
        self.timeout_timer.setSingleShot(True)
        self.timeout_timer.timeout.connect(self._timeout)

    def start(self) -> None:
        self.window.log_health("self-test starting")
        self.timeout_timer.start(int(self.timeout_seconds * 1000))
        QtCore.QTimer.singleShot(150, self._step_fit)
        QtCore.QTimer.singleShot(450, self._step_select_link)
        QtCore.QTimer.singleShot(800, self._step_select_joint)
        QtCore.QTimer.singleShot(1150, self._step_camera)
        QtCore.QTimer.singleShot(1450, self._step_toggle_mesh)
        QtCore.QTimer.singleShot(1750, self._step_pick_center)
        t = 2050
        if self.builder_insert:
            QtCore.QTimer.singleShot(t, self._step_builder_insert)
            t += 850
        if self.live_edit:
            QtCore.QTimer.singleShot(t, self._step_live_edit)
            t += 700
        if self.nudge_test:
            QtCore.QTimer.singleShot(t, self._step_nudge)
            t += 900
        if self.gizmo_test:
            QtCore.QTimer.singleShot(t, self._step_gizmo_drag)
            t += 1050
        if self.camera_controls_test:
            QtCore.QTimer.singleShot(t, self._step_camera_controls)
            t += 1100
        if not (self.builder_insert or self.live_edit or self.nudge_test or self.gizmo_test or self.camera_controls_test):
            QtCore.QTimer.singleShot(t, self._step_orbit_start)
            QtCore.QTimer.singleShot(t + 1250, self._step_orbit_stop)
            QtCore.QTimer.singleShot(t + 1550, self._step_render_snapshot)
            QtCore.QTimer.singleShot(t + 2250, self._finish)
            return
        QtCore.QTimer.singleShot(t, self._step_render_snapshot)
        QtCore.QTimer.singleShot(t + 300, self._step_orbit_start)
        QtCore.QTimer.singleShot(t + 1550, self._step_orbit_stop)
        QtCore.QTimer.singleShot(t + 1900, self._step_render_snapshot)
        QtCore.QTimer.singleShot(t + 2600, self._finish)

    def _step_fit(self) -> None:
        self.window.canvas.fit_view()
        self.window.log_health("self-test fit_view ok")

    def _step_select_link(self) -> None:
        if self.window.inspector.links:
            self.window.select_link(sorted(self.window.inspector.links)[0])
            self.window.log_health("self-test link selection ok")

    def _step_select_joint(self) -> None:
        if self.window.inspector.joints:
            self.window.select_joint(sorted(self.window.inspector.joints)[0])
            self.window.log_health("self-test joint selection ok")

    def _step_camera(self) -> None:
        pose = self.window.canvas.camera_pose()
        pose["azimuth"] += 25.0
        pose["elevation"] = max(-80.0, min(80.0, pose["elevation"] + 12.0))
        pose["distance"] *= 1.1
        self.window.canvas.apply_camera_pose(pose)
        self.window.camera_widget.set_pose(self.window.canvas.camera_pose())
        self.window.log_health("self-test camera apply ok")

    def _step_toggle_mesh(self) -> None:
        items = list(self.window.canvas.item_meta)
        if items:
            item_id = items[0]
            self.window.canvas.set_item_visible(item_id, False)
            self.window.mesh_list_set_checked(item_id, False)
            self.window.canvas.set_item_visible(item_id, True)
            self.window.mesh_list_set_checked(item_id, True)
            self.window.log_health(f"self-test mesh toggle ok for {item_id}")

    def _step_pick_center(self) -> None:
        projected = self.window.canvas._project_item_centers()
        link_name = None
        if projected:
            first_item_id = sorted(projected)[0]
            pt = projected[first_item_id]
            link_name = self.window.canvas.pick_link_at_canvas_pos((float(pt[0]), float(pt[1])), threshold_px=20.0)
        self.window.log_health(f"self-test targeted pick result: {link_name}")
        if link_name:
            self.window.select_link(link_name)

    def _step_live_edit(self) -> None:
        edited = make_self_test_edit(self.window.source_editor.toPlainText(), self.window.report["kind"])
        self.window.source_editor.setPlainText(edited)
        self.window.source_editor.document().setModified(True)
        draft_path = self.window.save_draft()
        self.window.log_health(f"self-test live edit draft: {draft_path}")

    def _step_builder_insert(self) -> None:
        if self.window.asset_files_list.count() == 0:
            self.window.log_health("self-test builder insert skipped: no assets found")
            return
        self.window.asset_files_list.setCurrentRow(0)
        self.window._on_asset_selected()
        self.window.builder_widget.parent_link_combo.setCurrentText("base_link")
        self.window.insert_builder_assembly()
        draft_path = self.window.save_draft()
        self.window.log_health(f"self-test builder draft: {draft_path}")

    def _step_nudge(self) -> None:
        target = self._preferred_link_target()
        if target is None:
            self.window.log_health("self-test nudge skipped: no selectable link")
            return
        self.window.select_link(target)
        self.window._set_nudge_mode(True)
        delta = np.array([0.004, 0.0, 0.003], dtype=float)
        self.window.canvas.apply_preview_nudge("link", target, delta)
        self.window._commit_nudge("link", target, delta.tolist())
        draft_path = self.window.save_draft()
        self.window.log_health(f"self-test nudge draft: {draft_path}")
        self.window.log_health(f"self-test nudge applied to {target}")

    def _step_gizmo_drag(self) -> None:
        target = self._preferred_link_target()
        if target is None:
            self.window.log_health("self-test gizmo skipped: no selectable link")
            return
        self.window.select_link(target)
        self.window.gizmo_action.setChecked(True)
        self.window._set_nudge_mode(True)
        projected = self.window.canvas.project_gizmo_handles()
        if "x" not in projected:
            self.window.log_health("self-test gizmo skipped: x-axis handle not projected")
            return
        origin = projected["x"]["origin"]
        start = projected["x"]["end"]
        axis = self.window.canvas._pick_gizmo_axis_at_canvas_pos((float(start[0]), float(start[1])), threshold_px=18.0)
        screen_dir = normalize(start - origin)
        if float(np.linalg.norm(screen_dir)) < 1e-6:
            self.window.log_health("self-test gizmo skipped: x-axis screen direction degenerate")
            return
        end = start + screen_dir * 26.0
        self.window.canvas._on_mouse_press(SimpleNamespace(button=1, pos=(float(start[0]), float(start[1]))))
        self.window.canvas._on_mouse_move(SimpleNamespace(pos=(float(end[0]), float(end[1]))))
        self.window.canvas._on_mouse_release(SimpleNamespace(button=1, pos=(float(end[0]), float(end[1]))))
        draft_path = self.window.save_draft()
        self.window.log_health(f"self-test gizmo picked axis: {axis}")
        self.window.log_health(f"self-test gizmo draft: {draft_path}")
        self.window.log_health(f"self-test gizmo drag applied to {target}")

    def _step_camera_controls(self) -> None:
        before = self.window.canvas.camera_pose()
        self.window.canvas.pan_by_pixels(28.0, -22.0)
        projected = self.window.canvas._project_item_centers()
        if projected:
            first_item_id = sorted(projected)[0]
            pt = projected[first_item_id]
            centered = self.window.canvas.center_orbit_on_canvas_pos((float(pt[0]), float(pt[1])), threshold_px=40.0)
            self.window.log_health(f"self-test click-center result: {centered}")
        self.window._apply_camera_preset("front")
        self.window._apply_camera_preset("top")
        self.window._focus_selection_camera()
        after = self.window.canvas.camera_pose()
        self.window.log_health(
            f"self-test camera controls ok before={json.dumps(before)} after={json.dumps(after)}"
        )

    def _preferred_link_target(self) -> Optional[str]:
        if (
            self.window.selected_kind == "link"
            and self.window.selected_name in self.window.inspector.links
            and self.window.inspector.links[self.window.selected_name].meshes
        ):
            return self.window.selected_name
        for link_name in sorted(self.window.inspector.links):
            if self.window.inspector.links[link_name].meshes:
                return link_name
        return sorted(self.window.inspector.links)[0] if self.window.inspector.links else None

    def _step_orbit_start(self) -> None:
        self.window.canvas.start_demo_orbit()

    def _step_orbit_stop(self) -> None:
        self.window.canvas.stop_demo_orbit()

    def _step_render_snapshot(self) -> None:
        info = self.window.canvas.render_health_snapshot()
        self.window.log_health(f"self-test render ok in {info['render_seconds']:.3f}s")

    def _finish(self) -> None:
        if self.done:
            return
        self.done = True
        self.timeout_timer.stop()
        elapsed = time.perf_counter() - self.started_at
        self.window.log_health(f"self-test passed in {elapsed:.3f}s")
        self.window.close()

    def _timeout(self) -> None:
        if self.done:
            return
        self.done = True
        elapsed = time.perf_counter() - self.started_at
        self.window.log_health(f"self-test timeout after {elapsed:.3f}s")
        self.window.close()


class InspectorMainWindow(QtWidgets.QMainWindow):
    def __init__(
        self,
        inspector: RobotStructureInspector,
        report: Dict[str, Any],
        json_path: Path,
        xml_path: Path,
        camera_pose: Optional[Dict[str, Any]] = None,
        live_reload: bool = True,
    ) -> None:
        super().__init__()
        self.inspector = inspector
        self.report = report
        self.json_path = json_path
        self.original_xml_path = xml_path.resolve()
        self.current_xml_path = xml_path.resolve()
        self.live_reload = live_reload
        self.mesh_list_items: Dict[str, QtWidgets.QListWidgetItem] = {}
        self.health_log: List[str] = []
        self.reloading = False
        self.reload_timer = QtCore.QTimer(self)
        self.reload_timer.setSingleShot(True)
        self.reload_timer.setInterval(350)
        self.reload_timer.timeout.connect(self.reload_from_disk)

        self.setWindowTitle(f"Robot Structure Inspector: {report['model_name']}")
        self.resize(1900, 1180)

        self.canvas = VispyRobotCanvas(inspector)
        self.source_editor = SourceEditor(report["xml_source"])
        self.link_tree = QtWidgets.QTreeWidget()
        self.link_tree.setHeaderLabels(["Structure"])
        self.links_table = QtWidgets.QTableWidget()
        self.joints_table = QtWidgets.QTableWidget()
        self.mesh_list = QtWidgets.QListWidget()
        self.asset_files_list = QtWidgets.QListWidget()
        self.selection_text = QtWidgets.QTextEdit()
        self.selection_text.setReadOnly(True)
        self.warnings_text = QtWidgets.QTextEdit()
        self.warnings_text.setReadOnly(True)
        self.health_text = QtWidgets.QTextEdit()
        self.health_text.setReadOnly(True)
        self.camera_widget = CameraPoseWidget()
        self.builder_widget = BuilderFormWidget()
        self.source_path_label = QtWidgets.QLabel()
        self.source_dirty_label = QtWidgets.QLabel("clean")
        self.save_draft_button = QtWidgets.QPushButton("Save Draft + Reload")
        self.revert_button = QtWidgets.QPushButton("Reload Current")
        self.original_button = QtWidgets.QPushButton("Load Original")
        self.insert_path_button = QtWidgets.QPushButton("Insert Path")
        self.insert_snippet_button = QtWidgets.QPushButton("Insert Scaffold")
        self.nudge_plane_combo = QtWidgets.QComboBox()
        self.nudge_plane_combo.addItems(["xy", "xz", "yz"])
        self.selected_kind: Optional[str] = None
        self.selected_name: Optional[str] = None
        self.shortcuts: List[QtGui.QShortcut] = []

        self._rebuild_views()
        self._build_layout()
        self._build_toolbar()
        self._connect_signals()
        self._install_shortcuts()
        self._setup_watcher()
        if camera_pose is not None:
            self.canvas.apply_camera_pose(camera_pose)
        self.camera_widget.set_pose(self.canvas.camera_pose())
        self._update_source_labels()
        self.statusBar().showMessage(f"JSON report: {json_path}")

    def _build_layout(self) -> None:
        tree_panel = QtWidgets.QWidget()
        tree_layout = QtWidgets.QVBoxLayout(tree_panel)
        tree_layout.setContentsMargins(0, 0, 0, 0)
        tree_layout.addWidget(QtWidgets.QLabel("Kinematic Structure"))
        tree_layout.addWidget(self.link_tree)

        tabs = QtWidgets.QTabWidget()
        tabs.addTab(self.links_table, "Links")
        tabs.addTab(self.joints_table, "Joints")
        tabs.addTab(self.mesh_list, "Meshes")
        asset_tab = QtWidgets.QWidget()
        asset_layout = QtWidgets.QVBoxLayout(asset_tab)
        asset_layout.setContentsMargins(4, 4, 4, 4)
        asset_buttons = QtWidgets.QHBoxLayout()
        asset_buttons.addWidget(self.insert_path_button)
        asset_buttons.addWidget(self.insert_snippet_button)
        asset_layout.addLayout(asset_buttons)
        asset_layout.addWidget(self.asset_files_list)
        tabs.addTab(asset_tab, "Asset Files")
        tabs.addTab(self.builder_widget, "Builder")
        tabs.addTab(self.selection_text, "Selection")
        tabs.addTab(self.warnings_text, "Warnings")
        tabs.addTab(self.health_text, "Health")

        top_split = QtWidgets.QSplitter(QtCore.Qt.Orientation.Horizontal)
        top_split.addWidget(tree_panel)
        top_split.addWidget(self.canvas)
        top_split.addWidget(tabs)
        top_split.setSizes([290, 1080, 460])

        bottom_panel = QtWidgets.QWidget()
        bottom_layout = QtWidgets.QVBoxLayout(bottom_panel)
        bottom_layout.setContentsMargins(0, 0, 0, 0)
        source_header = QtWidgets.QHBoxLayout()
        source_header.addWidget(QtWidgets.QLabel("Draft Editor"))
        source_header.addWidget(self.source_path_label, 1)
        source_header.addWidget(self.source_dirty_label)
        source_header.addWidget(self.original_button)
        source_header.addWidget(self.revert_button)
        source_header.addWidget(self.save_draft_button)
        bottom_layout.addLayout(source_header)
        bottom_layout.addWidget(self.source_editor)

        main_split = QtWidgets.QSplitter(QtCore.Qt.Orientation.Vertical)
        main_split.addWidget(top_split)
        main_split.addWidget(bottom_panel)
        main_split.setSizes([800, 320])
        self.setCentralWidget(main_split)

    def _rebuild_views(self) -> None:
        self._rebuild_tree()
        self._rebuild_tables()
        self._rebuild_mesh_list()
        self._rebuild_asset_files()
        self.builder_widget.set_parent_links(sorted(self.inspector.links) or ["base_link"])
        self.warnings_text.setPlainText("\n".join(self.report["warnings"] + self.report["notes"]) or "No warnings.")

    def _rebuild_tree(self) -> None:
        self.link_tree.clear()
        roots = [name for name, link in self.inspector.links.items() if link.parent is None]
        roots.sort()
        for root_name in roots:
            self._add_link_node(root_name, self.link_tree.invisibleRootItem())
        joints_root = QtWidgets.QTreeWidgetItem(["Joints"])
        for joint_name in sorted(self.inspector.joints):
            item = QtWidgets.QTreeWidgetItem([joint_name])
            item.setData(0, QtCore.Qt.ItemDataRole.UserRole, ("joint", joint_name))
            joints_root.addChild(item)
        self.link_tree.addTopLevelItem(joints_root)
        self.link_tree.expandToDepth(2)

    def _rebuild_tables(self) -> None:
        self.links_table.clear()
        self.links_table.setColumnCount(4)
        self.links_table.setHorizontalHeaderLabels(["Link", "Parent", "Meshes", "Children"])
        self.links_table.setRowCount(len(self.inspector.links))
        for row, link_name in enumerate(sorted(self.inspector.links)):
            link = self.inspector.links[link_name]
            values = [link_name, link.parent or "-", str(len(link.meshes)), str(len(link.child_links))]
            for col, value in enumerate(values):
                item = QtWidgets.QTableWidgetItem(value)
                item.setData(QtCore.Qt.ItemDataRole.UserRole, link_name)
                self.links_table.setItem(row, col, item)
        self.links_table.horizontalHeader().setSectionResizeMode(QtWidgets.QHeaderView.ResizeMode.Stretch)

        self.joints_table.clear()
        self.joints_table.setColumnCount(5)
        self.joints_table.setHorizontalHeaderLabels(["Joint", "Type", "Parent", "Child", "Axis"])
        self.joints_table.setRowCount(len(self.inspector.joints))
        for row, joint_name in enumerate(sorted(self.inspector.joints)):
            joint = self.inspector.joints[joint_name]
            axis = " ".join(f"{v:.3f}" for v in joint.axis_world)
            values = [joint_name, joint.joint_type, joint.parent or "-", joint.child or "-", axis]
            for col, value in enumerate(values):
                item = QtWidgets.QTableWidgetItem(value)
                item.setData(QtCore.Qt.ItemDataRole.UserRole, joint_name)
                self.joints_table.setItem(row, col, item)
        self.joints_table.horizontalHeader().setSectionResizeMode(QtWidgets.QHeaderView.ResizeMode.Stretch)

    def _rebuild_mesh_list(self) -> None:
        self.mesh_list.blockSignals(True)
        self.mesh_list.clear()
        self.mesh_list_items.clear()
        for item_id, item in sorted(self.canvas.item_meta.items()):
            label = f"{item.link_name} | {item.mesh_name} | {item.role}"
            entry = QtWidgets.QListWidgetItem(label)
            entry.setData(QtCore.Qt.ItemDataRole.UserRole, item_id)
            state = QtCore.Qt.CheckState.Checked if self.canvas.item_visibility.get(item_id, True) else QtCore.Qt.CheckState.Unchecked
            entry.setCheckState(state)
            self.mesh_list.addItem(entry)
            self.mesh_list_items[item_id] = entry
        self.mesh_list.blockSignals(False)

    def _rebuild_asset_files(self) -> None:
        self.asset_files_list.clear()
        for rel_path in self._discover_mesh_assets():
            item = QtWidgets.QListWidgetItem(rel_path)
            item.setData(QtCore.Qt.ItemDataRole.UserRole, rel_path)
            self.asset_files_list.addItem(item)

    def _discover_mesh_assets(self) -> List[str]:
        base = self.current_xml_path.parent
        roots = [base, base.parent]
        for child_name in ["meshes", "mesh", "assets", "asset"]:
            candidate = base / child_name
            if candidate.exists():
                roots.append(candidate)
            parent_candidate = base.parent / child_name
            if parent_candidate.exists():
                roots.append(parent_candidate)
        exts = {".stl", ".obj", ".dae", ".ply"}
        found: List[str] = []
        seen = set()
        for root in roots:
            for path in root.rglob("*"):
                if path.is_file() and path.suffix.lower() in exts:
                    try:
                        rel = path.relative_to(base).as_posix()
                    except ValueError:
                        rel = str(path)
                    if rel not in seen:
                        seen.add(rel)
                        found.append(rel)
                if len(found) >= 300:
                    break
        return sorted(found)

    def mesh_list_set_checked(self, item_id: str, checked: bool) -> None:
        item = self.mesh_list_items.get(item_id)
        if item is None:
            return
        self.mesh_list.blockSignals(True)
        item.setCheckState(QtCore.Qt.CheckState.Checked if checked else QtCore.Qt.CheckState.Unchecked)
        self.mesh_list.blockSignals(False)

    def _add_link_node(self, link_name: str, parent_item: QtWidgets.QTreeWidgetItem) -> None:
        item = QtWidgets.QTreeWidgetItem([link_name])
        item.setData(0, QtCore.Qt.ItemDataRole.UserRole, ("link", link_name))
        parent_item.addChild(item)
        for child in sorted(self.inspector.links[link_name].child_links):
            self._add_link_node(child, item)

    def _build_toolbar(self) -> None:
        tb = self.addToolBar("Viewer")
        tb.setMovable(False)

        self.fit_action = QtGui.QAction("Fit", self)
        self.fit_action.triggered.connect(self.canvas.fit_view)
        tb.addAction(self.fit_action)

        self.reload_action = QtGui.QAction("Reload", self)
        self.reload_action.triggered.connect(self.reload_from_disk)
        tb.addAction(self.reload_action)

        self.frames_action = QtGui.QAction("Frames", self)
        self.frames_action.setCheckable(True)
        self.frames_action.setChecked(True)
        self.frames_action.toggled.connect(self.canvas.toggle_frames)
        tb.addAction(self.frames_action)

        self.centers_action = QtGui.QAction("Centers", self)
        self.centers_action.setCheckable(True)
        self.centers_action.setChecked(True)
        self.centers_action.toggled.connect(self.canvas.toggle_centers)
        tb.addAction(self.centers_action)

        self.joints_action = QtGui.QAction("Joint Axes", self)
        self.joints_action.setCheckable(True)
        self.joints_action.setChecked(True)
        self.joints_action.toggled.connect(self.canvas.toggle_joints)
        tb.addAction(self.joints_action)

        self.collisions_action = QtGui.QAction("Collision Meshes", self)
        self.collisions_action.setCheckable(True)
        self.collisions_action.setChecked(False)
        self.collisions_action.toggled.connect(self._toggle_collisions)
        tb.addAction(self.collisions_action)

        self.nudge_action = QtGui.QAction("Nudge Drag", self)
        self.nudge_action.setCheckable(True)
        self.nudge_action.setChecked(False)
        self.nudge_action.toggled.connect(self._set_nudge_mode)
        tb.addAction(self.nudge_action)
        self.gizmo_action = QtGui.QAction("Axis Gizmo", self)
        self.gizmo_action.setCheckable(True)
        self.gizmo_action.setChecked(True)
        self.gizmo_action.toggled.connect(self.canvas.set_gizmo_enabled)
        tb.addAction(self.gizmo_action)
        tb.addWidget(QtWidgets.QLabel(" Plane "))
        tb.addWidget(self.nudge_plane_combo)
        tb.addSeparator()
        self.focus_action = QtGui.QAction("Focus Selection", self)
        self.focus_action.triggered.connect(self._focus_selection_camera)
        tb.addAction(self.focus_action)
        self.front_view_action = QtGui.QAction("Front", self)
        self.front_view_action.triggered.connect(lambda: self._apply_camera_preset("front"))
        tb.addAction(self.front_view_action)
        self.side_view_action = QtGui.QAction("Side", self)
        self.side_view_action.triggered.connect(lambda: self._apply_camera_preset("side"))
        tb.addAction(self.side_view_action)
        self.top_view_action = QtGui.QAction("Top", self)
        self.top_view_action.triggered.connect(lambda: self._apply_camera_preset("top"))
        tb.addAction(self.top_view_action)

        info = QtWidgets.QLabel(f"  {self.report['kind']}  |  links={self.report['link_count']}  joints={self.report['joint_count']}  ")
        tb.addWidget(info)
        hotkeys = QtWidgets.QLabel("  Hotkeys: F fit, C focus, 1 front, 3 side, 7 top, G gizmo, N nudge, RMB/Shift-drag pan, dbl-click orbit center  ")
        tb.addWidget(hotkeys)
        tb.addWidget(self.camera_widget)

    def _install_shortcuts(self) -> None:
        specs = [
            ("F", self.canvas.fit_view),
            ("C", self._focus_selection_camera),
            ("G", lambda: self.gizmo_action.setChecked(not self.gizmo_action.isChecked())),
            ("N", lambda: self.nudge_action.setChecked(not self.nudge_action.isChecked())),
            ("1", lambda: self._apply_camera_preset("front")),
            ("3", lambda: self._apply_camera_preset("side")),
            ("7", lambda: self._apply_camera_preset("top")),
        ]
        for key, handler in specs:
            shortcut = QtGui.QShortcut(QtGui.QKeySequence(key), self)
            shortcut.setContext(QtCore.Qt.ShortcutContext.WindowShortcut)
            shortcut.activated.connect(handler)
            self.shortcuts.append(shortcut)

    def _connect_signals(self) -> None:
        self.link_tree.itemSelectionChanged.connect(self._on_tree_select)
        self.links_table.itemSelectionChanged.connect(self._on_links_table_select)
        self.joints_table.itemSelectionChanged.connect(self._on_joints_table_select)
        self.mesh_list.itemChanged.connect(self._on_mesh_item_changed)
        self.mesh_list.itemSelectionChanged.connect(self._on_mesh_item_selected)
        self.insert_path_button.clicked.connect(self.insert_selected_asset_path)
        self.insert_snippet_button.clicked.connect(self.insert_selected_asset_snippet)
        self.asset_files_list.itemDoubleClicked.connect(lambda _: self.insert_selected_asset_snippet())
        self.asset_files_list.itemSelectionChanged.connect(self._on_asset_selected)
        self.builder_widget.insertLinkRequested.connect(self.insert_builder_link)
        self.builder_widget.insertJointRequested.connect(self.insert_builder_joint)
        self.builder_widget.insertInertialRequested.connect(self.insert_builder_inertial)
        self.builder_widget.insertAssemblyRequested.connect(self.insert_builder_assembly)
        self.canvas.linkPicked.connect(self.select_link)
        self.canvas.nudgeCommitted.connect(self._commit_nudge)
        self.canvas.healthEvent.connect(self.log_health)
        self.camera_widget.applyRequested.connect(self.canvas.apply_camera_pose)
        self.camera_widget.readRequested.connect(lambda: self.camera_widget.set_pose(self.canvas.camera_pose()))
        self.camera_widget.orbitRequested.connect(self.canvas.start_demo_orbit)
        self.camera_widget.stopOrbitRequested.connect(self.canvas.stop_demo_orbit)
        self.source_editor.document().modificationChanged.connect(self._on_source_modified)
        self.save_draft_button.clicked.connect(self.save_draft)
        self.revert_button.clicked.connect(self.reload_from_current_source)
        self.original_button.clicked.connect(self.load_original_source)
        self.nudge_plane_combo.currentTextChanged.connect(lambda plane: self.canvas.set_nudge_mode(self.nudge_action.isChecked(), plane))

    def _setup_watcher(self) -> None:
        if not self.live_reload:
            return
        self.file_watcher = QtCore.QFileSystemWatcher(self)
        paths = [str(self.current_xml_path.resolve())]
        for mesh_path in self.inspector.assets_meshes.values():
            paths.append(str(Path(mesh_path).resolve()))
        for link in self.inspector.links.values():
            for mesh in link.meshes:
                if mesh.mesh_file:
                    paths.append(str(Path(mesh.mesh_file).resolve()))
        unique_paths = [p for p in sorted(set(paths)) if Path(p).exists()]
        if unique_paths:
            self.file_watcher.addPaths(unique_paths)
            self.file_watcher.fileChanged.connect(self._schedule_reload)
        self.log_health(f"live reload watching {len(unique_paths)} files")

    def _schedule_reload(self, _: str) -> None:
        if not self.reload_timer.isActive():
            self.log_health("file change detected, scheduling reload")
            self.reload_timer.start()

    def reload_from_disk(self) -> None:
        if self.reloading:
            return
        self.reloading = True
        t0 = time.perf_counter()
        camera_pose = self.canvas.camera_pose()
        try:
            new_inspector = RobotStructureInspector(
                self.current_xml_path,
                mesh_mode=self.inspector.mesh_mode,
                max_faces_per_mesh=self.inspector.max_faces_per_mesh,
                max_points_per_mesh=self.inspector.max_points_per_mesh,
            )
            new_report = new_inspector.inspect()
            self.inspector = new_inspector
            self.report = new_report
            self.canvas.reload_scene(new_inspector)
            self.canvas.apply_camera_pose(camera_pose)
            self.source_editor.load_text(new_report["xml_source"])
            self._rebuild_views()
            self._setup_watcher()
            self.camera_widget.set_pose(self.canvas.camera_pose())
            self._update_source_labels()
            self.json_path, summary_path = output_paths(self.current_xml_path)
            self.json_path.write_text(json.dumps(new_report, indent=2), encoding="utf-8")
            summary_path.write_text(
                "\n".join(
                    [
                        f"source_file: {new_report['source_file']}",
                        f"kind: {new_report['kind']}",
                        f"model_name: {new_report['model_name']}",
                        f"links: {new_report['link_count']}",
                        f"joints: {new_report['joint_count']}",
                        f"warnings: {len(new_report['warnings'])}",
                    ]
                ),
                encoding="utf-8",
            )
            self.statusBar().showMessage(f"Reloaded in {time.perf_counter() - t0:.3f}s")
            self.log_health(f"reload complete in {time.perf_counter() - t0:.3f}s")
        except Exception as exc:
            self.log_health(f"reload failed: {exc}")
        finally:
            self.reloading = False

    def log_health(self, message: str) -> None:
        ts = time.strftime("%H:%M:%S")
        line = f"[{ts}] {message}"
        self.health_log.append(line)
        self.health_text.setPlainText("\n".join(self.health_log[-200:]))
        self.health_text.moveCursor(QtGui.QTextCursor.MoveOperation.End)
        print(line, flush=True)

    def _toggle_collisions(self, checked: bool) -> None:
        self.canvas.toggle_collision_meshes(checked)
        for item_id, item in self.canvas.item_meta.items():
            if item.role == "collision":
                self.mesh_list_set_checked(item_id, checked)

    def _set_nudge_mode(self, enabled: bool) -> None:
        self.canvas.set_nudge_mode(enabled, self.nudge_plane_combo.currentText())
        self.log_health(f"nudge mode -> {enabled} plane={self.nudge_plane_combo.currentText()}")

    def _focus_selection_camera(self) -> None:
        if self.selected_kind == "joint" and self.selected_name:
            self.canvas.focus_joint(self.selected_name)
        elif self.selected_kind == "link" and self.selected_name:
            self.canvas.focus_link(self.selected_name)
        else:
            self.canvas.fit_view()
        self.camera_widget.set_pose(self.canvas.camera_pose())
        self.log_health("camera focus selection")

    def _apply_camera_preset(self, preset: str) -> None:
        pose = self.canvas.camera_pose()
        if preset == "front":
            pose["azimuth"] = 0.0
            pose["elevation"] = 0.0
        elif preset == "side":
            pose["azimuth"] = 90.0
            pose["elevation"] = 0.0
        elif preset == "top":
            pose["azimuth"] = 0.0
            pose["elevation"] = 89.0
        else:
            return
        self.canvas.apply_camera_pose(pose)
        self.camera_widget.set_pose(self.canvas.camera_pose())
        self.log_health(f"camera preset -> {preset}")

    def _on_tree_select(self) -> None:
        items = self.link_tree.selectedItems()
        if not items:
            return
        payload = items[0].data(0, QtCore.Qt.ItemDataRole.UserRole)
        if not payload:
            return
        kind, name = payload
        if kind == "link":
            self.select_link(name)
        elif kind == "joint":
            self.select_joint(name)

    def _on_links_table_select(self) -> None:
        items = self.links_table.selectedItems()
        if not items:
            return
        self.select_link(items[0].data(QtCore.Qt.ItemDataRole.UserRole))

    def _on_joints_table_select(self) -> None:
        items = self.joints_table.selectedItems()
        if not items:
            return
        self.select_joint(items[0].data(QtCore.Qt.ItemDataRole.UserRole))

    def _on_mesh_item_changed(self, item: QtWidgets.QListWidgetItem) -> None:
        item_id = item.data(QtCore.Qt.ItemDataRole.UserRole)
        checked = item.checkState() == QtCore.Qt.CheckState.Checked
        self.canvas.set_item_visible(item_id, checked)
        self.log_health(f"mesh visibility {item_id} -> {checked}")

    def _on_mesh_item_selected(self) -> None:
        items = self.mesh_list.selectedItems()
        if not items:
            return
        item_id = items[0].data(QtCore.Qt.ItemDataRole.UserRole)
        meta = self.canvas.item_meta.get(item_id)
        if meta is None:
            return
        self.select_link(meta.link_name)
        self.source_editor.jump_to_token(meta.mesh_name)

    def insert_selected_asset_path(self) -> None:
        items = self.asset_files_list.selectedItems()
        if not items:
            return
        rel_path = items[0].data(QtCore.Qt.ItemDataRole.UserRole)
        self.source_editor.insertPlainText(rel_path)

    def insert_selected_asset_snippet(self) -> None:
        items = self.asset_files_list.selectedItems()
        if not items:
            return
        rel_path = items[0].data(QtCore.Qt.ItemDataRole.UserRole)
        mesh_name = safe_name(Path(rel_path).stem)
        metrics = self._metrics_for_asset_relpath(rel_path)
        if metrics is None:
            return
        scale = suggest_uniform_scale(metrics)
        metrics = self._metrics_for_asset_relpath(rel_path, scale=scale)
        if metrics is None:
            return
        parent_link = "base_link"
        selected_tree_items = self.link_tree.selectedItems()
        if selected_tree_items:
            payload = selected_tree_items[0].data(0, QtCore.Qt.ItemDataRole.UserRole)
            if payload and payload[0] == "link":
                parent_link = payload[1]
        if self.report["kind"] == "mjcf":
            snippet = make_mjcf_builder_scaffold(rel_path, mesh_name, f"{mesh_name}_body", metrics, scale=scale)
        else:
            snippet = make_urdf_builder_scaffold(rel_path, f"{mesh_name}_link", parent_link, f"{mesh_name}_joint", metrics, scale=scale)
        self.source_editor.insertPlainText(snippet)

    def _metrics_for_asset_relpath(self, rel_path: str, scale: Optional[List[float]] = None) -> Optional[Dict[str, Any]]:
        mesh_path = (self.current_xml_path.parent / rel_path).resolve()
        tmp_mesh = MeshRef(
            mesh_name=Path(rel_path).stem,
            mesh_file=str(mesh_path),
            visual_or_collision="visual",
            geom_type="mesh",
            mesh_scale=scale or [1.0, 1.0, 1.0],
        )
        return self.inspector.geometry_metrics_for_mesh(tmp_mesh)

    def _on_asset_selected(self) -> None:
        items = self.asset_files_list.selectedItems()
        if not items:
            return
        rel_path = items[0].data(QtCore.Qt.ItemDataRole.UserRole)
        raw_metrics = self._metrics_for_asset_relpath(rel_path)
        if raw_metrics is None:
            return
        scale = suggest_uniform_scale(raw_metrics)
        metrics = self._metrics_for_asset_relpath(rel_path, scale=scale)
        if metrics is None:
            return
        mesh_name = safe_name(Path(rel_path).stem)
        self.builder_widget.set_asset(rel_path, f"{mesh_name}_link", f"{mesh_name}_joint", scale)
        self.selection_text.setPlainText(
            json.dumps(
                {
                    "kind": "asset_mesh",
                    "path": rel_path,
                    "suggested_scale": scale,
                    "recommended_frame_origin": metrics.get("recommended_frame_origin"),
                    "center_mass": metrics.get("center_mass"),
                    "surface_centroid": metrics.get("surface_centroid"),
                    "bbox_center": metrics.get("bbox_center"),
                    "extents": metrics.get("extents"),
                    "volume": metrics.get("volume"),
                    "is_watertight": metrics.get("is_watertight"),
                },
                indent=2,
            )
        )

    def _builder_metrics(self) -> Optional[Tuple[Dict[str, Any], Dict[str, Any]]]:
        values = self.builder_widget.values()
        rel_path = values["asset_path"]
        if not rel_path or rel_path == "-":
            return None
        metrics = self._metrics_for_asset_relpath(rel_path, scale=values["scale"])
        if metrics is None:
            return None
        return values, metrics

    def _insert_before_close_tag(self, close_tag: str, snippet: str) -> None:
        text = self.source_editor.toPlainText()
        idx = text.rfind(close_tag)
        if idx >= 0:
            text = text[:idx] + snippet + ("\n" if not snippet.endswith("\n") else "") + text[idx:]
            self.source_editor.setPlainText(text)
        else:
            self.source_editor.insertPlainText(snippet)
        self.source_editor.document().setModified(True)

    def insert_builder_link(self) -> None:
        payload = self._builder_metrics()
        if payload is None:
            return
        values, metrics = payload
        center = np.array(metrics[values["origin_mode"]], dtype=float)
        origin = (-center).tolist()
        rel_path = values["asset_path"]
        snippet = (
            f'<link name="{values["link_name"]}">\n'
            f'  <visual>\n'
            f'    <origin xyz="{format_xyz(origin)}" rpy="0 0 0"/>\n'
            f'    <geometry>\n'
            f'      <mesh filename="{rel_path}" scale="{format_xyz(values["scale"])}"/>\n'
            f'    </geometry>\n'
            f'  </visual>\n'
            f'  <collision>\n'
            f'    <origin xyz="{format_xyz(origin)}" rpy="0 0 0"/>\n'
            f'    <geometry>\n'
            f'      <mesh filename="{rel_path}" scale="{format_xyz(values["scale"])}"/>\n'
            f'    </geometry>\n'
            f'  </collision>\n'
            f'</link>\n'
        )
        self._insert_before_close_tag("</robot>" if self.report["kind"] != "mjcf" else "</mujoco>", snippet)

    def insert_builder_joint(self) -> None:
        values = self.builder_widget.values()
        if self.report["kind"] == "mjcf":
            snippet = (
                f'<body name="{values["link_name"]}" pos="0 0 0">\n'
                f'  <joint name="{values["joint_name"]}" type="{values["joint_type"]}" axis="{format_xyz(values["axis"])}"/>\n'
                f'</body>\n'
            )
            self._insert_before_close_tag("</worldbody>", snippet)
            return
        snippet = make_urdf_joint_block(
            values["joint_name"],
            values["joint_type"],
            values["parent_link"],
            values["link_name"],
            [0.0, 0.0, 0.0],
            axis_xyz=values["axis"],
        )
        self._insert_before_close_tag("</robot>", snippet)

    def insert_builder_inertial(self) -> None:
        payload = self._builder_metrics()
        if payload is None:
            return
        values, metrics = payload
        snippet = make_urdf_inertial_block(metrics, values["density"])
        self.source_editor.insertPlainText(snippet)
        self.source_editor.document().setModified(True)

    def insert_builder_assembly(self) -> None:
        payload = self._builder_metrics()
        if payload is None:
            return
        values, metrics = payload
        center = np.array(metrics[values["origin_mode"]], dtype=float)
        origin = (-center).tolist()
        rel_path = values["asset_path"]
        if self.report["kind"] == "mjcf":
            mesh_name = safe_name(Path(rel_path).stem)
            asset_snippet = f'<mesh name="{mesh_name}" file="{rel_path}" scale="{format_xyz(values["scale"])}"/>\n'
            body_snippet = (
                f'<body name="{values["link_name"]}" pos="0 0 0">\n'
                f'  <joint name="{values["joint_name"]}" type="{values["joint_type"]}" axis="{format_xyz(values["axis"])}"/>\n'
                f'  <geom type="mesh" mesh="{mesh_name}" pos="{format_xyz(origin)}" quat="1 0 0 0"/>\n'
                f'</body>\n'
            )
            self._insert_before_close_tag("</asset>", asset_snippet)
            self._insert_before_close_tag("</worldbody>", body_snippet)
            return

        inertial_block = make_urdf_inertial_block(metrics, values["density"]) if values["include_inertial"] else ""
        if values["include_inertial"]:
            snippet = (
                f'<link name="{values["link_name"]}">\n'
                + "".join(f"  {line}\n" for line in inertial_block.strip().splitlines())
                + f'  <visual>\n'
                f'    <origin xyz="{format_xyz(origin)}" rpy="0 0 0"/>\n'
                f'    <geometry>\n'
                f'      <mesh filename="{rel_path}" scale="{format_xyz(values["scale"])}"/>\n'
                f'    </geometry>\n'
                f'  </visual>\n'
                f'  <collision>\n'
                f'    <origin xyz="{format_xyz(origin)}" rpy="0 0 0"/>\n'
                f'    <geometry>\n'
                f'      <mesh filename="{rel_path}" scale="{format_xyz(values["scale"])}"/>\n'
                f'    </geometry>\n'
                f'  </collision>\n'
                f'</link>\n'
            )
        else:
            snippet = (
                f'<link name="{values["link_name"]}">\n'
                f'  <visual>\n'
                f'    <origin xyz="{format_xyz(origin)}" rpy="0 0 0"/>\n'
                f'    <geometry>\n'
                f'      <mesh filename="{rel_path}" scale="{format_xyz(values["scale"])}"/>\n'
                f'    </geometry>\n'
                f'  </visual>\n'
                f'  <collision>\n'
                f'    <origin xyz="{format_xyz(origin)}" rpy="0 0 0"/>\n'
                f'    <geometry>\n'
                f'      <mesh filename="{rel_path}" scale="{format_xyz(values["scale"])}"/>\n'
                f'    </geometry>\n'
                f'  </collision>\n'
                f'</link>\n'
            )
        joint_snippet = make_urdf_joint_block(
            values["joint_name"],
            values["joint_type"],
            values["parent_link"],
            values["link_name"],
            [0.0, 0.0, 0.0],
            axis_xyz=values["axis"],
        )
        self._insert_before_close_tag("</robot>", snippet + joint_snippet)

    def select_link(self, name: str) -> None:
        self.selected_kind = "link"
        self.selected_name = name
        self.canvas.set_nudge_target("link", name)
        link = self.inspector.links[name]
        self.canvas.focus_link(name)
        self.source_editor.jump_to_token(f'name="{name}"')
        self.camera_widget.set_pose(self.canvas.camera_pose())
        idx = self.builder_widget.parent_link_combo.findText(name)
        if idx >= 0:
            self.builder_widget.parent_link_combo.setCurrentIndex(idx)
        mesh_metrics = []
        for mesh in link.meshes:
            metrics = self.inspector.geometry_metrics_for_mesh(mesh)
            if metrics is not None:
                mesh_metrics.append(
                    {
                        "mesh_name": mesh.mesh_name,
                        "role": mesh.visual_or_collision,
                        "recommended_frame_origin": metrics.get("recommended_frame_origin"),
                        "center_mass": metrics.get("center_mass"),
                        "surface_centroid": metrics.get("surface_centroid"),
                        "bbox_center": metrics.get("bbox_center"),
                        "extents": metrics.get("extents"),
                        "volume": metrics.get("volume"),
                    }
                )
        self.selection_text.setPlainText(
            json.dumps(
                {
                    "kind": "link",
                    "name": link.name,
                    "parent": link.parent,
                    "child_links": link.child_links,
                    "joints_out": link.joints_out,
                    "mesh_count": len(link.meshes),
                    "world_pos": link.world_pos,
                    "world_quat_wxyz": link.world_quat_wxyz,
                    "mesh_metrics": mesh_metrics,
                },
                indent=2,
            )
        )

    def select_joint(self, name: str) -> None:
        self.selected_kind = "joint"
        self.selected_name = name
        self.canvas.set_nudge_target("joint", name)
        joint = self.inspector.joints[name]
        self.canvas.focus_joint(name)
        self.source_editor.jump_to_token(f'name="{name}"')
        self.camera_widget.set_pose(self.canvas.camera_pose())
        self.selection_text.setPlainText(json.dumps(asdict(joint), indent=2))

    def _on_source_modified(self, modified: bool) -> None:
        self.source_dirty_label.setText("modified" if modified else "clean")

    def _update_source_labels(self) -> None:
        self.source_path_label.setText(str(self.current_xml_path))
        self.source_dirty_label.setText("modified" if self.source_editor.document().isModified() else "clean")

    def save_draft(self) -> Optional[Path]:
        draft_path = draft_descriptor_path(self.current_xml_path)
        try:
            draft_path.write_text(self.source_editor.toPlainText(), encoding="utf-8")
        except Exception as exc:
            self.log_health(f"save draft failed: {exc}")
            return None
        self.current_xml_path = draft_path
        self.source_editor.document().setModified(False)
        self._update_source_labels()
        self.log_health(f"saved draft to {draft_path}")
        self.reload_from_current_source()
        return draft_path

    def reload_from_current_source(self) -> None:
        self.reload_from_disk()

    def load_original_source(self) -> None:
        self.current_xml_path = self.original_xml_path
        self.log_health(f"reloading original source {self.original_xml_path}")
        self.reload_from_current_source()

    def _commit_nudge(self, kind: str, name: str, delta: List[float]) -> None:
        if self.report["kind"] != "urdf":
            self.log_health("nudge commit ignored: only URDF editing is supported")
            return
        if not self._apply_urdf_nudge_to_editor(kind, name, np.array(delta, dtype=float)):
            self.log_health(f"nudge commit failed for {kind} {name}")
            return
        self.log_health(f"nudge committed to editor for {kind} {name}: {delta}")
        self.preview_reload_from_editor()

    def _apply_urdf_nudge_to_editor(self, kind: str, name: str, delta: np.ndarray) -> bool:
        original_text = self.source_editor.toPlainText()
        try:
            root = ET.fromstring(original_text)
        except Exception as exc:
            self.log_health(f"editor parse failed during nudge: {exc}")
            return False

        changed = False
        if kind == "joint":
            for joint in root.findall("joint"):
                if joint.attrib.get("name") != name:
                    continue
                origin = joint.find("origin")
                if origin is None:
                    origin = ET.SubElement(joint, "origin")
                    origin.set("rpy", "0 0 0")
                    origin.set("xyz", "0 0 0")
                xyz = np.array(parse_floats(origin.attrib.get("xyz"), 3, [0.0, 0.0, 0.0]), dtype=float) + delta
                origin.set("xyz", format_xyz(xyz.tolist()))
                changed = True
                break
        elif kind == "link":
            for link in root.findall("link"):
                if link.attrib.get("name") != name:
                    continue
                for tag in ["visual", "collision"]:
                    for elem in link.findall(tag):
                        origin = elem.find("origin")
                        if origin is None:
                            origin = ET.SubElement(elem, "origin")
                            origin.set("rpy", "0 0 0")
                            origin.set("xyz", "0 0 0")
                        xyz = np.array(parse_floats(origin.attrib.get("xyz"), 3, [0.0, 0.0, 0.0]), dtype=float) + delta
                        origin.set("xyz", format_xyz(xyz.tolist()))
                        changed = True
                break
        if not changed:
            return False
        self.source_editor.setPlainText(serialize_xml_preserving_header(original_text, root))
        self.source_editor.document().setModified(True)
        return True

    def preview_reload_from_editor(self) -> None:
        t0 = time.perf_counter()
        camera_pose = self.canvas.camera_pose()
        try:
            preview_inspector = RobotStructureInspector(
                self.current_xml_path,
                mesh_mode=self.inspector.mesh_mode,
                max_faces_per_mesh=self.inspector.max_faces_per_mesh,
                max_points_per_mesh=self.inspector.max_points_per_mesh,
                xml_text=self.source_editor.toPlainText(),
            )
            preview_report = preview_inspector.inspect()
            self.inspector = preview_inspector
            self.report = preview_report
            self.canvas.reload_scene(preview_inspector)
            self.canvas.apply_camera_pose(camera_pose)
            self._rebuild_views()
            self.camera_widget.set_pose(self.canvas.camera_pose())
            self._update_source_labels()
            self.log_health(f"preview reload from editor in {time.perf_counter() - t0:.3f}s")
        except Exception as exc:
            self.log_health(f"preview reload failed: {exc}")


def output_paths(xml_path: Path, output_dir: Optional[Path] = None) -> Tuple[Path, Path]:
    out_dir = output_dir.resolve() if output_dir is not None else (xml_path.parent / "inspection")
    out_dir.mkdir(parents=True, exist_ok=True)
    stem = safe_name(xml_path.stem)
    return out_dir / f"{stem}_structure_report.json", out_dir / f"{stem}_structure_report_native.txt"


def draft_descriptor_path(path: Path) -> Path:
    if path.stem.endswith("-draft"):
        return path
    return path.with_name(f"{path.stem}-draft{path.suffix}")


def format_xyz(values: List[float]) -> str:
    return " ".join(f"{v:.6f}".rstrip("0").rstrip(".") if abs(v) > 1e-12 else "0" for v in values)


def suggest_uniform_scale(metrics: Dict[str, Any]) -> List[float]:
    max_extent = max(abs(v) for v in metrics.get("extents", [1.0]))
    if max_extent > 10.0:
        return [0.001, 0.001, 0.001]
    return [1.0, 1.0, 1.0]


def estimate_inertial_from_metrics(metrics: Dict[str, Any], density: float) -> Dict[str, Any]:
    extents = np.array(metrics.get("extents", [0.1, 0.1, 0.1]), dtype=float)
    volume = metrics.get("volume")
    if volume is None or volume <= 0:
        volume = float(np.prod(np.maximum(extents, 1e-6)))
    mass = float(max(volume * density, 1e-9))
    ixx = mass / 12.0 * float(extents[1] ** 2 + extents[2] ** 2)
    iyy = mass / 12.0 * float(extents[0] ** 2 + extents[2] ** 2)
    izz = mass / 12.0 * float(extents[0] ** 2 + extents[1] ** 2)
    return {
        "mass": mass,
        "origin_xyz": [0.0, 0.0, 0.0],
        "inertia": {
            "ixx": ixx,
            "iyy": iyy,
            "izz": izz,
            "ixy": 0.0,
            "ixz": 0.0,
            "iyz": 0.0,
        },
        "volume": volume,
    }


def make_urdf_inertial_block(metrics: Dict[str, Any], density: float) -> str:
    inertial = estimate_inertial_from_metrics(metrics, density)
    I = inertial["inertia"]
    return (
        f'<inertial>\n'
        f'  <origin xyz="{format_xyz(inertial["origin_xyz"])}" rpy="0 0 0"/>\n'
        f'  <mass value="{inertial["mass"]:.9g}"/>\n'
        f'  <inertia ixx="{I["ixx"]:.9g}" ixy="{I["ixy"]:.9g}" ixz="{I["ixz"]:.9g}" iyy="{I["iyy"]:.9g}" iyz="{I["iyz"]:.9g}" izz="{I["izz"]:.9g}"/>\n'
        f'</inertial>\n'
    )


def make_urdf_joint_block(joint_name: str, joint_type: str, parent_link: str, child_link: str, origin_xyz: List[float], axis_xyz: Optional[List[float]] = None) -> str:
    axis_line = ""
    if joint_type != "fixed" and axis_xyz is not None:
        axis_line = f'  <axis xyz="{format_xyz(axis_xyz)}"/>\n'
    return (
        f'<joint name="{joint_name}" type="{joint_type}">\n'
        f'  <origin xyz="{format_xyz(origin_xyz)}" rpy="0 0 0"/>\n'
        f'  <parent link="{parent_link}"/>\n'
        f'  <child link="{child_link}"/>\n'
        f'{axis_line}'
        f'</joint>\n'
    )


def make_urdf_builder_scaffold(
    rel_path: str,
    link_name: str,
    parent_link: str,
    joint_name: str,
    metrics: Dict[str, Any],
    scale: Optional[List[float]] = None,
) -> str:
    center = np.array(metrics["recommended_frame_origin"], dtype=float)
    mesh_origin = (-center).tolist()
    scale = scale or [1.0, 1.0, 1.0]
    return (
        f'<link name="{link_name}">\n'
        f'  <visual>\n'
        f'    <origin xyz="{format_xyz(mesh_origin)}" rpy="0 0 0"/>\n'
        f'    <geometry>\n'
        f'      <mesh filename="{rel_path}" scale="{format_xyz(scale)}"/>\n'
        f'    </geometry>\n'
        f'  </visual>\n'
        f'  <collision>\n'
        f'    <origin xyz="{format_xyz(mesh_origin)}" rpy="0 0 0"/>\n'
        f'    <geometry>\n'
        f'      <mesh filename="{rel_path}" scale="{format_xyz(scale)}"/>\n'
        f'    </geometry>\n'
        f'  </collision>\n'
        f'</link>\n'
        + make_urdf_joint_block(joint_name, "fixed", parent_link, link_name, [0.0, 0.0, 0.0])
    )


def make_mjcf_builder_scaffold(rel_path: str, mesh_name: str, body_name: str, metrics: Dict[str, Any], scale: Optional[List[float]] = None) -> str:
    center = np.array(metrics["recommended_frame_origin"], dtype=float)
    scale = scale or [1.0, 1.0, 1.0]
    return (
        f'<mesh name="{mesh_name}" file="{rel_path}" scale="{format_xyz(scale)}"/>\n'
        f'<!-- add the mesh above inside <asset>, then use the body block below inside <worldbody> or a parent <body> -->\n'
        f'<body name="{body_name}" pos="0 0 0">\n'
        f'  <geom type="mesh" mesh="{mesh_name}" pos="{format_xyz((-center).tolist())}" quat="1 0 0 0"/>\n'
        f'</body>\n'
    )


def make_self_test_edit(source_text: str, kind: str) -> str:
    attr_name = "pos" if kind == "mjcf" else "xyz"
    pattern = rf'{attr_name}="([^"]+)"'
    match = re.search(pattern, source_text)
    if match:
        vals = [float(x) for x in match.group(1).replace(",", " ").split()]
        if len(vals) >= 3:
            vals[0] += 0.01
            new_val = " ".join(f"{v:.6f}".rstrip("0").rstrip(".") for v in vals)
            return source_text[: match.start(1)] + new_val + source_text[match.end(1) :]
    close_tag = "</mujoco>" if kind == "mjcf" else "</robot>"
    if close_tag in source_text:
        return source_text.replace(close_tag, "  <!-- codex self-test draft edit -->\n" + close_tag, 1)
    return source_text + "\n<!-- codex self-test draft edit -->\n"


def serialize_xml_preserving_header(original_text: str, root: ET.Element) -> str:
    ET.indent(root)
    body = ET.tostring(root, encoding="unicode")
    stripped = original_text.lstrip()
    if stripped.startswith("<?xml"):
        return '<?xml version="1.0"?>\n' + body
    return body


def parse_camera_pose(text: Optional[str], default_pose: Optional[Dict[str, Any]] = None) -> Optional[Dict[str, Any]]:
    if text is None:
        return default_pose
    parts = [float(x) for x in text.replace(",", " ").split()]
    if len(parts) not in (3, 5):
        raise ValueError("camera pose expects 3 values (azimuth,elevation,distance) or 5 values (+roll,fov)")
    pose = {
        "azimuth": parts[0],
        "elevation": parts[1],
        "distance": parts[2],
        "roll": 0.0 if len(parts) == 3 else parts[3],
        "fov": 50.0 if len(parts) == 3 else parts[4],
        "center": [0.0, 0.0, 0.0],
    }
    if default_pose is not None:
        pose["center"] = list(default_pose["center"])
    return pose


def parse_center(text: Optional[str], default_center: List[float]) -> List[float]:
    if text is None:
        return default_center
    parts = [float(x) for x in text.replace(",", " ").split()]
    if len(parts) != 3:
        raise ValueError("camera center expects 3 comma-separated values")
    return parts


def main() -> None:
    ap = argparse.ArgumentParser(description="Native Qt + VisPy robot structure inspector for MJCF, URDF, and Xacro.")
    ap.add_argument("xml_path", type=Path, help="Path to MJCF/URDF/Xacro file")
    ap.add_argument("--mesh-mode", choices=["full", "wireframe", "points"], default="full")
    ap.add_argument("--max-faces-per-mesh", type=int, default=50000)
    ap.add_argument("--max-points-per-mesh", type=int, default=4000)
    ap.add_argument("--output-dir", type=Path, default=None, help="Optional directory for JSON/summary outputs")
    ap.add_argument("--camera-pose", type=str, default=None, help="Initial camera pose: azimuth,elevation,distance[,roll,fov]")
    ap.add_argument("--camera-center", type=str, default=None, help="Initial camera center: x,y,z")
    ap.add_argument("--auto-quit-seconds", type=float, default=0.0, help="Close the GUI automatically after N seconds")
    ap.add_argument("--demo-orbit-seconds", type=float, default=0.0, help="Run the demo orbit for N seconds after launch")
    ap.add_argument("--self-test", action="store_true", help="Run automated GUI smoke test and auto-exit")
    ap.add_argument("--self-test-live-edit", action="store_true", help="In self-test mode, edit source text, save a draft, reload, and render again")
    ap.add_argument("--self-test-builder", action="store_true", help="In self-test mode, use the form builder to insert an assembly, save a draft, reload, and render again")
    ap.add_argument("--self-test-nudge", action="store_true", help="In self-test mode, apply a viewport nudge to the selected link, update the editor, preview reload, and render again")
    ap.add_argument("--self-test-gizmo", action="store_true", help="In self-test mode, pick an axis gizmo handle, drag it through the live viewport path, save a draft, reload, and render again")
    ap.add_argument("--self-test-camera-controls", action="store_true", help="In self-test mode, exercise pan, click-to-center, camera presets, and selection focus in the live GUI")
    ap.add_argument("--self-test-timeout", type=float, default=8.0, help="Max seconds for GUI self-test before forced close")
    ap.add_argument("--no-live-reload", action="store_true", help="Disable file watching and auto-reload")
    ap.add_argument("--dump-only", action="store_true", help="Write JSON only and do not launch the GUI")
    args = ap.parse_args()

    inspector = RobotStructureInspector(
        args.xml_path,
        mesh_mode=args.mesh_mode,
        max_faces_per_mesh=args.max_faces_per_mesh,
        max_points_per_mesh=args.max_points_per_mesh,
    )
    report = inspector.inspect()
    json_path, summary_path = output_paths(args.xml_path.resolve(), args.output_dir)
    json_path.write_text(json.dumps(report, indent=2), encoding="utf-8")
    summary_path.write_text(
        "\n".join(
            [
                f"source_file: {report['source_file']}",
                f"kind: {report['kind']}",
                f"model_name: {report['model_name']}",
                f"links: {report['link_count']}",
                f"joints: {report['joint_count']}",
                f"warnings: {len(report['warnings'])}",
            ]
        ),
        encoding="utf-8",
    )

    print(f"Wrote JSON report: {json_path}")
    print(f"Wrote native summary: {summary_path}")

    if args.dump_only:
        return

    os.environ.setdefault("VISPY_APP", "pyside6")
    app = QtWidgets.QApplication(sys.argv)
    pose = {
        "azimuth": 35.0,
        "elevation": 20.0,
        "distance": 2.0,
        "roll": 0.0,
        "fov": 50.0,
        "center": [0.0, 0.0, 0.0],
    }
    pose = parse_camera_pose(args.camera_pose, pose) or pose
    pose["center"] = parse_center(args.camera_center, pose["center"])
    win = InspectorMainWindow(
        inspector,
        report,
        json_path,
        args.xml_path.resolve(),
        camera_pose=pose,
        live_reload=not args.no_live_reload,
    )
    win.show()
    win.log_health("window shown")
    if args.demo_orbit_seconds > 0:
        win.canvas.start_demo_orbit()
        QtCore.QTimer.singleShot(int(args.demo_orbit_seconds * 1000), win.canvas.stop_demo_orbit)
    if args.auto_quit_seconds > 0:
        QtCore.QTimer.singleShot(int(args.auto_quit_seconds * 1000), win.close)
    if args.self_test:
        tester = ViewerSelfTest(
            win,
            args.self_test_timeout,
            live_edit=args.self_test_live_edit,
            builder_insert=args.self_test_builder,
            nudge_test=args.self_test_nudge,
            gizmo_test=args.self_test_gizmo,
            camera_controls_test=args.self_test_camera_controls,
        )
        QtCore.QTimer.singleShot(120, tester.start)
        win._self_tester = tester
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
