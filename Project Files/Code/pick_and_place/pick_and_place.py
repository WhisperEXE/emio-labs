"""
High precision pick-and-place lab with configurable leg models, inverse formulations,
and benchmark logging.

Top-level layout:
- Scene and formulation configuration: all user-tuned options and thresholds.
- Optional dependency discovery: advanced leg-model factories and reduced formulation backends.
- Runtime controllers: camera bootstrapping, pick/place state machine, evaluation.
- Scene assembly: EMIO construction, inverse wiring, GUI registration.
"""

import csv
import importlib
import os
import sys
import numpy as np
import Sofa
import Sofa.ImGui as MyGui
import parameters

from datetime import datetime
from types import SimpleNamespace
from parts.controllers.assemblycontroller import AssemblyController
from parts.controllers.trackercontroller import DotTracker
from parts.emio import Emio, getParserArgs
from parts.gripper import Gripper
from utils.header import addHeader, addSolvers

try:
    import Sofa.SoftRobotsInverse as SoftRobotsInverse
except Exception as exc:
    SoftRobotsInverse = None
    SOFTROBOTS_INVERSE_IMPORT_ERROR = exc
else:
    SOFTROBOTS_INVERSE_IMPORT_ERROR = None


# Import-paths for local lab modules, shared assets, and external plugins
CURRENT_DIR = os.path.dirname(os.path.realpath(__file__))
ASSETS_DIR = os.path.abspath(os.path.join(CURRENT_DIR, "../../"))
EXTERNAL_PLUGINS_DIR = os.path.abspath(
    os.path.join(CURRENT_DIR, "../../../../external_plugins")
)

for extra_path in (CURRENT_DIR, ASSETS_DIR):
    if extra_path not in sys.path:
        sys.path.append(extra_path)


# ---------------------------------------------------------------------------
# Scene and IK solver configuration
# Keeps all user-tuned selections, thresholds, and UI labels in one place
# ---------------------------------------------------------------------------


# Model and IK solver selection
LEG_MODEL_OPTIONS = [
    "beam",
    "cosserat",
    "tetra",
    "tetra_linear",
    "hyper",
    "nonuniform",
]
INTEGRATED_FORMULATION = "integrated"
REDUCED_ACTUATOR_SPACE_FORMULATION = "reduced_actuator_space"
OIM_INSPIRED_FORMULATION = "oim_inspired"

IK_FORMULATION_OPTIONS = [
    INTEGRATED_FORMULATION,
    REDUCED_ACTUATOR_SPACE_FORMULATION,
    OIM_INSPIRED_FORMULATION,
]
IK_FORMULATION_LABELS = {
    INTEGRATED_FORMULATION: "Integrated inverse formulation",
    REDUCED_ACTUATOR_SPACE_FORMULATION: "Reduced actuator-space formulation",
    OIM_INSPIRED_FORMULATION: "OIM-inspired formulation",
}
IK_FORMULATION_SHORT_LABELS = {
    INTEGRATED_FORMULATION: "Formulation I",
    REDUCED_ACTUATOR_SPACE_FORMULATION: "Formulation II",
    OIM_INSPIRED_FORMULATION: "Formulation III",
}
ADVANCED_LEG_MODELS = {"tetra_linear", "hyper", "nonuniform"}
CUSTOM_REDUCED_FORMULATIONS = {
    REDUCED_ACTUATOR_SPACE_FORMULATION,
    OIM_INSPIRED_FORMULATION,
}
PASSIVE_INVERSE_TARGET_FORMULATIONS = {REDUCED_ACTUATOR_SPACE_FORMULATION}

# Change model/solver combination through these indexes:
LEG_MODEL_INDEX = 0
IK_FORMULATION_INDEX = 0

# Use the same timestep across all combinations
SCENE_DT = 0.03

if not 0 <= LEG_MODEL_INDEX < len(LEG_MODEL_OPTIONS):
    raise ValueError(
        f"LEG_MODEL_INDEX={LEG_MODEL_INDEX} is out of range for {LEG_MODEL_OPTIONS}"
    )
if not 0 <= IK_FORMULATION_INDEX < len(IK_FORMULATION_OPTIONS):
    raise ValueError(
        f"IK_FORMULATION_INDEX={IK_FORMULATION_INDEX} is out of range "
        f"for {IK_FORMULATION_OPTIONS}"
    )

SELECTED_LEG_MODEL = LEG_MODEL_OPTIONS[LEG_MODEL_INDEX]
SELECTED_IK_FORMULATION = IK_FORMULATION_OPTIONS[IK_FORMULATION_INDEX]
SELECTED_IK_FORMULATION_LABEL = IK_FORMULATION_LABELS[SELECTED_IK_FORMULATION]
SELECTED_IK_FORMULATION_SHORT_LABEL = IK_FORMULATION_SHORT_LABELS[
    SELECTED_IK_FORMULATION
]
SELECTED_SCENE_DT = SCENE_DT
USES_CUSTOM_REDUCED_FORMULATION = SELECTED_IK_FORMULATION in CUSTOM_REDUCED_FORMULATIONS
USES_PASSIVE_INVERSE_TARGET = (
    SELECTED_IK_FORMULATION in PASSIVE_INVERSE_TARGET_FORMULATIONS
)

IK_SOLVER_OPTIONS = IK_FORMULATION_OPTIONS
CUSTOM_IK_SOLVERS = CUSTOM_REDUCED_FORMULATIONS
PASSIVE_INVERSE_TARGET_SOLVERS = PASSIVE_INVERSE_TARGET_FORMULATIONS
IK_SOLVER_INDEX = IK_FORMULATION_INDEX
SELECTED_IK_SOLVER = SELECTED_IK_FORMULATION
USES_CUSTOM_INVERSE_SOLVER = USES_CUSTOM_REDUCED_FORMULATION


# Benchmark export, marker mappings and timing thresholds
BENCHMARK_OUTPUT_DIR = os.path.join(CURRENT_DIR, "benchmark_outputs")
BENCHMARK_SUMMARY_FILENAME = "benchmark_summary.csv"
BENCHMARK_MAX_CYCLE_TIME_S = 35.0

# Configure these to turn on/off evaluation:
BENCHMARK_ENABLED = True
EXPORT_BENCHMARK_CSV = True
ADD_TO_SUMMARY_CSV = True

# Marker mappings for camera (not tested)
OBJECT_MARKER_INDEX = 0
PLACE_MARKER_INDEX = 1
USE_CAMERA_MARKERS = False

# Pick-and-place constants and state thresholds
HOME_XYZ = [0.0, -120.0, 0.0]
PREGRASP_HEIGHT_MM = 35.0
GRASP_HEIGHT_ABOVE_MARKER_MM = 15.0
LIFT_HEIGHT_MM = 70.0
PLACE_RELEASE_HEIGHT_MM = 15.0
MOVE_THRESHOLD_MM = 25.0
GRASP_DISTANCE_THRESHOLD_MM = 15.0
PLACE_XZ_THRESHOLD_MM = 15.0
PLACE_HEIGHT_THRESHOLD_MM = 20.0
GRIPPER_OPEN_MM = 58.0
GRIPPER_CLOSED_MM = 14.0
GRIPPER_CLOSED_TOLERANCE_MM = 2.0
STARTUP_WAIT_S = 3.0
GRIPPER_SETTLE_TIME_S = 0.8
RELEASE_SETTLE_TIME_S = 0.6
OBJECT_FOLLOW_OFFSET = [0.0, 0.0, 0.0]
CUSTOM_FORMULATION_GRIP_WEIGHT_OPEN = 0.5
CUSTOM_FORMULATION_GRIP_WEIGHT_CLOSED = 2.0
CUSTOM_FORMULATION_GRIP_WEIGHT_SWITCH_MM = 0.5 * (GRIPPER_OPEN_MM + GRIPPER_CLOSED_MM)
CUSTOM_FORMULATION_TARGET_STEP_MM = 5.0
CUSTOM_FORMULATION_GRIP_STEP_MM = 10.0

# Backward-compatible aliases for the previous tuning constant names
CUSTOM_SOLVER_GRIP_WEIGHT_OPEN = CUSTOM_FORMULATION_GRIP_WEIGHT_OPEN
CUSTOM_SOLVER_GRIP_WEIGHT_CLOSED = CUSTOM_FORMULATION_GRIP_WEIGHT_CLOSED
CUSTOM_SOLVER_GRIP_WEIGHT_SWITCH_MM = CUSTOM_FORMULATION_GRIP_WEIGHT_SWITCH_MM
CUSTOM_SOLVER_TARGET_STEP_MM = CUSTOM_FORMULATION_TARGET_STEP_MM
CUSTOM_SOLVER_GRIP_STEP_MM = CUSTOM_FORMULATION_GRIP_STEP_MM


# Configure preprogrammed pick and place targets:
SIM_PROXY_PICK_XYZ = [27.0, -170.0, 0.0]
SIM_PROXY_PLACE_XYZ = [-26.0, -170.0, -13.0]

# Scene-level visuals, retry timing, and rigid-pose defaults
SCENE_LOGGER_NAME = "PickAndPlace"
SCENE_GRAVITY = [0.0, -9810.0, 0.0]
SCENE_VISUAL_FLAGS = ["hideBehavior", "hideWireframe"]
RIGID_POSE_ORIENTATION = [0.0, 0.0, 0.0, 1.0]
TCP_RIGID_MAPPING_INDICES = [0, 1, 2, 3]
TRACKER_RETRY_FRAME_INTERVAL = 40
TRACKER_INITIAL_RETRY_FRAME = -1000

TRACKED_MARKER_SCALE = 8
TARGET_MARKER_SCALE = 15
TCP_MARKER_SCALE = 6
TRACKED_OBJECT_COLOR = [0.95, 0.2, 0.2, 1.0]
TRACKED_PLACE_COLOR = [0.1, 0.9, 0.1, 1.0]
TCP_MARKER_COLOR = [0.2, 0.6, 1.0, 1.0]
TARGET_LINEAR_SOLVER_SETTINGS = {
    "iterations": 50,
    "tolerance": 1e-10,
    "threshold": 1e-10,
}

# GUI and plotting layout.
PROGRAM_STATUS_GROUP = "Pick & Place"
PROGRAM_IO_PREFIX = "/PickAndPlace"
PROGRAM_STATUS_FIELDS = [
    ("State index", "stateIndex"),
    ("State time", "stateTime"),
    ("Tracking active", "cameraTrackingActive"),
    ("Tracker count", "cameraTrackerCount"),
    ("Object attached", "objectAttached"),
]

BENCHMARK_STATUS_GROUP = "Evaluation"
BENCHMARK_IO_PREFIX = "/PickAndPlace/Evaluation"
BENCHMARK_STATUS_FIELDS = [
    ("Elapsed time", "elapsedTime"),
    ("TCP target error", "tcpTargetError"),
    ("Peak abs motor input", "peakAbsMotorInput"),
    ("Integrated abs motor input", "integratedAbsMotorInput"),
    ("Cycle time", "cycleTime"),
    ("Timed out", "timedOut"),
]
BENCHMARK_PLOT_FIELDS = [
    ("Program state index", "stateIndex", "program"),
    ("Benchmark TCP target error (mm)", "tcpTargetError", "benchmark"),
    ("Benchmark integrated abs motor input", "integratedAbsMotorInput", "benchmark"),
]

# Optional modules used to unlock advanced leg models and reduced formulation backends
ADVANCED_MODEL_IMPORTS = {
    "tetra_linear": "modeling_techniques.volume_models.tetra_linear_fem",
    "hyper": "modeling_techniques.volume_models.hyperelastic_fem",
    "nonuniform": "modeling_techniques.volume_models.tetra_nonuniform",
}
FORMULATION_BACKEND_IMPORTS = {
    REDUCED_ACTUATOR_SPACE_FORMULATION: "myQP_lab_inversekinematics",
    OIM_INSPIRED_FORMULATION: "myOIM_lab_inversekinematics",
}


# ---------------------------------------------------------------------------
# Optional dependency discovery
# Advanced leg models and reduced formulation backends are configurable in this workspace,
# so they are resolved once up front and validated before scene assembly.
# ---------------------------------------------------------------------------
def _import_optional(module_name):
    try:
        return importlib.import_module(module_name), None
    except Exception as exc:
        return None, exc


OPTION_IMPORT_ERRORS = {}
ADVANCED_MODEL_FACTORIES = {}
FORMULATION_BACKENDS = {}
CUSTOM_IK_BACKENDS = FORMULATION_BACKENDS

for model_name, module_name in ADVANCED_MODEL_IMPORTS.items():
    module, error = _import_optional(module_name)
    if module is not None:
        ADVANCED_MODEL_FACTORIES[model_name] = module.create_model
    else:
        OPTION_IMPORT_ERRORS[model_name] = error

for formulation_name, module_name in FORMULATION_BACKEND_IMPORTS.items():
    module, error = _import_optional(module_name)
    if module is not None:
        FORMULATION_BACKENDS[formulation_name] = module
    else:
        OPTION_IMPORT_ERRORS[formulation_name] = error

# Backward-compatible alias for the previous backend import map name
CUSTOM_IK_BACKEND_IMPORTS = FORMULATION_BACKEND_IMPORTS

if SOFTROBOTS_INVERSE_IMPORT_ERROR is not None:
    OPTION_IMPORT_ERRORS["softrobots_inverse"] = SOFTROBOTS_INVERSE_IMPORT_ERROR


def _require_supported_option(option_key, dependency, kind):
    if dependency is not None:
        return dependency

    detail = OPTION_IMPORT_ERRORS.get(option_key)
    detail_text = f" ({detail})" if detail else ""
    raise RuntimeError(
        f"Selected {kind} '{option_key}' is unavailable in this environment{detail_text}."
    )


def _ensure_external_plugin_repository(node):
    if not os.path.isdir(EXTERNAL_PLUGINS_DIR):
        return

    if node.getObject("GeneralExternalPluginRepository") is None:
        node.addObject("RequiredPlugin", name="Sofa.Component.SceneUtility")
        node.addObject(
            "AddPluginRepository",
            name="GeneralExternalPluginRepository",
            path=EXTERNAL_PLUGINS_DIR,
        )


# ---------------------------------------------------------------------------
# Formulation adapters
# These wrappers let the scene swap between the integrated inverse formulation
# and the reduced actuator-space backends without changing the rest of the lab
# wiring.
# ---------------------------------------------------------------------------
if SoftRobotsInverse is not None:

    class SelectableReducedActuatorSpaceFormulation(
        SoftRobotsInverse.QPInverseProblemSolver
    ):
        def __init__(
            self,
            emio,
            assembly_controller,
            sensor_mo,
            target_mo,
            effector_mo,
            gripper_distance_mo,
            gripper_opening_data,
            get_torques,
            *args,
            **kwargs,
        ):
            SoftRobotsInverse.QPInverseProblemSolver.__init__(self, *args, **kwargs)
            self.name = "ConstraintSolver"
            self.emio = emio
            self.assembly = assembly_controller
            self.sensor = sensor_mo
            self.target = target_mo
            self.effector = effector_mo
            self.gripper_distance_mo = gripper_distance_mo
            self.gripper_opening_data = gripper_opening_data
            self.get_torques = get_torques
            self.last_torques = None

        def _gripper_objective_weight(self):
            if self.gripper_opening_data is None:
                return CUSTOM_FORMULATION_GRIP_WEIGHT_OPEN

            if (
                _scalar(self.gripper_opening_data)
                <= CUSTOM_FORMULATION_GRIP_WEIGHT_SWITCH_MM
            ):
                return CUSTOM_FORMULATION_GRIP_WEIGHT_CLOSED

            return CUSTOM_FORMULATION_GRIP_WEIGHT_OPEN

        def solveSystem(self):
            W = self.W()
            dfree = self.dfree()
            torques = self.lambda_force()
            actuator_count = len(self.emio.motors)
            iA = list(range(actuator_count))
            iE = list(range(actuator_count, actuator_count + 3))
            q_t = list(self.target.position.value[0][0:3])
            q_e = list(self.effector.position.value[0][0:3])
            weights = [1.0, 1.0, 1.0]
            gripper_row_index = actuator_count + 3
            gripper_error = _read_gripper_distance_error(self.gripper_distance_mo)
            if gripper_error is not None and gripper_row_index < len(dfree):
                iE.append(gripper_row_index)
                q_t.append(0.0)
                q_e.append(gripper_error)
                weights.append(self._gripper_objective_weight())
            q_a = [
                self.emio.motors[i].JointActuator.angle.value
                for i in range(actuator_count)
            ]
            dq_free = np.copy(dfree)
            dq_free[iA] -= q_a

            if getattr(self.assembly, "done", True):
                try:
                    sensor_xyz = self.sensor.position.value[0][0:3]
                    torques[iA] = self.get_torques(
                        W=W,
                        dq_free=dq_free,
                        iE=iE,
                        iA=iA,
                        q_s=sensor_xyz,
                        q_t=q_t,
                        q_e=q_e,
                        q_a=q_a,
                        weights=weights,
                    )
                except Exception:
                    if self.last_torques is not None:
                        torques[iA] = np.copy(self.last_torques)
                    return True

            self.last_torques = np.copy(torques[iA])
            return True

    class SelectableOIMInspiredFormulation(SoftRobotsInverse.QPInverseProblemSolver):
        def __init__(
            self,
            emio,
            assembly_controller,
            sensor_mo,
            target_mo,
            effector_mo,
            gripper_distance_mo,
            gripper_opening_data,
            get_torques,
            *args,
            **kwargs,
        ):
            SoftRobotsInverse.QPInverseProblemSolver.__init__(self, *args, **kwargs)
            self.name = "ConstraintSolver"
            self.emio = emio
            self.assembly = assembly_controller
            self.sensor = sensor_mo
            self.target = target_mo
            self.effector = effector_mo
            self.gripper_distance_mo = gripper_distance_mo
            self.gripper_opening_data = gripper_opening_data
            self.get_torques = get_torques
            self.last_torques = None

        def _gripper_objective_weight(self):
            if self.gripper_opening_data is None:
                return CUSTOM_FORMULATION_GRIP_WEIGHT_OPEN

            if (
                _scalar(self.gripper_opening_data)
                <= CUSTOM_FORMULATION_GRIP_WEIGHT_SWITCH_MM
            ):
                return CUSTOM_FORMULATION_GRIP_WEIGHT_CLOSED

            return CUSTOM_FORMULATION_GRIP_WEIGHT_OPEN

        def solveSystem(self):
            W = self.W()
            dfree = self.dfree()
            torques = self.lambda_force()
            actuator_count = len(self.emio.motors)
            iA = list(range(actuator_count))
            iE = list(range(actuator_count, actuator_count + 3))
            q_t = list(self.target.position.value[0][0:3])
            q_e = list(self.effector.position.value[0][0:3])
            weights = [1.0, 1.0, 1.0]
            gripper_row_index = actuator_count + 3
            gripper_error = _read_gripper_distance_error(self.gripper_distance_mo)
            if gripper_error is not None and gripper_row_index < len(dfree):
                iE.append(gripper_row_index)
                q_t.append(0.0)
                q_e.append(gripper_error)
                weights.append(self._gripper_objective_weight())
            q_a = [motor.JointActuator.angle.value for motor in self.emio.motors]
            dq_free = np.copy(dfree)
            dq_free[iA] -= q_a

            if getattr(self.assembly, "done", True):
                try:
                    sensor_xyz = self.sensor.position.value[0][0:3]
                    torques[iA] = self.get_torques(
                        W=W,
                        dq_free=dq_free,
                        iE=iE,
                        iA=iA,
                        q_s=sensor_xyz,
                        q_t=q_t,
                        q_e=q_e,
                        q_a=q_a,
                        weights=weights,
                    )
                except Exception:
                    if self.last_torques is not None:
                        torques[iA] = np.copy(self.last_torques)
                    return True

            self.last_torques = np.copy(torques[iA])
            return True

else:
    SelectableReducedActuatorSpaceFormulation = None
    SelectableOIMInspiredFormulation = None

# Backward-compatible aliases for old imports.
SelectableQPInverseProblemSolver = SelectableReducedActuatorSpaceFormulation
SelectableOIMInverseProblemSolver = SelectableOIMInspiredFormulation


# ---------------------------------------------------------------------------
# Shared runtime helpers
# Both runtime controllers read and write SOFA data fields, rigid poses, and
# simple distances; centralizing that logic keeps those responsibilities aligned.
# ---------------------------------------------------------------------------
def _data_read(data):
    if hasattr(data, "value"):
        return data.value
    return data


def _data_write(data, value):
    if hasattr(data, "value"):
        data.value = value
        return

    try:
        data[:] = value
        return
    except Exception:
        pass

    try:
        data[...] = value
        return
    except Exception as exc:
        raise RuntimeError(
            f"Unsupported data write for type {type(data)} with value {value}"
        ) from exc


def _scalar(data):
    raw = _data_read(data)
    try:
        return float(raw)
    except Exception:
        return float(raw[0])


def _optional_scalar(data):
    raw = _data_read(data)
    for _ in range(4):
        try:
            return float(raw)
        except Exception:
            pass

        try:
            raw = raw[0]
        except Exception:
            return None

    return None


def _rigid_pose(xyz):
    return [float(xyz[0]), float(xyz[1]), float(xyz[2]), *RIGID_POSE_ORIENTATION]


def _read_pose_xyz(mechanical_object):
    pose = _data_read(mechanical_object.position)[0]
    return [float(pose[0]), float(pose[1]), float(pose[2])]


def _distance(a, b):
    return ((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2 + (a[2] - b[2]) ** 2) ** 0.5


def _distance_xz(a, b):
    return ((a[0] - b[0]) ** 2 + (a[2] - b[2]) ** 2) ** 0.5


# ---------------------------------------------------------------------------
# Runtime controllers
# HardwareTrackerBridge attaches the camera only when hardware is ready.
# CameraDrivenPickAndPlaceController owns the task state machine.
# BenchmarkRecorder logs per-step metrics and end-of-cycle summaries.
# ---------------------------------------------------------------------------
class HardwareTrackerBridge(Sofa.Core.Controller):
    """
    Lazily create the camera tracker once the EMIO connection is active.
    This keeps the lab loadable even when the camera is not yet available.
    """

    def __init__(self, root, tracker_factory, *args, **kwargs):
        Sofa.Core.Controller.__init__(self, *args, **kwargs)
        self.name = "HardwareTrackerBridge"
        self.root = root
        self.tracker_factory = tracker_factory
        self.tracker = None
        self._frame = 0
        self._last_retry_frame = TRACKER_INITIAL_RETRY_FRAME

    def get_trackers_mo(self):
        if self.root.getChild("DepthCamera") is not None:
            if hasattr(self.root.DepthCamera, "Trackers"):
                return self.root.DepthCamera.Trackers
        if self.tracker is not None and hasattr(self.tracker, "mo"):
            return self.tracker.mo
        return None

    def onAnimateBeginEvent(self, _):
        self._frame += 1

        if self.get_trackers_mo() is not None:
            return

        motor_controller = self.root.getObject("MotorController")
        if motor_controller is None or not motor_controller.emiomotors.is_connected:
            return

        if self._frame - self._last_retry_frame < TRACKER_RETRY_FRAME_INTERVAL:
            return
        self._last_retry_frame = self._frame

        try:
            self.tracker = self.root.addObject(
                self.tracker_factory(
                    name="DotTracker",
                    root=self.root,
                    nb_tracker=2,
                    show_video_feed=False,
                    track_colors=True,
                    compute_point_cloud=False,
                    scale=1,
                )
            )
        except RuntimeError:
            Sofa.msg_error(self.name, "Camera not detected")


class CameraDrivenPickAndPlaceController(Sofa.Core.Controller):
    """
    One-shot pick-and-place state machine.

    States:
    - STARTING_UP
    - WAIT_FOR_MARKERS
    - PREGRASP_OPEN
    - APPROACH_PICK
    - CLOSE_GRIPPER
    - LIFT_OBJECT
    - MOVE_ABOVE_PLACE
    - LOWER_TO_PLACE
    - RELEASE_OBJECT
    - RETURN_IDLE
    - DONE
    """

    STATE_ORDER = [
        "STARTING_UP",
        "WAIT_FOR_MARKERS",
        "PREGRASP_OPEN",
        "APPROACH_PICK",
        "CLOSE_GRIPPER",
        "LIFT_OBJECT",
        "MOVE_ABOVE_PLACE",
        "LOWER_TO_PLACE",
        "RELEASE_OBJECT",
        "RETURN_IDLE",
        "DONE",
    ]

    def __init__(
        self,
        root,
        target_mo,
        tcp_mo,
        passive_inverse_target_mo,
        gripper_opening_data,
        gripper_distance_mo,
        object_mo,
        place_marker_mo,
        tracker_bridge,
        dt,
        *args,
        **kwargs,
    ):
        Sofa.Core.Controller.__init__(self, *args, **kwargs)
        self.name = "CameraDrivenPickAndPlaceController"

        self.root = root
        self.target_mo = target_mo
        self.tcp_mo = tcp_mo
        self.passive_inverse_target_mo = passive_inverse_target_mo
        self.gripper_opening_data = gripper_opening_data
        self.gripper_distance_mo = gripper_distance_mo
        self.object_mo = object_mo
        self.place_marker_mo = place_marker_mo
        self.tracker_bridge = tracker_bridge
        self.dt = float(dt.value)

        # Read-only telemetry for the GUI.
        self.state_index_data = self.addData(name="stateIndex", type="float", value=0.0)
        self.state_time_data = self.addData(name="stateTime", type="float", value=0.0)
        self.camera_tracking_active = self.addData(
            name="cameraTrackingActive", type="float", value=0.0
        )
        self.camera_tracker_count = self.addData(
            name="cameraTrackerCount", type="float", value=0.0
        )
        self.object_attached_data = self.addData(
            name="objectAttached", type="float", value=0.0
        )

        self._state = "STARTING_UP"
        self._time_in_state = 0.0
        self._object_attached = False
        self._cycle_done = False
        self._cycle_pick_xyz = None
        self._cycle_place_xyz = None
        self._state_handlers = {
            "STARTING_UP": self._run_starting_up,
            "WAIT_FOR_MARKERS": self._run_wait_for_markers,
            "PREGRASP_OPEN": self._run_pregrasp_open,
            "APPROACH_PICK": self._run_approach_pick,
            "CLOSE_GRIPPER": self._run_close_gripper,
            "LIFT_OBJECT": self._run_lift_object,
            "MOVE_ABOVE_PLACE": self._run_move_above_place,
            "LOWER_TO_PLACE": self._run_lower_to_place,
            "RELEASE_OBJECT": self._run_release_object,
            "RETURN_IDLE": self._run_return_idle,
            "DONE": self._run_done,
        }

        self._set_target_pose(HOME_XYZ)
        self._set_gripper_opening(GRIPPER_OPEN_MM)
        self._publish_runtime_status()

    def _publish_runtime_status(self):
        _data_write(self.state_index_data, float(self.STATE_ORDER.index(self._state)))
        _data_write(self.state_time_data, float(self._time_in_state))
        _data_write(self.object_attached_data, 1.0 if self._object_attached else 0.0)

        tracker_count = self._tracker_count()
        _data_write(self.camera_tracker_count, float(tracker_count))
        tracking_active = self._read_tracker_xyz(OBJECT_MARKER_INDEX) is not None
        tracking_active = (
            tracking_active and self._read_tracker_xyz(PLACE_MARKER_INDEX) is not None
        )
        _data_write(self.camera_tracking_active, 1.0 if tracking_active else 0.0)

    def _set_state(self, state_name):
        self._state = state_name
        self._time_in_state = 0.0
        self._publish_runtime_status()

        msg = f"[CameraDrivenPickAndPlace] state={state_name}"
        print(msg)
        try:
            Sofa.msg_info(self.name, msg)
        except Exception:
            pass

    def _get_trackers_mo(self):
        if self.tracker_bridge is None:
            return None
        return self.tracker_bridge.get_trackers_mo()

    def _hardware_connected(self):
        motor_controller = self.root.getObject("MotorController")
        if motor_controller is None:
            return False
        return motor_controller.emiomotors.is_connected

    def _tracker_count(self):
        trackers_mo = self._get_trackers_mo()
        if trackers_mo is None:
            return 0
        try:
            return len(_data_read(trackers_mo.position))
        except Exception:
            return 0

    def _read_tracker_xyz(self, index):
        trackers_mo = self._get_trackers_mo()
        if trackers_mo is None:
            return None

        positions = _data_read(trackers_mo.position)
        if index < 0 or index >= len(positions):
            return None

        p = positions[index]
        return [float(p[0]), float(p[1]), float(p[2])]

    def _read_tcp_xyz(self):
        return _read_pose_xyz(self.tcp_mo)

    def _read_object_xyz(self):
        return _read_pose_xyz(self.object_mo)

    def _read_place_xyz(self):
        """
        Read the current proxy placement pose from the scene.

        This is only used by the optional simulation fallback path so the lab
        can be tested without a live camera connection.
        """
        return _read_pose_xyz(self.place_marker_mo)

    def _set_target_pose(self, xyz):
        _data_write(self.target_mo.position, [_rigid_pose(xyz)])

    def _set_smoothed_target_pose(self, xyz):
        if not USES_CUSTOM_REDUCED_FORMULATION:
            self._set_target_pose(xyz)
            return

        current_xyz = _read_pose_xyz(self.target_mo)
        delta = [float(xyz[i]) - current_xyz[i] for i in range(3)]
        distance = _distance(current_xyz, xyz)
        if distance <= CUSTOM_FORMULATION_TARGET_STEP_MM or distance <= 1e-9:
            self._set_target_pose(xyz)
            return

        scale = CUSTOM_FORMULATION_TARGET_STEP_MM / distance
        stepped_xyz = [current_xyz[i] + delta[i] * scale for i in range(3)]
        self._set_target_pose(stepped_xyz)

    def _sync_passive_inverse_target(self):
        if self.passive_inverse_target_mo is None:
            return

        tcp_pose = list(_data_read(self.tcp_mo.position)[0])
        _data_write(self.passive_inverse_target_mo.position, [tcp_pose])

    def _set_object_pose(self, xyz):
        _data_write(self.object_mo.position, [_rigid_pose(xyz)])

    def _set_place_marker_pose(self, xyz):
        _data_write(self.place_marker_mo.position, [_rigid_pose(xyz)])

    def _set_gripper_opening(self, opening):
        if not USES_CUSTOM_REDUCED_FORMULATION:
            _data_write(self.gripper_opening_data, [opening])
            return

        current_opening = _scalar(self.gripper_opening_data)
        delta = float(opening) - current_opening
        if abs(delta) <= CUSTOM_FORMULATION_GRIP_STEP_MM:
            _data_write(self.gripper_opening_data, [opening])
            return

        stepped_opening = (
            current_opening + np.sign(delta) * CUSTOM_FORMULATION_GRIP_STEP_MM
        )
        _data_write(self.gripper_opening_data, [stepped_opening])

    def _read_actual_gripper_gap(self):
        return _read_actual_gripper_gap(
            self.gripper_opening_data, self.gripper_distance_mo
        )

    def _gripper_is_closed(self):
        if USES_CUSTOM_REDUCED_FORMULATION:
            actual_gap = self._read_actual_gripper_gap()
            if actual_gap is None:
                return False
            return actual_gap <= (GRIPPER_CLOSED_MM + GRIPPER_CLOSED_TOLERANCE_MM)

        return _scalar(self.gripper_opening_data) <= (
            GRIPPER_CLOSED_MM + GRIPPER_CLOSED_TOLERANCE_MM
        )

    def _is_near(self, target_xyz, threshold_mm):
        return _distance(self._read_tcp_xyz(), target_xyz) <= threshold_mm

    def _sync_live_camera_overlays(self):
        """
        Mirror live marker positions into the visible scene markers before the
        one-shot cycle starts.
        """
        pick_xyz = self._read_tracker_xyz(OBJECT_MARKER_INDEX)
        if pick_xyz is not None and not self._object_attached and not self._cycle_done:
            self._set_object_pose(pick_xyz)

        place_xyz = self._read_tracker_xyz(PLACE_MARKER_INDEX)
        if place_xyz is not None and not self._cycle_done:
            self._set_place_marker_pose(place_xyz)

    def _capture_camera_targets(self):
        """
        Freeze the first valid object/place markers for the full cycle.

        Minimal marker assumption:
        - tracker 0 = object marker
        - tracker 1 = place marker
        """
        pick_xyz = self._read_tracker_xyz(OBJECT_MARKER_INDEX)
        place_xyz = self._read_tracker_xyz(PLACE_MARKER_INDEX)
        if pick_xyz is None or place_xyz is None:

            if self.tracker_bridge is None or not self._hardware_connected():
                pick_xyz = self._read_object_xyz()
                place_xyz = self._read_place_xyz()
            else:
                return False

        self._cycle_pick_xyz = list(pick_xyz)
        self._cycle_place_xyz = list(place_xyz)
        self._set_object_pose(self._cycle_pick_xyz)
        self._set_place_marker_pose(self._cycle_place_xyz)
        return True

    def _current_pregrasp_xyz(self):
        return [
            self._cycle_pick_xyz[0],
            self._cycle_pick_xyz[1] + PREGRASP_HEIGHT_MM,
            self._cycle_pick_xyz[2],
        ]

    def _current_grasp_xyz(self):
        return [
            self._cycle_pick_xyz[0],
            self._cycle_pick_xyz[1] + GRASP_HEIGHT_ABOVE_MARKER_MM,
            self._cycle_pick_xyz[2],
        ]

    def _current_lift_xyz(self):
        return [
            self._cycle_pick_xyz[0],
            self._cycle_pick_xyz[1] + LIFT_HEIGHT_MM,
            self._cycle_pick_xyz[2],
        ]

    def _current_transport_xyz(self):
        transport_y = (
            max(self._cycle_pick_xyz[1], self._cycle_place_xyz[1]) + LIFT_HEIGHT_MM
        )
        return [
            self._cycle_place_xyz[0],
            transport_y,
            self._cycle_place_xyz[2],
        ]

    def _current_release_above_xyz(self):
        return [
            self._cycle_place_xyz[0],
            self._cycle_place_xyz[1] + PLACE_RELEASE_HEIGHT_MM,
            self._cycle_place_xyz[2],
        ]

    def _attach_object(self):
        self._object_attached = True
        _data_write(self.object_attached_data, 1.0)

    def _detach_object_at_place(self):
        self._object_attached = False
        _data_write(self.object_attached_data, 0.0)
        self._set_object_pose(self._cycle_place_xyz)

    def _follow_object_with_tcp(self):
        if not self._object_attached:
            return

        tcp_xyz = self._read_tcp_xyz()
        followed = [
            tcp_xyz[0] + OBJECT_FOLLOW_OFFSET[0],
            tcp_xyz[1] + OBJECT_FOLLOW_OFFSET[1],
            tcp_xyz[2] + OBJECT_FOLLOW_OFFSET[2],
        ]
        self._set_object_pose(followed)

    def _run_wait_for_markers(self):

        self._set_smoothed_target_pose(HOME_XYZ)
        self._set_gripper_opening(GRIPPER_OPEN_MM)
        self._sync_live_camera_overlays()

        if self._capture_camera_targets():
            self._set_state("PREGRASP_OPEN")

    def _run_starting_up(self):

        self._set_smoothed_target_pose(HOME_XYZ)
        self._set_gripper_opening(GRIPPER_OPEN_MM)

        if self._time_in_state >= STARTUP_WAIT_S:
            self._set_state("WAIT_FOR_MARKERS")

    def _run_pregrasp_open(self):

        pregrasp_xyz = self._current_pregrasp_xyz()
        self._set_gripper_opening(GRIPPER_OPEN_MM)
        self._set_smoothed_target_pose(pregrasp_xyz)

        if self._is_near(pregrasp_xyz, MOVE_THRESHOLD_MM):
            self._set_state("APPROACH_PICK")

    def _run_approach_pick(self):

        grasp_xyz = self._current_grasp_xyz()
        self._set_gripper_opening(GRIPPER_OPEN_MM)
        self._set_smoothed_target_pose(grasp_xyz)

        if self._is_near(grasp_xyz, GRASP_DISTANCE_THRESHOLD_MM):
            self._set_state("CLOSE_GRIPPER")

    def _run_close_gripper(self):

        grasp_xyz = self._current_grasp_xyz()
        self._set_smoothed_target_pose(grasp_xyz)
        self._set_gripper_opening(GRIPPER_CLOSED_MM)

        if (
            self._time_in_state >= GRIPPER_SETTLE_TIME_S
            and self._is_near(grasp_xyz, GRASP_DISTANCE_THRESHOLD_MM)
            and self._gripper_is_closed()
        ):
            self._attach_object()
            self._set_state("LIFT_OBJECT")

    def _run_lift_object(self):

        lift_xyz = self._current_lift_xyz()
        self._set_gripper_opening(GRIPPER_CLOSED_MM)
        self._set_smoothed_target_pose(lift_xyz)

        if self._is_near(lift_xyz, MOVE_THRESHOLD_MM):
            self._set_state("MOVE_ABOVE_PLACE")

    def _run_move_above_place(self):

        transport_xyz = self._current_transport_xyz()
        self._set_gripper_opening(GRIPPER_CLOSED_MM)
        self._set_smoothed_target_pose(transport_xyz)

        if self._is_near(transport_xyz, MOVE_THRESHOLD_MM):
            self._set_state("LOWER_TO_PLACE")

    def _run_lower_to_place(self):

        release_above_xyz = self._current_release_above_xyz()
        self._set_gripper_opening(GRIPPER_CLOSED_MM)
        self._set_smoothed_target_pose(release_above_xyz)

        tcp_xyz = self._read_tcp_xyz()
        close_in_xz = _distance_xz(tcp_xyz, release_above_xyz) <= PLACE_XZ_THRESHOLD_MM
        close_in_height = (
            abs(tcp_xyz[1] - release_above_xyz[1]) <= PLACE_HEIGHT_THRESHOLD_MM
        )
        if close_in_xz and close_in_height:

            self._set_gripper_opening(GRIPPER_OPEN_MM)
            self._set_state("RELEASE_OBJECT")

    def _run_release_object(self):

        self._set_smoothed_target_pose(self._current_release_above_xyz())
        self._set_gripper_opening(GRIPPER_OPEN_MM)

        if self._time_in_state >= RELEASE_SETTLE_TIME_S:
            self._detach_object_at_place()
            self._set_state("RETURN_IDLE")

    def _run_return_idle(self):

        self._set_gripper_opening(GRIPPER_OPEN_MM)
        self._set_smoothed_target_pose(HOME_XYZ)

        if self._is_near(HOME_XYZ, MOVE_THRESHOLD_MM):
            self._cycle_done = True
            self._set_state("DONE")

    def _run_done(self):

        self._set_gripper_opening(GRIPPER_OPEN_MM)
        self._set_smoothed_target_pose(HOME_XYZ)

    def onAnimateBeginEvent(self, _):
        self._time_in_state += self.dt
        self._sync_passive_inverse_target()
        self._publish_runtime_status()
        self._follow_object_with_tcp()
        self._state_handlers.get(self._state, self._run_done)()

        self._publish_runtime_status()


class BenchmarkRecorder(Sofa.Core.Controller):
    """
    Collect task-level and control-level evaluation data without changing the
    one-shot program logic.

    The recorder samples once per simulation step, publishes a few live metrics
    to the GUI/plotting windows, and finalizes a summary when the cycle reaches
    DONE or when the benchmark timeout is exceeded.
    """

    def __init__(
        self,
        root,
        program,
        emio,
        target_mo,
        tcp_mo,
        object_mo,
        place_marker_mo,
        gripper_opening_data,
        gripper_distance_mo,
        dt,
        *args,
        **kwargs,
    ):
        Sofa.Core.Controller.__init__(self, *args, **kwargs)
        self.name = "BenchmarkRecorder"

        self.root = root
        self.program = program
        self.emio = emio
        self.target_mo = target_mo
        self.tcp_mo = tcp_mo
        self.object_mo = object_mo
        self.place_marker_mo = place_marker_mo
        self.gripper_opening_data = gripper_opening_data
        self.gripper_distance_mo = gripper_distance_mo
        self.dt = float(dt.value)

        self.elapsed_time_data = self.addData(
            name="elapsedTime", type="float", value=0.0
        )
        self.tcp_target_error_data = self.addData(
            name="tcpTargetError", type="float", value=0.0
        )
        self.cycle_time_data = self.addData(name="cycleTime", type="float", value=0.0)
        self.peak_abs_motor_input_data = self.addData(
            name="peakAbsMotorInput", type="float", value=0.0
        )
        self.integrated_abs_motor_input_data = self.addData(
            name="integratedAbsMotorInput", type="float", value=0.0
        )
        self.timed_out_data = self.addData(name="timedOut", type="float", value=0.0)
        self.finalized_data = self.addData(name="finalized", type="float", value=0.0)

        self.samples = []
        self.state_durations = {
            state_name: 0.0 for state_name in self.program.STATE_ORDER
        }
        self._elapsed_time = 0.0
        self._peak_abs_motor_input = 0.0
        self._integrated_abs_motor_input = 0.0
        self._sum_tcp_target_error = 0.0
        self._sample_count = 0
        self._summary_finalized = False
        self.summary = None

    def _state_name(self):
        return getattr(self.program, "_state", "UNKNOWN")

    def _current_motor_angles(self):
        return [float(motor.JointActuator.angle.value) for motor in self.emio.motors]

    def _current_motor_inputs(self):
        constraint_solver = self.root.getObject("ConstraintSolver")
        if constraint_solver is None or not hasattr(constraint_solver, "lambda_force"):
            return [0.0] * len(self.emio.motors)

        try:
            raw = (
                np.array(constraint_solver.lambda_force(), dtype=float)
                .flatten()
                .tolist()
            )
        except Exception:
            return [0.0] * len(self.emio.motors)

        torques = raw[: len(self.emio.motors)]
        if len(torques) < len(self.emio.motors):
            torques.extend([0.0] * (len(self.emio.motors) - len(torques)))
        return [float(value) for value in torques]

    def _build_sample(self):
        state_name = self._state_name()
        target_xyz = _read_pose_xyz(self.target_mo)
        tcp_xyz = _read_pose_xyz(self.tcp_mo)
        object_xyz = _read_pose_xyz(self.object_mo)
        place_xyz = _read_pose_xyz(self.place_marker_mo)
        gripper_opening = _scalar(self.gripper_opening_data)
        gripper_distance_error = _read_gripper_distance_error(self.gripper_distance_mo)
        gripper_gap = _read_actual_gripper_gap(
            self.gripper_opening_data, self.gripper_distance_mo
        )
        attached = _scalar(self.program.objectAttached) > 0.5
        tcp_target_error = _distance(tcp_xyz, target_xyz)
        motor_angles = self._current_motor_angles()
        motor_inputs = self._current_motor_inputs()

        return {
            "time_s": self._elapsed_time,
            "state": state_name,
            "state_index": int(_scalar(self.program.stateIndex)),
            "target_x": target_xyz[0],
            "target_y": target_xyz[1],
            "target_z": target_xyz[2],
            "tcp_x": tcp_xyz[0],
            "tcp_y": tcp_xyz[1],
            "tcp_z": tcp_xyz[2],
            "object_x": object_xyz[0],
            "object_y": object_xyz[1],
            "object_z": object_xyz[2],
            "place_x": place_xyz[0],
            "place_y": place_xyz[1],
            "place_z": place_xyz[2],
            "gripper_opening_mm": gripper_opening,
            "gripper_gap_mm": "" if gripper_gap is None else gripper_gap,
            "gripper_distance_error_mm": (
                "" if gripper_distance_error is None else gripper_distance_error
            ),
            "attached": 1.0 if attached else 0.0,
            "tcp_target_error_mm": tcp_target_error,
            "motor0_angle_rad": motor_angles[0],
            "motor1_angle_rad": motor_angles[1],
            "motor2_angle_rad": motor_angles[2],
            "motor3_angle_rad": motor_angles[3],
            "motor0_input": motor_inputs[0],
            "motor1_input": motor_inputs[1],
            "motor2_input": motor_inputs[2],
            "motor3_input": motor_inputs[3],
        }

    def _update_live_data(self, sample):
        _data_write(self.elapsed_time_data, float(sample["time_s"]))
        _data_write(self.tcp_target_error_data, float(sample["tcp_target_error_mm"]))
        _data_write(self.peak_abs_motor_input_data, float(self._peak_abs_motor_input))
        _data_write(
            self.integrated_abs_motor_input_data,
            float(self._integrated_abs_motor_input),
        )

    def _record_sample(self):
        sample = self._build_sample()
        self.samples.append(sample)
        self.state_durations[sample["state"]] = (
            self.state_durations.get(sample["state"], 0.0) + self.dt
        )

        abs_motor_inputs = [
            abs(sample["motor0_input"]),
            abs(sample["motor1_input"]),
            abs(sample["motor2_input"]),
            abs(sample["motor3_input"]),
        ]
        self._peak_abs_motor_input = max(
            self._peak_abs_motor_input, max(abs_motor_inputs)
        )
        self._integrated_abs_motor_input += sum(abs_motor_inputs) * self.dt
        self._sum_tcp_target_error += sample["tcp_target_error_mm"]
        self._sample_count += 1

        self._update_live_data(sample)

    def _summary_stem(self):
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        return f"{SELECTED_LEG_MODEL}_{SELECTED_IK_FORMULATION}_" f"{timestamp}"

    def _summary_combo_key(self):
        return f"{SELECTED_LEG_MODEL}_{SELECTED_IK_FORMULATION}"

    def _summary_metric_descriptors(self):
        descriptors = [
            {
                "section": "Minimal baseline",
                "metric": "cycle_time_s",
                "display_name": "Cycle time (s)",
                "notes": "Total task duration.",
            },
            {
                "section": "Minimal baseline",
                "metric": "mean_tcp_target_error_mm",
                "display_name": "Mean TCP-target error (mm)",
                "notes": "Average tracking quality during the run.",
            },
            {
                "section": "Minimal baseline",
                "metric": "integrated_abs_motor_input",
                "display_name": "Integrated abs motor input",
                "notes": "Total actuation effort across the run.",
            },
            {
                "section": "Useful diagnostics",
                "metric": "timed_out",
                "display_name": "Timed out",
                "notes": "Whether the benchmark exceeded the configured time limit.",
            },
            {
                "section": "Useful diagnostics",
                "metric": "final_state",
                "display_name": "Final state",
                "notes": "Controller state where the run ended.",
            },
            {
                "section": "Useful diagnostics",
                "metric": "peak_abs_motor_input",
                "display_name": "Peak abs motor input",
                "notes": "Highest instantaneous motor effort observed.",
            },
        ]

        for state_name in self.program.STATE_ORDER:
            descriptors.append(
                {
                    "section": "Useful diagnostics",
                    "metric": f"state_{state_name.lower()}_s",
                    "display_name": f"Time in {state_name} (s)",
                    "notes": "Total time spent in this controller state.",
                }
            )

        return descriptors

    def _summary_metric_rows(self):
        if self.summary is None:
            return {}

        metric_rows = {
            "cycle_time_s": self.summary["cycle_time_s"],
            "timed_out": float(self.summary["timed_out"]),
            "final_state": self.summary["final_state"],
            "peak_abs_motor_input": self.summary["peak_abs_motor_input"],
            "integrated_abs_motor_input": self.summary["integrated_abs_motor_input"],
            "mean_tcp_target_error_mm": self.summary["mean_tcp_target_error_mm"],
        }

        for state_name, duration in self.summary["state_durations_s"].items():
            metric_rows[f"state_{state_name.lower()}_s"] = duration

        return metric_rows

    def _export_samples_if_requested(self):
        if not EXPORT_BENCHMARK_CSV:
            return

        os.makedirs(BENCHMARK_OUTPUT_DIR, exist_ok=True)
        stem = self._summary_stem()

        if self.samples:
            csv_path = os.path.join(BENCHMARK_OUTPUT_DIR, f"{stem}_benchmark.csv")
            with open(csv_path, "w", newline="", encoding="utf-8") as csv_file:
                writer = csv.DictWriter(
                    csv_file, fieldnames=list(self.samples[0].keys())
                )
                writer.writeheader()
                writer.writerows(self.samples)

    def _update_shared_summary_csv_if_requested(self):
        if not ADD_TO_SUMMARY_CSV or self.summary is None:
            return

        os.makedirs(BENCHMARK_OUTPUT_DIR, exist_ok=True)
        summary_path = os.path.join(BENCHMARK_OUTPUT_DIR, BENCHMARK_SUMMARY_FILENAME)
        combo_key = self._summary_combo_key()
        metric_rows = self._summary_metric_rows()
        metric_descriptors = self._summary_metric_descriptors()

        ordered_metrics = [descriptor["metric"] for descriptor in metric_descriptors]
        descriptor_map = {
            descriptor["metric"]: descriptor for descriptor in metric_descriptors
        }
        row_map = {}
        fixed_fields = ["section", "metric", "display_name", "notes"]
        legacy_metadata_fields = ["metric_role", "baseline_notes"]
        fieldnames = list(fixed_fields)

        if os.path.exists(summary_path):
            with open(summary_path, "r", newline="", encoding="utf-8") as csv_file:
                reader = csv.DictReader(csv_file)
                if reader.fieldnames:
                    existing_fieldnames = list(reader.fieldnames)
                    combo_fields = [
                        field
                        for field in existing_fieldnames
                        if field not in fixed_fields
                        and field not in legacy_metadata_fields
                    ]
                    fieldnames = list(fixed_fields) + combo_fields
                for row in reader:
                    metric_name = row.get("metric")
                    if not metric_name:
                        continue
                    row_map[metric_name] = dict(row)

        if combo_key not in fieldnames:
            fieldnames.append(combo_key)

        for metric_name in ordered_metrics:
            if metric_name not in row_map:
                row_map[metric_name] = {"metric": metric_name}
            row_map[metric_name].update(descriptor_map[metric_name])
            row_map[metric_name][combo_key] = metric_rows.get(metric_name, "")

        with open(summary_path, "w", newline="", encoding="utf-8") as csv_file:
            writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
            writer.writeheader()
            for metric_name in ordered_metrics:
                row = {"metric": metric_name}
                row.update(row_map.get(metric_name, {}))
                writer.writerow({field: row.get(field, "") for field in fieldnames})

    def _finalize_summary(self, timed_out):
        if self._summary_finalized:
            return

        mean_tcp_target_error = (
            self._sum_tcp_target_error / self._sample_count
            if self._sample_count > 0
            else 0.0
        )

        self.summary = {
            "leg_model": SELECTED_LEG_MODEL,
            "ik_formulation": SELECTED_IK_FORMULATION,
            "ik_formulation_label": SELECTED_IK_FORMULATION_LABEL,
            "ik_formulation_short_label": SELECTED_IK_FORMULATION_SHORT_LABEL,
            "cycle_time_s": self._elapsed_time,
            "timed_out": bool(timed_out),
            "final_state": self._state_name(),
            "peak_abs_motor_input": self._peak_abs_motor_input,
            "integrated_abs_motor_input": self._integrated_abs_motor_input,
            "mean_tcp_target_error_mm": mean_tcp_target_error,
            "state_durations_s": self.state_durations,
        }

        _data_write(self.cycle_time_data, float(self._elapsed_time))
        _data_write(self.timed_out_data, 1.0 if timed_out else 0.0)
        _data_write(self.finalized_data, 1.0)

        self._summary_finalized = True
        self._export_samples_if_requested()
        self._update_shared_summary_csv_if_requested()

        summary_message = (
            "[BenchmarkRecorder] "
            f"formulation={SELECTED_IK_FORMULATION}, model={SELECTED_LEG_MODEL}, "
            f"cycle_time={self._elapsed_time:.2f}s, "
            f"peak_abs_motor_input={self._peak_abs_motor_input:.4f}, "
            f"integrated_abs_motor_input={self._integrated_abs_motor_input:.4f}, "
            f"timed_out={int(timed_out)}"
        )
        print(summary_message)
        try:
            Sofa.msg_info(self.name, summary_message)
        except Exception:
            pass

    def onAnimateEndEvent(self, _):
        if self._summary_finalized:
            return

        self._elapsed_time += self.dt
        self._record_sample()

        if self._state_name() == "DONE":
            self._finalize_summary(timed_out=False)
            return

        if self._elapsed_time >= BENCHMARK_MAX_CYCLE_TIME_S:
            self._finalize_summary(timed_out=True)


def _get_gripper_opening_data(emio):
    if hasattr(emio.centerpart, "Effector"):
        return emio.centerpart.Effector.Distance.DistanceMapping.restLengths
    return emio.centerpart.effector.Distance.DistanceMapping.restLengths


def _get_gripper_distance_node(emio):
    gripper_effector = None
    if hasattr(emio.centerpart, "Effector"):
        gripper_effector = emio.centerpart.Effector
    elif hasattr(emio.centerpart, "effector"):
        gripper_effector = emio.centerpart.effector

    if gripper_effector is None:
        return None

    if hasattr(gripper_effector, "Distance"):
        return gripper_effector.Distance
    return gripper_effector.getChild("Distance")


def _get_gripper_distance_mo(emio):
    distance_node = _get_gripper_distance_node(emio)
    if distance_node is None:
        return None

    mechanical_state = distance_node.getMechanicalState()
    if mechanical_state is not None:
        return mechanical_state

    return distance_node.getObject("MechanicalObject")


def _read_gripper_distance_error(gripper_distance_mo):
    if gripper_distance_mo is None or not hasattr(gripper_distance_mo, "position"):
        return None

    return _optional_scalar(gripper_distance_mo.position)


def _read_actual_gripper_gap(gripper_opening_data, gripper_distance_mo):
    distance_error = _read_gripper_distance_error(gripper_distance_mo)
    if distance_error is None:
        return None

    # The DistanceMapping output is zero when the fingertip distance matches
    # the commanded rest length, so the physical gap is command + error.
    return _scalar(gripper_opening_data) + distance_error


# ---------------------------------------------------------------------------
# Scene assembly helpers
# This section keeps formulation/model selection, visible scene markers, and GUI
# registration separated from the runtime controllers so createScene() stays
# as a short orchestration entry point.
# ---------------------------------------------------------------------------
def _selected_proxy_leg_model():
    if SELECTED_LEG_MODEL in ADVANCED_LEG_MODELS:
        return "tetra"
    return SELECTED_LEG_MODEL


def _log_selected_setup():
    proxy_model = _selected_proxy_leg_model()
    message = (
        f"[{SCENE_LOGGER_NAME}] "
        f"legModel={SELECTED_LEG_MODEL} "
        f"(emioProxy={proxy_model}) "
        f"ikFormulation={SELECTED_IK_FORMULATION} "
        f"({SELECTED_IK_FORMULATION_SHORT_LABEL}: {SELECTED_IK_FORMULATION_LABEL}) "
        f"dt={SELECTED_SCENE_DT:.2f}"
    )
    print(message)
    try:
        Sofa.msg_info(SCENE_LOGGER_NAME, message)
    except Exception:
        pass


def _build_configured_emio(simulation):
    requested_model = SELECTED_LEG_MODEL
    proxy_model = _selected_proxy_leg_model()

    if requested_model in ADVANCED_LEG_MODELS:
        _require_supported_option(
            requested_model,
            ADVANCED_MODEL_FACTORIES.get(requested_model),
            "leg model",
        )
        _ensure_external_plugin_repository(simulation)

    legs_young_modulus = (
        [parameters.youngModulus * parameters.tetraYMFactor]
        if proxy_model == "tetra"
        else [parameters.youngModulus]
    )

    emio = Emio(
        name="Emio",
        legsName=["blueleg"],
        legsModel=[proxy_model],
        legsPositionOnMotor=[
            "counterclockwisedown",
            "clockwisedown",
            "counterclockwisedown",
            "clockwisedown",
        ],
        legsYoungModulus=legs_young_modulus,
        centerPartName="whitepart",
        centerPartType="deformable",
        centerPartModel="beam",
        centerPartClass=Gripper,
        platformLevel=2,
        extended=True,
    )
    if not emio.isValid():
        return None

    if requested_model in ADVANCED_LEG_MODELS:
        create_model = ADVANCED_MODEL_FACTORIES[requested_model]
        model_args = SimpleNamespace(legName="blueleg")
        for leg in emio.legs:
            create_model(leg, model_args, parameters, [0, 0, 0])

    return emio


def _add_pose_marker(parent, name, xyz, color, scale=TRACKED_MARKER_SCALE):
    marker_node = parent.addChild(name)
    marker_node.addObject(
        "MechanicalObject",
        name="Pose",
        template="Rigid3",
        position=_rigid_pose(xyz),
        showObject=True,
        showObjectScale=scale,
        drawMode=1,
        showColor=color,
    )
    return marker_node


def _add_effector_target(parent):
    target_node = parent.addChild("Target")
    target_node.addObject("EulerImplicitSolver", firstOrder=True)
    target_node.addObject("CGLinearSolver", **TARGET_LINEAR_SOLVER_SETTINGS)
    target_node.addObject(
        "MechanicalObject",
        template="Rigid3",
        position=_rigid_pose(HOME_XYZ),
        showObject=True,
        showObjectScale=TARGET_MARKER_SCALE,
    )
    return target_node


def _configure_inverse_target_link(modelling, effector_target):
    passive_inverse_target = None
    inverse_goal_link = effector_target.getMechanicalState().position.linkpath
    if USES_PASSIVE_INVERSE_TARGET:
        passive_inverse_target = modelling.addChild("PassiveInverseTarget")
        passive_inverse_target.addObject(
            "MechanicalObject",
            template="Rigid3",
            position=_rigid_pose(HOME_XYZ),
            showObject=False,
        )
        inverse_goal_link = (
            passive_inverse_target.getMechanicalState().position.linkpath
        )
    return passive_inverse_target, inverse_goal_link


def _add_tcp_visual(parent, emio):
    tcp_node = parent.addChild("TCP")
    tcp_node.addObject(
        "MechanicalObject",
        template="Rigid3",
        position=emio.effector.EffectorCoord.barycenter.linkpath,
        showObject=True,
        showObjectScale=TCP_MARKER_SCALE,
        drawMode=1,
        showColor=TCP_MARKER_COLOR,
    )
    return tcp_node


def _register_gui_group(data_source, group_name, io_prefix, fields):
    for label, attr_name in fields:
        data = getattr(data_source, attr_name)
        MyGui.MyRobotWindow.addInformationInGroup(label, data, group_name)
        MyGui.IOWindow.addSubscribableData(f"{io_prefix}/{attr_name}", data)


def _register_benchmark_plots(program, benchmark):
    data_sources = {
        "program": program,
        "benchmark": benchmark,
    }
    for label, attr_name, source_name in BENCHMARK_PLOT_FIELDS:
        MyGui.PlottingWindow.addData(
            label,
            getattr(data_sources[source_name], attr_name),
        )


def _replace_constraint_solver(rootnode, solver_object):
    current_solver = rootnode.getObject("ConstraintSolver")
    if current_solver is not None:
        rootnode.removeObject(current_solver)
    rootnode.addObject(solver_object)


def _configure_selected_inverse_formulation(
    rootnode,
    emio,
    effector_target_mo,
    tcp_mo,
    gripper_distance_mo,
    gripper_opening_data,
    assembly_controller,
):
    selected_formulation = SELECTED_IK_FORMULATION
    if selected_formulation == INTEGRATED_FORMULATION:
        return

    _require_supported_option(
        "softrobots_inverse", SoftRobotsInverse, "IK formulation base plugin"
    )
    if selected_formulation == REDUCED_ACTUATOR_SPACE_FORMULATION:
        reduced_backend = _require_supported_option(
            REDUCED_ACTUATOR_SPACE_FORMULATION,
            FORMULATION_BACKENDS.get(REDUCED_ACTUATOR_SPACE_FORMULATION),
            "IK formulation backend",
        )
        _replace_constraint_solver(
            rootnode,
            SelectableReducedActuatorSpaceFormulation(
                emio=emio,
                assembly_controller=assembly_controller,
                sensor_mo=tcp_mo,
                target_mo=effector_target_mo,
                effector_mo=tcp_mo,
                gripper_distance_mo=gripper_distance_mo,
                gripper_opening_data=gripper_opening_data,
                get_torques=reduced_backend.getTorques,
            ),
        )
        if rootnode.ConstraintSolver.findData("epsilon"):
            rootnode.ConstraintSolver.epsilon.value = 1
        return

    if selected_formulation == OIM_INSPIRED_FORMULATION:
        oim_backend = _require_supported_option(
            OIM_INSPIRED_FORMULATION,
            FORMULATION_BACKENDS.get(OIM_INSPIRED_FORMULATION),
            "IK formulation backend",
        )
        _replace_constraint_solver(
            rootnode,
            SelectableOIMInspiredFormulation(
                emio=emio,
                assembly_controller=assembly_controller,
                sensor_mo=tcp_mo,
                target_mo=effector_target_mo,
                effector_mo=tcp_mo,
                gripper_distance_mo=gripper_distance_mo,
                gripper_opening_data=gripper_opening_data,
                get_torques=oim_backend.getTorques,
            ),
        )
        return

    raise RuntimeError(
        f"Unsupported IK formulation selection '{selected_formulation}'."
    )


# Backward-compatible alias for the previous configuration helper name.
_configure_selected_inverse_solver = _configure_selected_inverse_formulation


# ---------------------------------------------------------------------------
# Create the scene.
# ---------------------------------------------------------------------------
def createScene(rootnode):
    args = getParserArgs()

    _settings, modelling, simulation = addHeader(rootnode, inverse=True)
    addSolvers(simulation)

    rootnode.dt = SELECTED_SCENE_DT
    rootnode.gravity = SCENE_GRAVITY
    rootnode.VisualStyle.displayFlags.value = SCENE_VISUAL_FLAGS

    _log_selected_setup()

    emio = _build_configured_emio(simulation)
    if emio is None or not emio.isValid():
        return

    simulation.addChild(emio)
    emio.attachCenterPartToLegs()
    assembly_controller = emio.addObject(AssemblyController(emio))

    object_node = _add_pose_marker(
        modelling,
        "TrackedObject",
        SIM_PROXY_PICK_XYZ,
        TRACKED_OBJECT_COLOR,
    )
    place_node = _add_pose_marker(
        modelling,
        "TrackedPlace",
        SIM_PROXY_PLACE_XYZ,
        TRACKED_PLACE_COLOR,
    )

    emio.effector.addObject(
        "MechanicalObject",
        template="Rigid3",
        position=_rigid_pose([0.0, 0.0, 0.0]) * len(TCP_RIGID_MAPPING_INDICES),
    )
    emio.effector.addObject(
        "RigidMapping", rigidIndexPerPoint=TCP_RIGID_MAPPING_INDICES
    )

    effector_target = _add_effector_target(modelling)
    passive_inverse_target, inverse_goal_link = _configure_inverse_target_link(
        modelling,
        effector_target,
    )

    emio.addInverseComponentAndGUI(
        inverse_goal_link,
        barycentric=True,
        withGUI=False,
    )

    tcp = _add_tcp_visual(modelling, emio)
    gripper_opening = _get_gripper_opening_data(emio)
    gripper_distance_mo = _get_gripper_distance_mo(emio)
    _configure_selected_inverse_formulation(
        rootnode=rootnode,
        emio=emio,
        effector_target_mo=effector_target.getMechanicalState(),
        tcp_mo=tcp.getMechanicalState(),
        gripper_distance_mo=gripper_distance_mo,
        gripper_opening_data=gripper_opening,
        assembly_controller=assembly_controller,
    )

    tracker_bridge = None
    if args.connection:
        emio.addConnectionComponents()
        if USE_CAMERA_MARKERS:
            tracker_bridge = rootnode.addObject(
                HardwareTrackerBridge(root=rootnode, tracker_factory=DotTracker)
            )

    program = rootnode.addObject(
        CameraDrivenPickAndPlaceController(
            root=rootnode,
            target_mo=effector_target.getMechanicalState(),
            tcp_mo=tcp.getMechanicalState(),
            passive_inverse_target_mo=(
                passive_inverse_target.getMechanicalState()
                if passive_inverse_target is not None
                else None
            ),
            gripper_opening_data=gripper_opening,
            gripper_distance_mo=gripper_distance_mo,
            object_mo=object_node.Pose,
            place_marker_mo=place_node.Pose,
            tracker_bridge=tracker_bridge,
            dt=rootnode.dt,
        )
    )

    benchmark = None
    if BENCHMARK_ENABLED:
        benchmark = rootnode.addObject(
            BenchmarkRecorder(
                root=rootnode,
                program=program,
                emio=emio,
                target_mo=effector_target.getMechanicalState(),
                tcp_mo=tcp.getMechanicalState(),
                object_mo=object_node.Pose,
                place_marker_mo=place_node.Pose,
                gripper_opening_data=gripper_opening,
                gripper_distance_mo=gripper_distance_mo,
                dt=rootnode.dt,
            )
        )

    _register_gui_group(
        program,
        PROGRAM_STATUS_GROUP,
        PROGRAM_IO_PREFIX,
        PROGRAM_STATUS_FIELDS,
    )

    if benchmark is not None:
        _register_gui_group(
            benchmark,
            BENCHMARK_STATUS_GROUP,
            BENCHMARK_IO_PREFIX,
            BENCHMARK_STATUS_FIELDS,
        )
        _register_benchmark_plots(program, benchmark)

    return rootnode