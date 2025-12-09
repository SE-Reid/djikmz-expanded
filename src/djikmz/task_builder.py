"""
TaskBuilder - Fluent API for building DJI KMZ mission files.

This module provides a user-friendly interface for creating DJI drone missions
with support for enterprise drones and comprehensive validation.
"""

from typing import List, Dict, Any, Optional, Union
from datetime import datetime

from .model.kml import KML, WaypointTurnMode, GimbalPitchMode
from .model.waypoint import Waypoint, Point, WaypointTurnParam
from .model.mission_config import (
    MissionConfig, DroneModel, DroneInfo, PayloadInfo, PayloadModel, FlyToWaylineMode, FinishAction, RCLostAction
)
from .model.coordinate_system_param import CoordinateSystemParam, CoordinateModeEnum, HeightModeEnum, PositionTypeEnum
from .model.action_group import ActionGroup, ActionTrigger, TriggerType
from .model.action import  RotateYawAction, GimbalRotateAction, HoverAction, TakePhotoAction, StartRecordAction, StopRecordAction, FocusAction, ZoomAction, OrientedShootAction
import zipfile
import io

# Drone configurations with defaults
DRONE_CONFIGS = {
    "M350": {
        "model": DroneModel.M350,
        "default_height": 100.0,
        "default_speed": 10.0,
        "max_speed": 17.0,
        "supports_rtk": True,
        "takeoff_security_height": 5.0,
        "default_payload": PayloadModel.H20  # H20 commonly used on M350
    },
    "M300": {
        "model": DroneModel.M300,
        "default_height": 100.0,
        "default_speed": 10.0,
        "max_speed": 17.0,
        "supports_rtk": True,
        "takeoff_security_height": 5.0,
        "default_payload": PayloadModel.H20  # H20 commonly used on M300
    },
    "M30": {
        "model": DroneModel.M30,
        "default_height": 80.0,
        "default_speed": 8.0,
        "max_speed": 15.0,
        "supports_rtk": True,
        "takeoff_security_height": 3.0,
        "default_payload": PayloadModel.M30  # Integrated M30 camera
    },
    "M30T": {
        "model": DroneModel.M30T,
        "default_height": 80.0,
        "default_speed": 8.0,
        "max_speed": 15.0,
        "supports_rtk": True,
        "takeoff_security_height": 3.0,
        "default_payload": PayloadModel.M30T  # Integrated M30T thermal camera
    },
    "M3E": {
        "model": DroneModel.M3E,
        "default_height": 80.0,
        "default_speed": 8.0,
        "max_speed": 15.0,
        "supports_rtk": False,
        "takeoff_security_height": 3.0,
        "default_payload": PayloadModel.M3E  # Integrated M3E camera
    },
    "M3T": {
        "model": DroneModel.M3T,
        "default_height": 80.0,
        "default_speed": 8.0,
        "max_speed": 15.0,
        "supports_rtk": False,
        "takeoff_security_height": 3.0,
        "default_payload": PayloadModel.M3T  # Integrated M3T thermal camera
    },
    "M3M": {
        "model": DroneModel.M3M,
        "default_height": 80.0,
        "default_speed": 8.0,
        "max_speed": 15.0,
        "supports_rtk": False,
        "takeoff_security_height": 3.0,
        "default_payload": PayloadModel.M3M  # Integrated M3M multispectral camera
    },
    "M3D": {
        "model": DroneModel.M3D,
        "default_height": 80.0,
        "default_speed": 8.0,
        "max_speed": 15.0,
        "supports_rtk": False,
        "takeoff_security_height": 3.0,
        "default_payload": PayloadModel.M3D  # Integrated M3D camera
    },
    "M3TD": {
        "model": DroneModel.M3TD,
        "default_height": 80.0,
        "default_speed": 8.0,
        "max_speed": 15.0,
        "supports_rtk": False,
        "takeoff_security_height": 3.0,
        "default_payload": PayloadModel.M3TD  # Integrated M3TD thermal camera
    }
}


class ValidationError(Exception):
    """Raised when mission validation fails."""
    pass


class HardwareError(Exception):
    """Raised when hardware configuration is invalid."""
    pass


class WaypointBuilder:
    """Builder for configuring individual waypoints with actions."""
    
    def __init__(self, parent_task: 'DroneTask', waypoint: Waypoint):
        self._task = parent_task
        self._waypoint = waypoint
        self._actions: List[Any] = []

    def turn_mode(self, turn_mode: str) -> 'WaypointBuilder':
        """Set the turn mode for this waypoint."""
        if turn_mode == 'default':
            self._waypoint.use_global_turn_param = 1
            self._waypoint.turn_param = None
            return self
        mapping = {
            "turn_at_point": WaypointTurnMode.TURN_AT_POINT,
            "early_turn": WaypointTurnMode.COORDINATED_TURN,
            "curve_and_stop": WaypointTurnMode.CURVED_TURN_WITH_STOP,
            "curve_and_pass": WaypointTurnMode.CURVED_TURN_WITHOUT_STOP
        }
        if turn_mode not in mapping:
            raise ValueError(f"Invalid turn mode: {turn_mode}. Supported: {', '.join(mapping.keys())}")
        self._waypoint.use_global_turn_param = 0
        self._waypoint.turn_param = WaypointTurnParam(
            waypoint_turn_mode=mapping[turn_mode],
            waypoint_turn_damping_dist=0.2 if turn_mode == "early_turn" else None  
        )
        return self
    
    def height(self, height: float) -> 'WaypointBuilder':
        """Set the height for this waypoint (overrides global height)."""
        self._waypoint.height = height
        self._waypoint.use_global_height = 0
        return self
    
    def speed(self, speed: float) -> 'WaypointBuilder':
        """Set the flight speed for this waypoint (overrides global speed)."""
        self._waypoint.speed = speed
        self._waypoint.use_global_speed = 0
        return self
    
    def take_photo(self, suffix: Optional[str] = "", lens: Optional[str] = None) -> 'WaypointBuilder':
        """Add a take photo action at this waypoint."""
        action = TakePhotoAction(
            action_id=0,  # Will be assigned at build time
            file_suffix=suffix,
            payload_lens=lens
        )
        self._actions.append(action)
        return self
    
    def start_video_recording(self, suffix: Optional[str] = "", lens: Optional[str] = None) -> 'WaypointBuilder':
        """Add a start video recording action at this waypoint."""
        action = StartRecordAction(
            action_id=0,  # Will be assigned at build time
            file_suffix=suffix,
            payload_lens=lens          
        )
        self._actions.append(action)
        return self
    
    def stop_video_recording(self, lens: Optional[str] = None) -> 'WaypointBuilder':
        """Add a stop video recording action at this waypoint."""
        action = StopRecordAction(
            action_id=0,  # Will be assigned at build time
            payload_lens=lens          
        )
        self._actions.append(action)
        return self

    def hover(self, duration: float) -> 'WaypointBuilder':
        """Add a hover action at this waypoint."""
        action = HoverAction(
            action_id=0,  # Will be assigned at build time
            hover_time=duration  # HoverAction expects float in seconds
        )
        self._actions.append(action)
        return self

    def rotate_yaw(self, angle: float) -> 'WaypointBuilder':
        """Set the drone's heading (yaw) at this waypoint.
        
        Args:
            angle: Heading angle in degrees (-180~180, 0 = North, 90 = East)
        """
        # This can be implemented using waypointHeadingParam which support
        # automatic rotation direction. But the timing of yaw rotation is ambiguous
        if not (-180 <= angle <= 180):
            raise ValueError("Heading angle must be between -180 and 180 degrees")
        action = RotateYawAction(
            action_id=0,  # Will be assigned at build time
            aircraft_heading= angle,
            direction= 'clockwise')  
        self._actions.append(action)
        return self
    
    def gimbal_rotate(self, pitch: float = None, yaw: float = None, roll: float = None) -> 'WaypointBuilder':
        """Set gimbal to specific pitch, yaw, and/or roll angles.
        TODO: seems in the App, the roll is not used. 
        TODO: in the dji pilot app, rotation at many axes is not supported. this need to be changed to multiple actions. 
        Args:
            pitch: Pitch angle in degrees (-90 to +90, negative = down)
            yaw: Yaw angle in degrees (0-360, relative to north)  
            roll: Roll angle in degrees
        """
        action = GimbalRotateAction(action_id=0)  # Will be assigned at build time

        # Check which axes are specified
        axes = []
        if pitch is not None:
            axes.append('pitch')
        if yaw is not None:
            axes.append('yaw')
        if roll is not None:
            axes.append('roll')

        # If more than one axis is specified, split into separate actions
        if len(axes) >= 1:
            if pitch is not None:
                action = GimbalRotateAction(action_id=0)
                action.gimbal_pitch_rotate_enable = 1
                action.gimbal_pitch_rotate_angle = pitch
                self._actions.append(action)
            if yaw is not None:
                action = GimbalRotateAction(action_id=0)
                action.gimbal_yaw_rotate_enable = 1
                action.gimbal_yaw_rotate_angle = yaw
                self._actions.append(action)
            if roll is not None:
                action = GimbalRotateAction(action_id=0)
                action.gimbal_roll_rotate_enable = 1
                action.gimbal_roll_rotate_angle = roll
                self._actions.append(action)
        return self
    
    def set_focus(self, is_point_focus: bool, focus_position_x: float, focus_position_y: float, focus_region_width: Optional[float] = 0.0, focus_region_height: Optional[float] = 0.0, is_infinite: bool = False) -> 'WaypointBuilder':
        """Set camera focus distance."""
        if is_point_focus:
            point_focus = 1
            if focus_region_width != 0.0 or focus_region_height != 0.0:
                raise ValueError("Focus region size should not be set when using point focus")
        else:
            point_focus = 0
            if focus_region_width == 0.0 or focus_region_height == 0.0:
                raise ValueError("Focus region size must be set when not using point focus")
        
        infinite_focus = int(is_infinite)
        action = FocusAction(
            action_id=0,  # Will be assigned at build time
            is_point_focus=point_focus,
            focus_x=focus_position_x,
            focus_y=focus_position_y,
            focus_region_width=focus_region_width,
            focus_region_height=focus_region_height,
            is_infinite_focus=infinite_focus
        )
        self._actions.append(action)
        return self
    
    def fly_to(self, latitude: float, longitude: float, height: float|None=None) -> 'WaypointBuilder':
        """Add another waypoint and return its builder."""
        return self._task.fly_to(latitude, longitude, height)
    
    def build(self) -> KML:
        """Build the final KML mission."""
        return self._task.build()
    
    def to_kmz(self, filename: str) -> None:
        """Save the mission as a KMZ file."""
        self._task.to_kmz(filename)
    
    def _finalize_actions(self, action_id_start: int) -> int:
        """Finalize actions for this waypoint and return next action ID."""
        if not self._actions:
            return action_id_start
        
        # Assign action IDs
        for i, action in enumerate(self._actions):
            action.action_id = action_id_start + i
        
        # Create action group
        self._waypoint.action_group = ActionGroup(
            group_id=self._waypoint.waypoint_id,
            trigger=ActionTrigger(type=TriggerType.REACH_POINT),
            actions=self._actions
        )
        
        return action_id_start + len(self._actions)


class DroneTask:
    """Main builder for creating DJI drone missions."""
    
    def __init__(self, drone_model: str, pilot: str = "Pilot"):
        """Initialize a new drone mission.
        
        Args:
            drone_model: Model of the drone (e.g., "M30T", "M350")
            pilot: Name of the pilot
        """
        # Validate drone model
        if drone_model not in DRONE_CONFIGS:
            supported = ", ".join(DRONE_CONFIGS.keys())
            raise ValueError(
                f"Unsupported drone model: {drone_model}. "
                f"Supported models: {supported}"
            )
        
        self.drone_model = drone_model
        self.drone_config = DRONE_CONFIGS[drone_model]
        
        # Mission metadata
        self.pilot = pilot
        self.mission_name = "Untitled Mission"
        
        # Flight parameters with defaults
        self._flight_speed = self.drone_config["default_speed"]
        self._flight_height = self.drone_config["default_height"]
        self._turn_mode = WaypointTurnMode.TURN_AT_POINT
        
        # Waypoint storage
        self._waypoints: List[Waypoint] = []
        self._current_waypoint: Optional[Waypoint] = None
        
        # Technical configuration with defaults
        self._coordinate_system = CoordinateSystemParam(
            coordinate_system=CoordinateModeEnum.WGS84,
            height_mode=HeightModeEnum.RELATIVE,
            position_type=PositionTypeEnum.GPS
        )
        
        self._mission_config = MissionConfig(
            fly_to_wayline_mode=FlyToWaylineMode.SAFELY,
            finish_action=FinishAction.GO_HOME,
            rclost_action=RCLostAction.CONTINUE,
            take_off_height=self.drone_config["takeoff_security_height"],
            drone_info=DroneInfo(drone_model=self.drone_config["model"]),
            payload_info=PayloadInfo(
                payload_model=self.drone_config["default_payload"],
                position=0  # Default position
            )
        )

    def __len__(self) -> int:
        """Return the number of waypoints in the mission."""
        return len(self._waypoints)
    
    def name(self, mission_name: str) -> 'DroneTask':
        """Set the mission name."""
        self.mission_name = mission_name
        return self
    
    def speed(self, speed: float) -> 'DroneTask':
        """Set the global flight speed in m/s."""
        self._flight_speed = speed
        return self
    
    def altitude(self, height: float) -> 'DroneTask':
        """Set the global flight altitude in meters."""
        self._flight_height = height
        return self

    def turn_mode(self, turn_mode: str):
        mapping = {
            "turn_at_point": WaypointTurnMode.TURN_AT_POINT,
            "early_turn": WaypointTurnMode.COORDINATED_TURN,
            "curve_and_stop": WaypointTurnMode.CURVED_TURN_WITH_STOP,
            "curve_and_pass": WaypointTurnMode.CURVED_TURN_WITHOUT_STOP
        }
        if turn_mode not in mapping:
            raise ValueError(f"Invalid turn mode: {turn_mode}. Supported: {', '.join(mapping.keys())}")
        self._turn_mode = mapping[turn_mode]
        return self

    
    def coordinate_system(self, positioning_type: str = "GPS") -> 'DroneTask':
        """Configure coordinate system (GPS, RTK, or Qianxun).
        This is not useful in current DJI Pilot app, and not affect the task. so it's not included in readme.
            but kept it here for now."""
        mapping = {
            "GPS": PositionTypeEnum.GPS,
            "RTK": PositionTypeEnum.RTK,
            "Qianxun": PositionTypeEnum.QIANXUN
        }
        
        if positioning_type not in mapping:
            raise ValueError(f"Invalid positioning type: {positioning_type}")
        
        self._coordinate_system.position_type = mapping[positioning_type]
        return self
    
    def payload(self, payload_model: str, position: int = 0) -> 'DroneTask':
        """Configure payload/camera system.
        
        Args:
            payload_model: Payload model (e.g., "H20T", "M30T", "M3E")
            position: Payload position (0=default, 1=front right, 2=top)
        """
        # Automatically generate mapping from PayloadModel enum
        payload_mapping = {member.name: member for member in PayloadModel}
        
        if payload_model not in payload_mapping:
            supported = ", ".join(payload_mapping.keys())
            raise ValueError(f"Unsupported payload model: {payload_model}. Supported: {supported}")
        
        self._mission_config.payload_info = PayloadInfo(
            payload_model=payload_mapping[payload_model],
            position=position
        )
        return self
    
    def return_home_on_signal_loss(self, enable: bool = True) -> 'DroneTask':
        """Configure behavior when RC signal is lost."""
        if enable:
            self._mission_config.rclost_action = RCLostAction.CONTINUE
        else:
            self._mission_config.rclost_action = RCLostAction.HOVER
        return self
    
    def finish_action(self, action: str = "return_home") -> 'DroneTask':
        """Set what to do when mission is complete."""
        mapping = {
            "return_home": FinishAction.GO_HOME,
            "hover": FinishAction.NO_ACTION,
            "land": FinishAction.AUTOLAND,
            "restart": FinishAction.GOTO_FIRST_WAYPOINT
        }
        
        if action not in mapping:
            raise ValueError(f"Invalid finish action: {action}")
        
        self._mission_config.finish_action = mapping[action]
        return self
    
    def fly_to(self, latitude: float, longitude: float, height: float|None = None) -> WaypointBuilder:
        """Add a waypoint to fly to."""
        waypoint = Waypoint(
            latitude=latitude,
            longitude=longitude,
            height=height if height is not None else None,
            use_global_height=1 if height is None else 0,
            waypoint_id=len(self._waypoints)
        )
        self._waypoints.append(waypoint)
        self._current_waypoint = waypoint
        
        builder = WaypointBuilder(self, waypoint)
        # Store reference for later action finalization
        setattr(waypoint, '_waypoint_builder', builder)
        return builder
    
    def build(self) -> KML:
        """Build the final KML mission file."""
        # Validation
        errors = self._validate_configuration()
        if errors:
            raise ValidationError(f"Mission validation failed: {errors}")
        
        # Hardware detection
        self._detect_and_validate_hardware()
        
        # Assign waypoint indices
        for index, waypoint in enumerate(self._waypoints):
            waypoint.waypoint_id = index
        
        # Assign action IDs globally
        global_action_id = 0
        for waypoint in self._waypoints:
            if hasattr(waypoint, '_waypoint_builder'):
                builder = getattr(waypoint, '_waypoint_builder')
                global_action_id = builder._finalize_actions(global_action_id)
        
        # Build KML with correct field names
        kml = KML(
            author=self.pilot,
            create_time=int(datetime.now().timestamp() * 1000),
            update_time=int(datetime.now().timestamp() * 1000),
            mission_config=self._mission_config,
            coordinate_system_param=self._coordinate_system,
            global_turn_mode= self._turn_mode,
            global_speed=self._flight_speed,
            global_height=self._flight_height,
            waypoints=self._waypoints
        )
        
        return kml
    
    def to_kmz(self, filename: Optional[str]) -> None:
        """Save the mission as a KMZ file."""
        if not filename:
            filename = f"{self.mission_name}.kmz"
        kml = self.build()
        # Serialize KML to XML string
        kml_xml = kml.to_xml()

        # Write to KMZ (ZIP) with structure wpmz/template.kml
        with zipfile.ZipFile(filename, "w", zipfile.ZIP_DEFLATED) as kmz:
            kmz.writestr("wpmz/template.kml", kml_xml)
    
    def _validate_configuration(self) -> List[str]:
        """Validate mission configuration and return list of errors."""
        errors = []
        
        # Basic mission validation
        if not self._waypoints:
            errors.append("Mission must have at least one waypoint")
        
        # Speed validation
        max_speed = self.drone_config["max_speed"]
        if self._flight_speed > max_speed:
            errors.append(f"Speed {self._flight_speed} m/s exceeds drone limit of {max_speed} m/s")
        
        # Height validation (basic sanity check)
        if self._flight_height < 0:
            errors.append("Flight altitude cannot be negative")
        
        # RTK validation
        if (self._coordinate_system.position_type == PositionTypeEnum.RTK and 
            not self.drone_config["supports_rtk"]):
            errors.append(f"RTK positioning not supported on {self.drone_model}")
        
        # Waypoint validation
        for i, waypoint in enumerate(self._waypoints):
            if waypoint.speed and waypoint.speed > max_speed:
                errors.append(f"Waypoint {i} speed exceeds drone limit")
        
        return errors
    
    def _detect_and_validate_hardware(self):
        """Validate hardware configuration."""
        # TODO: Additional hardware checks go here:
        # - Payload compatibility
        # - Camera availability
        # - Gimbal configuration
        pass
