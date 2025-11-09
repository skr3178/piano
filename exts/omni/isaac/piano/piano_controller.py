# Copyright 2024
# Isaac Sim Piano Controller
# Full feature parity with RoboPianist Piano class

"""Piano controller class for Isaac Sim."""

from typing import Optional, Sequence, Dict, Callable, TYPE_CHECKING
import numpy as np

# Delay imports of Isaac Sim modules until runtime
if TYPE_CHECKING:
    from omni.isaac.core.articulations import Articulation
    from pxr import UsdGeom, Gf

from . import piano_constants as piano_consts
from . import midi_module


class IsaacPianoController:
    """A full-size standard (88-key) digital piano controller for Isaac Sim.
    
    This class provides the same functionality as the RoboPianist Piano class,
    but adapted for Isaac Sim's PhysX engine and USD-based scene management.
    
    Features:
    - Real-time key state tracking and activation detection
    - MIDI message generation (NoteOn, NoteOff, SustainOn, SustainOff)
    - Dynamic key color changes on activation
    - Observable state (joint positions, activations, normalized states)
    - Support for both passive (touch-based) and self-actuated modes
    - Callback hooks for audio synthesizer integration
    """

    def __init__(
        self,
        piano_articulation: "Articulation",
        prim_path: str = "/Piano",
        change_color_on_activation: bool = True,
        add_actuators: bool = False,
    ) -> None:
        """Initializes the piano controller.

        Args:
            piano_articulation: The Isaac Sim Articulation object for the piano.
            prim_path: USD path to the piano prim.
            change_color_on_activation: If True, the color of the key changes when it
                becomes activated.
            add_actuators: If True, enables self-actuated mode where keys can be
                controlled via actions.
        """
        # Import at runtime when Isaac Sim is available
        from omni.isaac.core.utils.prims import get_prim_at_path
        from pxr import UsdGeom, Gf
        
        # Store these for later use
        self._get_prim_at_path = get_prim_at_path
        self._UsdGeom = UsdGeom
        self._Gf = Gf
        
        self._articulation = piano_articulation
        self._prim_path = prim_path
        self._change_color_on_activation = change_color_on_activation
        self._add_actuators = add_actuators
        
        # Initialize MIDI module
        self._midi_module = midi_module.MidiModule()
        
        # Flag to track initialization
        self._is_initialized = False
        
        # Initialize these as empty dicts - will be populated on first update
        self._joint_indices: Dict[int, int] = {}
        self._key_ids: Dict[int, int] = {}
        self._key_mesh_paths: Dict[int, str] = {}
        self._original_colors: Dict[int, tuple] = {}
        
        # Track current action for visual feedback (keys pressed if action > threshold)
        self._current_action = np.zeros(piano_consts.NUM_KEYS)
        
        # Initialize state arrays
        self._initialize_state()

    def _setup_joints(self) -> None:
        """Setup joint mappings and limits."""
        # Get all joint names
        joint_names = self._articulation.dof_names
        
        # Check if articulation is initialized
        if joint_names is None:
            return  # Will try again later
        
        # Map joints to key indices
        self._joint_indices.clear()
        self._key_ids.clear()
        
        for idx, joint_name in enumerate(joint_names):
            # Extract key number from joint name (e.g., "white_joint_0", "black_joint_1")
            if "joint" in joint_name:
                parts = joint_name.split("_")
                key_id = int(parts[-1])
                self._joint_indices[key_id] = idx
                self._key_ids[idx] = key_id
        
        # Get joint limits from USD directly (Isaac Sim 4.5+ API changed)
        # Read limits from the USD stage since SingleArticulation doesn't expose get_dof_limits()
        from pxr import UsdPhysics, Usd
        from omni.isaac.core.utils.stage import get_current_stage
        
        stage = get_current_stage()
        num_dofs = len(joint_names)
        self._qpos_lower = np.zeros(num_dofs)
        self._qpos_upper = np.zeros(num_dofs)
        
        # Read limits from each joint in USD
        for idx, joint_name in enumerate(joint_names):
            # Find the joint prim
            joint_path = f"{self._prim_path}/{joint_name}"
            joint_prim = stage.GetPrimAtPath(joint_path)
            if joint_prim and joint_prim.IsValid():
                joint = UsdPhysics.RevoluteJoint(joint_prim)
                if joint:
                    lower_limit_attr = joint.GetLowerLimitAttr()
                    upper_limit_attr = joint.GetUpperLimitAttr()
                    if lower_limit_attr and lower_limit_attr.IsValid():
                        lower_val = lower_limit_attr.Get()
                        # Check if value is not None (0.0 is a valid value!)
                        if lower_val is not None:
                            self._qpos_lower[idx] = np.radians(lower_val)  # Convert degrees to radians
                        else:
                            self._qpos_lower[idx] = 0.0  # Default to 0° if not set
                    else:
                        self._qpos_lower[idx] = 0.0  # Default to 0° if attribute doesn't exist
                    
                    if upper_limit_attr and upper_limit_attr.IsValid():
                        upper_val = upper_limit_attr.Get()
                        if upper_val is not None:
                            self._qpos_upper[idx] = np.radians(upper_val)
                        else:
                            # Default to max angle if not set (calculate from constants)
                            from . import piano_constants as pc
                            is_black = idx in self._key_ids and self._key_ids[idx] in pc.BLACK_KEY_INDICES
                            max_angle = pc.BLACK_KEY_JOINT_MAX_ANGLE if is_black else pc.WHITE_KEY_JOINT_MAX_ANGLE
                            self._qpos_upper[idx] = max_angle
                    else:
                        # Default to max angle if attribute doesn't exist
                        from . import piano_constants as pc
                        is_black = idx in self._key_ids and self._key_ids[idx] in pc.BLACK_KEY_INDICES
                        max_angle = pc.BLACK_KEY_JOINT_MAX_ANGLE if is_black else pc.WHITE_KEY_JOINT_MAX_ANGLE
                        self._qpos_upper[idx] = max_angle
        
        # Store joint ranges
        self._qpos_range = np.column_stack([self._qpos_lower, self._qpos_upper])
        
        if self._add_actuators:
            # Calculate control range midpoints for actuator mode
            self._ctrl_midpoint = np.mean(
                np.column_stack([self._qpos_lower, self._qpos_upper]), axis=1
            )

    def _setup_key_visuals(self) -> None:
        """Setup key visual elements for color changes."""
        self._key_mesh_paths.clear()
        self._original_colors.clear()
        
        for key_id in range(piano_consts.NUM_KEYS):
            # Determine key type
            is_white = key_id in piano_consts.WHITE_KEY_INDICES
            key_type = "white" if is_white else "black"
            
            # Path to key mesh
            mesh_path = f"{self._prim_path}/{key_type}_key_{key_id}/{key_type}_key_{key_id}_mesh"
            self._key_mesh_paths[key_id] = mesh_path
            
            # Store original color
            prim = self._get_prim_at_path(mesh_path)
            if prim and prim.IsValid():
                mesh = self._UsdGeom.Mesh(prim)
                if mesh:
                    color_attr = mesh.GetDisplayColorAttr()
                    if color_attr:
                        colors = color_attr.Get()
                        if colors:
                            self._original_colors[key_id] = tuple(colors[0])

    def _initialize_state(self) -> None:
        """Initialize internal state arrays."""
        self._state = np.zeros(piano_consts.NUM_KEYS, dtype=np.float64)
        self._sustain_state = np.zeros(1, dtype=np.float64)
        self._activation = np.zeros(piano_consts.NUM_KEYS, dtype=bool)
        self._sustain_activation = np.zeros(1, dtype=bool)
        self._normalized_state = np.zeros(piano_consts.NUM_KEYS, dtype=np.float64)

    def initialize_episode(self, current_time: float = 0.0) -> None:
        """Initialize a new episode.
        
        Args:
            current_time: Current simulation time in seconds.
        """
        # Setup joints and visuals if not already done
        if not self._is_initialized:
            self._setup_joints()
            self._setup_key_visuals()
            self._is_initialized = True
        
        self._initialize_state()
        self._midi_module.initialize_episode(current_time)

    def is_key_black(self, key_id: int) -> bool:
        """Returns True if the piano key id corresponds to a black key.
        
        Args:
            key_id: Piano key number (0-87).
            
        Returns:
            True if the key is black, False if white.
        """
        black_keys = [0, 1, 0, 0, 1, 0, 1, 0, 0, 1, 0, 1]
        return bool(black_keys[key_id % 12])

    def update(self, dt: float) -> None:
        """Update piano state for the current timestep.
        
        This should be called every simulation step. It:
        1. Reads joint positions from the articulation
        2. Detects key activations
        3. Updates key colors
        4. Generates MIDI messages
        
        Args:
            dt: Time delta since last update (seconds).
        """
        # Ensure initialization has occurred
        if not self._is_initialized:
            self._setup_joints()
            self._setup_key_visuals()
            self._is_initialized = True
        
        # Get current simulation time from world
        from omni.isaac.core import World
        world = World.instance()
        current_time = world.current_time if world else 0.0
        
        # Update key state from joint positions
        self._update_key_state()
        
        # Update visual colors
        self._update_key_color()
        
        # Generate MIDI messages
        self._midi_module.after_substep(
            current_time, self._activation, self._sustain_activation
        )

    def _update_key_state(self) -> None:
        """Updates the state of the piano keys from joint positions."""
        # Get current joint positions
        joint_positions = self._articulation.get_joint_positions()
        
        # Always use position-based detection for more accurate state tracking
        # Sort joint positions by key ID
        sorted_positions = np.zeros(piano_consts.NUM_KEYS)
        for key_id, joint_idx in self._joint_indices.items():
            if joint_idx < len(joint_positions):
                sorted_positions[key_id] = joint_positions[joint_idx]
        
        # Clip joint positions to their limits
        self._state[:] = np.clip(sorted_positions, *self._qpos_range.T)
        
        # Normalize state
        range_span = self._qpos_range[:, 1] - self._qpos_range[:, 0]
        # Avoid division by zero
        range_span = np.where(range_span == 0, 1.0, range_span)
        self._normalized_state[:] = (
            (self._state - self._qpos_range[:, 0]) / range_span
        )
        
        # Detect activation: key is near its upper limit (fully pressed)
        # Upper limit is the maximum rotation (pressed position)
        self._activation[:] = (
            np.abs(self._state - self._qpos_range[:, 1]) <= piano_consts.KEY_THRESHOLD
        )
        
        # Debug: Track stuck keys
        if not hasattr(self, '_stuck_key_debug_counter'):
            self._stuck_key_debug_counter = 0
            self._last_active_keys = set()
        
        current_active = set(np.where(self._activation)[0])
        if len(current_active) > 0 and current_active == self._last_active_keys:
            self._stuck_key_debug_counter += 1
            if self._stuck_key_debug_counter == 100:  # Report after 100 consecutive steps
                stuck_key = list(current_active)[0]
                print(f"DEBUG: Key {stuck_key} seems stuck!")
                print(f"  Position: {self._state[stuck_key]:.6f} rad")
                print(f"  Upper limit: {self._qpos_range[stuck_key, 1]:.6f} rad")
                print(f"  Distance from limit: {abs(self._state[stuck_key] - self._qpos_range[stuck_key, 1]):.6f} rad")
                print(f"  Threshold: {piano_consts.KEY_THRESHOLD:.6f} rad")
                self._stuck_key_debug_counter = 0  # Reset
        else:
            self._stuck_key_debug_counter = 0
            self._last_active_keys = current_active
        
        # Update sustain state
        self._sustain_activation[:] = self._sustain_state >= piano_consts.SUSTAIN_THRESHOLD

    def _update_key_color(self) -> None:
        """Colors the piano keys if they are pressed."""
        if not self._change_color_on_activation:
            return
        
        from omni.isaac.core.utils.stage import get_current_stage
        stage = get_current_stage()
        
        if stage is None:
            return
        
        # Determine which keys should be colored as active
        # In actuator mode, use action values for visual feedback (more reliable than position)
        keys_to_color_red = np.zeros(piano_consts.NUM_KEYS, dtype=bool)
        
        if self._add_actuators and hasattr(self, '_current_action'):
            # Use action values: key is "active" if action > threshold
            # Action values are in Nm (effort), typically 0-2.0 for pressed keys
            action_threshold = 0.1  # If effort > 0.1 Nm, consider key as pressed/active
            keys_to_color_red = self._current_action > action_threshold
        else:
            # Use position-based activation detection
            keys_to_color_red = self._activation
        
        for key_id in range(piano_consts.NUM_KEYS):
            mesh_path = self._key_mesh_paths.get(key_id)
            if not mesh_path:
                continue
            
            prim = stage.GetPrimAtPath(mesh_path)
            if not prim or not prim.IsValid():
                continue
            
            mesh = self._UsdGeom.Mesh(prim)
            if not mesh:
                continue
            
            # Get or create displayColor attribute
            color_attr = mesh.GetDisplayColorAttr()
            if not color_attr:
                color_attr = mesh.CreateDisplayColorAttr()
            
            if keys_to_color_red[key_id]:
                # Set activation color (bright red for high visibility)
                color = self._Gf.Vec3f(
                    piano_consts.ACTIVATION_COLOR[0],
                    piano_consts.ACTIVATION_COLOR[1],
                    piano_consts.ACTIVATION_COLOR[2]
                )
            else:
                # Restore original color
                original = self._original_colors.get(key_id)
                if original:
                    color = self._Gf.Vec3f(*original[:3])
                else:
                    # Default gray (triggers material color)
                    color = self._Gf.Vec3f(0.5, 0.5, 0.5)
            
            color_attr.Set([color])

    def apply_action(self, action: np.ndarray) -> None:
        """Apply control action to the piano using position control.
        
        This mimics MuJoCo's position-controlled actuators (servos).
        Only works if add_actuators=True.
        
        Args:
            action: Control action array of shape (89,) where:
                - action[0:88] are key control signals (treated as position targets)
                - action[88] is the sustain pedal
        """
        if not self._add_actuators:
            raise ValueError("Cannot apply action if `add_actuators` is False.")
        
        # Ensure joints are set up
        if not hasattr(self, '_joint_indices') or not self._joint_indices:
            return  # Will be set up on first update
        
        # Convert action values to target joint positions
        # Action > threshold (0.1) = pressed → upper limit (fully pressed)
        # Action <= threshold = released → lower limit (rest position)
        action_threshold = 0.1
        target_positions = np.zeros(piano_consts.NUM_KEYS)
        
        for key_id in range(piano_consts.NUM_KEYS):
            if key_id in self._joint_indices:
                joint_idx = self._joint_indices[key_id]
                if action[key_id] > action_threshold:
                    # Pressed: set to upper limit (fully pressed position)
                    target_positions[key_id] = self._qpos_range[joint_idx, 1]
                else:
                    # Released: set to lower limit (rest position)
                    target_positions[key_id] = self._qpos_range[joint_idx, 0]
        
        # Map target positions to joint indices and set them
        joint_names = self._articulation.dof_names
        if joint_names is not None:
            sorted_positions = np.zeros(len(joint_names))
            for key_id, joint_idx in self._joint_indices.items():
                if joint_idx < len(sorted_positions):
                    sorted_positions[joint_idx] = target_positions[key_id]
            
            # Apply position control (like MuJoCo's position servo)
            self._articulation.set_joint_positions(sorted_positions)
        
        # Store current action for visual feedback (keys pressed if action > threshold)
        # This allows keys to turn red based on action, not just position
        if not hasattr(self, '_current_action'):
            self._current_action = np.zeros(piano_consts.NUM_KEYS)
        self._current_action[:] = action[:-1]  # Store key actions (exclude sustain)
        
        # Set sustain state
        self._sustain_state[0] = action[-1]

    def apply_sustain(self, sustain: float) -> None:
        """Apply sustain pedal value.
        
        Args:
            sustain: Sustain value between 0 and 1.
        """
        self._sustain_state[0] = sustain

    # Accessors (observables)

    @property
    def n_keys(self) -> int:
        """Number of piano keys."""
        return piano_consts.NUM_KEYS

    @property
    def activation(self) -> np.ndarray:
        """Boolean array of key activation states (88 keys)."""
        return self._activation.copy()

    @property
    def sustain_activation(self) -> np.ndarray:
        """Boolean array of sustain pedal activation (1 element)."""
        return self._sustain_activation.copy()

    @property
    def state(self) -> np.ndarray:
        """Joint position states (88 keys)."""
        return self._state.copy()

    @property
    def normalized_state(self) -> np.ndarray:
        """Normalized joint position states between 0 and 1 (88 keys)."""
        return self._normalized_state.copy()

    @property
    def sustain_state(self) -> np.ndarray:
        """Sustain pedal state (1 element)."""
        return self._sustain_state.copy()

    @property
    def midi_module(self) -> midi_module.MidiModule:
        """The MIDI module for sound generation."""
        return self._midi_module

    @property
    def articulation(self) -> "Articulation":
        """The underlying Isaac Sim Articulation object."""
        return self._articulation

    def get_joint_positions(self) -> np.ndarray:
        """Returns the piano key joint positions.
        
        Returns:
            Array of joint positions for all 88 keys.
        """
        joint_positions = self._articulation.get_joint_positions()
        
        # Sort by key ID
        sorted_positions = np.zeros(piano_consts.NUM_KEYS)
        for key_id, joint_idx in self._joint_indices.items():
            if joint_idx < len(joint_positions):
                sorted_positions[key_id] = joint_positions[joint_idx]
        
        return sorted_positions

    def get_latest_midi_messages(self):
        """Get MIDI messages from the last update."""
        return self._midi_module.get_latest_midi_messages()

    def get_all_midi_messages(self):
        """Get all MIDI messages since episode start."""
        return self._midi_module.get_all_midi_messages()

    def register_note_on_callback(self, callback: Callable[[int, int], None]) -> None:
        """Register callback for note on events.
        
        Args:
            callback: Function(note, velocity) to call when a note is pressed.
        """
        self._midi_module.register_synth_note_on_callback(callback)

    def register_note_off_callback(self, callback: Callable[[int], None]) -> None:
        """Register callback for note off events.
        
        Args:
            callback: Function(note) to call when a note is released.
        """
        self._midi_module.register_synth_note_off_callback(callback)

    def register_sustain_on_callback(self, callback: Callable[[], None]) -> None:
        """Register callback for sustain pedal on events.
        
        Args:
            callback: Function() to call when sustain pedal is pressed.
        """
        self._midi_module.register_synth_sustain_on_callback(callback)

    def register_sustain_off_callback(self, callback: Callable[[], None]) -> None:
        """Register callback for sustain pedal off events.
        
        Args:
            callback: Function() to call when sustain pedal is released.
        """
        self._midi_module.register_synth_sustain_off_callback(callback)

