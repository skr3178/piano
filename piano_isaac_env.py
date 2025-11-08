"""
Isaac Sim environment for piano with shadow hands, matching MuJoCo reward structure.
"""

import numpy as np
from typing import Optional, List, Tuple
from scipy.optimize import linear_sum_assignment
from dataclasses import dataclass

# Reward constants matching MuJoCo
FINGER_CLOSE_ENOUGH_TO_KEY = 0.01
KEY_CLOSE_ENOUGH_TO_PRESSED = 0.05
ENERGY_PENALTY_COEF = 5e-3


def tolerance(x, bounds=(0, 0.05), margin=0.5, sigmoid="gaussian"):
    """
    Tolerance function matching dm_control.utils.rewards.tolerance.
    
    Args:
        x: Value to evaluate
        bounds: (lower, upper) bounds for full reward
        margin: Margin beyond bounds where reward decays
        sigmoid: Type of sigmoid ("gaussian" or "linear")
    
    Returns:
        Reward value between 0 and 1
    """
    lower, upper = bounds
    if sigmoid == "gaussian":
        # Gaussian tolerance function
        if x < lower:
            return np.exp(-0.5 * ((x - lower) / margin) ** 2)
        elif x > upper:
            return np.exp(-0.5 * ((x - upper) / margin) ** 2)
        else:
            return 1.0
    elif sigmoid == "linear":
        # Linear tolerance function
        if x < lower:
            return max(0, 1.0 - (lower - x) / margin)
        elif x > upper:
            return max(0, 1.0 - (x - upper) / margin)
        else:
            return 1.0
    else:
        raise ValueError(f"Unknown sigmoid type: {sigmoid}")


@dataclass
class RewardComponents:
    """Container for individual reward components."""
    key_press: float = 0.0
    sustain: float = 0.0
    energy: float = 0.0
    fingering: float = 0.0
    ot_fingering: float = 0.0
    forearm: float = 0.0
    
    def total(self) -> float:
        """Compute total reward."""
        return (
            self.key_press +
            self.sustain +
            self.energy +
            self.fingering +
            self.ot_fingering +
            self.forearm
        )


class PianoRewardFunction:
    """
    Reward function matching MuJoCo piano_with_shadow_hands task.
    
    This class implements the same reward components as the MuJoCo environment:
    - Key press reward
    - Sustain reward
    - Energy penalty
    - Fingering reward (optional)
    - OT fingering reward (optional)
    - Forearm collision reward (optional)
    """
    
    def __init__(
        self,
        energy_penalty_coef: float = ENERGY_PENALTY_COEF,
        key_close_enough: float = KEY_CLOSE_ENOUGH_TO_PRESSED,
        finger_close_enough: float = FINGER_CLOSE_ENOUGH_TO_KEY,
        disable_fingering_reward: bool = False,
        disable_forearm_reward: bool = False,
    ):
        self.energy_penalty_coef = energy_penalty_coef
        self.key_close_enough = key_close_enough
        self.finger_close_enough = finger_close_enough
        self.disable_fingering_reward = disable_fingering_reward
        self.disable_forearm_reward = disable_forearm_reward
    
    def compute_key_press_reward(
        self,
        goal_state: np.ndarray,
        key_states: np.ndarray,
        key_activations: np.ndarray,
        key_qpos_range: Optional[np.ndarray] = None,
    ) -> float:
        """
        Reward for pressing the right keys at the right time.
        
        Matches MuJoCo _compute_key_press_reward:
        - 50% reward for correct keys pressed (tolerance on activation)
        - 50% reward for no false positives
        
        Args:
            goal_state: (n_keys + 1,) array, 1 for keys to press, 0 otherwise (+ sustain)
            key_states: (n_keys,) array of normalized key positions [0, 1]
            key_activations: (n_keys,) array of key activation (pressed = 1, not pressed = 0)
            key_qpos_range: (n_keys, 2) array of [min, max] for normalization, optional
        """
        goal_keys = goal_state[:-1]  # Exclude sustain
        on = np.flatnonzero(goal_keys)
        rew = 0.0
        
        # Reward for pressing correct keys
        if on.size > 0:
            if key_qpos_range is not None:
                # Normalize key states using qpos_range
                actual = key_states
            else:
                # Assume key_states are already normalized [0, 1]
                actual = key_states
            
            # Tolerance on difference between goal and actual
            diffs = goal_keys[on] - actual[on]
            rews = np.array([
                tolerance(diff, bounds=(0, self.key_close_enough), 
                         margin=(self.key_close_enough * 10), sigmoid="gaussian")
                for diff in diffs
            ])
            rew += 0.5 * rews.mean()
        
        # Penalty for false positives (keys pressed when they shouldn't be)
        off = np.flatnonzero(1 - goal_keys)
        if off.size > 0:
            rew += 0.5 * (1.0 - float(key_activations[off].any()))
        
        return rew
    
    def compute_sustain_reward(
        self,
        goal_sustain: float,
        sustain_activation: float,
    ) -> float:
        """
        Reward for pressing the sustain pedal at the right time.
        
        Args:
            goal_sustain: Target sustain activation (0 or 1)
            sustain_activation: Current sustain activation [0, 1]
        """
        diff = goal_sustain - sustain_activation
        return tolerance(
            diff,
            bounds=(0, self.key_close_enough),
            margin=(self.key_close_enough * 10),
            sigmoid="gaussian",
        )
    
    def compute_energy_reward(
        self,
        left_hand_power: np.ndarray,
        right_hand_power: np.ndarray,
    ) -> float:
        """
        Energy penalty for minimizing actuator power.
        
        Args:
            left_hand_power: (n_actuators,) array of power for left hand
            right_hand_power: (n_actuators,) array of power for right hand
        """
        total_power = np.sum(np.abs(left_hand_power)) + np.sum(np.abs(right_hand_power))
        return -self.energy_penalty_coef * total_power
    
    def compute_fingering_reward(
        self,
        left_fingertip_positions: np.ndarray,  # (n_fingers, 3)
        right_fingertip_positions: np.ndarray,  # (n_fingers, 3)
        key_positions: np.ndarray,  # (n_keys, 3) - positions of key centers
        goal_keys: List[Tuple[int, int]],  # List of (key_idx, finger_idx) pairs
        hand_side: str = "both",  # "left", "right", or "both"
    ) -> float:
        """
        Reward for minimizing distance between fingertips and target keys.
        
        Args:
            left_fingertip_positions: (n_fingers, 3) array
            right_fingertip_positions: (n_fingers, 3) array
            key_positions: (n_keys, 3) array
            goal_keys: List of (key_idx, finger_idx, hand_side) tuples
            hand_side: Which hand to use
        """
        distances = []
        
        for key_idx, finger_idx, side in goal_keys:
            if side == "left":
                fingertip_pos = left_fingertip_positions[finger_idx]
            elif side == "right":
                fingertip_pos = right_fingertip_positions[finger_idx]
            else:
                continue
            
            key_pos = key_positions[key_idx]
            diff = key_pos - fingertip_pos
            distances.append(float(np.linalg.norm(diff)))
        
        if not distances:
            return 0.0
        
        rews = np.array([
            tolerance(d, bounds=(0, self.finger_close_enough),
                     margin=(self.finger_close_enough * 10), sigmoid="gaussian")
            for d in distances
        ])
        return float(rews.mean())
    
    def compute_ot_fingering_reward(
        self,
        left_fingertip_positions: np.ndarray,  # (n_fingers, 3)
        right_fingertip_positions: np.ndarray,  # (n_fingers, 3)
        key_positions: np.ndarray,  # (n_keys_to_press, 3)
    ) -> float:
        """
        Optimal Transport-based fingering reward using Hungarian algorithm.
        
        Matches MuJoCo _compute_ot_fingering_reward.
        Uses Hungarian algorithm to optimally assign fingers to keys.
        
        Args:
            left_fingertip_positions: (n_fingers, 3) array
            right_fingertip_positions: (n_fingers, 3) array
            key_positions: (n_keys_to_press, 3) array
        """
        # Combine all fingertip positions
        all_fingertips = np.vstack([left_fingertip_positions, right_fingertip_positions])
        n_fingers = all_fingertips.shape[0]
        n_keys = key_positions.shape[0]
        
        if n_keys == 0:
            return 1.0
        
        # Compute distance matrix
        # distances[i, j] = distance from finger i to key j
        distances = np.zeros((n_fingers, n_keys))
        for i in range(n_fingers):
            for j in range(n_keys):
                diff = key_positions[j] - all_fingertips[i]
                distances[i, j] = np.linalg.norm(diff)
        
        # Use Hungarian algorithm for optimal assignment
        if n_fingers >= n_keys:
            # Pad with dummy fingers if needed
            row_indices, col_indices = linear_sum_assignment(distances[:n_keys, :])
        else:
            # More keys than fingers - assign best fingers to keys
            row_indices, col_indices = linear_sum_assignment(distances.T)
            row_indices, col_indices = col_indices, row_indices
        
        # Compute reward based on assigned distances
        assigned_distances = distances[row_indices, col_indices]
        rews = np.array([
            tolerance(d, bounds=(0, self.finger_close_enough),
                     margin=(self.finger_close_enough * 10), sigmoid="gaussian")
            for d in assigned_distances
        ])
        
        return float(rews.mean())
    
    def compute_forearm_reward(
        self,
        has_collision: bool,
    ) -> float:
        """
        Reward for not colliding forearms.
        
        Args:
            has_collision: True if forearms are colliding
        """
        return 0.0 if has_collision else 0.5
    
    def compute(
        self,
        goal_state: np.ndarray,
        key_states: np.ndarray,
        key_activations: np.ndarray,
        sustain_activation: float,
        left_hand_power: np.ndarray,
        right_hand_power: np.ndarray,
        left_fingertip_positions: Optional[np.ndarray] = None,
        right_fingertip_positions: Optional[np.ndarray] = None,
        key_positions: Optional[np.ndarray] = None,
        goal_keys: Optional[List[Tuple[int, int, str]]] = None,
        has_forearm_collision: bool = False,
        use_ot_fingering: bool = False,
        key_qpos_range: Optional[np.ndarray] = None,
    ) -> RewardComponents:
        """
        Compute all reward components.
        
        Returns:
            RewardComponents object with individual rewards and total
        """
        rewards = RewardComponents()
        
        # Key press reward
        rewards.key_press = self.compute_key_press_reward(
            goal_state, key_states, key_activations, key_qpos_range
        )
        
        # Sustain reward
        rewards.sustain = self.compute_sustain_reward(
            goal_state[-1], sustain_activation
        )
        
        # Energy reward (penalty)
        rewards.energy = self.compute_energy_reward(
            left_hand_power, right_hand_power
        )
        
        # Fingering rewards (optional)
        if not self.disable_fingering_reward:
            if use_ot_fingering and left_fingertip_positions is not None:
                keys_to_press = np.flatnonzero(goal_state[:-1])
                if keys_to_press.size > 0 and key_positions is not None:
                    key_pos_to_press = key_positions[keys_to_press]
                    rewards.ot_fingering = self.compute_ot_fingering_reward(
                        left_fingertip_positions,
                        right_fingertip_positions,
                        key_pos_to_press,
                    )
            elif goal_keys is not None and key_positions is not None:
                rewards.fingering = self.compute_fingering_reward(
                    left_fingertip_positions,
                    right_fingertip_positions,
                    key_positions,
                    goal_keys,
                )
        
        # Forearm reward (optional)
        if not self.disable_forearm_reward:
            rewards.forearm = self.compute_forearm_reward(has_forearm_collision)
        
        return rewards

