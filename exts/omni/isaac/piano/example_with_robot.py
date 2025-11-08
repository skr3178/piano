#!/usr/bin/env python3
"""Example of using the Isaac Piano with a robot hand."""

import sys
import numpy as np
from pathlib import Path

from omni.isaac.kit import SimulationApp
simulation_app = SimulationApp({"headless": False})

from omni.isaac.core import World
from omni.isaac.core.articulations import Articulation
from omni.isaac.core.utils.stage import add_reference_to_stage

# Import our piano extension
sys.path.insert(0, str(Path(__file__).parent.parent.parent.parent.parent))
from omni.isaac.piano import IsaacPianoController


def main():
    """Main simulation loop with robot hand."""
    
    print("=" * 70)
    print("Isaac Sim Piano - Robot Hand Example")
    print("=" * 70)
    
    # Create world
    world = World(stage_units_in_meters=1.0)
    
    # Path to piano USD file
    piano_usd_path = Path(__file__).parent.parent.parent.parent.parent.parent / "piano_with_physics.usda"
    
    if not piano_usd_path.exists():
        print(f"\nERROR: Piano USD file not found at: {piano_usd_path}")
        print("Please run generate_piano_usda_with_physics.py first.")
        simulation_app.close()
        return
    
    print(f"\nLoading piano from: {piano_usd_path}")
    
    # Add piano to stage
    piano_prim_path = "/World/Piano"
    add_reference_to_stage(str(piano_usd_path), piano_prim_path)
    
    # Create piano articulation
    piano_articulation = world.scene.add(
        Articulation(
            prim_path=piano_prim_path,
            name="piano",
        )
    )
    
    # Create piano controller
    piano_controller = IsaacPianoController(
        piano_articulation=piano_articulation,
        prim_path=piano_prim_path,
        change_color_on_activation=True,
        add_actuators=False,
    )
    
    # Register MIDI callbacks
    midi_events = []
    
    def on_note_on(note: int, velocity: int):
        event = f"Note ON: {note} (velocity: {velocity})"
        print(f"♪ {event}")
        midi_events.append(event)
    
    def on_note_off(note: int):
        event = f"Note OFF: {note}"
        print(f"♪ {event}")
        midi_events.append(event)
    
    piano_controller.register_note_on_callback(on_note_on)
    piano_controller.register_note_off_callback(on_note_off)
    
    # TODO: Add robot hand loading and control here
    # Example:
    # from omni.isaac.core.utils.nucleus import get_assets_root_path
    # hand_usd_path = get_assets_root_path() + "/Isaac/Robots/Franka/franka.usd"
    # add_reference_to_stage(hand_usd_path, "/World/Hand")
    # hand = world.scene.add(Articulation(prim_path="/World/Hand", name="hand"))
    
    # Reset world
    world.reset()
    
    print("\n" + "=" * 70)
    print("Simulation Started")
    print("=" * 70)
    print("\nInteract with the piano keys using the robot hand!")
    print("MIDI messages will be printed to console.")
    print("\n" + "=" * 70)
    
    # Initialize episode
    piano_controller.initialize_episode()
    
    step_count = 0
    
    # Simulation loop
    while simulation_app.is_running():
        world.step(render=True)
        
        if world.is_playing():
            # Update piano controller
            dt = world.get_physics_dt()
            piano_controller.update(dt)
            
            step_count += 1
            
            # Control robot hand here
            # Example: hand.apply_action(...)
            
            # Print status
            if step_count % 100 == 0:
                activated_keys = np.sum(piano_controller.activation)
                total_events = len(midi_events)
                print(f"[Step {step_count}] Active keys: {activated_keys}, Total MIDI events: {total_events}")
    
    print("\nSimulation ended.")
    print(f"Total MIDI events generated: {len(midi_events)}")
    simulation_app.close()


if __name__ == "__main__":
    main()

