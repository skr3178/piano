#!/usr/bin/env python3
"""Standalone example of using the Isaac Piano extension."""

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
    """Main simulation loop."""
    
    print("=" * 70)
    print("Isaac Sim Piano - Standalone Example")
    print("=" * 70)
    
    # Create world
    world = World(stage_units_in_meters=1.0)
    
    # Path to piano USD file
    piano_usd_path = Path(__file__).parent.parent.parent.parent.parent.parent / "piano_with_physics.usda"
    
    if not piano_usd_path.exists():
        print(f"\nERROR: Piano USD file not found at: {piano_usd_path}")
        print("Please run generate_piano_usda_with_physics.py first to create the piano model.")
        simulation_app.close()
        return
    
    print(f"\nLoading piano from: {piano_usd_path}")
    
    # Add piano to stage
    piano_prim_path = "/World/Piano"
    add_reference_to_stage(str(piano_usd_path), piano_prim_path)
    
    # Create articulation
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
    def on_note_on(note: int, velocity: int):
        print(f"♪ Note ON:  {note} (velocity: {velocity})")
    
    def on_note_off(note: int):
        print(f"♪ Note OFF: {note}")
    
    def on_sustain_on():
        print("🎹 Sustain pedal: ON")
    
    def on_sustain_off():
        print("🎹 Sustain pedal: OFF")
    
    piano_controller.register_note_on_callback(on_note_on)
    piano_controller.register_note_off_callback(on_note_off)
    piano_controller.register_sustain_on_callback(on_sustain_on)
    piano_controller.register_sustain_off_callback(on_sustain_off)
    
    # Reset world
    world.reset()
    
    print("\n" + "=" * 70)
    print("Simulation Started")
    print("=" * 70)
    print("\nInstructions:")
    print("  • Click and drag on piano keys to press them")
    print("  • Watch them turn green when activated")
    print("  • MIDI messages will be printed to console")
    print("  • Press ESC or close window to exit")
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
            
            # Print status every 100 steps
            if step_count % 100 == 0:
                activated_keys = np.sum(piano_controller.activation)
                if activated_keys > 0:
                    print(f"[Step {step_count}] Active keys: {activated_keys}")
    
    print("\nSimulation ended.")
    simulation_app.close()


if __name__ == "__main__":
    main()


