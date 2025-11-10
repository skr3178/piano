"""Configure collision groups for piano keys and hands in Isaac Sim.

This script sets collision groups on all collision prims to ensure
piano keys (group 0) collide with hands (group 1) but not with each other.

Usage in Isaac Sim:
    from configure_collisions import configure_collisions
    import omni.usd
    stage = omni.usd.get_context().get_stage()
    configure_collisions(stage)
"""

from pxr import Usd, UsdPhysics, Sdf


def configure_collisions(stage, load_payloads=True):
    """Configure collision groups recursively on all collision prims.
    
    Args:
        stage: USD stage to configure
        load_payloads: If True, load all payloads before configuring (required for hands)
    """
    if not stage:
        print("Error: No stage provided")
        return
    
    # Load all payloads first (required for hand collision prims)
    if load_payloads:
        print("Loading payloads (required for hand collision prims)...")
        world_prim = stage.GetPrimAtPath("/World")
        if world_prim and world_prim.IsValid():
            for prim in Usd.PrimRange(world_prim):
                if prim.HasPayload():
                    prim.Load()
        print("  ✓ Payloads loaded")
    
    collision_count = [0]  # Use list to allow modification in nested function
    
    def set_collision_groups(prim, is_hand=False, is_piano=False):
        """Recursively set collision groups on collision prims."""
        if not prim or not prim.IsValid():
            return
        
        # Determine if this is a hand or piano prim based on path
        prim_path = str(prim.GetPath())
        is_hand_prim = "hand" in prim_path.lower() or is_hand
        is_piano_prim = "piano" in prim_path.lower() or is_piano
        
        # Set collision groups on collision prims
        if prim.HasAPI(UsdPhysics.CollisionAPI):
            override_prim = stage.OverridePrim(prim.GetPath())
            if override_prim:
                try:
                    # Ensure collision is enabled
                    collision_api = UsdPhysics.CollisionAPI(override_prim)
                    if collision_api:
                        try:
                            collision_api.GetCollisionEnabledAttr().Set(True)
                        except:
                            collision_api.CreateCollisionEnabledAttr(True)
                    else:
                        collision_api = UsdPhysics.CollisionAPI.Apply(override_prim)
                        collision_api.CreateCollisionEnabledAttr(True)
                    
                    # Set collision groups
                    if is_hand_prim:
                        # Hand: group 1, filter 1 (collide with group 0)
                        override_prim.CreateAttribute("physx:collisionGroup", Sdf.ValueTypeNames.Int).Set(1)
                        override_prim.CreateAttribute("physx:collisionFilter", Sdf.ValueTypeNames.Int).Set(1)  # 0b01 = collide with group 0
                        collision_count[0] += 1
                    elif is_piano_prim:
                        # Piano: group 0, filter 2 (collide with group 1)
                        override_prim.CreateAttribute("physx:collisionGroup", Sdf.ValueTypeNames.Int).Set(0)
                        override_prim.CreateAttribute("physx:collisionFilter", Sdf.ValueTypeNames.Int).Set(2)  # 0b10 = collide with group 1
                        collision_count[0] += 1
                except Exception as e:
                    print(f"Warning: Could not set collision groups on {prim.GetPath()}: {e}")
        
        # Recursively process children (including those in payloads)
        for child in prim.GetChildren():
            set_collision_groups(child, is_hand_prim, is_piano_prim)
    
    # Configure collisions starting from World
    world_prim = stage.GetPrimAtPath("/World")
    if world_prim and world_prim.IsValid():
        # Process piano
        piano_prim = world_prim.GetChild("Piano")
        if piano_prim and piano_prim.IsValid():
            set_collision_groups(piano_prim, is_piano=True)
        
        # Process hands
        for hand_name in ["left_hand", "right_hand"]:
            hand_prim = world_prim.GetChild(hand_name)
            if hand_prim and hand_prim.IsValid():
                set_collision_groups(hand_prim, is_hand=True)
        
        print(f"✓ Configured collision groups on {collision_count[0]} collision prims")
        return collision_count[0]
    else:
        print("Error: Could not find /World prim")
        return 0


if __name__ == "__main__":
    # This can be called directly if running in Isaac Sim
    try:
        import omni.usd
        stage = omni.usd.get_context().get_stage()
        if stage:
            configure_collisions(stage)
        else:
            print("Error: No stage available. Make sure you're running this in Isaac Sim.")
    except ImportError:
        print("Error: This script must be run in Isaac Sim environment")
        print("Usage: from configure_collisions import configure_collisions")

