#!/usr/bin/env python3
"""
Mirror PathPlanner paths from right to left side of the field.
This script reads Right_*.path files and creates mirrored Left_* variants.

FRC 2026 field width: 8.07 meters
"""

import json
import sys
from pathlib import Path

FIELD_WIDTH = 8.07  # Meters
PATHS_DIR = Path("src/main/deploy/pathplanner/paths")


def mirror_waypoint(waypoint):
    """Mirror a waypoint by flipping Y coordinates and unlinking it."""
    # Mirror anchor point
    if "anchor" in waypoint:
        waypoint["anchor"]["y"] = FIELD_WIDTH - waypoint["anchor"]["y"]
    
    # Mirror prev control point
    if "prevControl" in waypoint and waypoint["prevControl"] is not None:
        waypoint["prevControl"]["y"] = FIELD_WIDTH - waypoint["prevControl"]["y"]
    
    # Mirror next control point
    if "nextControl" in waypoint and waypoint["nextControl"] is not None:
        waypoint["nextControl"]["y"] = FIELD_WIDTH - waypoint["nextControl"]["y"]
    
    # Unlink the waypoint
    waypoint["linkedName"] = None
    
    return waypoint


def mirror_path(input_name, output_name):
    """Mirror a single path file."""
    input_path = PATHS_DIR / f"{input_name}.path"
    output_path = PATHS_DIR / f"{output_name}.path"
    
    if not input_path.exists():
        print(f"❌ Path file not found: {input_path}")
        return False
    
    try:
        # Read JSON
        with open(input_path, 'r') as f:
            data = json.load(f)
        
        # Mirror waypoints
        if "waypoints" in data:
            for waypoint in data["waypoints"]:
                mirror_waypoint(waypoint)
        
        # Mirror rotation targets
        if "rotationTargets" in data:
            for target in data["rotationTargets"]:
                if "position" in target:
                    target["position"]["y"] = FIELD_WIDTH - target["position"]["y"]
                if "rotation" in target:
                    target["rotation"] = -target["rotation"]
        
        # Mirror goal rotation
        if "goalEndState" in data and "rotation" in data["goalEndState"]:
            data["goalEndState"]["rotation"] = -data["goalEndState"]["rotation"]
        
        # Mirror ideal starting rotation
        if "idealStartingState" in data and "rotation" in data["idealStartingState"]:
            data["idealStartingState"]["rotation"] = -data["idealStartingState"]["rotation"]
        
        # Update folder name
        if "folder" in data and "Right" in data["folder"]:
            data["folder"] = data["folder"].replace("Right", "Left")
        
        # Write output
        with open(output_path, 'w') as f:
            json.dump(data, f, indent=2)
        
        print(f"✓ Mirrored: {input_name} → {output_name}")
        return True
    
    except Exception as e:
        print(f"❌ Error mirroring {input_name}: {e}")
        return False


def mirror_all_right_paths():
    """Mirror all Right_*.path files to Left_* variants."""
    if not PATHS_DIR.exists():
        print(f"❌ Paths directory not found: {PATHS_DIR}")
        return False
    
    right_paths = sorted(PATHS_DIR.glob("Right_*.path"))
    
    if not right_paths:
        print("❌ No Right_* paths found to mirror.")
        return False
    
    success_count = 0
    for path_file in right_paths:
        input_name = path_file.stem
        output_name = input_name.replace("Right_", "Left_")
        
        if mirror_path(input_name, output_name):
            success_count += 1
    
    print(f"\n✓ Successfully mirrored {success_count}/{len(right_paths)} paths")
    return True


def main():
    """Main entry point."""
    print("🔄 Mirroring PathPlanner paths...\n")
    
    if len(sys.argv) == 1:
        # Mirror all Right_* paths
        mirror_all_right_paths()
    elif len(sys.argv) == 3:
        # Mirror specific path
        input_name = sys.argv[1]
        output_name = sys.argv[2]
        mirror_path(input_name, output_name)
    else:
        print("Usage:")
        print("  ./mirror-paths.py                    # Mirror all Right_* paths")
        print("  ./mirror-paths.py <input> <output>   # Mirror specific path")
        sys.exit(1)


if __name__ == "__main__":
    main()
