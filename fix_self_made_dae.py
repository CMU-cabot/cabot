#!/usr/bin/env python3
"""
Fix self_made child DAE files for Gazebo compatibility.

Problems addressed:
1. Nested animation structure: Gazebo's COLLADA parser does not support
   <animation> elements nested inside another <animation>. This script
   flattens the structure so all bone animations are direct children of
   <library_animations>.

2. Relative texture paths: Replace bare filenames with absolute file:// URIs
   so Gazebo/Ogre can always resolve textures regardless of working directory.
"""

import xml.etree.ElementTree as ET
import os
import glob
import shutil

COLLADA_NS = "http://www.collada.org/2005/11/COLLADASchema"
NS = {"c": COLLADA_NS}

# In-container model path (Gazebo runs inside Docker with this mount)
CONTAINER_BASE = "/home/developer/models/LIRS-HMLG/Children/self_made"

# Host path to map from
HOST_BASE = "/home/ai-suitcase-1/nitta_workspace/cabot/cabot-navigation/LIRS-HMLG/Children/self_made"


def flatten_animations(dae_path):
    """Flatten nested <animation> structure and fix texture paths."""
    print(f"Processing: {dae_path}")

    # Register namespace to avoid ns0: prefix in output
    ET.register_namespace("", COLLADA_NS)

    tree = ET.parse(dae_path)
    root = tree.getroot()

    changed = False

    # ---- Fix 1: Flatten nested animations --------------------------------
    lib_anims = root.find(f".//{{{COLLADA_NS}}}library_animations")
    if lib_anims is not None:
        # Collect top-level <animation> children that themselves contain
        # nested <animation> children (container nodes)
        containers = []
        for anim in list(lib_anims):
            tag = anim.tag.replace(f"{{{COLLADA_NS}}}", "")
            if tag != "animation":
                continue
            nested = anim.findall(f"{{{COLLADA_NS}}}animation")
            if nested:
                containers.append((anim, nested))

        for container, nested_anims in containers:
            # Remove container from library_animations
            lib_anims.remove(container)
            # Add nested animations directly to library_animations
            for nested in nested_anims:
                lib_anims.append(nested)
            changed = True
            print(f"  Flattened container: {container.get('id')} "
                  f"({len(nested_anims)} animations extracted)")

    # ---- Fix 2: Convert relative texture paths to absolute file:// URIs --
    child_dir_name = os.path.basename(os.path.dirname(dae_path))  # e.g. "child1"
    container_dir = os.path.join(CONTAINER_BASE, child_dir_name)
    host_dir = os.path.dirname(dae_path)

    lib_images = root.find(f".//{{{COLLADA_NS}}}library_images")
    if lib_images is not None:
        for image in lib_images.findall(f"{{{COLLADA_NS}}}image"):
            init_from = image.find(f"{{{COLLADA_NS}}}init_from")
            if init_from is None or init_from.text is None:
                continue
            src = init_from.text.strip()
            # Skip already-absolute paths
            if src.startswith("file://") or src.startswith("/"):
                continue
            # src is a bare filename (e.g. "afro_diffuse.png")
            # Check if file exists at host path (for validation)
            host_tex = os.path.join(host_dir, src)
            if not os.path.exists(host_tex):
                # Try textures/ subdirectory
                host_tex_sub = os.path.join(host_dir, "textures", src)
                if os.path.exists(host_tex_sub):
                    # File is in textures/ subdir
                    abs_uri = f"file://{container_dir}/textures/{src}"
                else:
                    print(f"  WARNING: texture not found on host: {src}")
                    abs_uri = f"file://{container_dir}/{src}"
            else:
                abs_uri = f"file://{container_dir}/{src}"

            init_from.text = abs_uri
            changed = True
            print(f"  Texture: {src!r} -> {abs_uri!r}")

    if changed:
        # Back up original
        backup = dae_path + ".bak"
        if not os.path.exists(backup):
            shutil.copy2(dae_path, backup)
            print(f"  Backed up to: {backup}")

        tree.write(dae_path, encoding="utf-8", xml_declaration=True)
        print(f"  Saved: {dae_path}")
    else:
        print("  No changes needed.")

    return changed


def main():
    base_dir = os.path.join(
        os.path.dirname(os.path.abspath(__file__)),
        "cabot-navigation/LIRS-HMLG/Children/self_made"
    )
    pattern = os.path.join(base_dir, "*", "*_walk.dae")
    files = sorted(glob.glob(pattern))

    if not files:
        print(f"No *_walk.dae files found under {base_dir}")
        return

    print(f"Found {len(files)} file(s) to process.\n")
    total_changed = 0
    for f in files:
        if flatten_animations(f):
            total_changed += 1
        print()

    print(f"Done. {total_changed}/{len(files)} file(s) modified.")


if __name__ == "__main__":
    main()
