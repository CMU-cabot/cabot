import os
import glob
import math
import re

def get_hips_rotation(dae_path):
    with open(dae_path, 'r', encoding='utf-8') as f:
        content = f.read()
        
    # Find Hips node and its matrix
    # Looking for structure like: <node id="Hips" ...> ... <matrix ...>NUMBERS</matrix>
    # Note: XML parsing is safer but regex is quicker for this specific structure if consistent
    
    # Locate Hips node start
    hips_match = re.search(r'<node [^>]*id="Hips"[^>]*>', content)
    if not hips_match:
        return None
        
    start_idx = hips_match.end()
    # Find the next matrix element
    matrix_match = re.search(r'<matrix[^>]*>(.*?)</matrix>', content[start_idx:], re.DOTALL)
    if not matrix_match:
        return None
        
    matrix_str = matrix_match.group(1).strip()
    values = [float(x) for x in matrix_str.split()]
    
    if len(values) != 16:
        return None
        
    # Assuming row-major order as per standard
    # m0  m1  m2  m3
    # m4  m5  m6  m7
    # m8  m9  m10 m11
    # m12 m13 m14 m15
    
    # Rotation around X axis check
    # [ 1  0  0 ]
    # [ 0  c -s ]
    # [ 0  s  c ]
    
    # m5 = cos(theta), m9 = sin(theta)
    # Check if m0 is Approx 1 and matrix looks like X-rotation
    
    m5 = values[5]
    m9 = values[9]
    
    # atan2(y, x) -> atan2(sin, cos)
    theta = math.atan2(m9, m5)
    
    # Translation
    tx = values[3]
    ty = values[7]
    tz = values[11]
    
    return theta, tx, ty, tz

def main():
    base_dir = "/home/developer/models/LIRS-HMLG"
    print(f"Searching in {base_dir}")
    
    files = glob.glob(os.path.join(base_dir, "*", "*", "walk.dae"))
    print(f"Found {len(files)} files")
    
    for p in sorted(files):
        # Extract meaningful name part
        parts = p.split("/")
        name = "/".join(parts[-3:-1])
        
        result = get_hips_rotation(p)
        if result:
            theta, tx, ty, tz = result
            print(f"{name}: theta={theta:.4f} ({math.degrees(theta):.1f} deg), tx={tx:.4f}, ty={ty:.4f}, tz={tz:.4f}")
        else:
            print(f"{name}: Failed to parse Hips matrix")

if __name__ == "__main__":
    main()

search_path = "/home/ai-suitcase-1/nitta_workspace/cabot/cabot-navigation/LIRS-HMLG"
dae_files = glob.glob(os.path.join(search_path, "**", "*.dae"), recursive=True)

offsets = {}

target_rad = 1.57079632679 # 90 degrees

print("model_offsets = {")
for dae_file in sorted(dae_files):
    fname = os.path.basename(dae_file)
    if fname in ["walk.dae", "run.dae", "talk.dae", "sit.dae"]:
        # These are commonly named files, might be duplicated across folders. 
        # The manager.py uses os.path.join(model_path, "*", "*", "walk.dae")
        # And it uses `skin_file = random.choice(self.models)` where self.models is list of full paths
        # BUT manager.py code:
        # skin_file = "walk.dae" -> then later: if self.models: skin_file = random.choice(self.models)
        # However, the XML generation uses: <filename>{skin_file}</filename>
        # If skin_file is a full path, that's fine.
        # But wait, manager.py does `models = glob.glob(os.path.join(model_path, "*", "*", "walk.dae"))`
        # So it selects specific files.
        # The user request implies we should map specific files or perhaps parent folders?
        # The loop in manager.py seems to pick a full path.
        pass

    theta = get_hips_rotation(dae_file)
    
    if theta is not None:
        correction = target_rad - theta
        # Normalize to -pi to pi if needed, though here we likely just want small additive value
        # Actually LIRS models seem to be 0 < theta < pi/2 mostly.
        
        # We only really care about walk.dae if that's what's being used, but the manager code
        # picks any .dae found in the glob.
        # Actually manager.py: "self.models = glob.glob(.../walk.dae)"
        # So it ONLY picks walk.dae files.
        pass
    else:
        correction = 0.0

    # Key by full path or relative path?
    # manager.py selects `skin_file` as absolute path.
    # But inside the Docker container, paths might differ slightly ("/home/developer/...").
    # The current manager.py is running on the host? No, inside container probably or mapped.
    # The file path used in manager.py `self.models` comes from `/home/developer/models/LIRS-HMLG` which is mapped.
    # Let's use the file name first, or partial path.
    # Since `walk.dae` is not unique, we need uniqueness.
    # The `self.models` glob yields `.../Children/c_casual/walk.dae`.
    # Let's use "Category/ModelName/filename" as key, as that is unique.
    
    clean_key = dae_file.replace(search_path + "/", "")
    if "walk.dae" in fname and theta is not None:
        print(f"    '{clean_key}': {correction:.4f},")

print("}")
