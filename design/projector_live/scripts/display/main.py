#!/usr/bin/env python3
import argparse
import sys
from pathlib import Path


def _ensure_local_import_path() -> None:
    root = Path(__file__).resolve().parent
    if str(root) not in sys.path:
        sys.path.insert(0, str(root))


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Step 1 display demo: animated projector arrow")
    p.add_argument("--screen", type=int, default=1, help="Target screen index")
    # p.add_argument("--backend", choices=["opengl", "qt"], default=None, help="Rendering backend override")
    # p.add_argument("--fps", type=int, default=60, help="Render FPS")
    p.add_argument("--windowed", action="store_true", help="Run windowed instead of fullscreen")
    # p.add_argument("--line-width", type=float, default=20.0, help="Arrow body width in pixels")
    # p.add_argument("--flow-speed", type=float, default=380.0, help="Flow animation speed (px/s)")
    # p.add_argument("--smoothing-hz", type=float, default=9.0, help="Heading smoothing response (Hz)")
    return p.parse_args()

def main() -> int:
    _ensure_local_import_path()
    args = parse_args()

    from config import DisplayConfig
    from runtime import DisplayRuntime

    config_kwargs = {
        "screen_index": args.screen,
        "show_fullscreen": (not args.windowed),
    }

    # if args.backend is not None:
    #     config_kwargs["render_backend"] = args.backend
    cfg = DisplayConfig(**config_kwargs)
    return DisplayRuntime(cfg).run()

if __name__ == "__main__":
    raise SystemExit(main())
