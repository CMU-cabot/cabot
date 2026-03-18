import os, sys
from PyQt6 import QtWidgets
from control_window import ControlWindow
from projector_window import ProjectorWindow

MEDIA = {
    "RIGHT": "videos/right.mp4",   
    "LEFT": "videos/left.mp4",
    "FORWARD": "videos/forward.mp4",
    # "TEXT": "../text_compose_bounce/text.mp4",
    "TEXT_ENG": "imgs/text_eng.png",
    "TEXT_JP": "imgs/text_jp.png",
    # "FIELD": "../field_compose_pulse/field.mp4",
    "BLOCK_FIELD": "imgs/block_field.png",
    "BLOCK_FIELD_ENG": "imgs/block_field_eng.png",
    "BLOCK_FIELD_JP": "imgs/block_field_jp.png",
    "BLANK": "imgs/blank.png",
    # "MUSIC": "audio/music.mp3", 
    "BELLABOT_MUSIC": "audio/bellabot_music.mp3", 
    # "ROBOT": "audio/robot.mp3"
    "ROBOT_JP": "audio/ping_guiding_jp.mp3",
    "ROBOT_ENG": "audio/ping_guiding_en.mp3"
}
PROJECTOR_INDEX = 1 # 0 = main screen, 1 = projector; auto-falls back to 0 if only 1 screen   
START_KEY = "FORWARD" # which clip to auto-start

def main():
    app = QtWidgets.QApplication(sys.argv)

    # choose screen index (fallback to 0 if only one screen / mirroring)
    screens = app.screens()
    use_idx = PROJECTOR_INDEX if len(screens) > 1 else 0

    proj = ProjectorWindow(use_idx)
    ctrl = ControlWindow(proj, MEDIA)
    ctrl.show()

    def start_default():
        p = MEDIA.get(START_KEY)
        if p and os.path.exists(p):
            ctrl._log(f"Auto-start: {START_KEY}")
            proj.play_loop(p)
        else:
            ctrl._log("⚠️ No START_KEY clip found.")
    proj.ready.connect(start_default)

    sys.exit(app.exec())

if __name__ == "__main__":
    main()