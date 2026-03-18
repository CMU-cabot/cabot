
import os
from PyQt6 import QtWidgets, QtCore, QtGui
from projector_window import ProjectorWindow, ap

class ControlWindow(QtWidgets.QWidget):
    def __init__(self, proj: ProjectorWindow, media_path: dict):
        super().__init__()
        self.proj = proj
        self.media_path = media_path
        self.setWindowTitle("Design Controller")

        self.resize(980, 620)
        self.single_screen = (len(QtWidgets.QApplication.screens()) == 1)

        root = QtWidgets.QHBoxLayout(self)
        root.setContentsMargins(10, 10, 10, 10)

        splitter = QtWidgets.QSplitter(QtCore.Qt.Orientation.Horizontal)
        root.addWidget(splitter)

        # Left pane: LOG
        left = QtWidgets.QWidget()
        lytL = QtWidgets.QVBoxLayout(left)
        lytL.setContentsMargins(0, 0, 0, 0)
        self.log = QtWidgets.QTextEdit(readOnly = True)
        self.log.setStyleSheet("font-family: Menlo, Monaco, monospace; font-size: 12pt;")
        lytL.addWidget(self.log)

        # Right pane: CONTROLS
        right = QtWidgets.QWidget()
        r = QtWidgets.QVBoxLayout(right)
        r.setContentsMargins(0, 0, 0, 0)
        r.setSpacing(10)

        def hline():
            line = QtWidgets.QFrame()
            line.setFrameShape(QtWidgets.QFrame.Shape.HLine)
            line.setFrameShadow(QtWidgets.QFrame.Shadow.Sunken)
            return line

        # --- ARROWS ---
        grp_arrows = QtWidgets.QGroupBox("ARROWS")
        lay_ar = QtWidgets.QHBoxLayout(grp_arrows)

        btn_left  = QtWidgets.QPushButton("← Left")
        btn_fwd   = QtWidgets.QPushButton("↑ Forward")
        btn_right = QtWidgets.QPushButton("Right →")
        for b in (btn_left, btn_fwd, btn_right):
            b.setMinimumHeight(42)

        lay_ar.addWidget(btn_left)
        lay_ar.addWidget(btn_fwd)
        lay_ar.addWidget(btn_right)

        # --- TEXT ---
        grp_text = QtWidgets.QGroupBox("TEXT")
        lay_tx = QtWidgets.QHBoxLayout(grp_text)

        btn_text_eng = QtWidgets.QPushButton("📝 English Text")
        btn_text_jp = QtWidgets.QPushButton("📝 Japanese Text")
        lay_tx.addWidget(btn_text_eng)
        lay_tx.addWidget(btn_text_jp)

        # --- FIELD ---
        grp_hl = QtWidgets.QGroupBox("FIELD")
        lay_hl = QtWidgets.QHBoxLayout(grp_hl)

        btn_block_field = QtWidgets.QPushButton("Block Field")
        # btn_block_field_eng = QtWidgets.QPushButton("Block Field English")
        # btn_block_field_jp = QtWidgets.QPushButton("Block Field Japanese")
        # btn_stop_field = QtWidgets.QPushButton("Stop Field")
        lay_hl.addWidget(btn_block_field)
        # lay_hl.addWidget(btn_block_field_eng)
        # lay_hl.addWidget(btn_block_field_jp)
        # lay_hl.addWidget(btn_stop_field)

        # --- SOUNDS ---
        grp_sounds = QtWidgets.QGroupBox("SOUNDS")
        lay_sd = QtWidgets.QHBoxLayout(grp_sounds)

        btn_music  = QtWidgets.QPushButton("🎵 Music")
        btn_robot_eng = QtWidgets.QPushButton("🤖 English Robot")
        btn_robot_jp = QtWidgets.QPushButton("🤖 Japanese Robot")
        for b in (btn_music, btn_robot_eng , btn_robot_jp):
            b.setMinimumHeight(42)

        self.chk_with_visual = QtWidgets.QCheckBox("Play with visuals")
        self.chk_with_visual.setChecked(False)
        # r.addWidget(self.chk_with_visual)

        lay_sd.addWidget(btn_music)
        lay_sd.addWidget(btn_robot_eng)
        lay_sd.addWidget(btn_robot_jp)
        lay_sd.addWidget(self.chk_with_visual)

        r.addWidget(grp_arrows)
        r.addWidget(hline())
        r.addWidget(grp_text)
        r.addWidget(hline())
        r.addWidget(grp_hl)
        r.addWidget(hline())
        r.addWidget(grp_sounds)
        r.addStretch(1)

        # Wire buttons
        btn_left.clicked.connect(lambda: self._play("LEFT"))
        btn_fwd.clicked.connect(lambda: self._play("FORWARD"))
        btn_right.clicked.connect(lambda: self._play("RIGHT"))
        btn_text_eng.clicked.connect(lambda: self._show_img("TEXT_ENG"))
        btn_text_jp.clicked.connect(lambda: self._show_img("TEXT_JP"))
        btn_block_field.clicked.connect(lambda: self._show_img("BLOCK_FIELD"))
        # btn_block_field_eng.clicked.connect(lambda: self._show_img("BLOCK_FIELD_ENG"))
        # btn_block_field_jp.clicked.connect(lambda: self._show_img("BLOCK_FIELD_JP"))
        # btn_stop_field.clicked.connect(lambda: self._show_img("STOP_FIELD"))
        btn_music.clicked.connect(lambda: self._audio_only("BELLABOT_MUSIC"))
        btn_robot_eng.clicked.connect(lambda: self._audio_only("ROBOT_ENG"))
        btn_robot_jp.clicked.connect(lambda: self._audio_only("ROBOT_JP"))

        # Shortcuts
        QtGui.QShortcut(QtGui.QKeySequence("Left"), self, activated=lambda: self._play("LEFT"))
        QtGui.QShortcut(QtGui.QKeySequence("Up"), self, activated=lambda: self._play("FORWARD"))
        QtGui.QShortcut(QtGui.QKeySequence("Right"), self, activated=lambda: self._play("RIGHT"))
        QtGui.QShortcut(QtGui.QKeySequence("1"), self, activated=lambda: self._show_img("TEXT_ENG"))
        QtGui.QShortcut(QtGui.QKeySequence("2"), self, activated=lambda: self._show_img("TEXT_JP"))
        QtGui.QShortcut(QtGui.QKeySequence("3"), self, activated=lambda: self._show_img("BLOCK_FIELD"))
        # QtGui.QShortcut(QtGui.QKeySequence("4"), self, activated=lambda: self._show_img("BLOCK_FIELD_ENG"))
        # QtGui.QShortcut(QtGui.QKeySequence("5"), self, activated=lambda: self._show_img("BLOCK_FIELD_JP"))
        # QtGui.QShortcut(QtGui.QKeySequence("6"), self, activated=lambda: self._show_img("STOP_FIELD"))
        QtGui.QShortcut(QtGui.QKeySequence("4"), self, activated=lambda: self._audio_only("BELLABOT_MUSIC"))
        QtGui.QShortcut(QtGui.QKeySequence("5"), self, activated=lambda: self._audio_only("ROBOT_ENG"))
        QtGui.QShortcut(QtGui.QKeySequence("6"), self, activated=lambda: self._audio_only("ROBOT_JP"))
        QtGui.QShortcut(QtGui.QKeySequence("Space"), self, activated=lambda: self._show_img("BLANK"))
        QtGui.QShortcut(QtGui.QKeySequence("0"), self, activated=lambda: self.enable_mix_modalities())
        QtGui.QShortcut(QtGui.QKeySequence("Esc"), self, activated=self.close)

        splitter.addWidget(left)
        splitter.addWidget(right)
        splitter.setSizes([640, 340])

        self._log(f"Controller ready. Use ← ↑ → or [0 - 6] or Space for Black Screen. ESC to exit. (single_screen = {self.single_screen})")

    def _log(self, msg: str):
        self.log.append(msg)
        sb = self.log.verticalScrollBar()
        sb.setValue(sb.maximum())

    def _play(self, key: str):
        path = self.media_path.get(key)
        if not path or not os.path.exists(path):
            self._log(f"❌ Missing file for {key}: {ap(path or '(unset)')}")
            QtWidgets.QMessageBox.critical(self, "File not found", ap(path or "(unset)"))
            return

        ext = os.path.splitext(path)[1].lower()
        combine = self.chk_with_visual.isChecked()

        # IMAGE
        if ext in (".png", ".jpg", ".jpeg"):
            # if combining and audio is playing, don't stop it
            self._log(f"🖼️ Image {key}")
            self.proj.show_image(path, stop_audio = not combine)
            return

        # AUDIO
        if ext in (".mp3", ".wav", ".m4a"):
            self._log(f"🔊 Audio {key}")
            if self.single_screen and not combine:
                self._show_blank_if_available()
                self.proj.play_audio_only(path, hide_window = False, pause_video = True)
            else:
                hide = not combine or not self.proj.is_image_visible()
                self.proj.play_audio_only(path, hide_window = hide, pause_video = not combine)
            return
        
        self._log(f"▶️ Switching to {key}: {os.path.basename(path)}")
        self.proj.play_loop(path, stop_audio = not combine)

    def _show_img(self, key: str):
        path = self.media_path.get(key)
        if not path or not os.path.exists(path):
            self._log(f"❌ Missing file for {key}: {ap(path or '(unset)')}")
            QtWidgets.QMessageBox.critical(self, "File not found", ap(path or "(unset)"))
            return
        
        combine = self.chk_with_visual.isChecked()
        self._log(f"🖼️ Switching to {key}: {os.path.basename(path)} (combine = {combine})")

        self.proj.show_image(path, stop_audio = not combine)

    def _audio_only(self, key: str):
        path = self.media_path.get(key)
        if not path or not os.path.exists(path):
            self._log(f"❌ Missing file for {key}: {ap(path or '(unset)')}")
            QtWidgets.QMessageBox.critical(self, "File not found", ap(path or "(unset)"))
            return
        
        combine = self.chk_with_visual.isChecked()
        self._log(f"🔊 Switching to {key}: {os.path.basename(path)} (combine = {combine})")

        if self.single_screen and not combine:
            self._show_blank_if_available()
            self.proj.play_audio_only(path, hide_window = False, pause_video = True)
        else:
            self.proj.play_audio_only(path, hide_window = not combine, pause_video = not combine)

    def _show_blank_if_available(self):
        blank = self.media_path.get("BLANK")

        if blank and os.path.exists(blank):
            self.proj.show_image(blank, stop_audio=True)
            self._log("🖼️ Showing BLANK background.")
            return True
        else:
            self._log("⚠️ BLANK image not found—keeping current visual.")
            return False

    
    def enable_mix_modalities(self):
        self.chk_with_visual.setChecked(False) if self.chk_with_visual.isChecked() else self.chk_with_visual.setChecked(True)

    def closeEvent(self, e):
        try: self.proj.close()
        except: pass
        e.accept()