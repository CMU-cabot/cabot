import os
from PyQt6.QtCore import QUrl
from PyQt6.QtGui import QPixmap
from PyQt6 import QtGui, QtCore, QtWidgets
from PyQt6.QtMultimediaWidgets import QVideoWidget
from PyQt6.QtMultimedia import QMediaPlayer, QAudioOutput

def ap(p: str) -> str:
    return os.path.abspath(p) if p else p

class ProjectorWindow(QtWidgets.QWidget):
    ready = QtCore.pyqtSignal()

    def __init__(self, screen_index: int):
        super().__init__()
        # self.setWindowTitle("Projector")
        self.setCursor(QtCore.Qt.CursorShape.BlankCursor)
        self.setWindowFlag(QtCore.Qt.WindowType.FramelessWindowHint, True)

        # Choose screen (fallback to 0 if mirroring)
        screens = QtWidgets.QApplication.screens()
        use_idx = screen_index if len(screens) > 1 else 0
        self._target_screen = screens[use_idx]
        self.setGeometry(self._target_screen.geometry())

        self.video = QVideoWidget(self)
        self.image_label = QtWidgets.QLabel(self)
        self.image_label.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)
        # self.image_label.hide()

        stack = QtWidgets.QStackedLayout(self)
        stack.setContentsMargins(0, 0, 0, 0)
        stack.addWidget(self.video)
        stack.addWidget(self.image_label)
        self.stack = stack

        # Show first, then attach to target screen and go fullscreen (fixes windowHandle None)
        self.show()
        def _attach():
            wh = self.windowHandle()
            if wh is not None:
                wh.setScreen(self._target_screen)
            self.showFullScreen()
            self.ready.emit()
        QtCore.QTimer.singleShot(0, _attach)

        # Player
        self.player = QMediaPlayer(self)
        self.player.setVideoOutput(self.video)
        self.player.mediaStatusChanged.connect(self._status_changed)

        self.audio_only = QMediaPlayer(self)
        self.audio_out = QAudioOutput(self)
        self.audio_only.setAudioOutput(self.audio_out)
        self.audio_only.mediaStatusChanged.connect(self._on_audio_status)

        self._last_video = None

        # ESC closes projector
        QtGui.QShortcut(QtGui.QKeySequence("Esc"), self, activated = self.close)

    def play_loop(self, path: str, stop_audio: bool = True):
        p = ap(path)
        if not p or not os.path.exists(p):
            print("❌ File not found: ", p)
            return
        
        self._last_video = p

        if not self.isVisible():
            self.showFullScreen()
        
        if stop_audio and self.is_audio_playing():
            self.audio_only.stop()
            self.audio_only.setPosition(0)
        
        self.image_label.hide()
        self.video.show()

        self.player.setSource(QUrl.fromLocalFile(p))
        self.player.play()
        print(f"▶️ Playing (loop): {p} | stop_audio = {stop_audio}")

    def _status_changed(self, status):
        if status == QMediaPlayer.MediaStatus.EndOfMedia:
            self.player.setPosition(0)
            self.player.play()

    def show_image(self, path: str, stop_audio: bool = True):
        p = ap(path)
        if not p or not os.path.exists(p):
            print("❌ Image not found:", p)
            return
        
        if not self.isVisible():
            self.showFullScreen()

        if stop_audio and self.is_audio_playing():
            self.audio_only.stop()
            self.audio_only.setPosition(0)

        '''if self.audio_only.playbackState() == QMediaPlayer.PlaybackState.PlayingState:
            self.audio_only.stop()
            self.audio_only.setPosition(0)'''

        if self.player.playbackState() == QMediaPlayer.PlaybackState.PlayingState:
            self.player.pause()

        pix = QPixmap(p)
        if pix.isNull():
            print("⚠️ Failed to load image:", p)
            return
        
        self.image_label.setGeometry(self.rect())
        scaled = pix.scaled(self.size(), QtCore.Qt.AspectRatioMode.KeepAspectRatio, QtCore.Qt.TransformationMode.SmoothTransformation)

        self.image_label.setPixmap(scaled)
        self.video.hide()  
        self.image_label.show()
        print("🖼️ Showing image:", p)

    '''def resizeEvent(self, e: QtGui.QResizeEvent) -> None:
        # keep currently shown image fitting the window
        if self.image_label.isVisible() and (pm := self.image_label.pixmap()):
            self.image_label.setPixmap(
                pm.scaled(self.size(),
                        QtCore.Qt.AspectRatioMode.KeepAspectRatio,
                        QtCore.Qt.TransformationMode.SmoothTransformation)
            )
        return super().resizeEvent(e)'''

    def play_audio_only(self, audio_path: str, volume: float = 1.0, hide_window: bool = True, pause_video: bool = True):
        p = ap(audio_path)
        if not p or not os.path.exists(p):
            print("❌ audio not found:", p)
            return
        
        # self.hide()
        if hide_window:
            self.hide()
        else:
            if not self.isVisible():
                self.showFullScreen()

        if pause_video and self.player.playbackState() == QMediaPlayer.PlaybackState.PlayingState:
            self.player.pause()

        self.audio_out.setVolume(max(0.0, min(1.0, volume)))
        self.audio_only.stop()
        self.audio_only.setSource(QUrl.fromLocalFile(p))
        self.audio_only.setPosition(0)
        self.audio_only.play()
        print(f"🔊 audio-only: {p} | hide_window = {hide_window} | pause_video = {pause_video}")
    
    def _on_audio_status(self, status):
        if status == QMediaPlayer.MediaStatus.EndOfMedia:
            pass

    def is_audio_playing(self) -> bool:
        return self.audio_only.playbackState() == QMediaPlayer.PlaybackState.PlayingState

    def is_image_visible(self) -> bool:
        return self.image_label.isVisible()

    def closeEvent(self, e):
        try: self.player.stop()
        except: pass
        e.accept()