import time
import signal

from PyQt6 import QtCore, QtGui, QtWidgets

from adapters import RosServoSource
from config import DisplayConfig
from model.arrow_state import ArrowState
from render.arrow_widget import ArrowWidget

class DisplayRuntime:
    def __init__(self, cfg: DisplayConfig):
        self._cfg = cfg
        if self._cfg.render_backend == "opengl":
            surface_format = QtGui.QSurfaceFormat()
            surface_format.setSwapBehavior(QtGui.QSurfaceFormat.SwapBehavior.DoubleBuffer)
            surface_format.setSwapInterval(1 if self._cfg.opengl_vsync else 0)
            QtGui.QSurfaceFormat.setDefaultFormat(surface_format)
        self._qt_app = QtWidgets.QApplication([])

        available_screens = self._qt_app.screens()
        screen_index = cfg.screen_index if 0 <= cfg.screen_index < len(available_screens) else 0
        self._screen = available_screens[screen_index]

        self._state = ArrowState()
        self._servo_source = None
        self._init_input_sources()
        self._widget = self._create_widget()
        self._attach_window()
        self._install_exit_controls()

        self._timer = QtCore.QTimer()
        self._timer.setTimerType(QtCore.Qt.TimerType.PreciseTimer)
        self._timer.setSingleShot(True)
        self._timer.timeout.connect(self._on_tick)
        self._frame_period_sec = 1.0 / max(1.0, float(self._cfg.fps))
        self._last_tick = time.perf_counter()
        self._next_tick = self._last_tick
        self._perf_window_start = self._last_tick
        self._tick_samples_ms = []
        self._late_tick_count = 0

    def _init_input_sources(self) -> None:
        if not self._cfg.use_servo_topic:
            return
        self._servo_source = RosServoSource(self._cfg.servo_topic)
        self._servo_source.start()

    def _create_widget(self):
        if self._cfg.render_backend != "opengl":
            return ArrowWidget(self._cfg, self._state)
        try:
            from render.arrow_gl_widget import ArrowOpenGLWidget
            return ArrowOpenGLWidget(self._cfg, self._state)
        except Exception as ex:
            print(f"[display_runtime] OpenGL backend unavailable, falling back to qt: {ex}")
            return ArrowWidget(self._cfg, self._state)

    def _attach_window(self):
        if self._cfg.show_fullscreen:
            # OpenGL fullscreen is more stable without aggressive WM bypass flags on some setups.
            if self._cfg.render_backend == "qt":
                self._widget.setWindowFlag(QtCore.Qt.WindowType.FramelessWindowHint, True)
                self._widget.setWindowFlag(QtCore.Qt.WindowType.WindowStaysOnTopHint, True)
                if hasattr(QtCore.Qt.WindowType, "X11BypassWindowManagerHint"):
                    self._widget.setWindowFlag(QtCore.Qt.WindowType.X11BypassWindowManagerHint, True)
            handle = self._widget.windowHandle()
            if handle is not None:
                handle.setScreen(self._screen)
            self._widget.setGeometry(self._screen.geometry())
            self._widget.showFullScreen()
            self._widget.raise_()
            self._widget.activateWindow()
            self._widget.setFocus(QtCore.Qt.FocusReason.ActiveWindowFocusReason)
        else:
            handle = self._widget.windowHandle()
            if handle is not None:
                handle.setScreen(self._screen)
            self._widget.setGeometry(self._screen.geometry())
            self._widget.show()
            self._widget.setFocus(QtCore.Qt.FocusReason.ActiveWindowFocusReason)

    def _install_exit_controls(self) -> None:
        self._exit_shortcuts = []
        for sequence in ("Esc", "Ctrl+Q"):
            shortcut = QtGui.QShortcut(QtGui.QKeySequence(sequence), self._widget)
            shortcut.setContext(QtCore.Qt.ShortcutContext.ApplicationShortcut)
            shortcut.activated.connect(self._request_quit)
            self._exit_shortcuts.append(shortcut)

        # Keep Python signal handling responsive while Qt event loop is running.
        self._signal_pump = QtCore.QTimer()
        self._signal_pump.setInterval(200)
        self._signal_pump.timeout.connect(lambda: None)
        self._signal_pump.start()

        self._prev_sigint_handler = None
        self._prev_sigterm_handler = None
        try:
            self._prev_sigint_handler = signal.getsignal(signal.SIGINT)
            self._prev_sigterm_handler = signal.getsignal(signal.SIGTERM)
            signal.signal(signal.SIGINT, self._on_process_signal)
            signal.signal(signal.SIGTERM, self._on_process_signal)
        except Exception:
            # Not fatal if we cannot install signal handlers in this runtime context.
            pass

    def _on_process_signal(self, signum, _frame) -> None:
        QtCore.QMetaObject.invokeMethod(self._qt_app, "quit", QtCore.Qt.ConnectionType.QueuedConnection)

    def _restore_signal_handlers(self) -> None:
        try:
            if self._prev_sigint_handler is not None:
                signal.signal(signal.SIGINT, self._prev_sigint_handler)
            if self._prev_sigterm_handler is not None:
                signal.signal(signal.SIGTERM, self._prev_sigterm_handler)
        except Exception:
            pass

    def _request_quit(self) -> None:
        self._qt_app.quit()

    def _on_tick(self):
        current_time_sec = time.perf_counter()
        frame_dt_sec = max(0.0, min(0.05, current_time_sec - self._last_tick))
        self._last_tick = current_time_sec
        self._update_heading_from_sources(current_time_sec)
        if self._cfg.enable_perf_log:
            frame_dt_ms = frame_dt_sec * 1000.0
            self._tick_samples_ms.append(frame_dt_ms)
            if frame_dt_ms > (self._frame_period_sec * 1000.0 * 1.5):
                self._late_tick_count += 1

        self._state.update(dt_sec = frame_dt_sec, smoothing_hz = self._cfg.heading_smoothing_hz,
            arrow_speed_px_s = self._cfg.arrow_speed_px_s, flow_speed_px_s = self._cfg.flow_speed_px_s)
        
        self._widget.update()
        self._maybe_log_perf(current_time_sec)
        self._next_tick += self._frame_period_sec
        if self._next_tick < current_time_sec:
            missed_frame_slots = int((current_time_sec - self._next_tick) / self._frame_period_sec) + 1
            self._next_tick += missed_frame_slots * self._frame_period_sec
        self._schedule_next_tick()

    def _update_heading_from_sources(self, now_sec: float) -> None:
        if self._servo_source is None:
            return
        latest = self._servo_source.get_latest()
        if latest is None:
            return
        servo_deg, stamp_sec = latest
        if (now_sec - stamp_sec) <= max(0.0, float(self._cfg.servo_fresh_sec)):
            self._state.arrow_point_towards = servo_deg

    def _maybe_log_perf(self, now_sec: float) -> None:
        if not self._cfg.enable_perf_log:
            return
        window_sec = now_sec - self._perf_window_start
        if window_sec < 1.0:
            return

        tick_count = len(self._tick_samples_ms)
        if tick_count > 0:
            sorted_ticks = sorted(self._tick_samples_ms)
            p95_index = min(tick_count - 1, max(0, int(tick_count * 0.95) - 1))
            tick_avg_ms = sum(sorted_ticks) / tick_count
            tick_p95_ms = sorted_ticks[p95_index]
            tick_max_ms = sorted_ticks[-1]
            fps_actual = tick_count / max(1e-6, window_sec)
        else:
            tick_avg_ms = 0.0
            tick_p95_ms = 0.0
            tick_max_ms = 0.0
            fps_actual = 0.0

        paint_stats = self._widget.consume_paint_stats() or {"avg_ms": 0.0, "p95_ms": 0.0, "max_ms": 0.0}
        print(
            "[perf] "
            f"fps={fps_actual:.1f} "
            f"tick_ms(avg/p95/max)={tick_avg_ms:.2f}/{tick_p95_ms:.2f}/{tick_max_ms:.2f} "
            f"late={self._late_tick_count} "
            f"paint_ms(avg/p95/max)={paint_stats['avg_ms']:.2f}/{paint_stats['p95_ms']:.2f}/{paint_stats['max_ms']:.2f}"
        )

        self._perf_window_start = now_sec
        self._tick_samples_ms = []
        self._late_tick_count = 0

    def _schedule_next_tick(self) -> None:
        current_time_sec = time.perf_counter()
        delay_sec = max(0.0, self._next_tick - current_time_sec)
        delay_ms = max(0, int(round(delay_sec * 1000.0)))
        self._timer.start(delay_ms)

    def run(self) -> int:
        self._last_tick = time.perf_counter()
        self._next_tick = self._last_tick
        self._schedule_next_tick()
        try:
            return self._qt_app.exec()
        finally:
            self._restore_signal_handlers()
            if self._servo_source is not None:
                self._servo_source.stop()
