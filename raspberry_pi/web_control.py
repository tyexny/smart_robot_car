"""Small authenticated web UI for text and browser-provided speech-to-text."""

from __future__ import annotations

import queue
import logging
import secrets
import os
import re
import threading
from pathlib import Path

from task_commands import parse_text
from vlm_tasks import parse_visual_goal

PAGE = Path(__file__).with_name("control_page.html").read_text(encoding="utf-8")
UI_REVISION = "20261102-thermal-toggle-auto"


def load_web_token(path=None):
    """Persist a random access credential so a receiver restart keeps its URL."""
    path = Path(path) if path is not None else Path(__file__).with_name('.web_access_token')
    try:
        with path.open(encoding='ascii') as saved:
            token = saved.read(130).strip()
    except FileNotFoundError:
        token = secrets.token_urlsafe(24)
        try:
            fd = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
        except FileExistsError:
            return load_web_token(path)
        with os.fdopen(fd, 'w', encoding='ascii') as saved:
            saved.write(token + '\n')
    if not re.fullmatch(r'[A-Za-z0-9_-]{24,128}', token):
        raise ValueError('Invalid .web_access_token; remove it to generate a new access URL')
    return token

PHASE_TEXT = {
    "thermal": "已到达，正在检测温度", "arrival_pending": "等待目标到达确认",
    "idle": "等待任务", "understand": "正在理解任务", "search": "正在寻找目标",
    "turning": "正在转动寻找", "verify": "正在确认目标", "select": "正在确认目标",
    "track": "正在识别目标", "approach": "正在靠近目标", "reacquire": "正在重新定位",
    "found": "已找到目标", "failed": "任务已暂停", "stopped": "已停车",
    "manual": "手动控制中", "detect": "持续识别中", "following": "正在跟随目标",
}


def public_status(raw):
    """Present only user-facing state; raw diagnostics remain in terminal logs."""
    phase = str(raw.get("phase", "idle")).lower()
    state_text = PHASE_TEXT.get(phase, "等待任务")
    if phase == "approach" and not raw.get("lock_confirmed", raw.get("fpga_lock") not in (None, "未锁定")):
        state_text = "正在确认目标"
    terminal = phase in {"idle", "failed", "stopped", "understand", "reconnecting"}
    bbox = raw.get("bbox") if not terminal else None
    lock = raw.get("fpga_lock")
    locked = not terminal and lock not in (None, "", "未锁定")
    message = str(raw.get("message") or "")
    if phase == "reconnecting":
        state_text = "正在重新连接电脑"
        message = "车辆已停车，正在自动重连；恢复后将重新确认目标。"
    if message:
        if "PC request failed: HTTP 403" in message:
            message = "电脑认证失败（403），请同步两端共享令牌并重启任务服务。"
        elif "PC request failed: HTTP 401" in message:
            message = "电脑认证失败（401），请检查电脑任务服务的访问配置。"
        elif "connection failed or timed out" in message:
            message = "无法连接电脑任务服务，请检查电脑服务、地址、端口和防火墙。"
        elif "did not acknowledge target lock" in message or "未回报目标锁定" in message:
            message = "目标确认未完成，请检查小车状态后重新发送任务。"
        elif "PC connection lost" in message:
            message = "连接已中断，已停止任务；连接恢复后请重新发送。"
        elif "display mode mismatch" in message or "PC request failed: HTTP 400" in message:
            message = "两端任务模式或程序版本不一致，请匹配后重新发送任务。"
        elif "HDMI" in message:
            message = "已切换显示页面，请返回摄像头页面后重新发送任务。"
        elif "Camera" in message or "camera" in message:
            message = "画面连接已变化，请重新发送任务。"
        elif "restarted" in message or "PC task changed" in message:
            message = "服务已重新连接，请重新发送任务。"
        elif "Search exhausted" in message:
            message = "本轮搜索未找到目标，请调整位置后重试。"
        elif "Mock" in message:
            message = "当前为模拟演示，任务已暂停。"
        elif "FPGA" in message:
            message = "任务已停止，请确认小车状态后重新发送。"
        elif any(name in message.lower() for name in ("deepseek", "qwen", "api key", "api_key")):
            message = "视觉识别服务请求失败，请检查电脑端API配置和日志后重新发送任务。"
        elif message.isascii():
            message = "任务处理失败，请查看电脑端和树莓派任务日志中的具体原因后重试。"
    if phase == "manual":
        message = ""
    kind = "attention" if phase == "failed" else "success" if phase in {"found", "approach", "following"} else "idle" if phase in {"idle", "stopped"} else "active"
    return dict(phase=phase, state_text=state_text, state_kind=kind,
                target="全部识别目标" if raw.get("mode") == "yolo" else raw.get("target", ""),
                bbox=bbox, fpga_lock="已锁定" if locked else "未锁定", message=message,
                mock=bool(raw.get("mock", False)),
                thermal=raw.get("thermal", {"enabled": False}),
                communication_state=raw.get("communication_state", "正常"))

def create_web_app(command_queue: queue.Queue, status_provider, token: str,
                   allow_vlm: bool = True, allow_yolo: bool = True,
                   thermal_setter=None, thermal_frames=None, thermal_manual=None):
    try:
        from flask import Flask, abort, jsonify, request
    except ImportError as error:
        raise RuntimeError("--web-port requires Flask; install requirements-web.txt") from error
    app = Flask(__name__)
    app.config["MAX_CONTENT_LENGTH"] = 1024
    from thermal_preview import register_thermal_preview
    register_thermal_preview(app, token, thermal_frames)

    def authorized():
        return secrets.compare_digest(request.headers.get("X-Task-Token", ""), token)

    @app.get("/<access_token>")
    def page(access_token):
        if not secrets.compare_digest(access_token, token):
            abort(403)
        return PAGE, 200, {"Cache-Control": "no-store", "X-Robot-UI-Revision": UI_REVISION}

    @app.post("/api/command")
    def command():
        if not authorized():
            abort(403)
        body = request.get_json(silent=True)
        if not isinstance(body, dict):
            return jsonify(error="指令格式错误"), 400
        text = body.get("text")
        intent = parse_text(text) if isinstance(text, str) else None
        allowed_fixed = intent is not None and (intent[0] == "motion" or allow_yolo)
        if not isinstance(text, str) or len(text) > 200 or not (allowed_fixed or (allow_vlm and parse_visual_goal(text))):
            return jsonify(error="不支持的指令；请使用“寻找红色球”或“寻找红色球并移动到它附近”。纯 YOLO 模式不接受寻找任务。"), 400
        try:
            command_queue.put_nowait(text)
        except queue.Full:
            return jsonify(error="指令队列繁忙，请重试"), 503
        return jsonify(message="任务已提交"), 202

    @app.post("/api/thermal/manual")
    def thermal_manual_control():
        if not authorized():
            abort(403)
        body = request.get_json(silent=True)
        value = body.get('enabled') if isinstance(body, dict) else None
        if type(value) is not bool:
            return jsonify(error="热成像开关状态必须为开启或关闭"), 400
        if thermal_manual is None:
            return jsonify(error="红外监测未启用"), 503
        try:
            state = thermal_manual(value)
        except (ValueError, RuntimeError) as error:
            return jsonify(error=str(error)), 503
        return jsonify(thermal=state, message="热成像已开启，正在等待新画面" if value else "热成像已关闭，旧报警已清除")

    @app.post("/api/thermal/threshold")
    def thermal_threshold():
        if not authorized():
            abort(403)
        body = request.get_json(silent=True)
        value = body.get("threshold_c") if isinstance(body, dict) else None
        if type(value) is not int or value not in (40, 50, 60):
            return jsonify(error="温度阈值只能选择 40、50 或 60°C"), 400
        if thermal_setter is None:
            return jsonify(error="红外监测未启用"), 503
        thermal_setter(value)
        return jsonify(threshold_c=value, message=f"报警阈值已设为 {value}°C")

    @app.get("/api/status")
    def status():
        if not authorized():
            abort(403)
        return jsonify(public_status(status_provider()))

    return app


def start_web_control(command_queue: queue.Queue, status_provider, port: int,
                      token: str | None = None, allow_vlm: bool = True,
                      allow_yolo: bool = True, thermal_setter=None, thermal_frames=None, thermal_manual=None):
    token = token or load_web_token()
    logging.getLogger("fpga_yolo").info("Web UI revision=%s source=%s",
        UI_REVISION, Path(__file__).with_name("control_page.html").resolve())
    app = create_web_app(command_queue, status_provider, token, allow_vlm, allow_yolo,
                         thermal_setter=thermal_setter, thermal_frames=thermal_frames, thermal_manual=thermal_manual)
    server = threading.Thread(target=lambda: app.run(host="0.0.0.0", port=port,
                                                       threaded=True, use_reloader=False),
                              name="task-web", daemon=True)
    server.start()
    return token
