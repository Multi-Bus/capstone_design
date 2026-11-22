"""FastAPI 서버: 가상 시뮬을 실시간으로 돌리고 WebSocket(/ws)으로 상태를 보내고 명령을 받는다."""
import asyncio
import contextlib
import json
from pathlib import Path

from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.staticfiles import StaticFiles

from .sim import SimSwarm

WEB = Path(__file__).resolve().parents[1] / "web"


def handle_command(sim, msg):
    """명령 하나를 적용한다. 문제가 있으면 오류 문자열, 없으면 None. 예외를 던지지 않는다."""
    try:
        m = json.loads(msg)
    except ValueError:
        return "JSON이 아닌 메시지"
    if not isinstance(m, dict):
        return "명령은 JSON 객체여야 한다"
    cmd = m.get("cmd")
    if cmd == "start":
        sim.start()
    elif cmd == "stop":
        sim.stop()
    elif cmd == "home":
        sim.home()
    elif cmd == "formation":
        try:
            sim.set_formation(m.get("name"))
        except ValueError as e:
            return str(e)
    elif cmd in ("goal", "obstacle"):
        x, y = m.get("x"), m.get("y")
        if not all(isinstance(v, (int, float)) and not isinstance(v, bool) for v in (x, y)):
            return "x, y는 숫자여야 한다"
        try:
            return sim.goto(x, y) if cmd == "goal" else sim.add_obstacle(x, y)
        except ValueError as e:
            return str(e)
    elif cmd == "clear_obstacles":
        sim.clear_obstacles()
    else:
        return f"모르는 명령: {cmd}"
    return None


def create_app(sim=None, sim_hz=50, send_hz=20):
    sim = sim or SimSwarm()
    clients = set()

    async def broadcast(state):
        for ws in list(clients):
            try:
                await ws.send_json(state)
            except Exception:  # 끊긴 소켓은 빼고 계속 돈다
                clients.discard(ws)

    async def loop():
        clock = asyncio.get_running_loop().time
        t0, last_send = clock(), 0.0
        while True:
            await asyncio.sleep(1 / sim_hz)
            now = clock() - t0
            due = int(now / sim.dt)
            for _ in range(min(due - sim.k, 50)):  # 실시간 따라잡기, 한 번에 최대 1초
                sim.step()
            if now - last_send >= 1 / send_hz:
                last_send = now
                await broadcast(sim.state())

    @contextlib.asynccontextmanager
    async def lifespan(_app):
        task = asyncio.create_task(loop())
        yield
        task.cancel()
        with contextlib.suppress(asyncio.CancelledError):
            await task

    app = FastAPI(lifespan=lifespan)

    @app.websocket("/ws")
    async def ws_endpoint(ws: WebSocket):
        await ws.accept()
        clients.add(ws)
        try:
            await ws.send_json(sim.state())
            while True:
                err = handle_command(sim, await ws.receive_text())
                if err:
                    await ws.send_json({"error": err})
        except WebSocketDisconnect:
            pass
        finally:
            clients.discard(ws)

    app.mount("/", StaticFiles(directory=WEB, html=True), name="web")
    return app
