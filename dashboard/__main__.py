"""실행: python -m dashboard [--port 8000]. 서버가 뜨면 브라우저를 연다."""
import argparse
import threading
import webbrowser

import uvicorn

from .server.app import create_app


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", type=int, default=8000)
    a = ap.parse_args()
    url = f"http://127.0.0.1:{a.port}/"
    threading.Timer(1.0, webbrowser.open, [url]).start()  # ponytail: 1초 뒤 열기, 서버가 늦으면 새로고침
    print(f"대시보드: {url}  (끄기: Ctrl+C)")
    uvicorn.run(create_app(), host="127.0.0.1", port=a.port, log_level="warning")


if __name__ == "__main__":
    main()
