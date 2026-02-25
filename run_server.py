"""
run_server.py
=============
Launch the PLC Conveyor Sortation HMI server.

Usage:
    python run_server.py
    python run_server.py --port 8080
    python run_server.py --host 0.0.0.0 --port 8080

Opens http://localhost:8080 in your browser.
"""

import argparse
import uvicorn


def main():
    parser = argparse.ArgumentParser(description="PLC Conveyor Sortation Server")
    parser.add_argument("--host", default="127.0.0.1", help="Bind host (default: 127.0.0.1)")
    parser.add_argument("--port", type=int, default=8080, help="Bind port (default: 8080)")
    parser.add_argument("--reload", action="store_true", help="Enable auto-reload for development")
    args = parser.parse_args()

    print(f"Starting PLC Conveyor Sortation Server on http://{args.host}:{args.port}")
    print("Press Ctrl+C to stop.")

    uvicorn.run(
        "server.api:app",
        host=args.host,
        port=args.port,
        reload=args.reload,
        log_level="info",
    )


if __name__ == "__main__":
    main()
