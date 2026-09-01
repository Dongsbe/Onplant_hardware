from __future__ import annotations

import argparse
import json
import os
from urllib import request


def send_remote_key(server: str, robot_id: str, key: str) -> dict:
    url = f"{server.rstrip('/')}/api/robots/{robot_id}/remote"
    payload = json.dumps({"key": key}).encode("utf-8")
    req = request.Request(
        url,
        data=payload,
        headers={"Content-Type": "application/json"},
        method="POST",
    )
    with request.urlopen(req, timeout=5) as response:
        return json.loads(response.read().decode("utf-8"))


def main() -> None:
    parser = argparse.ArgumentParser(description="Send OnPlant remote key to the server.")
    parser.add_argument("key", choices=["3", "4", "5"], help="3=state report, 4=camera on, 5=camera off")
    parser.add_argument("--server", default=os.getenv("ONPLANT_SERVER", "http://192.168.100.6:5050"))
    parser.add_argument("--robot-id", default=os.getenv("ONPLANT_ROBOT_ID", "raspbot-a"))
    args = parser.parse_args()

    result = send_remote_key(args.server, args.robot_id, args.key)
    print(json.dumps(result, ensure_ascii=False))


if __name__ == "__main__":
    main()
