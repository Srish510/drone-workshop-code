import json
from typing import Optional


def encode_message(msg_type: str, data: dict) -> bytes: #Return JSON
    message = {"type": msg_type, "data": data}
    return (json.dumps(message) + "\n").encode("utf-8")


def decode_message(raw: bytes) -> Optional[dict]:
    try:
        text = raw.decode("utf-8").strip()
        if not text:
            return None
        return json.loads(text)
    except (json.JSONDecodeError, UnicodeDecodeError):
        return None