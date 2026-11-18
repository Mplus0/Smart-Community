"""Shared helpers for route modules: read-only mode and the optional token."""

from __future__ import annotations

import hmac
from typing import Optional

from fastapi import Header, HTTPException, WebSocket, status

from . import config


def _token_ok(value: Optional[str]) -> bool:
    return not config.AUTH_TOKEN or hmac.compare_digest(value or "", config.AUTH_TOKEN)


def check_writable(x_infra_token: Optional[str] = Header(default=None)) -> None:
    """Dependency for mutating HTTP endpoints.

    * ``409 Conflict`` when :data:`config.READONLY` is on.
    * ``401 Unauthorized`` when :data:`config.AUTH_TOKEN` is set and the
      request lacks a matching ``X-Infra-Token`` header.
    """
    if config.READONLY:
        raise HTTPException(status_code=status.HTTP_409_CONFLICT, detail="工作台为只读模式")
    if not _token_ok(x_infra_token):
        raise HTTPException(status_code=status.HTTP_401_UNAUTHORIZED, detail="Token 无效")


async def accept_ws(ws: WebSocket, writable: bool = False) -> bool:
    """Accept a WebSocket after checking the token (``?token=``).

    Browsers cannot set headers on WebSockets, so the token travels as a
    query parameter. ``writable`` sockets (teleop) are also refused in
    read-only mode. Returns ``False`` after closing a rejected socket.
    """
    await ws.accept()
    if not _token_ok(ws.query_params.get("token")):
        await ws.close(code=4401, reason="bad token")
        return False
    if writable and config.READONLY:
        await ws.close(code=4409, reason="read-only")
        return False
    return True
