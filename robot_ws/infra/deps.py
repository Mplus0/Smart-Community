"""Shared helpers for route modules."""

from __future__ import annotations

from typing import Optional

from fastapi import Header, HTTPException, status

from . import config


def check_writable(x_infra_token: Optional[str] = Header(default=None)) -> None:
    """Dependency enforcing read-only mode and the optional auth token.

    Mutating endpoints declare ``Depends(check_writable)`` which:

    * Returns ``409 Conflict`` when :data:`config.READONLY` is on.
    * Returns ``401 Unauthorized`` when :data:`config.AUTH_TOKEN` is set
      and the caller did not forward a matching ``X-Infra-Token``.

    The read-only / no-token combination used in-vehicle today passes
    through silently (per WORKBENCH.md §6).
    """
    if config.READONLY:
        raise HTTPException(status_code=status.HTTP_409_CONFLICT, detail="infra is read-only")
    if config.AUTH_TOKEN:
        if not x_infra_token or x_infra_token != config.AUTH_TOKEN:
            raise HTTPException(status_code=status.HTTP_401_UNAUTHORIZED, detail="bad token")
