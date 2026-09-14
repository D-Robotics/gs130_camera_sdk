"""Errors returned by libgs130."""

from ._enums import ErrorCode


class GS130Error(RuntimeError):
    """An error returned by a libgs130 function."""

    def __init__(self, code, func=None):
        self.code = int(code)
        self.func = func
        try:
            reason = ErrorCode(code).name
        except ValueError:
            reason = "error code %s" % code
        super().__init__("%s() -> %s" % (func, reason) if func else reason)


def check(code, func):
    if code != ErrorCode.OK:
        raise GS130Error(code, func)
