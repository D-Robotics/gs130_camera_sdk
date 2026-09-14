"""The exception the gs130 calls raise."""

from ._enums import ErrorCode


class GS130Error(RuntimeError):
    """A gs130 call failed.

    code is the ErrorCode the library returned, and func the C function that
    returned it, or None.
    """

    def __init__(self, code, func=None):
        self.code = int(code)
        self.func = func
        try:
            reason = ErrorCode(code).name
        except ValueError:
            reason = "error code %s" % (code,)
        super().__init__(
            "%s() -> %s" % (func, reason) if func else reason
        )


def check(error, func):
    """Raise GS130Error unless the library returned ErrorCode.OK."""
    if error != ErrorCode.OK:
        raise GS130Error(error, func)
