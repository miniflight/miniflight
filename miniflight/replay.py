"""Record external calls; replay their order, arguments, results and clock."""

import builtins
from enum import Enum, auto
import json
import time


class CallPhase(Enum):
    RECORD_IDLE = auto()
    RECORD_ACTIVE = auto()
    REPLAYING = auto()
    FAILED = auto()
    EXHAUSTED = auto()


class Calls:
    def __init__(self, record=None, rows=()):
        self.record, self.rows = record, iter(rows)
        self.count, self.mismatch = 0, None
        self.phase = CallPhase.REPLAYING if record is None else CallPhase.RECORD_IDLE

    def reject(self, reason):
        if self.phase is not CallPhase.FAILED:
            self.mismatch = reason
            self.phase = CallPhase.FAILED
        raise AssertionError(self.mismatch)

    def __call__(self, name, arguments, function=None, encode=lambda value: value):
        if self.phase is CallPhase.RECORD_ACTIVE:  # One external call owns nested cleanup.
            return function()
        if self.phase is CallPhase.FAILED:
            raise AssertionError(self.mismatch)
        if self.phase is CallPhase.EXHAUSTED:
            raise RuntimeError("call replay is finished")
        arguments = json.loads(json.dumps(arguments))
        if self.phase is CallPhase.REPLAYING:
            row = next(self.rows, None)
            self.count += 1
            if row is None or (row["name"], row["arguments"]) != (name, arguments):
                expected = None if row is None else (row["name"], row["arguments"])
                self.reject(f"call {self.count}: expected {expected}, got {name} {arguments}")
            if "error" in row:
                error = row["error"]
                kind = getattr(builtins, error["type"], None)
                if not isinstance(kind, type) or not issubclass(kind, BaseException):
                    raise ValueError(f"unsupported recorded exception: {error['type']}")
                raise kind(error["message"])
            return row["result"]
        self.phase = CallPhase.RECORD_ACTIVE
        try:
            result = function()
        except BaseException as error:
            self.record(event="call", name=name, arguments=arguments,
                        error=dict(type=type(error).__name__, message=str(error)))
            raise
        else:
            self.record(event="call", name=name, arguments=arguments, result=encode(result))
            return result
        finally:
            self.phase = CallPhase.RECORD_IDLE

    def monotonic(self):
        return self("clock", {}, time.monotonic)

    def sleep(self, seconds):
        return self("sleep", dict(seconds=seconds), lambda: time.sleep(seconds))

    def finish(self):
        if self.phase is CallPhase.FAILED:
            raise AssertionError(self.mismatch)
        if next(self.rows, None) is not None:
            self.reject(f"replay stopped after {self.count} calls with unread input")
        if self.phase is CallPhase.REPLAYING:
            self.phase = CallPhase.EXHAUSTED
