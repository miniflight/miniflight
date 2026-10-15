"""Record external calls; replay their order, arguments, results and clock."""

import builtins
import json
import time


class Calls:
    def __init__(self, record=None, rows=()):
        self.record, self.rows = record, iter(rows)
        self.count, self.active, self.mismatch = 0, False, None

    def __call__(self, name, arguments, function=None, encode=lambda value: value):
        if self.active:  # One external call owns any nested transport cleanup.
            return function()
        arguments = json.loads(json.dumps(arguments))
        if self.record is None:
            row = next(self.rows, None)
            self.count += 1
            if row is None or (row["name"], row["arguments"]) != (name, arguments):
                expected = None if row is None else (row["name"], row["arguments"])
                self.mismatch = self.mismatch or f"call {self.count}: expected {expected}, got {name} {arguments}"
            if self.mismatch:
                raise AssertionError(self.mismatch)
            if "error" in row:
                error = row["error"]
                kind = getattr(builtins, error["type"], None)
                if not isinstance(kind, type) or not issubclass(kind, BaseException):
                    raise ValueError(f"unsupported recorded exception: {error['type']}")
                raise kind(error["message"])
            return row["result"]
        self.active = True
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
            self.active = False

    def monotonic(self):
        return self("clock", {}, time.monotonic)

    def sleep(self, seconds):
        return self("sleep", dict(seconds=seconds), lambda: time.sleep(seconds))

    def finish(self):
        if self.mismatch:
            raise AssertionError(self.mismatch)
        if next(self.rows, None) is not None:
            raise AssertionError(f"replay stopped after {self.count} calls with unread input")
