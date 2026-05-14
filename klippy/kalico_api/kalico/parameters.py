import typing

Number = typing.TypeVar("Number", int, float)


class Above(typing.Generic[Number]):
    def __init__(self, above: Number):
        self._above = above
        self.description = {"above": above}

    def __call__(self, v):
        return self._above < v

    def __repr__(self):
        return f"{self._above} < v"


class Below(typing.Generic[Number]):
    def __init__(self, below: Number):
        self._below = below
        self.description = {"below": below}

    def __call__(self, v):
        return v < self._below

    def __repr__(self):
        return f"v < {self._below}"


class Minimum(typing.Generic[Number]):
    def __init__(self, minimum: Number):
        self._minimum = minimum
        self.description = {"minimum": minimum}

    def __call__(self, v):
        return self._minimum <= v

    def __repr__(self):
        return f"{self._minimum} <= v"


class Maximum(typing.Generic[Number]):
    def __init__(self, maximum: Number):
        self._maximum = maximum
        self.description = {"maximum": maximum}

    def __call__(self, v):
        return v <= self._maximum

    def __repr__(self):
        return f"v <= {self._maximum}"


class Range(typing.Generic[Number]):
    def __init__(self, lower: Number, upper: Number):
        self._lower = lower
        self._upper = upper
        self.description = {"minimum": lower, "maximum": upper}

    def __call__(self, v: Number) -> bool:
        return self._lower <= v <= self._upper

    def __repr__(self):
        return f"{self._lower} <= v <= {self._upper}"


class Between(typing.Generic[Number]):
    def __init__(self, above: Number, below: Number):
        self._above = above
        self._below = below
        self.description = {"above": above, "below": below}

    def __call__(self, v: Number) -> bool:
        return self._above < v < self._below

    def __repr__(self):
        return f"{self._above} < v < {self._below}"


class IntRange:
    def __class_getitem__(cls, range: tuple[int, int]):
        return typing.Annotated[int, Range(*range)]


class IntBetween:
    def __class_getitem__(cls, range: tuple[int, int]):
        return typing.Annotated[int, Between(*range)]


class FloatRange:
    def __class_getitem__(cls, range: tuple[int, int]):
        return typing.Annotated[float, Range(*range)]


class FloatBetween:
    def __class_getitem__(cls, range: tuple[int, int]):
        return typing.Annotated[float, Between(*range)]
