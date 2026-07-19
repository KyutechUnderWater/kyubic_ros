"""Parser for the ASCII UDP messages emitted by a Cerulean Sonar DVL-75."""

from __future__ import annotations

from dataclasses import dataclass


class Dvl75ParseError(ValueError):
    """Raised when a DVL-75 sentence is malformed or fails validation."""


@dataclass(frozen=True)
class Dvext:
    """Measurements carried by one ``$DVEXT`` sentence."""

    bottom_lock: bool
    data_skips: int
    velocity_up: float
    altitude: float
    velocity_north: float
    velocity_east: float
    beam_lock: tuple[bool, bool, bool, bool]
    beam_velocity: tuple[float, float, float, float]
    beam_range: tuple[float, float, float, float]


@dataclass(frozen=True)
class Dvpdl:
    """Incremental navigation data carried by one ``$DVPDL`` sentence."""

    device_time_us: int
    delta_time_us: int
    angle_delta: tuple[float, float, float]
    position_delta: tuple[float, float, float]
    confidence: int


def parse_sentence(packet: bytes, validate_checksum: bool = True) -> Dvext | Dvpdl | None:
    """Parse one DVL-75 UDP packet.

    Args:
        packet: One UDP payload received from the DVL.
        validate_checksum: Whether to reject sentences with an invalid NMEA checksum.

    Returns:
        A supported DVL-75 measurement, or ``None`` for an unrelated message.

    Raises:
        Dvl75ParseError: If a supported sentence is malformed or invalid.
    """
    try:
        sentence = packet.decode("ascii").strip()
    except UnicodeDecodeError as error:
        raise Dvl75ParseError("packet is not ASCII") from error

    # The DVL can emit boot and informational text alongside NMEA sentences.
    # It is not a measurement and should not be reported as a malformed packet.
    if not sentence.startswith("$"):
        return None

    body = _extract_body(sentence, validate_checksum)
    fields = body.split(",")
    if not fields:
        raise Dvl75ParseError("sentence has no message identifier")

    if fields[0] == "DVEXT":
        return _parse_dvext(fields)
    if fields[0] == "DVPDL":
        return _parse_dvpdl(fields)
    return None


def _extract_body(sentence: str, validate_checksum: bool) -> str:
    """Return a sentence body after checking its optional NMEA checksum.

    Args:
        sentence: ASCII NMEA sentence beginning with ``$``.
        validate_checksum: Whether a missing or mismatched checksum is invalid.

    Returns:
        Sentence content between the leading ``$`` and the checksum delimiter.

    Raises:
        Dvl75ParseError: If the checksum is required, malformed, or mismatched.
    """
    payload = sentence[1:]
    if "*" not in payload:
        if validate_checksum:
            raise Dvl75ParseError("sentence has no checksum")
        return payload

    body, checksum = payload.rsplit("*", maxsplit=1)
    if len(checksum) != 2:
        raise Dvl75ParseError("checksum must have two hexadecimal digits")
    try:
        expected = int(checksum, 16)
    except ValueError as error:
        raise Dvl75ParseError("checksum is not hexadecimal") from error

    actual = 0
    for character in body:
        actual ^= ord(character)
    if validate_checksum and actual != expected:
        raise Dvl75ParseError(f"checksum mismatch: expected {expected:02X}, got {actual:02X}")
    return body


def _parse_dvext(fields: list[str]) -> Dvext:
    """Parse a ``DVEXT`` field array.

    Args:
        fields: Comma-separated fields with ``DVEXT`` as the first element.

    Returns:
        Parsed extended DVL measurement.

    Raises:
        Dvl75ParseError: If a field count, boolean, or numeric value is invalid.
    """
    # DVL-75 firmware commonly emits a comma immediately before ``*hh``.
    # ``str.split`` represents it as one trailing empty field.
    if len(fields) == 36 and fields[-1] == "":
        fields = fields[:-1]
    if len(fields) != 35:
        raise Dvl75ParseError(f"DVEXT needs 35 fields, got {len(fields)}")

    beam_lock = tuple(_parse_bool(value) for value in fields[23:27])
    beam_velocity = tuple(_parse_float(value, "beam_velocity") for value in fields[27:31])
    beam_range = tuple(_parse_float(value, "beam_range") for value in fields[31:35])
    return Dvext(
        bottom_lock=_parse_bool(fields[1]),
        data_skips=_parse_int(fields[7], "data_skips"),
        velocity_up=_parse_float(fields[8], "velocity_up"),
        altitude=_parse_float(fields[9], "altitude"),
        velocity_north=_parse_float(fields[10], "velocity_north"),
        velocity_east=_parse_float(fields[11], "velocity_east"),
        beam_lock=beam_lock,  # type: ignore[arg-type]
        beam_velocity=beam_velocity,  # type: ignore[arg-type]
        beam_range=beam_range,  # type: ignore[arg-type]
    )


def _parse_dvpdl(fields: list[str]) -> Dvpdl:
    """Parse a ``DVPDL`` field array.

    Args:
        fields: Comma-separated fields with ``DVPDL`` as the first element.

    Returns:
        Parsed incremental DVL measurement.

    Raises:
        Dvl75ParseError: If a field count, range, or numeric value is invalid.
    """
    if len(fields) != 10:
        raise Dvl75ParseError(f"DVPDL needs 10 fields, got {len(fields)}")

    device_time_us = _parse_int(fields[1], "device_time_us")
    delta_time_us = _parse_int(fields[2], "delta_time_us")
    if not 0 <= device_time_us <= (2**64 - 1):
        raise Dvl75ParseError("device_time_us must fit uint64")
    if not 0 <= delta_time_us <= (2**32 - 1):
        raise Dvl75ParseError("delta_time_us must fit uint32")

    confidence = _parse_int(fields[9], "confidence")
    if not 0 <= confidence <= 100:
        raise Dvl75ParseError("confidence must be in the range 0..100")
    angle_delta = tuple(_parse_float(value, "angle_delta") for value in fields[3:6])
    position_delta = tuple(_parse_float(value, "position_delta") for value in fields[6:9])
    return Dvpdl(
        device_time_us=device_time_us,
        delta_time_us=delta_time_us,
        angle_delta=angle_delta,  # type: ignore[arg-type]
        position_delta=position_delta,  # type: ignore[arg-type]
        confidence=confidence,
    )


def _parse_bool(value: str) -> bool:
    """Parse a DVL boolean field represented by ``T`` or ``F``.

    Args:
        value: DVL boolean field.

    Returns:
        ``True`` for ``T`` and ``False`` for ``F``.

    Raises:
        Dvl75ParseError: If the value is neither ``T`` nor ``F``.
    """
    if value == "T":
        return True
    if value == "F":
        return False
    raise Dvl75ParseError(f"expected T or F, got {value!r}")


def _parse_float(value: str, name: str) -> float:
    """Parse one finite floating-point field.

    Args:
        value: Decimal DVL field.
        name: Field name included in a validation error.

    Returns:
        Finite floating-point value.

    Raises:
        Dvl75ParseError: If the value is non-numeric or non-finite.
    """
    try:
        result = float(value)
    except ValueError as error:
        raise Dvl75ParseError(f"{name} is not numeric") from error
    if result != result or result in (float("inf"), float("-inf")):
        raise Dvl75ParseError(f"{name} must be finite")
    return result


def _parse_int(value: str, name: str) -> int:
    """Parse one integer field.

    Args:
        value: Integer DVL field.
        name: Field name included in a validation error.

    Returns:
        Parsed integer value.

    Raises:
        Dvl75ParseError: If the value is not an integer.
    """
    try:
        return int(value)
    except ValueError as error:
        raise Dvl75ParseError(f"{name} is not an integer") from error
