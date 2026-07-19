"""Unit tests for DVL-75 ASCII message parsing."""

import unittest

from dvl75_driver.parser import Dvext, Dvl75ParseError, Dvpdl, parse_sentence


def sentence(body: str) -> bytes:
    """Build an NMEA sentence with the correct checksum.

    Args:
        body: NMEA sentence content without leading ``$`` or checksum.

    Returns:
        ASCII NMEA sentence including the leading ``$`` and checksum.
    """
    checksum = 0
    for character in body:
        checksum ^= ord(character)
    return f"${body}*{checksum:02X}".encode()


class TestParser(unittest.TestCase):
    """Verify DVL-75 message parsing and checksum validation."""

    def test_parse_dvext(self) -> None:
        """Parse bottom-lock, altitude, and beam values from DVEXT."""
        body = (
            "DVEXT,T,V,0000,1.0,2.0,3.0,4,0.1,5.0,0.2,0.3,35.0,139.0,0.05,"
            "1.0,0.0,0.0,0.0,6,7,8,9,T,F,T,T,0.4,0.5,0.6,0.7,8,9,10,11"
        )
        result = parse_sentence(sentence(body))

        self.assertIsInstance(result, Dvext)
        assert isinstance(result, Dvext)
        self.assertTrue(result.bottom_lock)
        self.assertEqual(result.data_skips, 4)
        self.assertEqual(result.beam_lock, (True, False, True, True))
        self.assertEqual(result.beam_range, (8.0, 9.0, 10.0, 11.0))

    def test_parse_dvext_with_trailing_comma(self) -> None:
        """Accept the DVL firmware variant with a comma before the checksum."""
        body = (
            "DVEXT,T,V,0000,1.0,2.0,3.0,4,0.1,5.0,0.2,0.3,35.0,139.0,0.05,"
            "1.0,0.0,0.0,0.0,6,7,8,9,T,F,T,T,0.4,0.5,0.6,0.7,8,9,10,11,"
        )
        result = parse_sentence(sentence(body))

        self.assertIsInstance(result, Dvext)

    def test_parse_dvpdl(self) -> None:
        """Parse incremental position and angle data from DVPDL."""
        result = parse_sentence(sentence("DVPDL,101234000,50000,0.1,-0.2,0.3,1,2,3,95"))

        self.assertIsInstance(result, Dvpdl)
        assert isinstance(result, Dvpdl)
        self.assertEqual(result.delta_time_us, 50000)
        self.assertEqual(result.position_delta, (1.0, 2.0, 3.0))
        self.assertEqual(result.confidence, 95)

    def test_reject_invalid_checksum(self) -> None:
        """Reject a sentence whose checksum does not match its payload."""
        with self.assertRaises(Dvl75ParseError):
            parse_sentence(b"$DVPDL,1,1,0,0,0,0,0,0,100*00")

    def test_reject_out_of_range_unsigned_value(self) -> None:
        """Reject a negative delta time that cannot fit the ROS uint32 field."""
        with self.assertRaises(Dvl75ParseError):
            parse_sentence(sentence("DVPDL,1,-1,0,0,0,0,0,0,100"))

    def test_ignore_unrelated_sentence(self) -> None:
        """Ignore supported NMEA traffic that is not a DVL measurement."""
        self.assertIsNone(parse_sentence(sentence("DVTXT,booted")))

    def test_ignore_non_nmea_information_text(self) -> None:
        """Ignore DVL startup text that does not use the NMEA framing."""
        self.assertIsNone(parse_sentence(b"DVL system boot complete"))
