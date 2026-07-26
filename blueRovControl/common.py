def clamp(value, lo, hi):
    return max(lo, min(hi, value))


def normalize_deg(angle):
    """角度を -180〜180 度に正規化"""
    return (angle + 180.0) % 360.0 - 180.0
