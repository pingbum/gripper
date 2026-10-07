# -*- coding: utf-8 -*-
import math

def sine(t: float, amp: float, freq: float) -> float:
    """Sine: A * sin(2*pi*f*t)"""
    return amp * math.sin(2.0 * math.pi * freq * t)

def square(t: float, amp: float, freq: float) -> float:
    """Square (50% duty): A * sign(sin(2*pi*f*t))"""
    return amp * (1.0 if math.sin(2.0 * math.pi * freq * t) >= 0.0 else -1.0)

def triangle(t: float, amp: float, freq: float) -> float:
    """Triangle: A * (2/pi) * asin(sin(2*pi*f*t))  (-A..A)"""
    return amp * (2.0 / math.pi) * math.asin(math.sin(2.0 * math.pi * freq * t))
