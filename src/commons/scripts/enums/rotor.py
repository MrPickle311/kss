class CalibrationStatus:
    CalibrationInProgress = 0x00
    CalibrationCompletedSuccessfully = 0x01
    CalibrationFailed = 0x02


class MovementStatus:
    Stopped = 0x00
    MovingAnticlockwise = 0xFF
    MovingClockwise = 0x01


class RotationDirection:
    Clockwise = 0x00
    Anticlockwise = 0x01
