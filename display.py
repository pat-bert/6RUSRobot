from typing import List

from RPLCD.i2c import CharLCD


class LCD(CharLCD):
    AXIS = 'XYZABC'
    POST_DECIMAL_PLACE = 1
    PRE_DECIMAL_PLACE = 3
    # Sign + Decimal Point + Digits before + Digits after
    WIDTH = 1 + 1 * (PRE_DECIMAL_PLACE > 0) + PRE_DECIMAL_PLACE + POST_DECIMAL_PLACE

    CONNECTED_CHAR = 0
    DISCONNECTED_CHAR = 1

    CONNECTED_SYMBOL = (
        0b01110,
        0b10001,
        0b10001,
        0b10001,
        0b11111,
        0b11011,
        0b11011,
        0b11111,
    )

    DISCONNECTED_SYMBOL = (
        0b01110,
        0b10000,
        0b10000,
        0b10000,
        0b11111,
        0b11011,
        0b11011,
        0b11111,
    )

    def __init__(self):
        # Adress and port expander type are fixed
        # Hide the specific implementation used
        super().__init__('PCF8574', 0x26)
        self.create_char(self.CONNECTED_CHAR, self.CONNECTED_SYMBOL)
        self.create_char(self.DISCONNECTED_CHAR, self.DISCONNECTED_SYMBOL)

    def print_pose(self, pose: List[float]):
        """
        Prints a pose on the lower three rows of the display
        """
        rows = 3 * ['']

        for i, (ax, val) in enumerate(zip(self.AXIS, pose)):
            space = ' ' if i // 3 == 0 else ''
            rows[i % 3] += f'{ax}{val:+0{self.WIDTH}.{self.POST_DECIMAL_PLACE}f}{space}'

        display_str = '\r\n'.join(rows)

        # Set cursor to start of second row and write positions
        self.cursor_pos = (1, 0)
        self.write_string(display_str)

    def print_connection(self, is_connected: bool):
        # Move cursor to upper right corner
        self.cursor_pos = (0, 19)

        if is_connected:
            self.write_string(chr(self.CONNECTED_CHAR))
        else:
            self.write_string(chr(self.DISCONNECTED_CHAR))

    def print_status(self, status: str):
        self.home()
        self.write_string(status[:19])
