from typing import List

from RPLCD.i2c import CharLCD


class LCD:
    AXIS = 'XYZABC'
    POST_DECIMAL_PLACE = 1
    PRE_DECIMAL_PLACE = 3
    # Sign + Decimal Point + Digits before + Digits after
    WIDTH = 1 + 1 * (PRE_DECIMAL_PLACE > 0) + PRE_DECIMAL_PLACE + POST_DECIMAL_PLACE

    SW_I2C_PORT = 11
    SW_I2C_SDA = 23
    SW_I2C_SCL = 24

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
        try:
            self._lcd = CharLCD('PCF8574', 0x26, port=self.SW_I2C_PORT)
        except Exception:
            self.connected = False
        else:
            self.connected = True
            self._lcd.create_char(self.CONNECTED_CHAR, self.CONNECTED_SYMBOL)
            self._lcd.create_char(self.DISCONNECTED_CHAR, self.DISCONNECTED_SYMBOL)

    def print_pose(self, pose: List[float]):
        """
        Prints a pose on the lower three rows of the display
        """
        if self.connected:
            rows = 3 * ['']

            for i, (ax, val) in enumerate(zip(self.AXIS, pose)):
                space = ' ' if i // 3 == 0 else ''
                rows[i % 3] += f'{ax}{val:+0{self.WIDTH}.{self.POST_DECIMAL_PLACE}f}{space}'

            display_str = '\r\n'.join(rows)

            # Set cursor to start of second row and write positions
            self._lcd.cursor_pos = (1, 0)
            self._lcd.write_string(display_str)

    def print_connection(self, is_connected: bool):
        if self.connected:
            # Move cursor to upper right corner
            self._lcd.cursor_pos = (0, 19)

            if is_connected:
                self._lcd.write_string(chr(self.CONNECTED_CHAR))
            else:
                self._lcd.write_string(chr(self.DISCONNECTED_CHAR))

    def print_status(self, status: str):
        if self.connected:
            self._lcd.home()
            self._lcd.write_string(status[:19])

    def __del__(self):
        if self.connected:
            self._lcd.close()
