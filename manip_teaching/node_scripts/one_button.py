import time


class Button(object):

    def __init__(self, debounce_ms=50, click_ms=200, press_ms=800, long_press_interval_ms=1000):
        self._debounce_ms = debounce_ms
        self._click_ms = click_ms
        self._press_ms = press_ms
        self._long_press_interval_ms = long_press_interval_ms

        self._state = 'IDLE'
        self._last_time = 0
        self._click_count = 0
        self._press_start_time = 0
        self._is_long_press = False

        self._click_callback = None
        self._double_click_callback = None
        self._triple_click_callback = None
        self._long_press_callback = None
        self._long_press_stop_callback = None

    def set_click_callback(self, callback):
        self._click_callback = callback

    def set_double_click_callback(self, callback):
        self._double_click_callback = callback

    def set_triple_click_callback(self, callback):
        self._triple_click_callback = callback

    def set_long_press_callback(self, callback):
        self._long_press_callback = callback

    def set_long_press_stop_callback(self, callback):
        self._long_press_stop_callback = callback

    def tick(self, pressed):
        now = time.time() * 1000  # Current time in milliseconds

        print(self._state, pressed)
        if self._state == 'IDLE':
            if pressed:
                self._state = 'DEBOUNCE_PRESS'
                self._last_time = now
                self._press_start_time = now
                self._is_long_press = False
        elif self._state == 'DEBOUNCE_PRESS':
            if not pressed:
                self._state = 'IDLE'
            elif now - self._last_time >= self._debounce_ms:
                self._state = 'PRESS'
                self._click_count += 1
        elif self._state == 'PRESS':
            if not pressed:
                self._state = 'DEBOUNCE_RELEASE'
                self._last_time = now
            elif now - self._press_start_time >= self._press_ms:
                if not self._is_long_press and self._long_press_callback:
                    self._long_press_callback()
                self._is_long_press = True
        elif self._state == 'DEBOUNCE_RELEASE':
            # print(now - self._last_time, self._debounce_ms)
            if pressed:
                self._state = 'PRESS'
            elif now - self._last_time >= self._debounce_ms:
                if self._is_long_press:
                    if self._long_press_stop_callback:
                        self._long_press_stop_callback()
                else:
                    print(now - self._press_start_time, self._click_ms)
                    print(self._click_count)
                    if now - self._press_start_time >= self._click_ms:
                        if self._click_count == 1 and self._click_callback:
                            self._click_callback()
                        elif self._click_count == 2 and self._double_click_callback:
                            self._double_click_callback()
                        elif self._click_count == 3 and self._triple_click_callback:
                            self._triple_click_callback()
                        self._click_count = 0
                self._state = 'IDLE'

# Example usage
def on_click():
    print("Single click detected")

def on_double_click():
    print("Double click detected")

def on_triple_click():
    print("Triple click detected")

def on_long_press():
    print("Long press detected")

def on_long_press_stop():
    print("Long press stopped")

if __name__ == '__main__':
    button = Button()
    button.set_click_callback(on_click)
    button.set_double_click_callback(on_double_click)
    button.set_triple_click_callback(on_triple_click)
    button.set_long_press_callback(on_long_press)
    button.set_long_press_stop_callback(on_long_press_stop)

    # Simulating button press sequences
    print("Simulating button press")
    button.tick(True)  # Press
    time.sleep(0.2)
    button.tick(True)  # Press
    time.sleep(0.2)
    button.tick(False)  # Release
    time.sleep(0.05)
    button.tick(False)  # Release
    time.sleep(0.05)

    print("Simulating button press")
    button.tick(True)  # Press
    time.sleep(0.1)
    button.tick(True)  # Press
    time.sleep(0.1)
    button.tick(False)  # Release
    time.sleep(0.05)
    button.tick(False)  # Release
    time.sleep(0.05)

    button.tick(True)  # Press
    time.sleep(0.2)
    button.tick(True)  # Press
    time.sleep(0.2)
    button.tick(False)  # Release
    time.sleep(0.05)
    button.tick(False)  # Release
    time.sleep(0.05)

    button.tick(True)  # Press
    time.sleep(1.0)
    button.tick(True)  # Press
    time.sleep(1.0)
    button.tick(True)  # Press
    time.sleep(1.0)
    button.tick(False)  # Release
    time.sleep(0.05)
    button.tick(False)  # Release
    time.sleep(0.05)

    # button.tick(False)  # Release
    # time.sleep(0.05)
    # button.tick(False)  # Confirm Release
    # time.sleep(0.05)
    # button.tick(True)  # Press
    # time.sleep(0.05)
    # button.tick(True)  # Confirm press
    # time.sleep(0.05)
    # button.tick(False)  # Release
    # time.sleep(0.05)
    # button.tick(False)  # Confirm Release
    # time.sleep(0.05)

    # button.tick(True)  # Press again
    # time.sleep(0.1)
    # button.tick(True)  # Confirm press
    # time.sleep(0.1)
    # button.tick(False)  # Release
    # time.sleep(0.1)
    # button.tick(False)  # Confirm Release
    # time.sleep(0.1)
    # button.tick(True)  # Press again
    # time.sleep(0.1)
    # button.tick(True)  # Confirm press
    # time.sleep(0.1)
    # button.tick(False)  # Release
    # time.sleep(0.1)
    # button.tick(False)  # Confirm Release
    # time.sleep(1)

    # # Checking for long press
    # print("Simulating long press")
    # button.tick(True)  # Press
    # time.sleep(0.9)
    # button.tick(True)  # Confirm long press
    # time.sleep(0.2)
    # button.tick(False)  # Release
    # time.sleep(0.1)
