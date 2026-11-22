"""Screenshot-driven GUI automation steps for the CubeMX one-time ADC/DAC
configuration (knowledge #16: analog IPs can only be instantiated through
the GUI; this drives that single session).

Usage:
  python tools/gui.py shot <name>            # save a screenshot PNG
  python tools/gui.py click <x> <y>          # left click
  python tools/gui.py dclick <x> <y>         # double click
  python tools/gui.py rclick <x> <y>         # right click
  python tools/gui.py type <text>            # type text
  python tools/gui.py key <combo>            # e.g. ctrl+s, enter, tab
  python tools/gui.py windows                # list top-level window titles
"""
import sys
import time

import pyautogui

pyautogui.FAILSAFE = True  # slam mouse to corner aborts


def main() -> int:
    cmd = sys.argv[1]
    if cmd == "shot":
        name = sys.argv[2]
        img = pyautogui.screenshot()
        img.save(name)
        print("saved %s (%dx%d)" % (name, img.width, img.height))
    elif cmd == "click":
        x, y = int(sys.argv[2]), int(sys.argv[3])
        pyautogui.click(x, y)
        time.sleep(0.5)
        print("clicked %d,%d" % (x, y))
    elif cmd == "dclick":
        x, y = int(sys.argv[2]), int(sys.argv[3])
        pyautogui.doubleClick(x, y)
        time.sleep(0.5)
        print("dclicked %d,%d" % (x, y))
    elif cmd == "rclick":
        x, y = int(sys.argv[2]), int(sys.argv[3])
        pyautogui.rightClick(x, y)
        time.sleep(0.5)
        print("rclicked %d,%d" % (x, y))
    elif cmd == "type":
        pyautogui.typewrite(sys.argv[2], interval=0.02)
        time.sleep(0.3)
        print("typed")
    elif cmd == "key":
        keys = sys.argv[2].split("+")
        pyautogui.hotkey(*keys)
        time.sleep(0.5)
        print("key %s" % sys.argv[2])
    elif cmd == "windows":
        import pywinauto
        for w in pywinauto.findwindows.enum_windows():
            if w.title:
                print(repr(w.title))
    else:
        print("unknown command")
        return 2
    return 0


if __name__ == "__main__":
    sys.exit(main())
