import argparse
import getpass
import sys
import random
from os import system
from time import sleep
import curses

def getangulardata():
    option = [132, 100, 122, 321]
    return random.choice(option)

def getdistdata():
    option = [900, 867, 324, 134, 455]
    return random.choice(option)

def displaydata(angle: int, dist: int):
    try:
        stdscr = curses.initscr()
        curses.curs_set(0)  # Disable cursor visibility

        while True:
            mpu = getangulardata()
            tof = getdistdata()

            mpu_heading = "MPU"
            tof_heading = "TOF"
            stdscr.clear()
            stdscr.addstr(0, 0, "+-------+-------+")
            stdscr.addstr(1, 0, f"| {mpu_heading:^5} | {tof_heading:^5} |")
            stdscr.addstr(2, 0, "+-------+-------+")
            stdscr.addstr(3, 0, f"| {mpu:^5} | {tof:^5} |")
            stdscr.addstr(4, 0, "+-------+-------+")
            stdscr.refresh()
            sleep(1)

    except KeyboardInterrupt:
        pass

    finally:
        curses.endwin()  # Restore terminal settings

def main():
    parser = argparse.ArgumentParser(description="Display python version using CLI.")
    parser.add_argument("-mpu", "--Angle", type=int, help="1 to display, 0 to leave", default=1)
    parser.add_argument("-d", "--Distance", type=int, help="1 to display, 0 to leave", default=1)

    args = parser.parse_args()

    displaydata(args.Angle, args.Distance)

if __name__ == "__main__":
    try:
        main()
    except:
        pass
