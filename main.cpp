#include <iostream>
#include <ncurses.h>
#include <locale.h>
#include <unistd.h>
#include <cmath>

#include "Connection_information.h"
#include "Urg_driver.h"

#define LIDAR_MODEL "UTM-30LX-EW"

WINDOW *win;
WINDOW *info_win;

void ncurses_init() {
  /* -----< curses : START >----- */
  setlocale(LC_ALL, "");
  initscr();
  noecho();
  cbreak();
  keypad(stdscr, TRUE);
  timeout(0);
  curs_set(0);
  box(stdscr, ACS_VLINE, ACS_HLINE);
  if (has_colors()) start_color();
  init_pair(1, COLOR_WHITE,   COLOR_BLACK);
  init_pair(2, COLOR_YELLOW,  COLOR_BLACK);
  init_pair(3, COLOR_GREEN,   COLOR_BLACK);
  init_pair(4, COLOR_BLUE,    COLOR_BLACK);
  init_pair(5, COLOR_RED,     COLOR_BLACK);
  init_pair(6, COLOR_MAGENTA, COLOR_BLACK);
  init_pair(7, COLOR_CYAN,    COLOR_BLACK);

  /* -----< curses : END >----- */
}

void ncurses_close() {
  endwin();
}

int main(int argc, char *argv[]) {
  double MAX_RANGE = 35;
  double MIN_RANGE =  5;
  double start_angle = -100.0;
  double end_angle = +100.0;
  double step_angle = 0.25;

  qrk::Connection_information information(argc, argv);
  qrk::Urg_driver urg;
#if 0
  if (!urg.open(information.device_or_ip_name(),
                information.baudrate_or_port_number(),
                information.connection_type())) {
    std::cout << "Urg_driver::open(): " << information.device_or_ip_name() << ": " << urg.what() << std::endl;
    return 1;
  }
#endif
  urg.set_scanning_parameter(urg.deg2step(start_angle), urg.deg2step(end_angle), 0);

  ncurses_init();
  int width, height;
  getmaxyx(stdscr, height, width);
  double max_range = 10.0;  // [m]
  double csize_h = max_range / (height/2);
  double csize_w = max_range / (width/2);

  int info_win_width = 30;
  int info_win_height = 10;
  info_win = newwin(info_win_height, info_win_width, height/2 + 5, width/2 - info_win_width/2);
  bool DISPLAY_INFO = false;
  int time_count = 0;
  for (;;) {
#if 0
    urg.start_measurement(qrk::Urg_driver::Distance, qrk::Urg_driver::Infinity_times, 0);
    for (int i = 0; i < Capture_times; ++i) {
    std::vector<long> data;
      long time_stamp = 0;
      if (!urg.get_distance(data, &time_stamp)) {
        std::cout << "Urg_driver::get_distance(): " << urg.what() << std::endl;
        ncurses_close();
        return 1;
      }
    }
#endif

    for (int i = 1; i < height-1; i++) {
      for (int j = 1; j < width-1; j++) {
        double dist = std::hypot((j - width/2)*csize_w, (i - height/2)*csize_h);
        if (dist < 1.0) {
          attrset(COLOR_PAIR(2));
        } else if (dist < 2.0) {
          attrset(COLOR_PAIR(3));
        } else if (dist < 3.0) {
          attrset(COLOR_PAIR(4));
        } else if (dist < 4.0) {
          attrset(COLOR_PAIR(5));
        } else if (dist < 5.0) {
          attrset(COLOR_PAIR(6));
        } else {
          attrset(COLOR_PAIR(7));
        }
        mvprintw(i, j, "･");
      }
    }

    attrset(COLOR_PAIR(1));
    for (int i = 1; i < height-1; i++) {
      mvprintw(i, width/2, "│");
    }
    for (int i = 1; i < width-1; i++) {
      mvprintw(height/2, i, "─");
    }
    mvprintw(height/2, width/2, "┼");
    mvprintw(1,        width/2 + 1, "%1.0f",  max_range);
    mvprintw(height-2, width/2 - 3, "%1.0f", -max_range);
    mvprintw(height/2, width-3,     "%1.0f",  max_range);
    mvprintw(height/2,       1,     "%1.0f", -max_range);

    mvprintw(1, 1, "%d", time_count);
    refresh();

    time_count++;

    int key = getch();
    if ( key == 'q') {
      if (DISPLAY_INFO) {
        DISPLAY_INFO = false;
      } else {
        ncurses_close();
        break;
      }
    } else if (key == 'u') {
      if (max_range < MAX_RANGE) {
        max_range += 1.0;
      }
    } else if (key == 'd') {
      if (max_range > MIN_RANGE) {
        max_range -= 1.0;
      }
    } else if (key == 'I') {
      if (DISPLAY_INFO) DISPLAY_INFO = false;
      else DISPLAY_INFO = true;
    }

    if (DISPLAY_INFO) {
      box(info_win, ACS_VLINE, ACS_HLINE);
      mvwprintw(info_win, 1, 1, "LIDAR: %s", LIDAR_MODEL);
      mvwprintw(info_win, 2, 1, "START: %0.f[deg]", start_angle);
      mvwprintw(info_win, 3, 1, "  END: %0.f[deg]", end_angle);
      mvwprintw(info_win, 4, 1, " STEP: %0.f[deg]", step_angle);
      wrefresh(info_win);
    } else {
      ;
    }
    usleep(25000);
  }
  return 0;
}
