#include <cmath>
#include <iostream>
#include <locale.h>
#include <ncurses.h>
#include <unistd.h>

#include "Connection_information.h"
#include "Map.h"
#include "Urg_driver.h"

#define LIDAR_MODEL "UTM-30LX-EW"

WINDOW *win;
WINDOW *info_win;
WINDOW *world_win;

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
  if (has_colors())
    start_color();
  init_pair(1, COLOR_WHITE, COLOR_BLACK);
  init_pair(2, COLOR_YELLOW, COLOR_BLACK);
  init_pair(3, COLOR_GREEN, COLOR_BLACK);
  init_pair(4, COLOR_BLUE, COLOR_BLACK);
  init_pair(5, COLOR_RED, COLOR_BLACK);
  init_pair(6, COLOR_MAGENTA, COLOR_BLACK);
  init_pair(7, COLOR_CYAN, COLOR_BLACK);
  /* -----< curses : END >----- */
}

void ncurses_close() { endwin(); }

int main(int argc, char *argv[]) {
  bool USE_LIDAR = false;

  MapClass map;
  map.read_map("log.map");
  map.show_config();

  double MAX_RANGE = 35; // [m] 表示する範囲の最大値
  double MIN_RANGE = 5;  // [m] 表示する範囲の最小値
  double start_angle = -100.0;
  double end_angle = +100.0;
  double step_angle = 0.25;

  qrk::Connection_information information(argc, argv);
  qrk::Urg_driver urg;
  if (!urg.open(information.device_or_ip_name(),
                information.baudrate_or_port_number(),
                information.connection_type())) {
    std::cout << "Urg_driver::open(): " << information.device_or_ip_name()
              << ": " << urg.what() << std::endl;
  } else {
    USE_LIDAR = true;
  }
  urg.set_scanning_parameter(urg.deg2step(start_angle), urg.deg2step(end_angle),
                             0);

  ncurses_init();
  int width, height; // 主ウィンドウの幅・高さ
  getmaxyx(stdscr, height, width);
  double max_range = 10.0; // [m] 現在表示する範囲の最大値
  double csize_h = max_range / (height / 2); // 縦方向の解像度
  double csize_w = max_range / (width / 2);  // 横方向の解像度

  int info_win_width = 31;  // 情報ウィンドウの幅
  int info_win_height = 10; // 情報ウィンドウの高さ
  info_win = newwin(info_win_height, info_win_width, height / 2 + 5,
                    width / 2 - info_win_width / 2);
  bool DISPLAY_INFO = false; // 情報ウィンドウの表示フラグ

  int world_win_width = width;
  int world_win_height = height;
  world_win = newwin(world_win_height, world_win_width, 0, 0);
  bool DISPLAY_WORLD = false;

  int time_count = 0;
  if (USE_LIDAR) {
    urg.start_measurement(qrk::Urg_driver::Distance,
                          qrk::Urg_driver::Infinity_times, 0);
  }
  for (;;) {
    if (!DISPLAY_WORLD) {
      long time_stamp = 0;
      std::vector<long> data;
      if (USE_LIDAR) {
        if (!urg.get_distance(data, &time_stamp)) {
          std::cout << "Urg_driver::get_distance(): " << urg.what()
                    << std::endl;
          ncurses_close();
          return 1;
        }
      }

      // 背景の塗りつぶし
      for (int i = 1; i < height - 1; i++) {
        for (int j = 1; j < width - 1; j++) {
          double dist =
              std::hypot((j - width / 2) * csize_w, (i - height / 2) * csize_h);
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

      // 計測点の描画
      attrset(COLOR_PAIR(1));
      for (int i = 0; i < data.size(); i++) {
        if (i % 10 == 0) {
          double x = data[i] / 1000.0 *
                     cos((start_angle + i * step_angle) * M_PI / 180);
          double y = data[i] / 1000.0 *
                     sin((start_angle + i * step_angle) * M_PI / 180);
          int ix = (-y) / csize_w + width / 2;
          int iy = -(x) / csize_h + height / 2;
          if (ix > 0 && ix < width && iy > 0 && iy < height) {
            mvprintw(iy, ix, "@");
          }
        }
      }

      // 座標軸の描画
      attrset(COLOR_PAIR(1));
      for (int i = 1; i < height - 1; i++) {
        mvprintw(i, width / 2, "│");
      }
      for (int i = 1; i < width - 1; i++) {
        mvprintw(height / 2, i, "─");
      }
      mvprintw(height / 2, width / 2, "┼");
      mvprintw(1, width / 2 + 1, "%1.0f", max_range);
      mvprintw(height - 2, width / 2 - 3, "%1.0f", -max_range);
      mvprintw(height / 2, width - 3, "%1.0f", max_range);
      mvprintw(height / 2, 1, "%1.0f", -max_range);
      mvprintw(1, 1, "%d", time_count);
      refresh();

      int key = getch();
      if (key == 'q') {
        if (DISPLAY_INFO) {
          DISPLAY_INFO = false;
        } else if (DISPLAY_WORLD) {
          DISPLAY_WORLD = false;
        } else {
          ncurses_close();
          break;
        }
      } else if (key == 'u') {
        if (max_range < MAX_RANGE) {
          max_range += 1.0;
          csize_h = max_range / (height / 2);
          csize_w = max_range / (width / 2);
        }
      } else if (key == 'd') {
        if (max_range > MIN_RANGE) {
          max_range -= 1.0;
          csize_h = max_range / (height / 2);
          csize_w = max_range / (width / 2);
        }
      } else if (key == 'I') {
        if (DISPLAY_INFO)
          DISPLAY_INFO = false;
        else
          DISPLAY_INFO = true;
      } else if (key == 'w') {
        if (DISPLAY_WORLD)
          DISPLAY_WORLD = false;
        else
          DISPLAY_WORLD = true;
      }

      if (DISPLAY_INFO) {
        box(info_win, ACS_VLINE, ACS_HLINE);
        mvwprintw(info_win, 1, 1, "LIDAR: %s/%d", LIDAR_MODEL,
                  static_cast<int>(USE_LIDAR));
        mvwprintw(info_win, 2, 1, "START: %+0.f[deg]", start_angle);
        mvwprintw(info_win, 3, 1, "  END: %+0.f[deg]", end_angle);
        mvwprintw(info_win, 4, 1, " STEP: %0.2f[deg]", step_angle);
        wrefresh(info_win);
      }
    } else if (DISPLAY_WORLD) {
      box(world_win, ACS_VLINE, ACS_HLINE);
      mvwprintw(world_win, 1, 1, "WIDTH: %0.1f[m]", map.WIDTH * map.csize);
      mvwprintw(world_win, 2, 1, "HEIGHT: %0.1f[m]", map.HEIGHT * map.csize);
      double csize_w_world = 1.0 * width / map.WIDTH;
      double csize_h_world = 1.0 * height / map.HEIGHT;
      mvwprintw(world_win, 3, 1, "csize_w: %0.3f[m/pix]", csize_w_world);
      mvwprintw(world_win, 4, 1, "csize_h: %0.3f[m/pix]", csize_h_world);

      for (int y = 0; y < map.HEIGHT; y++) {
        int iy = y * csize_h_world;
        if (iy > 0 && iy < (height - 1)) {
          for (int x = 0; x < map.WIDTH; x++) {
            uint32_t val = map.map[y * map.WIDTH + x];
            uint8_t color = val >> 24;
            if (color < 10) {
              int ix = x * csize_w_world;
              if (ix > 0 && ix < (width - 1)) {
                mvwprintw(world_win, iy, ix, "･");
              }
            }
          }
        }
      }
      mvwprintw(world_win, map.ORIGIN_Y * csize_h_world,
                map.ORIGIN_X * csize_w_world, "@");

      int key = getch();
      if (key == 'q') {
        if (DISPLAY_WORLD) {
          DISPLAY_WORLD = false;
        }
      } else if (key == 'u') {
        if (max_range < MAX_RANGE) {
          max_range += 1.0;
        }
      } else if (key == 'd') {
        if (max_range > MIN_RANGE) {
          max_range -= 1.0;
        }
      } else if (key == 'w') {
        DISPLAY_WORLD = false;
      }
      wrefresh(world_win);
    }

    time_count++;

    usleep(2500);
  }

  return EXIT_SUCCESS;
}
