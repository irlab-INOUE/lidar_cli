#include "Map.h"

void MapClass::set_pixel(CH channel, int x, int y, uint8_t val) {
  if (map.size() < WIDTH * HEIGHT) {
    map.resize(WIDTH*HEIGHT);
    for(int i = 0; i < WIDTH*HEIGHT; i++) {
      map[i] = 0;
    }
  }
  uint32_t orig = map[y*WIDTH + x];
  switch (channel) {
    case CH::ch1:
      orig = orig & 0x0FFF;
      orig = orig | (val << 24);
      break;
    case CH::ch2:
      orig = orig & 0xF0FF;
      orig = orig | (val << 16);
      break;
    case CH::ch3:
      orig = orig & 0xFF0F;
      orig = orig | (val << 8);
      break;
    case CH::ch4:
      orig = orig & 0xFFF0;
      orig = orig | val;
      break;
    default:
      break;
  }
  map[y*WIDTH + x] = orig;
}

void MapClass::set_pixel_metric(CH channel, double x, double y, uint8_t val) {
  int ix = x/csize + ORIGIN_X;
  int iy =-y/csize + ORIGIN_Y;
  if (ix >= 0 && ix < WIDTH && iy >= 0 && iy < HEIGHT)
    set_pixel(channel, ix, iy, val);
}

void MapClass::output(std::ofstream &fout) {
  fout << "MAP"    << "\n";
  fout << HEIGHT   << "\n";
  fout << WIDTH    << "\n";
  fout << csize    << "\n";
  fout << num_channel  << "\n";
  fout << ORIGIN_X << "\n";
  fout << ORIGIN_Y << "\n";
  for (auto d: map) {
    fout.write(reinterpret_cast<char*>(&d), sizeof(d));
  }
}

void MapClass::read_map(std::string fname) {
  std::ifstream fin(fname, std::ios::binary);
  std::string ind, tmp;
  uint32_t a;
  char r;
  fin >> ind;
  if (ind != "MAP") {
    std::cout << fname << " is not map file\n";
    return;
  }
  fin >> HEIGHT >> WIDTH >> csize >> num_channel >> ORIGIN_X >> ORIGIN_Y;
  fin.read((char*)&r, sizeof(r));   // ORIGIN_Yの後ろの改行を読み飛ばす
  for (int y = 0; y < HEIGHT; y++) {
    for (int x = 0; x < WIDTH; x++) {
      fin.read((char*)&a, sizeof(a));
      map.emplace_back(a);
    }
  }
}

void MapClass::show_config() {
  std::cout 
    << "WIDTH:  " << WIDTH << "\n"
    << "HEIGHT: " << HEIGHT << "\n"
    << "csize: " << csize << "\n"
    << "channel: " << num_channel << "\n"
    << "ORIGIN_X: " << ORIGIN_X << "\n"
    << "ORIGIN_Y: " << ORIGIN_Y << "\n";
}
