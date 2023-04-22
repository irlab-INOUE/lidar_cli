#include <iostream>
#include <fstream>
#include <vector>

// 現時点では1格子に対して4つの状態を格納する
// 以下の列挙型では各状態に対して
// ch1 ~ ch4 の名前をつける
enum class CH : uint8_t {
  ch1,
  ch2,
  ch3,
  ch4,
};

class MapClass {
private:
  int num_channel;   // 0:占有値 1:作業1 2:作業2 3:作業3 

public:
  std::vector<uint32_t> map;
  int WIDTH;
  int HEIGHT;
  int ORIGIN_X;
  int ORIGIN_Y;
  double csize;

  MapClass() {};
  void set_WIDTH(int val) {WIDTH = val;};
  void set_HEIGHT(int val) {HEIGHT = val;};
  void set_csize(double val) {csize = val;};
  void set_channel(int val) {num_channel = val;};
  void set_ORIGIN_X(int val) {ORIGIN_X = val;};
  void set_ORIGIN_Y(int val) {ORIGIN_Y = val;};
  //void set_map(uint32_t val) {map.emplace_back(val);};
  void set_pixel(CH channel, int x, int y, uint8_t val);
  void set_pixel_metric(CH channel, double x, double y, uint8_t val);

  void output(std::ofstream &fout);
  void read_map(std::string fname);
  void show();
  void show_config();
};

