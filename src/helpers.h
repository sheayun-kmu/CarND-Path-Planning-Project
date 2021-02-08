#ifndef HELPERS_H
#define HELPERS_H

#include <string>
#include <vector>

// for convenience
using std::string;
using std::vector;

string hasData(string s);
inline constexpr double pi() { return M_PI; }
inline double deg2rad(double x) { return x * pi() / 180; }
inline double rad2deg(double x) { return x * 180 / pi(); }
inline double distance(double x1, double y1, double x2, double y2) {
  return sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
}
int ClosestWaypoint(double x, double y, const vector<double> &maps_x,
                    const vector<double> &maps_y);
int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x,
                 const vector<double> &maps_y);
vector<double> getFrenet(double x, double y, double theta,
                         const vector<double> &maps_x,
                         const vector<double> &maps_y);
vector<double> getXY(double s, double d, const vector<double> &maps_s,
                     const vector<double> &maps_x,
                     const vector<double> &maps_y);

#endif  // HELPERS_H