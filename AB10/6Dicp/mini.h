// g++ -c -o mini.o mini.c

#include <iostream>
using std::cout;
using std::cerr;
using std::endl;
#include <vector>
using std::vector;
#include <list>
using std::list;
#include <algorithm>
using std::swap;
#include <fstream>
using std::ifstream;
using std::ofstream;

typedef double point[3];
typedef double vec[3]; 

class PtPair {
  public:
    point p1, p2;
    float weight;
};

double Point_Point_Align(const vector<PtPair> &pairs, double *alignxf);
