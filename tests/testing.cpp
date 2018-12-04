#include <Eigen/Dense>
#include <MEL/Core/Console.hpp>


using namespace mel;
using Eigen::Matrix;

int main(int argc, char const* argv[]) {
  Matrix<double,3,3> M;
  Matrix<double,3,1> V;
  M << 25,16,33,
       34,55,69,
       72,38,79;

  V << 1,
       2,
       3;




  print(M*2);
  print(V);
  print(M*V);
}

