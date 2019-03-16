#include "eigen_helpers.h"
#include <iostream>

int main()
{
  EigenGateLocationList egll;

  rdr_spline_path::GateLocationList gll;

  rdr_spline_path::GateLocation gl;
  geometry_msgs::Point p;
  p.x = 1; p.y = 2; p.z = 3;
  gl.corners.push_back(p);
  p.x = 1; p.y = 2; p.z = 6;
  gl.corners.push_back(p);
  
  p.x = 4; p.y = 2; p.z = 3;
  gl.corners.push_back(p);
  p.x = 4; p.y = 2; p.z = 6;
  gl.corners.push_back(p);


  Eigen::Vector3d temp = to_eigen(p);

  std::cout << "last point: " << temp << std::endl;

  EigenGateLocation egl = to_eigen(gl);

  std::cout << "gl:\n" << egl.corners << std::endl;

  std::cout << "gl center:\n" << egl.center
    << "\n gl normal:\n" << egl.normal
    << "\n gl front point:\n" << egl.front_point
    << "\n gl back point:\n" << egl.back_point << std::endl;

  gll.gates.push_back(gl);

  std::cout << "gate location list size: " << gll.gates.size() << std::endl;

  egll = to_eigen(gll);

  std::cout << "eigen gatelist size: " << egll.gates.size() << std::endl;
  std::cout << "eigen corners:\n" << egll.gates[0].corners << std::endl;

  return 0;
}

