#include <hw_tool.h>

using namespace std;
using namespace Eigen;

void Homeworktool::initGridMap(double _resolution, Vector3d global_xyz_l, Vector3d global_xyz_u, int max_x_id,
                               int max_y_id, int max_z_id)
{
  gl_xl = global_xyz_l(0);
  gl_yl = global_xyz_l(1);
  gl_zl = global_xyz_l(2);

  gl_xu = global_xyz_u(0);
  gl_yu = global_xyz_u(1);
  gl_zu = global_xyz_u(2);

  GLX_SIZE = max_x_id;
  GLY_SIZE = max_y_id;
  GLZ_SIZE = max_z_id;
  GLYZ_SIZE = GLY_SIZE * GLZ_SIZE;
  GLXYZ_SIZE = GLX_SIZE * GLYZ_SIZE;

  resolution = _resolution;
  inv_resolution = 1.0 / _resolution;

  data = new uint8_t[GLXYZ_SIZE];
  memset(data, 0, GLXYZ_SIZE * sizeof(uint8_t));
}

void Homeworktool::setObs(const double coord_x, const double coord_y, const double coord_z)
{
  if (coord_x < gl_xl || coord_y < gl_yl || coord_z < gl_zl || coord_x >= gl_xu || coord_y >= gl_yu || coord_z >= gl_zu)
    return;

  int idx_x = static_cast<int>((coord_x - gl_xl) * inv_resolution);
  int idx_y = static_cast<int>((coord_y - gl_yl) * inv_resolution);
  int idx_z = static_cast<int>((coord_z - gl_zl) * inv_resolution);

  data[idx_x * GLYZ_SIZE + idx_y * GLZ_SIZE + idx_z] = 1;
}

bool Homeworktool::isObsFree(const double coord_x, const double coord_y, const double coord_z)
{
  Vector3d pt;
  Vector3i idx;

  pt(0) = coord_x;
  pt(1) = coord_y;
  pt(2) = coord_z;
  idx = coord2gridIndex(pt);

  int idx_x = idx(0);
  int idx_y = idx(1);
  int idx_z = idx(2);

  return (idx_x >= 0 && idx_x < GLX_SIZE && idx_y >= 0 && idx_y < GLY_SIZE && idx_z >= 0 && idx_z < GLZ_SIZE &&
          (data[idx_x * GLYZ_SIZE + idx_y * GLZ_SIZE + idx_z] < 1));
}

Vector3d Homeworktool::gridIndex2coord(const Vector3i& index)
{
  Vector3d pt;

  pt(0) = ((double)index(0) + 0.5) * resolution + gl_xl;
  pt(1) = ((double)index(1) + 0.5) * resolution + gl_yl;
  pt(2) = ((double)index(2) + 0.5) * resolution + gl_zl;

  return pt;
}

Vector3i Homeworktool::coord2gridIndex(const Vector3d& pt)
{
  Vector3i idx;
  idx << min(max(int((pt(0) - gl_xl) * inv_resolution), 0), GLX_SIZE - 1),
      min(max(int((pt(1) - gl_yl) * inv_resolution), 0), GLY_SIZE - 1),
      min(max(int((pt(2) - gl_zl) * inv_resolution), 0), GLZ_SIZE - 1);

  return idx;
}

Eigen::Vector3d Homeworktool::coordRounding(const Eigen::Vector3d& coord)
{
  return gridIndex2coord(coord2gridIndex(coord));
}

double Homeworktool::OptimalBVP(Eigen::Vector3d _start_position, Eigen::Vector3d _start_velocity,
                                Eigen::Vector3d _target_position)
{
  double optimal_cost = 100000;  // this just to initial the optimal_cost, you can delete it
  /*
  STEP 2: go to the hw_tool.cpp and finish the function Homeworktool::OptimalBVP
  the solving process has been given in the document
  because the final point of trajectory is the start point of OBVP, so we input the pos,vel to the OBVP
  after finish Homeworktool::OptimalBVP, the Trajctory_Cost will record the optimal cost of this trajectory
  */
  // init
  double alpha1, alpha2, alpha3, beta1, beta2, beta3;
  double px0, pxF, py0, pyF, pz0, pzF, vx0, vy0, vz0;
  double T = 0;
  double a, b, c, d;

  px0 = _start_position(0);
  py0 = _start_position(1);
  pz0 = _start_position(2);
  pxF = _target_position(0);
  pyF = _target_position(1);
  pzF = _target_position(2);
  vx0 = _start_velocity(0);
  vy0 = _start_velocity(1);
  vz0 = _start_velocity(2);

  // Get the solution by companion matrix
  // polynominal coefficients are calculated from Mablab, which is the 1st order diff or cost function J to t
  a = 0;
  b = -4 * (pow(vx0, 2) + pow(vy0, 2) + pow(vz0, 2));
  c = -24 * (px0 * vx0 - pxF * vx0 + py0 * vy0 - pyF * vy0 + pz0 * vz0 - pzF * vz0);
  d = 72 * (pz0 * pzF + py0 * pyF + px0 * pxF) -
      36 * (pow(pzF, 2) + pow(pz0, 2) + pow(pyF, 2) + pow(py0, 2) + pow(pxF, 2) + pow(px0, 2));

  // companion matrix method to get the roots of polynomial equation
  Matrix4d A;
  A << 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, -d, -c, -b, -a;

  // calculate eigen value
  EigenSolver<Eigen::MatrixXd> es(A);
  // get the real part of 4 eigen values
  MatrixXd r;
  r = es.eigenvalues().real();
  cout << "real part of eigen value = \n" << r << endl;

  // select the positive and real part of eigen value
  for (int i = 0; i < 4; i++)
  {
    if (r(i) > T)
      T = r(i);
  }
  cout << "optimal T = " << T << endl;

  // get the optimal cost
  alpha1 = (12 * (px0 - pxF + T * vx0)) / pow(T, 3) - (6 * vx0) / pow(T, 2);
  alpha2 = (12 * (py0 - pyF + T * vy0)) / pow(T, 3) - (6 * vy0) / pow(T, 2);
  alpha3 = (12 * (pz0 - pzF + T * vz0)) / pow(T, 3) - (6 * vz0) / pow(T, 2);
  beta1 = (2 * vx0) / T - (6 * (px0 - pxF + T * vx0)) / pow(T, 2);
  beta2 = (2 * vy0) / T - (6 * (py0 - pyF + T * vy0)) / pow(T, 2);
  beta3 = (2 * vz0) / T - (6 * (pz0 - pzF + T * vz0)) / pow(T, 2);

  optimal_cost = T + (1 / 3 * pow(alpha1, 2) * pow(T, 3) + alpha1 * beta1 * pow(T, 2) + pow(beta1, 2) * T) +
                 (1 / 3 * pow(alpha2, 2) * pow(T, 3) + alpha2 * beta2 * pow(T, 2) + pow(beta2, 2) * T) +
                 (1 / 3 * pow(alpha3, 2) * pow(T, 3) + alpha3 * beta3 * pow(T, 2) + pow(beta3, 2) * T);

  return optimal_cost;
}
