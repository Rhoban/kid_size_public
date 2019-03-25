#include <gtest/gtest.h>
#include <services/ViveService.h>

double epsilon = std::pow(10,-10);

bool verbose = false;

TEST(getViveToCamera, noOffset)
{
  // Default rotation is different between vive and camera basis
  ViveService vive;
  std::vector<Eigen::Vector3d> pos_in_vive =
    { Eigen::Vector3d::UnitX(), Eigen::Vector3d::UnitY(), Eigen::Vector3d::UnitZ() };
  std::vector<Eigen::Vector3d> expected_pos_in_camera =
    { Eigen::Vector3d::UnitZ(), -Eigen::Vector3d::UnitX(), -Eigen::Vector3d::UnitY() };
  for (size_t i = 0; i < pos_in_vive.size(); i++)
  {
    Eigen::Vector3d pos_in_camera = vive.getViveToCamera() * pos_in_vive[i];
    if (verbose)
    {
      std::cout << pos_in_vive[i].transpose() << " -> " << pos_in_camera.transpose() << std::endl;
    }
    for (int dim = 0; dim < 3; dim++)
    {
      EXPECT_NEAR(pos_in_camera(dim), expected_pos_in_camera[i](dim), epsilon);
    }
  }
}

TEST(getViveToCamera, zOffset)
{
  ViveService vive;
  double offset_z = -0.05;
  vive.setPosOffset(Eigen::Vector3d(0,0, offset_z));
  Eigen::Vector3d offset_in_camera(0, offset_z, 0);
  std::vector<Eigen::Vector3d> pos_in_vive =
    { Eigen::Vector3d::UnitX(), Eigen::Vector3d::UnitY(), Eigen::Vector3d::UnitZ() };
  std::vector<Eigen::Vector3d> expected_pos_in_camera =
    {
      Eigen::Vector3d::UnitZ() + offset_in_camera,
      -Eigen::Vector3d::UnitX() + offset_in_camera,
      -Eigen::Vector3d::UnitY() + offset_in_camera
    };
  for (size_t i = 0; i < pos_in_vive.size(); i++)
  {
    Eigen::Vector3d pos_in_camera = vive.getViveToCamera() * pos_in_vive[i];
    if (verbose)
    {
      std::cout << pos_in_vive[i].transpose() << " -> " << pos_in_camera.transpose() << std::endl;
    }
    for (int dim = 0; dim < 3; dim++)
    {
      EXPECT_NEAR(pos_in_camera(dim), expected_pos_in_camera[i](dim), epsilon);
    }
  }
}


int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

