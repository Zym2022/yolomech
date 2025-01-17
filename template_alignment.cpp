#include "template_alignment.h"

void
TemplateAlign(char* target_pcd_path)
{
  // load template file
  std::vector<FeatureCloud> object_templates;
  std::ifstream input_stream(py2cpp_.template_list_path);
  object_templates.resize(0);
  std::string pcd_filename;
  while(input_stream.good())
  {
    std::getline(input_stream,pcd_filename);
    if(pcd_filename.empty()||pcd_filename.at(0) == '#')
      continue;

    FeatureCloud template_cloud;
    template_cloud.loadInputCloud(pcd_filename);
    object_templates.push_back(template_cloud);
  }

  input_stream.close();

  // load the traget cloud PCD file
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  // pcl::io::loadPCDFile(target_pcd_path,*cloud);

  mmind::eye::Camera camera;
  if (!findAndConnect(camera))
    {
      std::cout << "ERROR: Can't find camera";
      return;
    }

  if (!confirmCapture3D()) {
      camera.disconnect();
      return;
  }

  mmind::eye::UserSet& currentUserSet = camera.currentUserSet();

  // Set the exposure times for acquiring depth information.
  showError(currentUserSet.setFloatArrayValue(
      mmind::eye::scanning3d_setting::ExposureSequence::name, std::vector<double>{20, 10}));
  //    showError(currentUserSet.setFloatArrayValue(
  //        mmind::eye::scanning3d_setting::ExposureSequence::name, std::vector<double>{5, 10}));

  // Obtain the current exposure times for acquiring depth information to check if the setting was
  // successful.
  std::vector<double> exposureSequence;
  showError(currentUserSet.getFloatArrayValue(
      mmind::eye::scanning3d_setting::ExposureSequence::name, exposureSequence));
  std::cout << "3D scanning exposure multiplier : " << exposureSequence.size() << "."
            << std::endl;
  for (size_t i = 0; i < exposureSequence.size(); i++) {
      std::cout << "3D scanning exposure time " << i + 1 << ": " << exposureSequence[i] << " ms."
                << std::endl;
  }

  camera.setPointCloudUnit(mmind::eye::CoordinateUnit::Meter);
  mmind::eye::Frame2DAnd3D frame2DAnd3D;
  showError(camera.capture2DAnd3D(frame2DAnd3D));

  const mmind::eye::PointCloud pointCloud = frame2DAnd3D.frame3D().getUntexturedPointCloud();

  pcl::PointCloud<pcl::PointXYZ> pointCloudPCL(pointCloud.width(), pointCloud.height());
  convertToPCL(pointCloud, pointCloudPCL);

  // Preprocess the cloud by...
  // ...removing distant points
  const float depth_limit = 0.4;
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud (pointCloudPCL.makeShared());
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (0, depth_limit);
  pass.filter (*cloud);

  pcl::PassThrough<pcl::PointXYZ> pass_x;
  pass_x.setInputCloud(cloud);
  pass_x.setFilterFieldName("x");
  pass_x.setFilterLimits(-0.15, 0.35);
  pass_x.filter(*cloud);

  pcl::PassThrough<pcl::PointXYZ> pass_y;
  pass_y.setInputCloud(cloud);
  pass_y.setFilterFieldName("y");
  pass_y.setFilterLimits(-0.1, 0.5);
  pass_y.filter(*cloud);

  // ... and downsampling the point cloud
  const float voxel_grid_size = 0.002f;
  pcl::VoxelGrid<pcl::PointXYZ> vox_grid;
  vox_grid.setInputCloud (cloud);
  vox_grid.setLeafSize (voxel_grid_size, voxel_grid_size, voxel_grid_size);
  vox_grid.filter (*cloud);

  // Assign to the target FeatureCloud
  FeatureCloud target_cloud;
  target_cloud.setInputCloud(cloud);

  // Set the TemplateAlignment inputs
  TemplateAlignment template_align;
  for (size_t i = 0;i<object_templates.size();i++)
  {
    template_align.addTemplateCloud(object_templates[i]);
  }
  template_align.setTargetCloud(target_cloud);

  // Find the best template alignment
  TemplateAlignment::Result best_alignment;
  int best_index = template_align.findBestAlignment(best_alignment);
  const FeatureCloud &best_template = object_templates[best_index];

  // Print the alignment fitness score (values less than 0.00002 are good)
  printf ("Best fitness score: %f\n", best_alignment.fitness_score);

  // pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;
  // ndt.setTransformationEpsilon(0.01);
  // ndt.setResolution(1.0);

  // ndt.setInputSource(best_template.getPointCloud());
  // ndt.setInputTarget(target_cloud.getPointCloud());
  // pcl::PointCloud<pcl::PointXYZ>::Ptr unused_result_0(new pcl::PointCloud<pcl::PointXYZ>());

  // ndt.align(*unused_result_0, best_alignment.final_transformation);

  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
  icp.setMaxCorrespondenceDistance(100);
  icp.setMaximumIterations(100);
  icp.setTransformationEpsilon(1e-6);
  icp.setEuclideanFitnessEpsilon(1e-6);
  icp.setRANSACIterations(0);

  icp.setInputSource(best_template.getPointCloud());
  icp.setInputTarget(target_cloud.getPointCloud());
  pcl::PointCloud<pcl::PointXYZ>::Ptr unused_result(new pcl::PointCloud<pcl::PointXYZ>());
  icp.align(*unused_result, best_alignment.final_transformation);

  // Print the rotation matrix and translation vector
  Eigen::Matrix3f rotation = icp.getFinalTransformation().block<3,3>(0, 0);
  Eigen::Vector3f translation = icp.getFinalTransformation().block<3,1>(0, 3);
  // Eigen::Matrix3f rotation = best_alignment.final_transformation.block<3,3>(0, 0);
  // Eigen::Vector3f translation = best_alignment.final_transformation.block<3,1>(0, 3);

  printf ("\n");
  printf ("    | %6.3f %6.3f %6.3f | \n", rotation (0,0), rotation (0,1), rotation (0,2));
  printf ("R = | %6.3f %6.3f %6.3f | \n", rotation (1,0), rotation (1,1), rotation (1,2));
  printf ("    | %6.3f %6.3f %6.3f | \n", rotation (2,0), rotation (2,1), rotation (2,2));
  printf ("\n");
  printf ("t = < %0.3f, %0.3f, %0.3f >\n", translation (0), translation (1), translation (2));

  //////////////////////////
  // StandardTrans standardtrans_;
  // Eigen::Matrix4f trans_camera2target = icp.getFinalTransformation();
  // Eigen::Matrix4f trans_hole2target = trans_camera2target * standardtrans_.trans_hole2camera;
  // Eigen::Matrix4f trans_end2base = Eigen::Matrix4f::Identity(); // get from robot
  // trans_end2base << -0.4999368190765381, -1.5891189832473174e-05, 0.8660618662834167, 0.7147712707519531,
  //                   -1.0617844964144751e-05, 1.0, 1.221960974362446e-05, -0.1500995010137558,
  //                   -0.8660618662834167, -3.086678134422982e-06, -0.4999368190765381, 0.16625721752643585,
  //                   0.0, 0.0, 0.0, 1.0;    
  // Eigen::Matrix4f trans_target2base = trans_end2base * standardtrans_.trans_camera2end;
  // Eigen::Matrix4f trans_hole2base = trans_target2base * trans_hole2target;
  ///////////////////////////

  ////////////////
  StandardTrans standardtrans_;
  Eigen::Matrix4f trans_end2base = Eigen::Matrix4f::Identity();
  Eigen::Quaternionf quat_end2base(py2cpp_.qw, py2cpp_.qx, py2cpp_.qy, py2cpp_.qz);
  std::cout << py2cpp_.px << py2cpp_.py << py2cpp_.pz << std::endl;
  Eigen::Vector4f hole_pos_vec(py2cpp_.px, py2cpp_.py, py2cpp_.pz, 1.0);
  trans_end2base.block<3, 3>(0, 0) = quat_end2base.matrix();
  trans_end2base.block<4, 1>(0, 3) = hole_pos_vec;
  
  // trans_end2base << -0.4999368190765381, -1.5891189832473174e-05, 0.8660618662834167, 0.7147712707519531,
  //                   -1.0617844964144751e-05, 1.0, 1.221960974362446e-05, -0.1500995010137558,
  //                   -0.8660618662834167, -3.086678134422982e-06, -0.4999368190765381, 0.16625721752643585,
  //                   0.0, 0.0, 0.0, 1.0;    

  std::cout << trans_end2base << std::endl;
  /////////////////////////////////////////////////////

  //////////////////////////////////////////////////
  Eigen::Matrix4f trans_target2camera = best_alignment.final_transformation * standardtrans_.trans_hole2camera;
  Eigen::Matrix4f trans_target2base = trans_end2base * standardtrans_.trans_camera2end * trans_target2camera;

  rotation = trans_target2base.block<3,3>(0, 0);
  translation = trans_target2base.block<3,1>(0, 3);

  printf ("\n");
  printf ("hole to base from template estimation: \n");
  printf ("    | %6.3f %6.3f %6.3f | \n", rotation (0,0), rotation (0,1), rotation (0,2));
  printf ("R = | %6.3f %6.3f %6.3f | \n", rotation (1,0), rotation (1,1), rotation (1,2));
  printf ("    | %6.3f %6.3f %6.3f | \n", rotation (2,0), rotation (2,1), rotation (2,2));
  printf ("\n");
  printf ("t = < %0.3f, %0.3f, %0.3f >\n", translation (0), translation (1), translation (2));
  //////////////////////////////////////////////////

  // for icp
  Eigen::Matrix4f tempR;
  tempR  << 0.9659258, -0.2588190,  0.0000000, 0.0,
            0.2588190,  0.9659258,  0.0000000, 0.0,
            0.0000000,  0.0000000,  1.0000000, 0.0,
            0.0,        0.0,        0.0,       1.0;
  Eigen::Matrix4f icp_init = icp.getFinalTransformation();
  double icp_best_score = icp.getFitnessScore();
  Eigen::Matrix4f icp_best_trans = icp.getFinalTransformation();

  if (icp_best_score > 0.000005) {

    for (int i = 0; i < 24; i++) {
      icp_init = icp_init * tempR;
      icp.align(*unused_result, icp_init);
      printf ("icp iter %d fitness score: %f\n", i, icp.getFitnessScore());

      if (icp.getFitnessScore() < icp_best_score) 
      {
          icp_best_score = icp.getFitnessScore();
          icp_best_trans = icp.getFinalTransformation();
      }
      if (icp_best_score <= 0.000005) break;

    }  
  }

  trans_target2camera = icp_best_trans * standardtrans_.trans_hole2camera;
  printf ("\n");
  printf ("hole to base from template estimation: \n");
  std::cout << trans_target2camera << std::endl;
  printf ("\n");
  // Eigen::Matrix4f trans_target2camera = best_alignment.final_transformation * standardtrans_.trans_hole2camera;
  ////////////////////////////////////////////////////
  trans_target2base = trans_end2base * standardtrans_.trans_camera2end * trans_target2camera;
  ////////////////

  rotation = trans_target2base.block<3,3>(0, 0);
  translation = trans_target2base.block<3,1>(0, 3);

  printf ("\n");
  printf ("hole to base from icp estimation: \n");
  printf ("icp best fitness score: %f\n", icp_best_score);
  printf ("    | %6.3f %6.3f %6.3f | \n", rotation (0,0), rotation (0,1), rotation (0,2));
  printf ("R = | %6.3f %6.3f %6.3f | \n", rotation (1,0), rotation (1,1), rotation (1,2));
  printf ("    | %6.3f %6.3f %6.3f | \n", rotation (2,0), rotation (2,1), rotation (2,2));
  printf ("\n");
  printf ("t = < %0.3f, %0.3f, %0.3f >\n", translation (0), translation (1), translation (2));

  Eigen::AngleAxisf rotationV(0, Eigen::Vector3f(0, 1, 0));
  Eigen::Matrix4f rotationMatrix = Eigen::Matrix4f::Identity();
  rotationMatrix.block<3, 3>(0, 0) = rotationV.toRotationMatrix();
  Eigen::Matrix4f Result = trans_target2base * rotationMatrix;

  rotation = Result.block<3,3>(0, 0);
  translation = Result.block<3,1>(0, 3);

  printf ("\n");
  printf ("transformed hole to base from estimation: \n");
  printf ("    | %6.3f %6.3f %6.3f | \n", rotation (0,0), rotation (0,1), rotation (0,2));
  printf ("R = | %6.3f %6.3f %6.3f | \n", rotation (1,0), rotation (1,1), rotation (1,2));
  printf ("    | %6.3f %6.3f %6.3f | \n", rotation (2,0), rotation (2,1), rotation (2,2));
  printf ("\n");
  printf ("t = < %0.3f, %0.3f, %0.3f >\n", translation (0), translation (1), translation (2));

  var2py_.best_fitness_score = best_alignment.fitness_score;
  Eigen::Quaternionf quat_result(Result.block<3,3>(0, 0));
  var2py_.px = Result.block<3,1>(0, 3)(0);
  var2py_.py = Result.block<3,1>(0, 3)(1);
  var2py_.pz = Result.block<3,1>(0, 3)(2);
  var2py_.qx = quat_result.x();
  var2py_.qy = quat_result.y();
  var2py_.qz = quat_result.z();
  var2py_.qw = quat_result.w();

  float hole_position_x_val = 0.728864848613739;
  float hole_position_y_val = -0.28153127431869507;
  float hole_position_z_val = 0.11900459975004196;
  float hole_orientation_x_val = -0.9327642917633057;
  float hole_orientation_y_val = -0.0017601799918338656;
  float hole_orientation_z_val = -0.35992735624313354;
  float hole_orientation_w_val = 0.019999029114842415;

  Eigen::Matrix4f trans_hole2world_val = Eigen::Matrix4f::Identity();
  Eigen::Quaternionf quat_hole2world_val(hole_orientation_w_val, hole_orientation_x_val, hole_orientation_y_val, hole_orientation_z_val);
  Eigen::Vector4f hole_pos_vec_val(hole_position_x_val, hole_position_y_val, hole_position_z_val, 1.0);
  trans_hole2world_val.block<3, 3>(0, 0) = quat_hole2world_val.matrix();
  trans_hole2world_val.block<4, 1>(0, 3) = hole_pos_vec_val;
  Eigen::Matrix4f trans_hole2base_val = standardtrans_.trans_base2world.inverse() * trans_hole2world_val;
  
  ////////////////////////////////////
  rotation = trans_hole2base_val.block<3,3>(0, 0);
  translation = trans_hole2base_val.block<3,1>(0, 3);

  printf ("\n");
  printf ("hole to base from optitrack:\n");
  printf ("    | %6.3f %6.3f %6.3f | \n", rotation (0,0), rotation (0,1), rotation (0,2));
  printf ("R = | %6.3f %6.3f %6.3f | \n", rotation (1,0), rotation (1,1), rotation (1,2));
  printf ("    | %6.3f %6.3f %6.3f | \n", rotation (2,0), rotation (2,1), rotation (2,2));
  printf ("\n");
  printf ("t = < %0.3f, %0.3f, %0.3f >\n", translation (0), translation (1), translation (2));
  //////////////////////////////////////////////////////
  
  trans_hole2base_val = trans_hole2base_val * rotationMatrix;

  rotation = trans_hole2base_val.block<3,3>(0, 0);
  translation = trans_hole2base_val.block<3,1>(0, 3);

  printf ("\n");
  printf ("tranformed hole to base from optitrack:\n");
  printf ("    | %6.3f %6.3f %6.3f | \n", rotation (0,0), rotation (0,1), rotation (0,2));
  printf ("R = | %6.3f %6.3f %6.3f | \n", rotation (1,0), rotation (1,1), rotation (1,2));
  printf ("    | %6.3f %6.3f %6.3f | \n", rotation (2,0), rotation (2,1), rotation (2,2));
  printf ("\n");
  printf ("t = < %0.3f, %0.3f, %0.3f >\n", translation (0), translation (1), translation (2));

  // Save the aligned template for visualization
  pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::transformPointCloud (*best_template.getPointCloud (), *transformed_cloud, icp.getFinalTransformation());
  // pcl::transformPointCloud (*best_template.getPointCloud (), *transformed_cloud, best_alignment.final_transformation);
  pcl::io::savePCDFileBinary ("output.pcd", *transformed_cloud);
  
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = simpleVis(transformed_cloud, cloud);
  while (!viewer->wasStopped ())
  {
    viewer->spinOnce (100);
    boost::this_thread::sleep (boost::posix_time::microseconds (100000));
  }
}
