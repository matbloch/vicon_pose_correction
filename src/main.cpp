#include <iostream>


std::vector<Eigen::Isometry3d> vicon_poses;
std::vector<Eigen::Isometry3d> measured_poses;


void generate_poses(){
	
	
	double x_center_offset = 0;
	double y_center_offset = 0;
	double z_center_offset = 0;
	
	Eigen::Isometry3d center_offset;
	center_offset.setIdentity();
	center_offset.translate(Eigen::Vector3d(0,0,2));
	
	
	

	Eigen::Isometry3d pose;
	pose.setIdentity();
	
	vicon_poses.push_back(pose);
	
	pose.translate(Eigen::Vector3d(0,0,2));
	
	
	for(size_t i=0;i<vicon_poses.size();i++){
		measured_poses.push_back(center_offset * vicon_poses[i]);
	}
	
	// apply offset
	
	
}

int main() {

	// add estimates

	// add between factors

    return 0;
}
