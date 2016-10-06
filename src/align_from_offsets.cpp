#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include <algorithm>
#include <Eigen/Dense>

// command line parsing
#include <gflags/gflags.h>

// define parameters
DEFINE_string(dataset, "", "path to dataset");
DEFINE_string(ds_tracker, "", "path to tracker dataset");
DEFINE_string(ds_vicon, "", "path to vicon dataset");
DEFINE_string(output, "", "output filename (measured pose aligned to vicon data)");
DEFINE_double(scaling_factor, 1.0, "scaling factor of the measurements");
DEFINE_string(offsets, "", "world and camera offsets");

template <class T>
class CSVParser{
public:
	CSVParser(std::string filename):filename_(filename), line_number_(0){
		filehandle_ = new std::ifstream(filename);
	}
	bool OpenFile(){
		if(filehandle_->is_open()){
			return true;
		}else{
			return false;
		}
	}
  bool IterateRows()
  {

    line_number_++;

    std::string item_string;

    std::getline(*filehandle_, current_line_);

    if (!filehandle_->eof())
    {
      //std::cout << current_line_ << std::endl;

      current_items_.clear();
      std::istringstream inStream(current_line_);

      T value;
      while (std::getline(inStream, item_string, ',') && std::istringstream(item_string) >> value)
      {
        //std::cout << csvItem << std::endl;
        current_items_.push_back(value);
      }
      return true;
    }
    return false;
  }
	T GetVal(int col){
		return current_items_[col];
	}

	void GetPose(Eigen::Isometry3d& pose){

		pose.setIdentity();
		Eigen::Vector3d pos(GetVal(0),GetVal(1),GetVal(2));
		Eigen::AngleAxisd rot;

		rot = Eigen::AngleAxisd(
				GetVal(6), // angle
				Eigen::Vector3d(GetVal(3),GetVal(4),GetVal(5)).normalized()	// direction
				);
		pos = -rot.matrix().transpose() * pos;
		pose.fromPositionOrientationScale(pos, rot.inverse(), Eigen::Vector3d::Ones());

	}

	~CSVParser(){
		filehandle_->close();
		delete filehandle_;
	}
private:
	std::string filename_;
	int line_number_;
	std::string current_line_;
	std::vector<T> current_items_;
	T current_item_;

	std::ifstream* filehandle_;

};


class CSVWriter {
public:
	CSVWriter(std::string filename) :filename_(filename), col_nr(0), line_nr(0) {
		filehandle_ = new std::ofstream();
		filehandle_->open(filename);
	}
	void startNewRow() {
		line_nr++;
		col_nr = 0;
		*filehandle_ << "\n";
	}
	template<class T> void addEntry(T val) {
		if (col_nr > 0) {
			// add new column
			*filehandle_ << ",";
		}
		*filehandle_ << val;
		col_nr++;
	}
	~CSVWriter() {
		filehandle_->close();
	}
	void changeFile(std::string filename) {
		filehandle_->close();
		delete(filehandle_);
		filehandle_ = new std::ofstream();
		filehandle_->open(filename);
		filename_ = filename;
	}

	template<class T> void addPose(Eigen::Isometry3d pose) {

		Eigen::Vector3d pos;
		Eigen::AngleAxisd rot;
		Eigen::AngleAxis<double> aa;
		Eigen::Vector3d outputAngles;
		// reconstruct
		pos = pose.inverse(Eigen::Isometry).translation();

		Eigen::IOFormat CleanFmt(4, 0, ", ", "\n", "[", "]");
		//std::cout << "position in world frame: " << std::endl << pos.format(CleanFmt) << std::endl;

		addEntry<T>(pos[0]);
		addEntry<T>(pos[1]);
		addEntry<T>(pos[2]);

		// axis angle
		aa.fromRotationMatrix(pose.linear().transpose());
		outputAngles = aa.axis().normalized();

		// write
		addEntry<double>(outputAngles[0]);
		addEntry<double>(outputAngles[1]);
		addEntry<double>(outputAngles[2]);
		addEntry<double>(aa.angle());
	}

private:
	std::string filename_;
	std::ofstream* filehandle_;
	int col_nr;
	int line_nr;
};

std::vector<Eigen::Isometry3d> vicon_poses;
std::vector<Eigen::Isometry3d> measured_poses;

void read_poses_from_seperate_file(std::vector<Eigen::Isometry3d> &poses,std::string filename, bool quaternions = false){

	CSVParser<double> csv(filename);

	Eigen::Vector3d pos;
	Eigen::AngleAxisd rot;
	Eigen::Quaterniond rotq;
	Eigen::Isometry3d pose;
	pose.setIdentity();

	while(csv.IterateRows()){

		// tracker pose
		pos = Eigen::Vector3d(csv.GetVal(0), csv.GetVal(1), csv.GetVal(2));

		if(quaternions){
			rotq.x() = csv.GetVal(3);
			rotq.y() = csv.GetVal(4);
			rotq.z() = csv.GetVal(5);
			rotq.w() = csv.GetVal(6);
			rotq.normalize();
			pos = -rotq.matrix().transpose() * pos;
			pose.fromPositionOrientationScale(pos, rotq.inverse(), Eigen::Vector3d::Ones());
		} else {
			rot = Eigen::AngleAxisd(
					csv.GetVal(6), // angle
					Eigen::Vector3d(csv.GetVal(3),csv.GetVal(4),csv.GetVal(5)).normalized()	// direction
					);
			pos = -rot.matrix().transpose() * pos;
			pose.fromPositionOrientationScale(pos, rot.inverse(), Eigen::Vector3d::Ones());
		}
		poses.push_back(pose);
	}

	std::cout << "Finished pose read-in." << std::endl;
}

void read_poses_from_file(std::vector<Eigen::Isometry3d> &vicon_poses,std::vector<Eigen::Isometry3d> &tracker_poses,std::string filename){

	CSVParser<double> csv(filename);

	Eigen::Vector3d pos;
	Eigen::AngleAxisd rot;
	Eigen::Quaterniond rotq;
	Eigen::Isometry3d pose;
	pose.setIdentity();

	while(csv.IterateRows()){

		// vicon poses - quaternions
		pos = Eigen::Vector3d(csv.GetVal(0), csv.GetVal(1), csv.GetVal(2));
		rotq.x() = csv.GetVal(3);
		rotq.y() = csv.GetVal(4);
		rotq.z() = csv.GetVal(5);
		rotq.w() = csv.GetVal(6);
		rotq.normalize();
		pos = -rotq.matrix().transpose() * pos;
		pose.fromPositionOrientationScale(pos, rotq.inverse(), Eigen::Vector3d::Ones());
		vicon_poses.push_back(pose);

		// tracker pose (rotation inverted)
		pose.setIdentity();
		pos = Eigen::Vector3d(csv.GetVal(7), csv.GetVal(8), csv.GetVal(9));
		rot = Eigen::AngleAxisd(
				csv.GetVal(13), // angle
				Eigen::Vector3d(csv.GetVal(10),csv.GetVal(11),csv.GetVal(12)).normalized()	// direction
				);
		pos = -rot.matrix().transpose() * pos;
		pose.fromPositionOrientationScale(pos, rot.inverse(), Eigen::Vector3d::Ones());
		tracker_poses.push_back(pose);
	}

	std::cout << "Finished pose read-in." << std::endl;
}


int main (int argc, char** argv) {

	google::ParseCommandLineFlags(&argc, &argv, true);

	if(FLAGS_dataset != ""){
		// read poses
		read_poses_from_file(vicon_poses, measured_poses, FLAGS_dataset);
	}else if(FLAGS_ds_tracker != ""){
		// read poses from separate file
		read_poses_from_seperate_file(measured_poses, FLAGS_ds_tracker);
		read_poses_from_seperate_file(vicon_poses, FLAGS_ds_vicon);
	}

	// pre-align poses
	for(size_t i=0;i<vicon_poses.size();i++){
		//setViconDataToWorld(vicon_poses[i]);
		//setTrackerDataToWorld(measured_poses[i]);
	}


	if(FLAGS_offsets == ""){
		std::cout << "please provide the alignment transformations" << std::endl;
		return -1;
	}


	Eigen::Isometry3d world_offset;
	world_offset.setIdentity();
	Eigen::Isometry3d cam_offset;
	cam_offset.setIdentity();

	CSVParser<double> csv(FLAGS_offsets);


	for(int i=0;i<2;i++){
		csv.IterateRows();
		if(i==0){
			csv.GetPose(world_offset);
		}else if(i==1){
			csv.GetPose(cam_offset);
		}
	}


	Eigen::IOFormat CleanFmt(4, 0, ", ", "\n", "[", "]");
	std::cout << world_offset.matrix().format(CleanFmt) << std::endl;
	std::cout << cam_offset.matrix().format(CleanFmt) << std::endl;

	Eigen::Isometry3d pose;
	Eigen::Vector3d position;
	Eigen::Vector3d outputAngles;
	Eigen::AngleAxis<double> aa;


	if(FLAGS_output != "")
	{
		std::ofstream myfile;
		std::string filename = FLAGS_output;
    std::cout << "Start writing results to file. " << std::endl;

    // try to open file
    myfile.open(filename, std::fstream::out);// std::fstream::in | | std::fstream::app);

		// save corrected poses to file
		for(size_t i=0; i < measured_poses.size();i++)
		{
			// ====== vicon data
			pose = vicon_poses[i];
			position = pose.inverse(Eigen::Isometry).translation()/FLAGS_scaling_factor;
			// save position
			myfile << position[0];
			myfile << "," << position[1];
			myfile << "," << position[2];
			// angle axis
			aa.fromRotationMatrix(pose.linear().transpose());
			outputAngles = aa.axis().normalized();
			myfile << "," << outputAngles[0];
			myfile << "," << outputAngles[1];
			myfile << "," << outputAngles[2];
			myfile << "," << aa.angle();
			myfile << ",";

			// ====== marker pose in vicon frame
			// camera pose in vicon frame
			//pose = measured_poses[i] * world_offset.inverse(Eigen::Isometry);
			// vicon position from marker measurements
			pose = cam_offset * measured_poses[i] * world_offset;
			position = pose.inverse(Eigen::Isometry).translation()/FLAGS_scaling_factor;
			// save position
			myfile << "," << position[0];
			myfile << "," << position[1];
			myfile << "," << position[2];
			// angle axis
			aa.fromRotationMatrix(pose.linear().transpose());
			outputAngles = aa.axis().normalized();
			myfile << "," << outputAngles[0];
			myfile << "," << outputAngles[1];
			myfile << "," << outputAngles[2];
			myfile << "," << aa.angle();

      // new line
      myfile << "\n";
		}
    myfile.close();



    std::cout << "vicon data and aligned measured poses have been saved to file: " << filename << std::endl;
	}
  return 0;
}
