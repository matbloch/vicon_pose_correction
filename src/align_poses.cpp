#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include <algorithm>
#include <Eigen/Dense>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>

#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

#include <gtsam/nonlinear/DoglegOptimizer.h>
//#include <gtsam/slam/BetweenFactor.h>
#include <BetweenFactorVicon.h>
#include <OffsetFactorVicon.h>
#include <gtsam/slam/PriorFactor.h>

// command line parsing
#include <gflags/gflags.h>

// define parameters
DEFINE_string(dataset, "", "path to dataset");
DEFINE_string(ds_tracker, "", "path to tracker dataset");
DEFINE_string(ds_vicon, "", "path to vicon dataset");
DEFINE_string(output, "", "output filename (measured pose aligned to vicon data)");
DEFINE_double(scaling_factor, 1.0, "scaling factor of the measurements");
DEFINE_bool(fake_tracker_readings, false, "fake tracker readings");
DEFINE_string(offsets, "", "world and camera offsets output file");

//#define CAM_OFFSET_ONLY

class ViconAligner
{
public:
	ViconAligner() : mbConverged(false), mMaxIterations(20), mUpdateConvergenceLimit(1e-07), mBundleCout(0), mbStop(NULL)
	{

		gtsam::noiseModel::Diagonal::shared_ptr noise = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << gtsam::Vector3::Constant(0.1), gtsam::Vector3::Constant(50.)).finished());
		poseMeasurementNoise = gtsam::noiseModel::Robust::Create(gtsam::noiseModel::mEstimator::Huber::Create(1.3), noise);
//		poseMeasurementNoise = noise;

		Eigen::Isometry3d offset_c;
		offset_c.setIdentity();
		//offset_c.fromPositionOrientationScale(Eigen::Vector3d::Zero(), Eigen::AngleAxisd(-0.5*M_PI, Eigen::Vector3d(0,1,0)).matrix(), Eigen::Vector3d::Ones());
		if (FLAGS_fake_tracker_readings)
		  offset_c.setIdentity();
		Eigen::Vector3d t_c = offset_c.translation();
		Eigen::Matrix3d R_c = offset_c.linear();
		gtsam::Point3 position_c(t_c(0), t_c(1), t_c(2));
		gtsam::Rot3 rotation_c(R_c(0,0), R_c(0,1), R_c(0,2), R_c(1,0), R_c(1,1), R_c(1,2), R_c(2,0), R_c(2,1), R_c(2,2));

		/*
		 * p0: Camera offset: Camera > Vicon markers
		 * p1: World offset: Vicon origin > Camera origin
		 */

		gtsam::Pose3 pose_c(rotation_c, position_c);
		estimates.insert(gtsam::Symbol('p', 0), pose_c);

#ifndef CAM_OFFSET_ONLY
		Eigen::Isometry3d offset_w;
		offset_w.fromPositionOrientationScale(Eigen::Vector3d(0,0,20), Eigen::AngleAxisd(0.261799167, Eigen::Vector3d(0,0,1)).matrix(), Eigen::Vector3d::Ones());
    offset_w = offset_w.inverse(Eigen::Isometry);
    if (FLAGS_fake_tracker_readings)
      offset_w.setIdentity();
    Eigen::Vector3d t_w = offset_w.translation();
    Eigen::Matrix3d R_w = offset_w.linear();
    gtsam::Point3 position_w = gtsam::Point3(t_w(0), t_w(1), t_w(2));
    gtsam::Rot3 rotation_w = gtsam::Rot3(R_w(0,0), R_w(0,1), R_w(0,2), R_w(1,0), R_w(1,1), R_w(1,2), R_w(2,0), R_w(2,1), R_w(2,2));
		gtsam::Pose3 pose_w(rotation_w, position_w);
		estimates.insert(gtsam::Symbol('p', 1), pose_w);
#endif
	}

	void addMeasurements(const Eigen::Isometry3d &viconPose, const Eigen::Isometry3d &trackerPose)
	{
		// isometry to gtsam pose
		Eigen::Vector3d t = viconPose.translation();
		Eigen::Matrix3d R = viconPose.linear();
		gtsam::Point3 position(t(0), t(1), t(2));
		gtsam::Rot3 rotation(R(0,0), R(0,1), R(0,2), R(1,0), R(1,1), R(1,2), R(2,0), R(2,1), R(2,2));
		gtsam::Pose3 vicon(rotation, position);

		t = trackerPose.translation();
		R = trackerPose.linear();
		gtsam::Point3 position2(t(0), t(1), t(2));
		gtsam::Rot3 rotation2(R(0,0), R(0,1), R(0,2), R(1,0), R(1,1), R(1,2), R(2,0), R(2,1), R(2,2));
		gtsam::Pose3 tracker(rotation2, position2);

#ifndef CAM_OFFSET_ONLY
		graph.add(gtsam::BetweenFactorVicon<gtsam::Pose3>(gtsam::Symbol('p', 0), gtsam::Symbol('p', 1), vicon, tracker, poseMeasurementNoise));
#else
		graph.add(gtsam::OffsetFactorVicon<gtsam::Pose3>(gtsam::Symbol('p', 0), vicon, tracker, poseMeasurementNoise));
#endif
	}

	int Compute()
	{
		gtsam::DoglegParams params;
		params.maxIterations = 1000;
		params.absoluteErrorTol = 10e-8;
		params.relativeErrorTol = 10e-8;
		params.verbosity = gtsam::NonlinearOptimizerParams::LINEAR;
		gtsam::DoglegOptimizer opt(graph, estimates, params);
		try
		{
			results = opt.optimize();
			mbConverged = opt.iterations() < 10;

			return opt.iterations();
		}
		catch(gtsam::IndeterminantLinearSystemException)
		{
			printf("gtsam::IndeterminantLinearSystemException!\n");
			return 0;
		}
	}

	void GetOffsets(Eigen::Isometry3d &cam_offset, Eigen::Isometry3d &world_offset){
		/*
		 * p0: Camera offset: Camera > Vicon markers
		 * p1: World offset: Vicon origin > Camera origin
		 */
		  Eigen::Isometry3d ret = Eigen::Isometry3d::Identity();
		  try
		  {
			  gtsam::Pose3 pose = results.at<gtsam::Pose3>(gtsam::Symbol('p',0));
			  gtsam::Matrix3 rot = pose.rotation().matrix();
			  gtsam::Point3 position = pose.translation();
			  ret.translation()(0) = position.x();
			  ret.translation()(1) = position.y();
			  ret.translation()(2) = position.z();
			  ret.linear()(0,0) = rot(0,0);
			  ret.linear()(0,1) = rot(0,1);
			  ret.linear()(0,2) = rot(0,2);
			  ret.linear()(1,0) = rot(1,0);
			  ret.linear()(1,1) = rot(1,1);
			  ret.linear()(1,2) = rot(1,2);
			  ret.linear()(2,0) = rot(2,0);
			  ret.linear()(2,1) = rot(2,1);
			  ret.linear()(2,2) = rot(2,2);
			  cam_offset = ret;

#ifndef CAM_OFFSET_ONLY
			  pose = results.at<gtsam::Pose3>(gtsam::Symbol('p',1));
			  rot = pose.rotation().matrix();
			  position = pose.translation();
			  ret.translation()(0) = position.x();
			  ret.translation()(1) = position.y();
			  ret.translation()(2) = position.z();
			  ret.linear()(0,0) = rot(0,0);
			  ret.linear()(0,1) = rot(0,1);
			  ret.linear()(0,2) = rot(0,2);
			  ret.linear()(1,0) = rot(1,0);
			  ret.linear()(1,1) = rot(1,1);
			  ret.linear()(1,2) = rot(1,2);
			  ret.linear()(2,0) = rot(2,0);
			  ret.linear()(2,1) = rot(2,1);
			  ret.linear()(2,2) = rot(2,2);
			  world_offset = ret;
#endif
		  }
		  catch (std::exception)
		  {
		    printf("Keys do not exist\n");
		  }
	}

	bool HasConverged() const { return mbConverged; }

protected:
	// noise models
	gtsam::noiseModel::Base::shared_ptr poseMeasurementNoise;		// pose observation noise model
	// graph values
	gtsam::Values estimates;
	gtsam::Values results;
	gtsam::NonlinearFactorGraph graph;
	// status
	bool mbConverged;
	int mMaxIterations;
	double mUpdateConvergenceLimit;
	int mBundleCout;
	// optimization stop trigger
	const bool *mbStop;
};



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

	void GetPose(Eigen::Isometry3d pose){

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
	CSVWriter() :filehandle_(nullptr), filename_(""), col_nr(0), line_nr(0) {
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

/*
pos: in original frame
rot: in original frame (right-hand rule)
*/
void generatePoseFromWorld(Eigen::Isometry3d &pose, Eigen::Vector3d pos, Eigen::Vector3d rot_ax, double rot) {

	Eigen::IOFormat CleanFmt(4, 0, ", ", "\n", "[", "]");
	//std::cout << "Adding position:" << std::endl << pos.format(CleanFmt) << std::endl;

	pose.setIdentity();
	Eigen::AngleAxisd rotation;

	rot_ax.normalize();
	rotation = Eigen::AngleAxisd(rot*M_PI / 180.0, rot_ax);
	// R^-1
	// -R^-1*T
	pose.fromPositionOrientationScale(-rotation.inverse().toRotationMatrix()*pos, rotation.inverse(), Eigen::Vector3d::Ones());
}

/*
pos: in original frame
rot: in original frame (right-hand rule)
*/
void setTrackerDataToWorld(Eigen::Isometry3d &pose) {
	Eigen::Isometry3d offset;
	offset.setIdentity();
	// map offset
	Eigen::Vector3d pos(0, 0, 0);
	Eigen::AngleAxisd rotation = Eigen::AngleAxisd(0.0*M_PI / 180.0, Eigen::Vector3d(0, 0, 1));
	offset.fromPositionOrientationScale(-rotation.inverse().toRotationMatrix()*pos, rotation.inverse(), Eigen::Vector3d::Ones());
	pose = pose * offset;
}

void setViconDataToWorld(Eigen::Isometry3d &pose) {
	Eigen::Isometry3d offset;
	offset.setIdentity();
	// map offset
	Eigen::Vector3d pos(0., 0., 20.);
	Eigen::AngleAxisd rotation = Eigen::AngleAxisd(90.0*M_PI / 180.0, Eigen::Vector3d(0, 0, 1));
	offset.fromPositionOrientationScale(-rotation.inverse().toRotationMatrix()*pos, rotation.inverse(), Eigen::Vector3d::Ones());
	pose = pose * offset;
}

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

void generate_poses(){

	Eigen::Isometry3d center_offset;
	center_offset.setIdentity();
	Eigen::Vector3d pos(0.,0.,5.);
	Eigen::AngleAxisd rot(0.0, Eigen::Vector3d(0.2,0.3,0.4).normalized());
	center_offset.fromPositionOrientationScale(-(rot*pos), rot, Eigen::Vector3d::Ones());

	Eigen::Isometry3d cam_offset;
	cam_offset.setIdentity();
	Eigen::Vector3d pos2(0.1,0.,0.);
	Eigen::AngleAxisd rot2(0.0, Eigen::Vector3d(0.2,0.3,0.4).normalized());
	cam_offset.fromPositionOrientationScale(-(rot2*pos2), rot2, Eigen::Vector3d::Ones());

	Eigen::Isometry3d pose;
	pose.setIdentity();

	// generate vicon poses
	for(int i=0;i<20;i++){
		// start at identity
		vicon_poses.push_back(pose);

		// shift and rotate
		Eigen::Vector3d pos(0,0,0.1*i); // increase in z dim
		Eigen::AngleAxisd rot(0.05*i, Eigen::Vector3d(0.2,0.5,.3).normalized());	// rotate
		pose.fromPositionOrientationScale(pos, rot, Eigen::Vector3d::Ones());
	}

	// generate vicon poses
	for(int i=0;i<20;i++){
		Eigen::Isometry3d dp;
		Eigen::Vector3d pos(0.05*i,0.08*i,0);
		Eigen::AngleAxisd rot(0.05*i, Eigen::Vector3d(0.8,0.1,.2).normalized());
		dp.fromPositionOrientationScale(pos, rot, Eigen::Vector3d::Ones());
		vicon_poses.push_back(dp*pose);
	}

	// generate tracker poses - apply offset
	for(size_t i=0;i<vicon_poses.size();i++){
#ifndef CAM_OFFSET_ONLY
		measured_poses.push_back(cam_offset*vicon_poses[i]*center_offset);
#else
		measured_poses.push_back(cam_offset*vicon_poses[i]);
#endif
	}

	//print ground truth, the resulting offsets are the inverse of the given offsets here
  Eigen::IOFormat CleanFmt(4, 0, ", ", "\n", "[", "]");
  std::cout << "cam offset GT:" << std::endl;
  std::cout << cam_offset.inverse(Eigen::Isometry).matrix().format(CleanFmt) << std::endl;
#ifndef CAM_OFFSET_ONLY
  std::cout << "world offset GT:" << std::endl;
  std::cout << center_offset.inverse(Eigen::Isometry).matrix().format(CleanFmt) << std::endl;
#endif
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
	} else {
		std::cout << "Using generated test poses." << std::endl;
		// generate measurements
		generate_poses();
	}

	// pre-align poses
	for(size_t i=0;i<vicon_poses.size();i++){
		//setViconDataToWorld(vicon_poses[i]);
		setTrackerDataToWorld(measured_poses[i]);
	}


	ViconAligner* ba = new ViconAligner();

	for(size_t i=0;i<vicon_poses.size();i++){
		ba->addMeasurements(vicon_poses[i], measured_poses[i]);
	}

	int nr_iter = ba->Compute();
	std::cout << "Track alignment terminated in: " << nr_iter << " Iterations" << std::endl;

	Eigen::Isometry3d cam_offset, world_offset;
	ba->GetOffsets(cam_offset, world_offset);

  Eigen::IOFormat CleanFmt(4, 0, ", ", "\n", "[", "]");
  std::cout << "cam offset:" << std::endl;
	std::cout << cam_offset.matrix().format(CleanFmt) << std::endl;
#ifndef CAM_OFFSET_ONLY
	std::cout << "world offset:" << std::endl;
	std::cout << world_offset.matrix().format(CleanFmt) << std::endl;
#endif

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



    if(FLAGS_offsets != ""){
        CSVWriter out(FLAGS_offsets);
        out.addPose<double>(world_offset);
        out.startNewRow();
        out.addPose<double>(cam_offset);
    }

    std::cout << "vicon data and aligned measured poses have been saved to file: " << filename << std::endl;
	}
  return 0;
}
