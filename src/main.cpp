#include <iostream>
#include <algorithm>
#include <Eigen/Dense>


#include <gtsam/geometry/Point3.h>

#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>


#include <gtsam/nonlinear/DoglegOptimizer.h>
#include <gtsam/slam/BetweenFactor.h>
#include <BetweenFactorVicon.h>

#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Pose3.h>

class BundleAdjust
{
public:
	BundleAdjust() : mbConverged(false), mMaxIterations(20), mUpdateConvergenceLimit(1e-07), mBundleCout(0), mbStop(NULL)
	{

		gtsam::noiseModel::Diagonal::shared_ptr noise = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << gtsam::Vector3::Constant(2.0), gtsam::Vector3::Constant(0.1)).finished());
		poseMeasurementNoise = gtsam::noiseModel::Robust::Create(gtsam::noiseModel::mEstimator::Huber::Create(1.0), noise);

		Eigen::Isometry3d offset;
		offset.setIdentity();

		const Eigen::Vector3d t = offset.translation();
		const Eigen::Matrix3d R = offset.linear();
		gtsam::Point3 position(t(0), t(1), t(2));
		gtsam::Rot3 rotation(R(0,0), R(0,1), R(0,2), R(1,0), R(1,1), R(1,2), R(2,0), R(2,1), R(2,2));
		gtsam::Pose3 pose(rotation, position);

		// init: no offset
		estimates.insert(gtsam::Symbol('p', 0), pose);
		estimates.insert(gtsam::Symbol('p', 1), pose);

	}

	void addMeasurements(const Eigen::Isometry3d &viconPose, const Eigen::Isometry3d &trackerPose)
	{
		// isometry to gtsam pose
		Eigen::Vector3d t = viconPose.translation();
		Eigen::Matrix3d R = viconPose.linear();
		gtsam::Point3 position(t(0), t(1), t(2));
		gtsam::Rot3 rotation(R(0,0), R(0,1), R(0,2), R(1,0), R(1,1), R(1,2), R(2,0), R(2,1), R(2,2));
		gtsam::Pose3 meas1(rotation, position);

		t = trackerPose.translation();
		R = trackerPose.linear();
		gtsam::Point3 position2(t(0), t(1), t(2));
		gtsam::Rot3 rotation2(R(0,0), R(0,1), R(0,2), R(1,0), R(1,1), R(1,2), R(2,0), R(2,1), R(2,2));
		gtsam::Pose3 meas2(rotation2, position2);

		graph.add(gtsam::BetweenFactorVicon<gtsam::Pose3>(gtsam::Symbol('p', 0), gtsam::Symbol('p', 1), meas1, meas2, poseMeasurementNoise));
	}

	int Compute(const bool *bStop)
	{
		gtsam::DoglegParams params;
		params.maxIterations = 10;
		params.errorTol = 10e-6;
		params.relativeErrorTol = 10e-3;
		gtsam::DoglegOptimizer opt(graph, estimates, params);
		try
		{
			results = opt.optimize();
			//    mbConverged = opt.error() <= params.errorTol;
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
		  }
		  catch (std::exception)
		  {
			printf("Keys do not exist");
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

	BundleAdjust *ba = new BundleAdjust();

	for(size_t i=0;i<vicon_poses.size();i++){
		ba->addMeasurements(vicon_poses[i], measured_poses[i]);
	}

	ba->Compute();

    return 0;
}
