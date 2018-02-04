#include <Model/NamesModel.h>
#include "moves/CameraCalibration.hpp"
#include "scheduler/MoveScheduler.h"
#include "services/ModelService.h"

/**
 * Ground static target in meters 
 */
const static std::vector<double> GroundTargetsValue = {0.3, 0.6, 1.0, 1.5, 2.0, 2.5};

CameraCalibration::CameraCalibration() :
    _generator(),
    _container(),
    _pixelPoint(0.0, 0.0),
    _groundPoint(0.0, 0.0, 0.0),
    _outputDataFile("/tmp/cameraCalibration.data"),
    _calibIndex(0),
    _samplesNumber(7),
    _message("")
{
    //Random seed initialization
    std::random_device rd;
    _generator = std::default_random_engine(rd());
    Move::initializeBinding();
    //RhIO Binding
    bind->bindNew("message", _message, RhIO::Bind::PushOnly)
        ->comment("User message");
    bind->bindNew("outputFile", _outputDataFile, RhIO::Bind::PullOnly)
        ->comment("Output data written file")
        ->defaultValue(_outputDataFile);
    bind->bindNew("samplesNumber", _samplesNumber, RhIO::Bind::PullOnly)
        ->comment("Number of pixel points per ground distance")
        ->minimum(1)
        ->defaultValue(_samplesNumber);
    //Binding function
    bind->bindFunc("calibration_camera_start", 
        "Restart a new camera calibration proccess",
        &CameraCalibration::cmdStartCalib, *this);
    bind->bindFunc("calibration_camera_point", 
        "Save current state and compute next target point",
        &CameraCalibration::cmdNextPoint, *this);
    bind->bindFunc("calibration_camera_renew", 
        "Generate a new pixel space point",
        &CameraCalibration::cmdRenewPoint, *this);
    bind->bindFunc("calibration_camera_test", 
        "Estimate a point on the ground.",
        &CameraCalibration::cmdTestPoint, *this, {"0.0", "0.0"});
}

std::string CameraCalibration::getName()
{
    return "CameraCalibration";
}

void CameraCalibration::onStart()
{
    _container.clear();
    _calibIndex = 0;
    _message = "Camera calibration ready.";
    Move::bind->push();
    //Release head motors
    Helpers::getScheduler()->getManager()
        ->dev<RhAL::DXL>("head_pitch").torqueEnable() = false;
    Helpers::getScheduler()->getManager()
        ->dev<RhAL::DXL>("head_yaw").torqueEnable() = false;
}

void CameraCalibration::step(float elapsed)
{
    Move::bind->pull();

    //Write data into file
    if (_calibIndex == (size_t)-1) {
        std::cout << "Saving camera calibration to: " 
            << _outputDataFile << std::endl;
        _container.save(_outputDataFile);
        _calibIndex = 0;
    }

    //Compute current target estimation on 
    //pixel space
    double currentTargetX = RhIO::Root.getFloat(
        "Vision/Calibration/targetX");
    double currentTargetY = RhIO::Root.getFloat(
        "Vision/Calibration/targetY");
    if (
        currentTargetX > -1.0 && 
        currentTargetX < 1.0 &&
        currentTargetY > -1.0 && 
        currentTargetY < 1.0
    ) {
        //Retrieve model
        Leph::HumanoidModel* model = &(Helpers::getServices()
            ->model->readModel().get());
        //Compute ground point in world frame
        Eigen::Vector3d footInSelf = model
            ->frameInSelf("left_foot_tip");
        Eigen::Vector3d groundInWorld = model
            ->selfInFrame("origin", _groundPoint + footInSelf);
        //Compute the projection estimation on pixel space
        Eigen::Vector2d pixel;
        Leph::CameraParameters camParams = Helpers::getServices()
            ->model->getCameraParameters();
        bool isSuccess = model->cameraWorldToPixel(
            camParams, groundInWorld, pixel);
        //Write it to RhIO
        if (isSuccess) {
            RhIO::Root.setFloat(
                "Vision/Calibration/estimateX", pixel.x());
            RhIO::Root.setFloat(
                "Vision/Calibration/estimateY", pixel.y());
        } else {
            RhIO::Root.setFloat(
                "Vision/Calibration/estimateX", 2.0);
            RhIO::Root.setFloat(
                "Vision/Calibration/estimateY", 2.0);
        }
    }

    Move::bind->push();
}

bool CameraCalibration::setNextPoint()
{
    //Check for calibration end
    if (_calibIndex/_samplesNumber >= GroundTargetsValue.size()) {
        RhIO::Root.setFloat(
            "Vision/Calibration/targetX", 2.0);
        RhIO::Root.setFloat(
            "Vision/Calibration/targetY", 2.0);
        RhIO::Root.setFloat(
            "Vision/Calibration/estimateX", 2.0);
        RhIO::Root.setFloat(
            "Vision/Calibration/estimateY", 2.0);
        return false;
    }
    //Generate a random target in pixel space
    std::uniform_real_distribution<double> unif(-1.0, 1.0);
    _pixelPoint.x() = unif(_generator);
    _pixelPoint.y() = unif(_generator);
    //Assign the target to vision calibration frame
    RhIO::Root.setFloat(
        "Vision/Calibration/targetX", _pixelPoint.x());
    RhIO::Root.setFloat(
        "Vision/Calibration/targetY", _pixelPoint.y());
    //Assign ground target from static data
    _groundPoint.x() = GroundTargetsValue[_calibIndex/_samplesNumber];
    _groundPoint.y() = 0.0;
    _groundPoint.z() = 0.0;
    //Increment index and continue
    _calibIndex++;
    return true;
}
        
std::string CameraCalibration::cmdStartCalib()
{
    //Reset the process
    _container.clear();
    _calibIndex = 0;
    setNextPoint();
    _message = 
        std::string("[")
        + std::to_string(_container.size())
        + std::string("]. Next ground point: X=") 
        + std::to_string(_groundPoint.x()) 
        + std::string(" Y=")
        + std::to_string(_groundPoint.y()) 
        + std::string(" (left support foot).");
    return _message;
}
        
std::string CameraCalibration::cmdNextPoint()
{
    //Retrieve model and imu state
    RhAL::StandardManager* manager = 
        Helpers::getScheduler()->getManager();
    Leph::HumanoidModel* model = &(Helpers::getServices()
        ->model->readModel().get());
    double pitch = manager->dev<RhAL::GY85>("imu").getPitch();
    double roll = manager->dev<RhAL::GY85>("imu").getRoll();
    //Save current robot state
    Leph::VectorLabel data;
    for (const std::string name : Leph::NamesDOF) {
        data.append(name, model->getDOF(name));
    }
    data.append("imu_pitch", pitch);
    data.append("imu_roll", roll);
    data.append("ground_x", _groundPoint.x());
    data.append("ground_y", _groundPoint.y());
    data.append("ground_z", _groundPoint.z());
    data.append("pixel_x", _pixelPoint.x());
    data.append("pixel_y", _pixelPoint.y());
    _container.append(data);
    //Set a new target point
    bool isContinue = setNextPoint();
    if (!isContinue) {
        _calibIndex = (size_t)-1;
        _message = "Calibration finished.";
        return _message;
    } else {
        _message = 
            std::string("[")
            + std::to_string(_container.size())
            + std::string("]. Next ground point: X=") 
            + std::to_string(_groundPoint.x()) 
            + std::string(" Y=")
            + std::to_string(_groundPoint.y()) 
            + std::string(" (left support foot).");
        if (_calibIndex % _samplesNumber == 1) {
            _message += "\nNext target point.";
        }
        return _message;
    }
}
        
void CameraCalibration::cmdRenewPoint()
{
    //Generate a random target in pixel space
    std::uniform_real_distribution<double> unif(-1.0, 1.0);
    _pixelPoint.x() = unif(_generator);
    _pixelPoint.y() = unif(_generator);
    //Assign the target to vision calibration frame
    RhIO::Root.setFloat(
        "Vision/Calibration/targetX", _pixelPoint.x());
    RhIO::Root.setFloat(
        "Vision/Calibration/targetY", _pixelPoint.y());
}
        
std::string CameraCalibration::cmdTestPoint(
    double pixelX, double pixelY)
{
    //Retrieve model
    Leph::HumanoidModel* model = &(Helpers::getServices()
        ->model->readModel().get());
    Leph::CameraParameters camParams = Helpers::getServices()
        ->model->getCameraParameters();
    //Compute view vector in world frame
    Eigen::Vector3d viewInWorld = model->cameraPixelToViewVector(
        camParams, Eigen::Vector2d(pixelX, pixelY));
    //Compute ground point in world frame
    Eigen::Vector3d groundInWorld;
    bool isSuccess = model->cameraViewVectorToWorld(
        viewInWorld, groundInWorld);
    //Convertion in left foot frame
    Eigen::Vector3d footInSelf = model
        ->frameInSelf("left_foot_tip");
    Eigen::Vector3d groundInFoot = model
        ->frameInSelf("origin", groundInWorld) - footInSelf;
    if (!isSuccess) {
        footInSelf.setZero();
    }

    return 
        std::string("pixelX=") 
        + std::to_string(pixelX)
        + std::string(" pixelY=")
        + std::to_string(pixelY)
        + std::string(" ground=[")
        + std::to_string(groundInFoot.x())
        + std::string(", ")
        + std::to_string(groundInFoot.y())
        + std::string("]");
}

