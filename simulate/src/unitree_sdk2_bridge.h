#pragma once

#include <csignal>
#include <mujoco/mujoco.h>

#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/dds_wrapper/robots/go2/go2.h>
#include <unitree/dds_wrapper/robots/g1/g1.h>
#include <unitree/idl/hg/BmsState_.hpp>
#include <unitree/idl/hg/IMUState_.hpp>

#include <iostream>

#include "param.h"
#include "physics_joystick.h"

#ifdef LOGGER
    #include <logger/CsvLogger.h>
#endif

#define MOTOR_SENSOR_NUM 3

class UnitreeSDK2BridgeBase
{
public:
    UnitreeSDK2BridgeBase(mjModel *model, mjData *data)
    : mj_model_(model), mj_data_(data)
    {
        _check_sensor();
        if(param::config.print_scene_information == 1) {
            printSceneInformation();
        }
        if(param::config.use_joystick == 1) {
            if(param::config.joystick_type == "xbox") {
                joystick = std::make_shared<XBoxJoystick>(param::config.joystick_device, param::config.joystick_bits);
            } else if(param::config.joystick_type == "switch") {
                joystick  = std::make_shared<SwitchJoystick>(param::config.joystick_device, param::config.joystick_bits);
            } else {
                std::cerr << "Unsupported joystick type: " << param::config.joystick_type << std::endl;
                exit(EXIT_FAILURE);
            }
        }

    }

    virtual void start() {}

    void printSceneInformation()
    {
        auto printObjects = [this](const char* title, int count, int type, auto getIndex) {
            std::cout << "<<------------- " << title << " ------------->> " << std::endl;
            for (int i = 0; i < count; i++) {
                const char* name = mj_id2name(mj_model_, type, i);
                if (name) {
                    std::cout << title << "_index: " << getIndex(i) << ", " << "name: " << name;
                    if (type == mjOBJ_JOINT) {
                        double low = mj_model_->jnt_range[2 * i];
                        double high = mj_model_->jnt_range[2 * i + 1];
                        std::cout << ", jointrange: [" << low << ", " << high << "]";      
                    }
                    if (type == mjOBJ_ACTUATOR) {
                        double low = mj_model_->actuator_ctrlrange[2 * i];
                        double high = mj_model_->actuator_ctrlrange[2 * i + 1];
                        std::cout << ", ctrlrange: [" << low << ", " << high << "]";
                    }
                    if (type == mjOBJ_SENSOR) {
                        std::cout << ", dim: " << mj_model_->sensor_dim[i];
                    }
                    std::cout << std::endl;
                }
            }
            std::cout << std::endl;
        };
    
        printObjects("Link", mj_model_->nbody, mjOBJ_BODY, [](int i) { return i; });
        printObjects("Joint", mj_model_->njnt, mjOBJ_JOINT, [](int i) { return i; });
        printObjects("Actuator", mj_model_->nu, mjOBJ_ACTUATOR, [](int i) { return i; });
    
        int sensorIndex = 0;
        printObjects("Sensor", mj_model_->nsensor, mjOBJ_SENSOR, [&](int i) {
            int currentIndex = sensorIndex;
            sensorIndex += mj_model_->sensor_dim[i];
            return currentIndex;
        });
    }

protected:
    int num_motor_ = 0;
    int dim_motor_sensor_ = 0;

    mjData *mj_data_;
    mjModel *mj_model_;

    // Sensor data indices
    int imu_quat_adr_ = -1;
    int imu_gyro_adr_ = -1;
    int imu_acc_adr_ = -1;
    int frame_pos_adr_ = -1;
    int frame_vel_adr_ = -1;

    int secondary_imu_quat_adr_ = -1;
    int secondary_imu_gyro_adr_ = -1;
    int secondary_imu_acc_adr_ = -1;

    std::shared_ptr<unitree::common::UnitreeJoystick> joystick = nullptr;

    void _check_sensor()
    {
        num_motor_ = mj_model_->nu;
        dim_motor_sensor_ = MOTOR_SENSOR_NUM * num_motor_;
    
        // Find sensor addresses by name
        int sensor_id = -1;
        
        // IMU quaternion
        sensor_id = mj_name2id(mj_model_, mjOBJ_SENSOR, "imu_quat");
        if (sensor_id >= 0) {
            imu_quat_adr_ = mj_model_->sensor_adr[sensor_id];
        }
        
        // IMU gyroscope
        sensor_id = mj_name2id(mj_model_, mjOBJ_SENSOR, "imu_gyro");
        if (sensor_id >= 0) {
            imu_gyro_adr_ = mj_model_->sensor_adr[sensor_id];
        }
        
        // IMU accelerometer
        sensor_id = mj_name2id(mj_model_, mjOBJ_SENSOR, "imu_acc");
        if (sensor_id >= 0) {
            imu_acc_adr_ = mj_model_->sensor_adr[sensor_id];
        }
        
        // Frame position
        sensor_id = mj_name2id(mj_model_, mjOBJ_SENSOR, "frame_pos");
        if (sensor_id >= 0) {
            frame_pos_adr_ = mj_model_->sensor_adr[sensor_id];
        }
        
        // Frame velocity
        sensor_id = mj_name2id(mj_model_, mjOBJ_SENSOR, "frame_vel");
        if (sensor_id >= 0) {
            frame_vel_adr_ = mj_model_->sensor_adr[sensor_id];
        }

        // Secondary IMU quaternion
        sensor_id = mj_name2id(mj_model_, mjOBJ_SENSOR, "secondary_imu_quat");
        if (sensor_id >= 0) {
            secondary_imu_quat_adr_ = mj_model_->sensor_adr[sensor_id];
        }

        // Secondary IMU gyroscope
        sensor_id = mj_name2id(mj_model_, mjOBJ_SENSOR, "secondary_imu_gyro");
        if (sensor_id >= 0) {
            secondary_imu_gyro_adr_ = mj_model_->sensor_adr[sensor_id];
        }

        // Secondary IMU accelerometer
        sensor_id = mj_name2id(mj_model_, mjOBJ_SENSOR, "secondary_imu_acc");
        if (sensor_id >= 0) {
            secondary_imu_acc_adr_ = mj_model_->sensor_adr[sensor_id];
        }
    }
};

template <typename LowCmd_t, typename LowState_t>
class RobotBridge : public UnitreeSDK2BridgeBase
{
using HighState_t = unitree::robot::go2::publisher::SportModeState;
using WirelessController_t = unitree::robot::go2::publisher::WirelessController;

public:
    RobotBridge(mjModel *model, mjData *data) : UnitreeSDK2BridgeBase(model, data)
    {
        lowcmd = std::make_shared<LowCmd_t>("rt/lowcmd");
        lowstate = std::make_unique<LowState_t>();
        lowstate->joystick = joystick;
        highstate = std::make_unique<HighState_t>();
        wireless_controller = std::make_unique<WirelessController_t>();
        wireless_controller->joystick = joystick;

        foot_size = model->geom_size[mj_name2id(model, mjOBJ_GEOM, "FL") * 3 + 0];
        std::cout << "[RobotBridge] foot_size: " << foot_size << std::endl;

        std::cout << "[RobotBridge] nflex: " << model->nflex << std::endl;
        std::cout << "[RobotBridge] flex.raduis: " << model->flex_radius[0] << std::endl;

        #ifdef LOGGER
            const char* workspace = std::getenv("WORKSPACE");
            std::string csv_path = workspace ? std::string(workspace) + "/data/mujoco_data.csv" : "/data/mujoco_data.csv";
            std::cout << "[RobotBridge] Save log data to: " << csv_path << std::endl;
            CsvLogger& csvLogger = CsvLogger::getInstance();
            csvLogger.setCsvPath(csv_path);
            csvLogger.init();

            logThread_ = std::make_shared<unitree::common::RecurrentThread>(
                "unitree_bridge", UT_CPU_ID_NONE, 5000,  // 每 5000 µs (即 200 Hz) 执行一次
                [this]() {
                    CsvLogger& csvLogger = CsvLogger::getInstance();

                if (flex_flag && mj_data_->time > 5.0) {
                    std::cout << "[UnitreeSDK2Bridge] Stable static flex pos:" << std::endl;

                    size_t nvert = mj_model_->nflexvert;
                    double sum_z = 0.0;

                    size_t start_idx=0, end_idx=nvert, add_num=1, total_num=0;
                    if (mj_model_->flex_dim[0] == 3) {
                        end_idx = end_idx/2;
                    }

                    for (size_t i = start_idx; i < end_idx; i+=add_num) {
                        double x = mj_data_->flexvert_xpos[3 * i + 0];
                        double y = mj_data_->flexvert_xpos[3 * i + 1];
                        double z = mj_data_->flexvert_xpos[3 * i + 2];

                        // 这里只以 z 坐标为例计算高度统计，可根据需求改成 x/y/z 的任意分量
                        flex_pos_max = std::max(flex_pos_max, z);
                        flex_pos_min = std::min(flex_pos_min, z);
                        sum_z += z;
                        total_num++;

                        std::cout << std::fixed << std::setprecision(4)
                                << "v[" << i << "] = (" << x << ", " << y << ", " << z << ")\n";
                    }

                    flex_pos_mean = sum_z / total_num;

                    std::cout << std::setprecision(4)
                            << "[Flex Summary] z_max = " << flex_pos_max
                            << ", z_min = " << flex_pos_min
                            << ", z_mean = " << flex_pos_mean << std::endl;

                    if (std::abs(flex_pos_max - flex_pos_min) > threshold) {
                        std::cerr << "⚠️ [Warning] Flex deformation exceeds threshold: Δz = "
                                << (flex_pos_max - flex_pos_min)
                                << " > " << threshold << " m\n";
                    }

                    flex_flag = false;
                }

                    Eigen::VectorXd foot_force(4), foot_pos(4), foot_vel(4);

                    for (int j = 0; j < 4; j++) {
                        foot_pos[j] = mj_data_->sensordata[dim_motor_sensor_ + 48 + 3*j + 2] - foot_size - 2*mj_model_->flex_radius[0];
                        foot_vel[j] = mj_data_->sensordata[dim_motor_sensor_ + 60 + 3*j + 2];
                        foot_force[j] = - mj_data_->sensordata[dim_motor_sensor_ + 72 + 3*j + 2];
                    }

                    csvLogger.update("foot_force", foot_force);
                    csvLogger.update("foot_pos", foot_pos);
                    csvLogger.update("foot_vel", foot_vel);
                }
            );

            std::signal(SIGINT, +[](int signum) {
                std::cout << "[RobotBridge] Caught SIGINT." << std::endl;
                CsvLogger::getInstance().save();
                std::_Exit(signum);
            });
        #endif
    }

    void start()
    {
        thread_ = std::make_shared<unitree::common::RecurrentThread>(
            "unitree_bridge", UT_CPU_ID_NONE, 1000, [this]() { this->run(); });
    }

    virtual void run()
    {
        if(!mj_data_) return;
        if(lowstate->joystick) { lowstate->joystick->update(); }
        // lowcmd
        {
            std::lock_guard<std::mutex> lock(lowcmd->mutex_);
            for(int i(0); i<num_motor_; i++) {
                auto & m = lowcmd->msg_.motor_cmd()[i];
                mj_data_->ctrl[i] = m.tau() +
                                    m.kp() * (m.q() - mj_data_->sensordata[i]) +
                                    m.kd() * (m.dq() - mj_data_->sensordata[i + num_motor_]);
            }
        }

        // lowstate
        if(lowstate->trylock()) {
            for(int i(0); i<num_motor_; i++) {
                lowstate->msg_.motor_state()[i].q() = mj_data_->sensordata[i];
                lowstate->msg_.motor_state()[i].dq() = mj_data_->sensordata[i + num_motor_];
                lowstate->msg_.motor_state()[i].tau_est() = mj_data_->sensordata[i + 2 * num_motor_];
            }
            
            if(imu_quat_adr_ >= 0) {
                lowstate->msg_.imu_state().quaternion()[0] = mj_data_->sensordata[imu_quat_adr_ + 0];
                lowstate->msg_.imu_state().quaternion()[1] = mj_data_->sensordata[imu_quat_adr_ + 1];
                lowstate->msg_.imu_state().quaternion()[2] = mj_data_->sensordata[imu_quat_adr_ + 2];
                lowstate->msg_.imu_state().quaternion()[3] = mj_data_->sensordata[imu_quat_adr_ + 3];

                double w = lowstate->msg_.imu_state().quaternion()[0];
                double x = lowstate->msg_.imu_state().quaternion()[1];
                double y = lowstate->msg_.imu_state().quaternion()[2];
                double z = lowstate->msg_.imu_state().quaternion()[3];

                lowstate->msg_.imu_state().rpy()[0] = atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y));
                lowstate->msg_.imu_state().rpy()[1] = asin(2 * (w * y - z * x));
                lowstate->msg_.imu_state().rpy()[2] = atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z));
            }
            
            if(imu_gyro_adr_ >= 0) {
                lowstate->msg_.imu_state().gyroscope()[0] = mj_data_->sensordata[imu_gyro_adr_ + 0];
                lowstate->msg_.imu_state().gyroscope()[1] = mj_data_->sensordata[imu_gyro_adr_ + 1];
                lowstate->msg_.imu_state().gyroscope()[2] = mj_data_->sensordata[imu_gyro_adr_ + 2];
            }

            if(imu_acc_adr_ >= 0) {
                lowstate->msg_.imu_state().accelerometer()[0] = mj_data_->sensordata[imu_acc_adr_ + 0];
                lowstate->msg_.imu_state().accelerometer()[1] = mj_data_->sensordata[imu_acc_adr_ + 1];
                lowstate->msg_.imu_state().accelerometer()[2] = mj_data_->sensordata[imu_acc_adr_ + 2];
            }
            
            lowstate->msg_.tick() = std::round(mj_data_->time / 1e-3);
            lowstate->unlockAndPublish();
        }
        // highstate
        if(highstate->trylock()) {
            // stamp sec(int32) + nanosec(uint_32), time s double
            highstate->msg_.stamp().sec() = static_cast<int32_t>(mj_data_->time);
            highstate->msg_.stamp().nanosec() = static_cast<uint32_t>((mj_data_->time - highstate->msg_.stamp().sec()) * 1e9);

            if(frame_pos_adr_ >= 0) {
                highstate->msg_.position()[0] = mj_data_->sensordata[frame_pos_adr_ + 0];
                highstate->msg_.position()[1] = mj_data_->sensordata[frame_pos_adr_ + 1];
                highstate->msg_.position()[2] = mj_data_->sensordata[frame_pos_adr_ + 2];
            }
            if(frame_vel_adr_ >= 0) {
                highstate->msg_.velocity()[0] = mj_data_->sensordata[frame_vel_adr_ + 0];
                highstate->msg_.velocity()[1] = mj_data_->sensordata[frame_vel_adr_ + 1];
                highstate->msg_.velocity()[2] = mj_data_->sensordata[frame_vel_adr_ + 2];
            }
            highstate->unlockAndPublish();
        }
        // wireless_controller
        if(wireless_controller->joystick) {
            wireless_controller->unlockAndPublish();
        }
    }

    std::unique_ptr<HighState_t> highstate;
    std::unique_ptr<WirelessController_t> wireless_controller;
    std::shared_ptr<LowCmd_t> lowcmd;
    std::unique_ptr<LowState_t> lowstate;

    double foot_size = 0;
    
private:
    unitree::common::RecurrentThreadPtr thread_;
    unitree::common::RecurrentThreadPtr logThread_;
    bool flex_flag = true;
    double flex_pos_max = -1e9, flex_pos_min = 1e9, flex_pos_mean = 0.0;
    const double threshold = 0.05;
};

class Go2Bridge : public RobotBridge<unitree::robot::go2::subscription::LowCmd, unitree::robot::go2::publisher::LowState>
{
public:
    Go2Bridge(mjModel *model, mjData *data) : RobotBridge(model, data)
    {
        int sensor_id = -1;

        // Force sensor
        sensor_id = mj_name2id(mj_model_, mjOBJ_SENSOR, "FR_touch");
        if (sensor_id >= 0) {
            FR_touch_adr_ = mj_model_->sensor_adr[sensor_id];
        }

        // Foot position sensor
        sensor_id = mj_name2id(mj_model_, mjOBJ_SENSOR, "FR_pos");
        if (sensor_id >= 0) {
            FR_pos_adr_ = mj_model_->sensor_adr[sensor_id];
        }

        // Foot linear velocity sensor
        sensor_id = mj_name2id(mj_model_, mjOBJ_SENSOR, "FR_linvel");
        if (sensor_id >= 0) {
            FR_linvel_adr_ = mj_model_->sensor_adr[sensor_id];
        }
    }

    void run() override
    {
        // lowstate
        // Foot force: need correction in simulation
        lowstate->msg_.foot_force()[0] = (int) mj_data_->sensordata[FR_touch_adr_ + 0];
        lowstate->msg_.foot_force()[1] = (int) mj_data_->sensordata[FR_touch_adr_ + 1];
        lowstate->msg_.foot_force()[2] = (int) mj_data_->sensordata[FR_touch_adr_ + 2];
        lowstate->msg_.foot_force()[3] = (int) mj_data_->sensordata[FR_touch_adr_ + 3];
        // highstate
        // These can only be used in simulation for dapc data collection because:
        //  First, these fields are empty in hardware
        //  Second, these fields are expressed in body frame according to Unitree Go2 Doc, while mujoco sensor data is expressed in world frame
        for(int i=0;i<12;i++) highstate->msg_.foot_position_body()[i] = mj_data_->sensordata[FR_pos_adr_ + i];
        for(int i=0;i<12;i++) highstate->msg_.foot_speed_body()[i] = mj_data_->sensordata[FR_linvel_adr_ + i];
        for(int i=0;i<4;i++) highstate->msg_.foot_position_body()[3*i+2] -= foot_size;
        RobotBridge::run();
    }

private:
    int FR_touch_adr_ = -1;
    int FR_pos_adr_ = -1;
    int FR_linvel_adr_ = -1;
};

class G1Bridge : public RobotBridge<unitree::robot::g1::subscription::LowCmd, unitree::robot::g1::publisher::LowState>
{
public:
    G1Bridge(mjModel *model, mjData *data) : RobotBridge(model, data)
    {
        if (param::config.robot.find("g1") != std::string::npos) {
            auto* g1_lowstate = dynamic_cast<unitree::robot::g1::publisher::LowState*>(lowstate.get());
            if (g1_lowstate) {
                auto scene = param::config.robot_scene.filename().string();
                g1_lowstate->msg_.mode_machine() = scene.find("23") != std::string::npos ? 4 : 5;
            }
        }

        bmsstate = std::make_unique<BmsState_t>("rt/lf/bmsstate");
        bmsstate->msg_.soc() = 100;

        secondary_imustate = std::make_unique<IMUState_t>("rt/secondary_imu");
    }

    void run() override
    {
        RobotBridge::run();

        // secondary IMU state
        if (secondary_imustate->trylock()) {
            if(secondary_imu_quat_adr_ >= 0) {
                secondary_imustate->msg_.quaternion()[0] = mj_data_->sensordata[secondary_imu_quat_adr_ + 0];
                secondary_imustate->msg_.quaternion()[1] = mj_data_->sensordata[secondary_imu_quat_adr_ + 1];
                secondary_imustate->msg_.quaternion()[2] = mj_data_->sensordata[secondary_imu_quat_adr_ + 2];
                secondary_imustate->msg_.quaternion()[3] = mj_data_->sensordata[secondary_imu_quat_adr_ + 3];

                double w = secondary_imustate->msg_.quaternion()[0];
                double x = secondary_imustate->msg_.quaternion()[1];
                double y = secondary_imustate->msg_.quaternion()[2];
                double z = secondary_imustate->msg_.quaternion()[3];

                secondary_imustate->msg_.rpy()[0] = atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y));
                secondary_imustate->msg_.rpy()[1] = asin(2 * (w * y - z * x));
                secondary_imustate->msg_.rpy()[2] = atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z));
            }

            if(secondary_imu_gyro_adr_ >= 0) {
                secondary_imustate->msg_.gyroscope()[0] = mj_data_->sensordata[secondary_imu_gyro_adr_ + 0];
                secondary_imustate->msg_.gyroscope()[1] = mj_data_->sensordata[secondary_imu_gyro_adr_ + 1];
                secondary_imustate->msg_.gyroscope()[2] = mj_data_->sensordata[secondary_imu_gyro_adr_ + 2];
            }

            if(secondary_imu_acc_adr_ >= 0) {
                secondary_imustate->msg_.accelerometer()[0] = mj_data_->sensordata[secondary_imu_acc_adr_ + 0];
                secondary_imustate->msg_.accelerometer()[1] = mj_data_->sensordata[secondary_imu_acc_adr_ + 1];
                secondary_imustate->msg_.accelerometer()[2] = mj_data_->sensordata[secondary_imu_acc_adr_ + 2];
            }

            secondary_imustate->unlockAndPublish();
        }

        // In practice, bmsstate is sent at a low frequency; here it is sent with the main loop
        bmsstate->unlockAndPublish();
    }

    using BmsState_t = unitree::robot::RealTimePublisher<unitree_hg::msg::dds_::BmsState_>;
    using IMUState_t = unitree::robot::RealTimePublisher<unitree_hg::msg::dds_::IMUState_>;
    std::unique_ptr<BmsState_t> bmsstate;
    std::unique_ptr<IMUState_t> secondary_imustate;
};
