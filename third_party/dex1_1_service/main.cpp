#include <unitree/idl/go2/MotorCmds_.hpp>
#include <unitree/idl/go2/MotorStates_.hpp>
#include <unitree/common/thread/thread.hpp>
#include "dds/Publisher.h"
#include "dds/Subscription.h"

#include "unitreeMotor/unitreeMotor.h"
#include "serialPort/SerialPort.h"

#include "param.h"
#include <map>
#include <memory>
#include <spdlog/spdlog.h>
#include <unistd.h>
#include <iostream>
#include <vector>
#include <string>
#include <filesystem>

// Function to list available serial ports
std::vector<std::string> getAvailableSerialPorts() {
    std::vector<std::string> ports;
    constexpr const char* kSupportedPrefixes[] = {
        "/dev/ttyUSB",
        "/dev/ttyCH343USB", // support new serial hub
        "/dev/ttyACM", // support unitree H2
    };

    for (const auto& entry : std::filesystem::directory_iterator("/dev")) {
        std::string path = entry.path().string();
        for (const char* prefix : kSupportedPrefixes) {
            if (path.rfind(prefix, 0) == 0) {
                ports.push_back(path);
                break;
            }
        }
    }
    spdlog::info("Available Serial Ports: {}", fmt::join(ports, ", "));

    return ports;
}

// Motor control unit, handles dds command subscription, state publishing, and serial communication
class MotorUnit {
public:
    MotorUnit(int id, std::shared_ptr<SerialPort> serial, const std::string& cmdTopic, const std::string& stateTopic)
        : id_(id), serial_(serial), cmdTopic_(cmdTopic), stateTopic_(stateTopic) {

        cmd_.id = id_;
        cmd_.motorType = MotorType::M4010;
        cmd_.mode = queryMotorMode(cmd_.motorType, MotorMode::BRAKE);
        state_.motorType = cmd_.motorType;
        gear_ratio_ = queryGearRatio(cmd_.motorType);

        sub_ = std::make_shared<unitree::robot::SubscriptionBase<unitree_go::msg::dds_::MotorCmds_>>(cmdTopic_);
        pub_ = std::make_shared<unitree::robot::RealTimePublisher<unitree_go::msg::dds_::MotorStates_>>(stateTopic_);
        sub_->msg_.cmds().resize(1);
        pub_->msg_.states().resize(1);

        thread_ = std::make_unique<unitree::common::RecurrentThread>(2000, [&]() { this->loop_(); });
    }

private:
    void loop_() {
        if (sub_->isTimeout()) {
            cmd_.mode = queryMotorMode(cmd_.motorType, MotorMode::BRAKE);
            cmd_.kp = cmd_.kd = cmd_.q = cmd_.dq = cmd_.tau = 0.0f;
            cmd_.timeout = 1;
        } 
        else {
            auto& msg = sub_->msg_.cmds()[0];
            cmd_.mode = queryMotorMode(cmd_.motorType, MotorMode::FOC);
            cmd_.kp = msg.kp() / (gear_ratio_ * gear_ratio_);
            cmd_.kd = msg.kd() / (gear_ratio_ * gear_ratio_);
            cmd_.q = msg.q() * gear_ratio_;
            cmd_.dq = msg.dq() * gear_ratio_;
            cmd_.tau = msg.tau() / gear_ratio_;
            cmd_.timeout = 0;
        }

        serial_->sendRecv(&cmd_, &state_);
        if (pub_->trylock()) {
            pub_->msg_.states()[0].q() = state_.q / gear_ratio_;
            pub_->msg_.states()[0].dq() = state_.dq / gear_ratio_;
            pub_->msg_.states()[0].tau_est() = state_.tau * gear_ratio_;
            pub_->unlockAndPublish();
        }
    }

    int id_;
    float gear_ratio_;
    std::string cmdTopic_, stateTopic_;
    std::shared_ptr<SerialPort> serial_;
    MotorCmd cmd_;
    MotorData state_;
    std::shared_ptr<unitree::robot::SubscriptionBase<unitree_go::msg::dds_::MotorCmds_>> sub_;
    std::shared_ptr<unitree::robot::RealTimePublisher<unitree_go::msg::dds_::MotorStates_>> pub_;
    std::unique_ptr<unitree::common::RecurrentThread> thread_;
};

// Main server class, handles motor detection, DDS setup, and calibration
class Dex1GripperServer {
public:
    // Constructor attempts to detect motors on provided serial ports
    Dex1GripperServer(const std::vector<std::string>& ports) {
        bool found = false;
        for (int attempt = 0; attempt < 3 && !found; ++attempt) {
            detectMotors_(ports);
            found = !motors_.empty();
            if (!found) {
                usleep(50000);
            }
        }
        if (!found) {
            spdlog::error("Motors not found after multiple attempts.");
            exit(1);
        }
    }
    // Set up DDS publishers and subscribers for each detected motor
    void runDDS() {
        // Initialize DDS for each detected motor
        for (auto& [id, motor_info] : motors_) {
            std::string side = (id == 0) ? "right" : "left";
            std::string cmdTopic = "rt/dex1/" + side + "/cmd";
            std::string stateTopic = "rt/dex1/" + side + "/state";
            motor_info.unit = std::make_unique<MotorUnit>(id, motor_info.serial, cmdTopic, stateTopic);
        }
    }

    void calibrate() {
        char key;
        int total = motors_.size();
        int index = 1;
        for (auto& [id, motor_info] : motors_) {
            std::string side = (id == 0) ? "right" : "left";
            spdlog::info("========== Motor Calibration (Motor {} (index) of {} (total)) ==========", index, total);
            spdlog::info("  - Motor ID: {}, \t Side: {}, \t Port: {}", id, side, motor_info.port_name);
            spdlog::info("Please manually close the gripper tightly. \n \t\t\t\t Then press 's' + Enter to calibrate, or any other key to skip.");
            std::cout << "> ";
            std::cin >> key;
            if (key == 's'|| key == 'S') {
                spdlog::info("Calibrating motor {}...", id);
                bool res = motor_info.serial->calibration(MotorType::M4010, id, 0.0f, 322 * (M_PI / 180.));
                if (res)
                    spdlog::info("Motor {} calibration successful.", id);
                else
                    spdlog::error("Motor {} calibration failed.", id);
            } else {
                spdlog::info("Skipped calibration for motor {}.", id);
            }
            ++index;
        }
        spdlog::info("Calibration process completed.");
        exit(0);
    }

    // 清除电机锁存故障（复位指令 status=7）。
    //
    // 背景：M4010 在堵转/过流后会锁存 MError=0x200（绕组过热位），此后电机拒绝进入 FOC，
    // 表现为「夹爪完全不动，但服务、DDS、话题全部正常」，极难排查。
    // 该错误位断电不一定能清，但 status=7 复位指令实测能清（0x200 -> 0x0）。
    // 由 scripts/start_xr_teleop.sh 在拉起本服务前调用一次，实现自愈。
    void resetMotors() {
        for (auto& [id, motor_info] : motors_) {
            std::string side = (id == 0) ? "right" : "left";
            spdlog::info("Resetting motor {} ({}) on {} ...", id, side, motor_info.port_name);

            MotorCmd cmd;
            MotorData data;
            cmd.motorType = MotorType::M4010;
            cmd.id = id;
            cmd.mode = 7;               // status=7: 复位（重启电机并重新执行上电自检）
            cmd.tau = cmd.dq = cmd.q = cmd.kp = cmd.kd = 0.0f;
            cmd.timeout = 0;
            data.motorType = cmd.motorType;

            // 复位期间电机不响应其他指令（约 0.3s），多发一会儿再等。
            for (int i = 0; i < 20; ++i) {
                motor_info.serial->sendRecv(&cmd, &data);
                usleep(20000);
            }
            usleep(1000000);

            // 回读确认（status=0 停机，只读，不给电机任何力矩）
            cmd.mode = 0;
            bool ok = false;
            for (int i = 0; i < 5 && !ok; ++i) {
                ok = motor_info.serial->sendRecv(&cmd, &data);
                if (!ok) usleep(20000);
            }
            if (ok) {
                spdlog::info("  motor {} after reset: MError=0x{:X} temp={}C", id,
                             (unsigned int)data.merror, data.temp);
                if (data.merror != 0) {
                    spdlog::warn("  motor {} still reports MError=0x{:X} "
                                 "(故障未清除，可能需要给夹爪断电重启或检查硬件)", id,
                                 (unsigned int)data.merror);
                }
            } else {
                spdlog::warn("  motor {} no reply after reset", id);
            }
        }
        spdlog::info("Motor reset done.");
    }

private:
    struct MotorInfo {
        std::shared_ptr<SerialPort> serial;
        std::string port_name;
        std::unique_ptr<MotorUnit> unit;
    };

    std::map<int, MotorInfo> motors_; // key: motor id, value: MotorInfo(serial, port name, MotorUnit)

    // Attempt to detect motors on the given serial ports
    void detectMotors_(const std::vector<std::string>& ports) {
        // ===================== Silence begin =====================
        // Temporarily redirect stdout to /dev/null
        // This is only to suppress unwanted console output from serial->sendRecv()

        // int saved_stdout = dup(STDOUT_FILENO);
        // int null_fd = open("/dev/null", O_WRONLY);
        // dup2(null_fd, STDOUT_FILENO);
        // close(null_fd);

        // ===================== Silence begin =====================

        for (const auto& port : ports) {
            auto serial = std::make_shared<SerialPort>(port.c_str());
            for (int id = 0; id <= 1; ++id) {
                MotorCmd cmd;
                MotorData data;
                cmd.motorType = MotorType::M4010;
                cmd.id = id;
                cmd.mode = queryMotorMode(cmd.motorType, MotorMode::FOC);
                data.motorType = cmd.motorType;
                usleep(200);
                if (serial->sendRecv(&cmd, &data)) {
                    if (motors_.count(id)) {
                        spdlog::warn("Duplicate motor ID {} detected on port {}. Skipping!", id, port);
                        continue;
                    } 
                    motors_[id] = { serial, port, nullptr };
                }
            }
        }

        // ===================== Silence end =====================
        // Restore original stdout

        // dup2(saved_stdout, STDOUT_FILENO);
        // close(saved_stdout);
        
        // ===================== Silence end =====================
        
        if (!motors_.empty()) {
            spdlog::info("Detected motors:");
            for (const auto& [id, motor_info] : motors_) {
                std::string side = (id == 0) ? "right" : "left";
                std::string cmdTopic = "rt/dex1/" + side + "/cmd";
                std::string stateTopic = "rt/dex1/" + side + "/state";
                spdlog::info("  - Motor ID: {} \t Side: {} \t Port: {} \t cmdTopic: {} \t stateTopic: {}", id, side, motor_info.port_name, cmdTopic, stateTopic);
            }
        }
    }
};

int main(int argc, char** argv) {
    auto vm = param::helper(argc, argv);
    unitree::robot::ChannelFactory::Instance()->Init(0, vm["network"].as<std::string>());

    std::vector<std::string> ports = getAvailableSerialPorts();
    if (ports.empty()) {
        spdlog::warn("No ttyUSB serial ports found.");
        return 0;
    }

    Dex1GripperServer server(ports);

    if (vm.count("reset")) {
        server.resetMotors();
        return 0;
    }

    if (vm.count("calibration")) {
        server.calibrate();
        return 0;
    }

    server.runDDS();
    spdlog::info("Dex1-1 Gripper Server started.");

    while (true) sleep(1);
    return 0;
}