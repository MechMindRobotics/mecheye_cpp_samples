/*******************************************************************************
 *BSD 3-Clause License
 *
 *Copyright (c) 2016-2025, Mech-Mind Robotics Technologies Co., Ltd.
 *All rights reserved.
 *
 *Redistribution and use in source and binary forms, with or without
 *modification, are permitted provided that the following conditions are met:
 *
 *1. Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 *2. Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 *3. Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 *THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 *FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 *DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 *OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ******************************************************************************/

/*
With this sample, you can warm up a Mech-Eye camera by periodically performing image acquisition
over a specified duration to help the device reach stable operating conditions before use.
*/
#define NOMINMAX
#include <chrono>
#include <thread>
#include <iostream>
#include <atomic>
#include <future>
#include <csignal>
#include <iomanip>
#include <string>
#include <vector>
#include <random>
#include <sstream>
#include <algorithm>
// #include <opencv2/imgcodecs.hpp>
#include "area_scan_3d_camera/Camera.h"
#include "area_scan_3d_camera/api_util.h"

namespace {
constexpr int kDefaultWarmupMinutesOfUhpOrLsrs = 60;
constexpr int kDefaultWarmupMinutes = 30;
constexpr int kDefaultIntervalSeconds = 5;
constexpr int kDefaultIntervalSecondsOfUhp = 3;
constexpr int kMinWarmupMinutes = 15;
constexpr int kMaxWarmupMinutes = 90;
constexpr int kMinIntervalSeconds = 3;
constexpr int kMaxIntervalSeconds = 30;
std::atomic<bool> kStopFlag{false};

struct CommandLineArgs
{
    int warmupTimeMinutes = kDefaultWarmupMinutes;
    int sampleIntervalSeconds = kDefaultIntervalSeconds;
    bool userSetWarmupTime = false;
    bool userSetSampleInterval = false;
};

void handleInterrupt(int)
{
    std::cout << "\n[Interrupt] Ctrl+C received. Cleaning up..." << std::endl;
    kStopFlag.store(true);
}

CommandLineArgs parseArguments(int argc, char* argv[])
{
    CommandLineArgs args;
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if ((arg == "-w" || arg == "--warmup-time") && i + 1 < argc) {
            int val = std::atoi(argv[++i]);
            if (val < kMinWarmupMinutes || val > kMaxWarmupMinutes) {
                std::cerr << "[Error] Warmup time must be between " << kMinWarmupMinutes << " and "
                          << kMaxWarmupMinutes << " minutes.\n";
                std::exit(1);
            }
            args.warmupTimeMinutes = val;
            args.userSetWarmupTime = true;
        } else if ((arg == "-i" || arg == "--sample-interval") && i + 1 < argc) {
            int val = std::atoi(argv[++i]);
            if (val < kMinIntervalSeconds || val > kMaxIntervalSeconds) {
                std::cerr << "[Error] Sample interval must be between " << kMinIntervalSeconds
                          << " and " << kMaxIntervalSeconds << " seconds.\n";
                std::exit(1);
            }
            args.sampleIntervalSeconds = val;
            args.userSetSampleInterval = true;
        } else if (arg == "-h" || arg == "--help") {
            std::cout << "Usage: " << argv[0] << " [options]\n"
                      << "  -h, --help            show this help message and exit\n"
                      << "  -w minutes, --warmup-time minutes\n"
                      << "                        Warmup time in minutes (" << kMinWarmupMinutes
                      << "-" << kMaxWarmupMinutes << "), default: " << kDefaultWarmupMinutes
                      << " (UHP and LSRS camera default: " << kDefaultWarmupMinutesOfUhpOrLsrs
                      << "m)\n"
                      << "  -i seconds, --sample-interval seconds\n"
                      << "                        Sample interval in seconds ("
                      << kMinIntervalSeconds << "-" << kMaxIntervalSeconds
                      << "), default: " << kDefaultIntervalSeconds
                      << " (UHP camera default: " << kDefaultIntervalSecondsOfUhp << "s)\n";
            std::exit(0);
        } else {
            std::cerr << "[Error] Unknown argument: " << arg << "\n";
            std::exit(1);
        }
    }
    return args;
}

class AreaWarmUp
{
public:
    AreaWarmUp(int warmupMinutes, int intervalSeconds)
        : _warmupMinutes(warmupMinutes), _intervalSeconds(intervalSeconds)
    {
    }

    std::string createWarmupGroupName(const std::vector<std::string>& groupNames)
    {
        const std::string baseName = "warmup";
        if (std::find(groupNames.begin(), groupNames.end(), baseName) == groupNames.end()) {
            return baseName;
        }
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_int_distribution<> dis(0, 9999);

        std::string newName;
        do {
            int randomNum = dis(gen);
            std::ostringstream oss;
            oss << baseName << "_" << std::setw(4) << std::setfill('0') << randomNum;
            newName = oss.str();
        } while (std::find(groupNames.begin(), groupNames.end(), newName) != groupNames.end());

        return newName;
    }

    bool init()
    {
        if (!findAndConnect(_camera)) {
            std::cerr << "[Error] Failed to connect to camera.\n";
            return false;
        }

        if (!_camera.currentUserSet().getName(_originalUserSet).isOK()) {
            std::cerr << "[Warning] Failed to get original user set name.\n";
            return false;
        }

        std::vector<std::string> userSetNames;
        if (!_camera.userSetManager().getAllUserSetNames(userSetNames).isOK()) {
            std::cerr << "[Warning] Failed to get all user set names.\n";
            return false;
        }

        _warmupUserSet = createWarmupGroupName(userSetNames);
        if (!_camera.userSetManager().selectUserSet("calib").isOK()) {
            std::cerr << "[Warning] Failed to switch to user set : calib.\n";
            return false;
        }

        if (!_camera.userSetManager().addUserSet(_warmupUserSet).isOK()) {
            std::cerr << "[Warning] Failed to add to user set :" << _warmupUserSet << ".\n";
            return false;
        }

        if (!_camera.userSetManager().selectUserSet(_warmupUserSet).isOK()) {
            std::cerr << "[Warning] Failed to switch to user set '" << _warmupUserSet << "'.\n";
            return false;
        }

        mmind::eye::CameraInfo info;
        if (_camera.getCameraInfo(info).isOK()) {
            if (!_userSetWarmupTime) {
                if (info.model == "Mech-Eye LSR S" ||
                    info.model.find("Mech-Eye UHP") != std::string::npos) {
                    _warmupMinutes = kDefaultWarmupMinutesOfUhpOrLsrs;
                    std::cout << "[Info] Detected " << info.model << ", warmup time adjusted to "
                              << _warmupMinutes << " minutes.\n";
                } else {
                    _warmupMinutes = kDefaultWarmupMinutes;
                    std::cout << "[Info] Detected " << info.model << ", warmup time adjusted to "
                              << _warmupMinutes << " minutes.\n";
                }
            }

            if (!_userSetSampleInterval) {
                if (info.model.find("Mech-Eye UHP") != std::string::npos) {
                    _intervalSeconds = kDefaultIntervalSecondsOfUhp;
                    std::cout << "[Info] Detected UHP camera, sample interval adjusted to "
                              << _intervalSeconds << " seconds.\n";
                } else {
                    _intervalSeconds = kDefaultIntervalSeconds;
                    std::cout << "[Info] Detected " << info.model
                              << ", sample interval adjusted to " << _intervalSeconds
                              << " seconds.\n";
                }
            }
        }
        return true;
    }

    void run()
    {
        std::cout << "[Info] Starting warmup: " << _warmupMinutes
                  << " min, interval: " << _intervalSeconds << " sec." << std::endl;

        auto startTime = std::chrono::steady_clock::now();
        auto deadline = startTime + std::chrono::minutes(_warmupMinutes);
        auto nextTriggerTime = std::chrono::steady_clock::now();
        int sampleCount = 0;

        auto captureAndNotify = [&](int count) {
            captureAndPrintOnce(count, startTime);
            {
                std::lock_guard<std::mutex> lock(_mtx);
                _isCapturing = false;
            }
            _cv.notify_one();
        };

        while (!kStopFlag.load() && std::chrono::steady_clock::now() < deadline) {
            auto now = std::chrono::steady_clock::now();
            while (now < nextTriggerTime && !kStopFlag.load()) {
                auto remaining =
                    std::chrono::duration_cast<std::chrono::milliseconds>(nextTriggerTime - now);
                auto waitDuration =
                    remaining.count() > 100 ? std::chrono::milliseconds(100) : remaining;
                std::this_thread::sleep_for(waitDuration);
                now = std::chrono::steady_clock::now();
            }

            if (kStopFlag.load())
                break;

            if (_isCapturing) {
                std::unique_lock<std::mutex> lock(_mtx);
                if (_cv.wait_for(lock, std::chrono::seconds(_intervalSeconds),
                                 [&] { return !_isCapturing.load() || kStopFlag.load(); })) {
                    if (kStopFlag.load())
                        break;
                    nextTriggerTime += std::chrono::seconds(_intervalSeconds);
                    continue;
                }
            }

            {
                std::lock_guard<std::mutex> lock(_mtx);
                _isCapturing = true;
                sampleCount++;
                _task = std::async(std::launch::async, captureAndNotify, sampleCount);
            }

            nextTriggerTime += std::chrono::seconds(_intervalSeconds);
        }

        if (_isCapturing) {
            std::unique_lock<std::mutex> lock(_mtx);
            _cv.wait(lock, [&] { return !_isCapturing.load(); });
        }

        if (_task.valid())
            _task.wait();
        std::cout << "[Info] Warmup process completed." << std::endl;
    }

    ~AreaWarmUp()
    {
        if (!_originalUserSet.empty()) {
            _camera.userSetManager().selectUserSet(_originalUserSet);
            std::cout << "[Info] User set restored to: " << _originalUserSet << std::endl;
        }

        _camera.userSetManager().deleteUserSet(_warmupUserSet);
        std::cout << "[Info] delete user set: " << _warmupUserSet << std::endl;
        _camera.disconnect();
        std::cout << "[Info] Camera disconnected." << std::endl;
    }

    void setUserSetWarmupTimeFlag(bool flag) { _userSetWarmupTime = flag; }
    void setUserSetSampleIntervalFlag(bool flag) { _userSetSampleInterval = flag; }

private:
    void captureAndPrintOnce(int sampleCount,
                             const std::chrono::steady_clock::time_point& startTime)
    {
        mmind::eye::Frame2DAnd3D frame;
        auto status = _camera.capture2DAnd3D(frame);
        if (!status.isOK()) {
            std::cerr << "[Warning] Capture failed. Status: " << status.errorCode << std::endl;
            return;
        }

        mmind::eye::CameraStatus cameraStatus;
        mmind::eye::showError(_camera.getCameraStatus(cameraStatus));

        int totalSeconds = _warmupMinutes * 60;
        int elapsedSeconds = sampleCount * _intervalSeconds;
        double progress = std::min(100.0, (double)elapsedSeconds / totalSeconds * 100.0);

        if (progress > 100.0)
            progress = 100.0;
        auto systemNow = std::chrono::system_clock::now();
        auto systemTime = std::chrono::system_clock::to_time_t(systemNow);
        std::lock_guard<std::mutex> lock(_mtx);
        std::cout << "------------------------------\n"
                  << "[" << std::put_time(std::localtime(&systemTime), "%F %T")
                  << "] Warmup capture #" << sampleCount << " (Progress: " << std::fixed
                  << std::setprecision(1) << progress << "%) Completed\n";

        printCameraStatus(cameraStatus);
        std::cout << "------------------------------" << std::endl;
    }

private:
    mmind::eye::Camera _camera;
    std::string _originalUserSet;
    std::string _warmupUserSet;
    int _warmupMinutes;
    int _intervalSeconds;
    std::atomic<bool> _isCapturing{false};
    std::mutex _mtx;
    std::condition_variable _cv;
    std::future<void> _task;
    bool _userSetWarmupTime{false};
    bool _userSetSampleInterval{false};
};

} // namespace

int main(int argc, char** argv)
{
    std::signal(SIGINT, handleInterrupt);
    CommandLineArgs args = parseArguments(argc, argv);
    AreaWarmUp warmup(args.warmupTimeMinutes, args.sampleIntervalSeconds);
    warmup.setUserSetWarmupTimeFlag(args.userSetWarmupTime);
    warmup.setUserSetSampleIntervalFlag(args.userSetSampleInterval);
    if (!warmup.init())
        return 1;
    warmup.run();
    return 0;
}
