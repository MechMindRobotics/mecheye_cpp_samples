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
With this sample, you can periodically trigger data acquisition with signals input from an external
device at a fixed scan rate to warm up the Mech-Eye Profiler, while the acquired data is used only
for stabilization and not saved.
*/

// #include <opencv2/imgcodecs.hpp>
#define NOMINMAX
#include <iostream>
#include <iomanip>
#include <thread>
#include <chrono>
#include <atomic>
#include <csignal>
#include <mutex>
#include <limits>
#include <random>
#include <future>
#include <sstream>
#include <algorithm>
#include "profiler/Profiler.h"
#include "profiler/api_util.h"
#include "profiler/parameters/ProfileProcessingParameters.h"
#include "profiler/parameters/ProfileExtractionParameters.h"
#include "profiler/parameters/RawImageParameters.h"
#include "profiler/parameters/ScanParameters.h"

namespace {
std::atomic<bool> kStopWarmup{false};
const int kMinScanLineNum = 1;
const int kMaxScanLineNum = 20000;
constexpr int kDefaultWarmupMinutes = 30;
constexpr int kDefaultIntervalSeconds = 5;
constexpr int kMinWarmupMinutes = 30;
constexpr int kMaxWarmupMinutes = 90;
constexpr int kMinIntervalSeconds = 3;
constexpr int kMaxIntervalSeconds = 30;
std::condition_variable* g_captureCvPtr = nullptr;

void handleInterrupt(int)
{
    std::cout << "\n[Interrupt] Ctrl+C received. Preparing to stop warmup..." << std::endl;
    kStopWarmup.store(true);
    if (g_captureCvPtr)
        g_captureCvPtr->notify_all();
}
} // namespace

class WarmUp
{
public:
    WarmUp(int warmupMinutes, int intervalSeconds)
        : _warmupMinutes(warmupMinutes), _intervalSeconds(intervalSeconds)
    {
    }
    bool init()
    {
        if (!findAndConnect(_profiler)) {
            std::cerr << "[Error] Failed to connect to profiler." << std::endl;
            return false;
        }

        if (!confirmCapture()) {
            return false;
        }

        _isConnected = true;
        if (!switchToWarmupUserSet() || !setWarmupAutoParams()) {
            return false;
        }

        if (!interactiveSetScanParams()) {
            return false;
        }

        auto status = _profiler.registerAcquisitionCallback(
            [this](const mmind::eye::ProfileBatch& batch, void* pUser) {
                this->callbackFunc(pUser);
            },
            this);
        if (!status.isOK()) {
            showError(status);
            return false;
        }
        return true;
    }
    void run()
    {
        std::cout << "LNX Profiler warmup starting. Warmup time: " << _warmupMinutes
                  << " minutes, sample interval: " << _intervalSeconds << " seconds." << std::endl;
        std::cout << "[Info] Warmup loop started..." << std::endl;
        auto status = _profiler.startAcquisition();
        if (!status.isOK()) {
            showError(status);
            return;
        }

        executeWarmUpLoop();

        auto stopStatus = _profiler.stopAcquisition();
        if (!stopStatus.isOK()) {
            showError(stopStatus);
        }
        std::cout << "LNX Profiler warmup finished." << std::endl;
    }
    std::condition_variable& getConditionVariable() { return _captureCv; }
    ~WarmUp() { cleanup(); }

private:
    bool interactiveSetScanParams()
    {
        std::cout << "[Info] Entering interactive scan parameter input..." << std::endl;

        auto userSet = _profiler.currentUserSet();

        int currentScanLineCount = 0;
        double currentSoftwareTriggerRate = 0.0;

        {
            auto status = userSet.getIntValue(mmind::eye::scan_settings::ScanLineCount::name,
                                              currentScanLineCount);
            if (!status.isOK()) {
                showError(status);
                return false;
            }

            status = userSet.getFloatValue(mmind::eye::trigger_settings::SoftwareTriggerRate::name,
                                           currentSoftwareTriggerRate);
            if (!status.isOK()) {
                showError(status);
                return false;
            }
        }

        auto status = userSet.getFloatValue(mmind::eye::trigger_settings::MaxScanRate::name,
                                            _maxSoftwareTriggerRate);
        if (!status.isOK()) {
            showError(status);
            return false;
        }

        std::string input;
        int scanLineCount = currentScanLineCount;
        float softwareTriggerRate = currentSoftwareTriggerRate;
        while (true) {
            std::cout << "Please input scan line count (current: " << currentScanLineCount
                      << ", range: " << kMinScanLineNum << "-" << kMaxScanLineNum << "): ";
            std::cin.clear();
            std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
            std::getline(std::cin, input);
            if (input.empty()) {
                scanLineCount = currentScanLineCount;
                std::cout << "[Info] Using current scan line count: " << scanLineCount << std::endl;
                break;
            }
            try {
                scanLineCount = std::stoi(input);
                if (scanLineCount >= kMinScanLineNum && scanLineCount <= kMaxScanLineNum) {
                    break;
                } else {
                    std::cout << "[Error] Scan line count must be in range " << kMinScanLineNum
                              << "–" << kMaxScanLineNum << "." << std::endl;
                }
            } catch (...) {
                std::cout << "[Error] Invalid input. Please enter a valid integer." << std::endl;
            }
        }

        while (true) {
            std::cout << "Please input software trigger rate (Hz, current: "
                      << currentSoftwareTriggerRate << ", range: " << _minSoftwareTriggerRate << "-"
                      << _maxSoftwareTriggerRate << "): ";
            std::getline(std::cin, input);
            if (input.empty()) {
                softwareTriggerRate = currentSoftwareTriggerRate;
                std::cout << "[Info] Using current scan frequency: " << softwareTriggerRate << " Hz"
                          << std::endl;
                break;
            }
            try {
                softwareTriggerRate = std::stof(input);
                if (softwareTriggerRate >= _minSoftwareTriggerRate &&
                    softwareTriggerRate <= _maxSoftwareTriggerRate) {
                    break;
                } else {
                    std::cout << "[Error] Software trigger rate must be in range "
                              << _minSoftwareTriggerRate << "–" << _maxSoftwareTriggerRate << "."
                              << std::endl;
                }
            } catch (...) {
                std::cout << "[Error] Invalid input. Please enter a valid number." << std::endl;
            }
        }

        std::cout << "[Info] Received parameters: scanLineCount = " << scanLineCount
                  << ", softwareTriggerRate = " << softwareTriggerRate << " Hz" << std::endl;

        auto setStatus1 =
            userSet.setIntValue(mmind::eye::scan_settings::ScanLineCount::name, scanLineCount);
        if (!setStatus1.isOK()) {
            showError(setStatus1);
            return false;
        }

        auto setStatus2 = userSet.setFloatValue(
            mmind::eye::trigger_settings::SoftwareTriggerRate::name, softwareTriggerRate);
        if (!setStatus2.isOK()) {
            showError(setStatus2);
            return false;
        }

        return true;
    }
    bool setWarmupAutoParams()
    {
        std::cout << "[Info] Setting fixed frequency trigger, software trigger, single frame mode "
                     "for warmup."
                  << std::endl;
        auto userSet = _profiler.currentUserSet();
        auto status = userSet.setEnumValue(
            mmind::eye::trigger_settings::LineScanTriggerSource::name,
            static_cast<int>(
                mmind::eye::trigger_settings::LineScanTriggerSource::Value::FixedRate));
        if (!status.isOK()) {
            showError(status);
            return false;
        }

        status = userSet.setEnumValue(
            mmind::eye::trigger_settings::DataAcquisitionTriggerSource::name,
            static_cast<int>(
                mmind::eye::trigger_settings::DataAcquisitionTriggerSource::Value::Software));
        if (!status.isOK()) {
            showError(status);
            return false;
        }

        status = userSet.setEnumValue(
            mmind::eye::trigger_settings::DataAcquisitionMethod::name,
            static_cast<int>(
                mmind::eye::trigger_settings::DataAcquisitionMethod::Value::Frame_Based));
        if (!status.isOK()) {
            showError(status);
            return false;
        }

        return true;
    }
    bool captureSingleFrame()
    {
        std::lock_guard<std::mutex> lock(_captureMutex);
        auto status = _profiler.triggerSoftware();
        if (!status.isOK()) {
            showError(status);
            return false;
        }
        return true;
    }
    bool waitForNextCaptureOrStop()
    {
        const int pollIntervalMs = 100;
        int waitedMs = 0;
        const int totalWaitMs = _intervalSeconds * 1000;

        while (!kStopWarmup.load() && waitedMs < totalWaitMs) {
            std::this_thread::sleep_for(std::chrono::milliseconds(pollIntervalMs));
            waitedMs += pollIntervalMs;
        }
        return kStopWarmup.load();
    }

    void finishCaptureWithError(const char* msg)
    {
        std::cerr << msg << std::endl;
        finishCaptureSuccessfully();
    }

    void finishCaptureSuccessfully()
    {
        std::lock_guard<std::mutex> lock(_captureMutex);
        _isCapturing = false;
        _captureCv.notify_one();
    }

    void stopAcquisitionAndPrintInfo()
    {
        auto stopStatus = _profiler.stopAcquisition();
        if (!stopStatus.isOK()) {
            showError(stopStatus);
        } else {
            std::cout << "[Info] Acquisition stopped." << std::endl;
        }
    }

    void performCapture()
    {
        try {
            if (!captureSingleFrame()) {
                std::cerr << "[Error] Failed to trigger capture." << std::endl;
            } else {
                std::cout << "[Info] Capture triggered." << std::endl;
            }
        } catch (...) {
            std::cerr << "[Exception] Unknown error during capture." << std::endl;
        }
        finishCaptureSuccessfully();
    }

    bool tryStartCapture()
    {
        std::lock_guard<std::mutex> lock(_captureMutex);
        if (_isCapturing) {
            std::cout << "[Warning] Last capture not finished, skip this cycle." << std::endl;
            return false;
        }
        _isCapturing = true;

        _task = std::async(std::launch::async, [this]() { performCapture(); });

        return true;
    }

    void executeWarmUpLoop()
    {
        auto startTime = std::chrono::steady_clock::now();
        auto deadline = startTime + std::chrono::minutes(_warmupMinutes);

        while (!kStopWarmup.load() && std::chrono::steady_clock::now() < deadline) {
            if (!tryStartCapture()) {
                continue;
            }

            if (waitForNextCaptureOrStop()) {
                break;
            }
        }
        if (_task.valid()) {
            _task.wait();
        }
    }

    void cleanup()
    {
        try {
            if (!_originalUserSet.empty()) {
                _profiler.userSetManager().selectUserSet(_originalUserSet);
                _profiler.userSetManager().deleteUserSet(_warmupUserSet);
                std::cout << "[Info] User set restored to: " << _originalUserSet << std::endl;
            }
        } catch (...) {
            std::cerr << "[Error] Failed to restore user set." << std::endl;
        }
        if (_isConnected)
            _profiler.disconnect();
        std::cout << "[Info] Profiler disconnected." << std::endl;
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

    bool switchToWarmupUserSet()
    {
        auto status = _profiler.currentUserSet().getName(_originalUserSet);
        if (!status.isOK()) {
            showError(status);
            return false;
        }
        std::cout << "[Info] Original user set: " << _originalUserSet << std::endl;

        std::vector<std::string> userSetNames;
        if (!_profiler.userSetManager().getAllUserSetNames(userSetNames).isOK()) {
            std::cerr << "[Warning] Failed to get all user set names.\n";
            return false;
        }

        _warmupUserSet = createWarmupGroupName(userSetNames);
        if (!_profiler.userSetManager().selectUserSet("calib").isOK()) {
            std::cerr << "[Warning] Failed to switch to user set : calib.\n";
            return false;
        }

        if (!_profiler.userSetManager().addUserSet(_warmupUserSet).isOK()) {
            std::cerr << "[Warning] Failed to add to user set :" << _warmupUserSet << ".\n";
            return false;
        }

        if (!_profiler.userSetManager().selectUserSet(_warmupUserSet).isOK()) {
            std::cerr << "[Warning] Failed to switch to user set '" << _warmupUserSet << "'.\n";
            return false;
        }
        return true;
    }
    void callbackFunc(void* pUser)
    {
        int currentSample = ++_sampleCount;
        auto* profiler = static_cast<mmind::eye::Profiler*>(pUser);
        if (!profiler) {
            std::cerr << "[Error] Profiler pointer invalid in callback." << std::endl;
            return;
        }

        mmind::eye::ProfilerStatus profilerStatus;
        auto status = profiler->getProfilerStatus(profilerStatus);
        if (status.isOK()) {
            int totalSeconds = _warmupMinutes * 60;
            int elapsedSeconds = currentSample * _intervalSeconds;
            double progress = std::min(100.0, (double)elapsedSeconds / totalSeconds * 100.0);

            auto now_time_t =
                std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
            std::cout << "------------------------------" << std::endl;
            std::cout << "[" << std::put_time(std::localtime(&now_time_t), "%F %T")
                      << "] Warmup capture #" << currentSample << " (Progress: " << std::fixed
                      << std::setprecision(1) << progress << "%) Completed" << std::endl;

            printProfilerStatus(profilerStatus);
            std::cout << "------------------------------" << std::endl;
        }
        {
            std::lock_guard<std::mutex> lock(_captureMutex);
            _isCapturing = false;
        }
        _captureCv.notify_one();
    }

private:
    mmind::eye::Profiler _profiler;
    std::atomic<bool> _isCapturing{false};
    std::mutex _captureMutex;
    std::condition_variable _captureCv;
    std::string _originalUserSet;
    std::string _warmupUserSet;
    std::atomic<int> _sampleCount{0};
    bool _isConnected{false};
    int _warmupMinutes;
    int _intervalSeconds;

    const double _minSoftwareTriggerRate = 2.0;
    double _maxSoftwareTriggerRate = 0.0;
    std::future<void> _task;
};

struct CommandLineArgs
{
    int warmupTimeMinutes = kDefaultWarmupMinutes;
    int sampleIntervalSeconds = kDefaultIntervalSeconds;
};
CommandLineArgs parseArguments(int argc, char* argv[])
{
    CommandLineArgs args;

    auto isNumber = [](const std::string& s) {
        return !s.empty() && std::all_of(s.begin(), s.end(), ::isdigit);
    };

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];

        if ((arg == "-w" || arg == "--warmup-time") && i + 1 < argc) {
            std::string value = argv[++i];
            if (!isNumber(value)) {
                std::cerr << "[Error] Invalid value for " << arg << ": must be an integer.\n";
                std::exit(1);
            }
            int val = std::stoi(value);
            if (val < kMinWarmupMinutes || val > kMaxWarmupMinutes) {
                std::cerr << "[Error] Warmup time must be between " << kMinWarmupMinutes << " and "
                          << kMaxWarmupMinutes << " minutes.\n";
                std::exit(1);
            }
            args.warmupTimeMinutes = val;
        } else if ((arg == "-i" || arg == "--sample-interval") && i + 1 < argc) {
            std::string value = argv[++i];
            if (!isNumber(value)) {
                std::cerr << "[Error] Invalid value for " << arg << ": must be an integer.\n";
                std::exit(1);
            }
            int val = std::stoi(value);
            if (val < kMinIntervalSeconds || val > kMaxIntervalSeconds) {
                std::cerr << "[Error] Sample interval must be between " << kMinIntervalSeconds
                          << " and " << kMaxIntervalSeconds << " seconds.\n";
                std::exit(1);
            }
            args.sampleIntervalSeconds = val;
        } else if (arg == "-h" || arg == "--help") {
            std::cout
                << "Usage:\n"
                << "  " << argv[0] << " [options]\n"
                << "Options:\n"
                << "  -w, --warmup-time <minutes>     Set warmup duration (30–90 minutes, default: "
                << kDefaultWarmupMinutes << ")\n"
                << "  -i, --sample-interval <seconds> Set sample interval (3–30 seconds, default: "
                << kDefaultIntervalSeconds << ")\n"
                << "  -h, --help                      Show this help message\n";
            std::exit(0);

        } else {
            std::cerr << "[Error] Unknown argument: " << arg << "\n";
            std::cerr << "Use -h or --help to see usage.\n";
            std::exit(1);
        }
    }

    return args;
}

int main(int argc, char** argv)
{
    CommandLineArgs args = parseArguments(argc, argv);
    std::signal(SIGINT, handleInterrupt);
    WarmUp warmup(args.warmupTimeMinutes, args.sampleIntervalSeconds);
    g_captureCvPtr = &(warmup.getConditionVariable());
    if (!warmup.init())
        return -1;
    warmup.run();
    return 0;
}
