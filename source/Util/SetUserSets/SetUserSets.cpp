#include "MechEyeApi.h"
#include "SampleUtil.h"
#include <iostream>

int main()
{
    mmind::api::MechEyeDevice device;
    if (!findAndConnect(device))
        return -1;

    std::vector<std::string> userSets;
    showError(device.getAllUserSets(userSets));

    std::cout << "All user sets : ";
    for (size_t i = 0; i < userSets.size(); i++)
        std::cout << userSets[i] << "  ";
    std::cout << std::endl;

    std::string currentUserSet;
    showError(device.getCurrentUserSet(currentUserSet));
    std::cout << "Current user set : " << currentUserSet << std::endl;

    showError(device.setCurrentUserSet(userSets.front()));
    std::cout << "Set \"" << userSets.front() << "\" as the current user set." << std::endl
              << std::endl;

    showError(device.saveAllSettingsToUserSets());
    std::cout << "save all parammeters to current user set." << std::endl;

    device.disconnect();
    std::cout << "Disconnect Mech-Eye Success." << std::endl;
    return 0;
}
