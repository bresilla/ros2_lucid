#pragma once

#include <string>
#include <unordered_map>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <fstream>
#include <nlohmann/json.hpp>
using json = nlohmann::json;

namespace camset {
    const int GAIN = 1;
    const float EXP = 1.2;

    struct Data {
        std::string name;       // Camera name
        int group;              // Switch
        int scpd;               // StreamChannelPacketDelay
        double scftd;           // StreamChannelFrameTransmissionDelay
    };

    inline std::unordered_map<uint64_t, Data> by_mac = {
        {30853686642969, {"9b",    1,   240000,    0 * 80000}},
        {30853686643065, {"9a",    1,   240000,    1 * 80000}},
        {30853686646563, {"9c",    1,   240000,    2 * 80000}},
        {30853686653149, {"9d",    1,   240000,    3 * 80000}},
        {30853686643056, {"9e",    2,   240000,    0 * 80000}},
        {30853686646554, {"7a",    2,   240000,    1 * 80000}},
        {30853686653140, {"7e",    2,   240000,    2 * 80000}},
        {30853686445113, {"7c",    2,   240000,    3 * 80000}},
        {30853686646528, {"7d",    3,   240000,    0 * 80000}},
        {30853686652294, {"7b",    3,   240000,    2 * 80000}},
        {30853686643187, {"1",     3,   240000,    1 * 80000}},
        {30853686650340, {"4",     3,   240000,    3 * 80000}},
        {30853686646397, {"3",     4,   240000,    0 * 80000}},
        {30853686643152, {"12",    4,   240000,    1 * 80000}},
        {30853686646406, {"10",    4,   240000,    2 * 80000}},
        {30853686643161, {"13",    0,   240000,    0 * 80000}}, // directly
        {30853686650366, {"11b",   0,   240000,    0 * 80000}}, // directly
        {30853686650497, {"11a",   0,   240000,    0 * 80000}}  // directly
    };

    inline std::map<std::string, uint64_t> by_name = {
        {"9b", 30853686642969},
        {"9a", 30853686643065},
        {"9c", 30853686646563},
        {"9d", 30853686653149},
        {"9e", 30853686643056},
        {"7a", 30853686646554},
        {"7e", 30853686653140},
        {"7c", 30853686445113},
        {"7d", 30853686646528},
        {"7b", 30853686652294},
        {"1",  30853686643187},
        {"4",  30853686650340},
        {"3",  30853686646397},
        {"12", 30853686643152},
        {"10", 30853686646406},
        {"13", 30853686643161},
        {"11b",30853686650366},
        {"11a",30853686650497}
    };


    inline std::map<uint64_t, std::string> get_from_json() {
        std::string path = ament_index_cpp::get_package_share_directory("ros2_lucid") + "/config/camera.json";
        std::ifstream ifs(path);
        json j = json::parse(ifs);
        std::map<uint64_t, std::string> res;
        for (auto& [mac, name] : j.items()) {

        //     res[std::stoull(mac)] = name;
        }
        return res;
    }
}