#pragma once

#include <string>
#include <unordered_map>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <fmt/format.h>

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


    inline std::map<uint64_t, std::string> cam_list() {
        std::string path = ament_index_cpp::get_package_share_directory("ros2_lucid") + "/config/cameras.json";
        std::ifstream ifs(path);
        json j = json::parse(ifs);
        std::map<uint64_t, std::string> res;
        // fmt::print("{}", j.at("cameras").dump());
        for (auto& [key, value] : j.at("cameras").items()) {
            res[atoi(key.c_str())] = value;
        }
        return res;
    }
    
    inline std::map<uint64_t, Data> cam_list_anv() {
        std::string path = ament_index_cpp::get_package_share_directory("ros2_lucid") + "/config/cameras.json";
        std::ifstream ifs(path);
        json j = json::parse(ifs);

        // std::vector<std::pair<std::string, int8_t>> groups;

        // for (auto& cam : j.at("cams").items()) {
        //     auto iface = cam.value().at("iface").get<std::string>();
        //     if (std::find_if(groups.begin(), groups.end(), [&iface](const std::pair<std::string, int8_t>& p) { return p.first == iface; }) == groups.end()) {
        //         groups.push_back({iface, 1});
        //     } else {
        //         auto it = std::find_if(groups.begin(), groups.end(), [&iface](const std::pair<std::string, int8_t>& p) { return p.first == iface; });
        //         it->second++;
        //     }
        // }

        std::map<std::string, int8_t> new_group;
        for (auto& cam : j.at("cams").items()){
            //ig goups[i].first not in new_group add it
            //else increment the value
            auto iface = cam.value().at("iface").get<std::string>();
            if (new_group.find(iface) == new_group.end()) {
                new_group[iface] = 1;
            } else {
                new_group[iface]++;
            }
        }

        for (auto& cam : j.at("cams").items()) {
            auto name = cam.value().at("name").get<std::string>();
            auto mac = cam.key();
            auto iface = cam.value().at("iface").get<std::string>();
            // auto group = std::find_if(groups.begin(), groups.end(), [&iface](const std::pair<std::string, int8_t>& p) { return p.first == iface; })->second;
            auto group = new_group[iface];
            auto scpd = (group - 1) * 80000;
            auto scftd = scpd * 1.2;

            fmt::print("iface: {}, mac: {}, name: {}, group: {}, scpd: {}\n", iface, mac, name, group, scpd);
        }


        std::map<uint64_t, Data> res;
        return res;
    }
}