#pragma once

#include <string>
#include <unordered_map>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <fmt/format.h>

#include <fstream>
#include <nlohmann/json.hpp>
using json = nlohmann::json;

namespace camera {
    struct Data {
        std::string name;       // Camera name
        int group;              // Switch
        int scpd;               // StreamChannelPacketDelay
        double scftd;           // StreamChannelFrameTransmissionDelay
    };

    inline std::map<uint64_t, Data> by_mac() {
        std::string path = ament_index_cpp::get_package_share_directory("ros2_lucid") + "/config/cameras.json";
        std::ifstream ifs(path);
        json j = json::parse(ifs);
        std::map<uint64_t, Data> res;

        std::map<std::string, int8_t> new_group;
        for (auto& cam : j.at("cameras").items()){
            auto iface = cam.value().at("iface").get<std::string>();
            if (new_group.find(iface) == new_group.end()) {
                new_group[iface] = 1;
            } else {
                new_group[iface] = new_group[iface] + 1;
            }
        }
        std::map<std::string, int8_t> cloned_group = new_group;
        for (auto& cam : j.at("cameras").items()) {
            auto name = cam.value().at("name").get<std::string>();
            auto mac = atoi(cam.key().c_str());
            auto iface = cam.value().at("iface").get<std::string>();
            auto scpd = (new_group[iface] - 1) * 80000;
            cloned_group[iface]--;
            auto scftd = (cloned_group[iface]) * 80000;
            Data data = {name, new_group[iface], scpd, scftd};
            res[mac] = data;
        }
        return res;
    }

    inline std::map<std::string, uint64_t> by_name() {
        std::string path = ament_index_cpp::get_package_share_directory("ros2_lucid") + "/config/cameras.json";
        std::ifstream ifs(path);
        json j = json::parse(ifs);
        std::map<std::string, uint64_t> res;

        for (auto& cam : j.at("cameras").items()){
            auto name = cam.value().at("name").get<std::string>();
            auto mac = atoi(cam.key().c_str());
            res[name] = mac;
        }
        return res;
    }
}