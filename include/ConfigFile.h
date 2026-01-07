#pragma once

#include "CompileConfig.h"
#include "CommonDataType.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <unordered_map>
#include <string>
#include "log_manager.h"

namespace NoMapEFM{

class IniFile {
public:
    bool load(const std::string& filename) {
        std::ifstream file(filename);
        if (!file.is_open()) {
            std::cerr << "Failed to open file: " << filename << std::endl;
            return false;
        }

        std::string line;
        std::string currentSection;
        while (std::getline(file, line)) {
            line.erase(0, line.find_first_not_of(" \t\n\r"));
            line.erase(line.find_last_not_of(" \t\n\r") + 1);

            if (line.empty() || line[0] == ';' || line[0] == '#') {
                continue;
            }

            if (line[0] == '[' && line.back() == ']') {
                currentSection = line.substr(1, line.size() - 2);
            } else {
                std::istringstream iss(line);
                std::string key, value;
                if (std::getline(iss, key, '=') && std::getline(iss, value)) {
                    if (str_trim(key) == false || str_trim(value) == false) {
                        continue;
                    }
                    data[currentSection][key] = value;
                }
            }
        }
        return true;
    }

    std::string getValue(const std::string& section, const std::string& key) {
        if (data.find(section) == data.end()) {
            LOG_ERROR(std::string("not find! section: ") + section +std::string(", key: ") + key);
            return "0";
        }
        return data[section][key];
    }

    bool str_trim(std::string& str_org) {
        size_t start = str_org.find_first_not_of(" \t\n\r");
        if (start == std::string::npos) {
            return false;
        }
        size_t end = str_org.find_last_not_of(" \t\n\r");
        str_org = str_org.substr(start, end - start + 1);
        return true;
    }

private:
    std::unordered_map<std::string, std::unordered_map<std::string, std::string>> data;
};

}
