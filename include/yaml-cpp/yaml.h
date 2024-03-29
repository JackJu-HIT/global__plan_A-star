#ifndef STD_YAML_CPP_HEADER_H
#define STD_YAML_CPP_HEADER_H

#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <vector>

struct PMHeader {
    std::string image;
    double resolution;
    std::vector<double> origin;
    double occupied_thresh;
    double free_thresh;
    int negate;
};

PMHeader readPMHeader(const std::string& filename) {
    PMHeader header;
    std::ifstream file(filename);
    std::string line;

    while (std::getline(file, line)) {
        std::stringstream ss(line);
        std::string key;
        ss >> key;

        if (key == "image:") {
            std::string value;
            ss >> value;
            header.image = value;
        } else if (key == "resolution:") {
            double value;
            ss >> value;
            header.resolution = value;
        } else if (key == "origin:") {
            std::string values1;
            ss >> values1;  // 读取整个[...]部分
            // 去除 [ 和 ] 字符
            values1.erase(values1.begin());
            values1.pop_back();
           // std::cout << values1 << std::endl;
             std::string values2;
            ss >> values2;  // 读取整个[...]部分
            values2.pop_back();
            std::string values3;
            ss >> values3;
            values3.pop_back();
            //std::cout << values3 << std::endl;
            double value1 = std::stod(values1);
            double value2 = std::stod(values2);
            double value3 = std::stod(values3);
            header.origin.push_back(value1);
            header.origin.push_back(value2);
            header.origin.push_back(value3);

        } else if (key == "occupied_thresh:") {
            double value;
            ss >> value;
            header.occupied_thresh = value;
        } else if (key == "free_thresh:") {
            double value;
            ss >> value;
            header.free_thresh = value;
        } else if (key == "negate:") {
            int value;
            ss >> value;
            header.negate = value;
        }
    }

    file.close();
    return header;
}

#if 0

int main() {
    std::string filename = "PM.yaml";
    PMHeader header = readPMHeader(filename);

    std::cout << "image: " << header.image << std::endl;
    std::cout << "resolution: " << header.resolution << std::endl;
    std::cout << "origin: [";

    for (int i = 0; i < header.origin.size(); ++i) {
        if (i > 0) {
            std::cout << ", ";
        }
        std::cout << header.origin[i];
    }

    std::cout << "]" << std::endl;
    std::cout << "occupied_thresh: " << header.occupied_thresh << std::endl;
    std::cout << "free_thresh: " << header.free_thresh << std::endl;
    std::cout << "negate: " << header.negate << std::endl;

    return 0;
}
#endif
#endif // STD_MSGS_MESSAGE_HEADER_H

