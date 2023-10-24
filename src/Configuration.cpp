#include "Configuration.hpp"
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <fstream>
#include <iostream>

namespace Configuration
{
    double getVariable(std::string variableName) {
        std::string filename = "../config/configuration.xml";
        std::ifstream file(filename);
        std::stringstream buffer;
        buffer << file.rdbuf();
        
        boost::property_tree::ptree pt;
        boost::property_tree::read_xml(buffer, pt);

        for (auto& v : pt.get_child("properties")) {
            if (v.first == "property") {
                if (v.second.get<std::string>("<xmlattr>.name") == variableName) {
                    return v.second.get<double>("<xmlattr>.value");
                }
            }
        }
        std::cout << "Configuration variable not found." << std::endl;
        return 0.0;
    }   
}
