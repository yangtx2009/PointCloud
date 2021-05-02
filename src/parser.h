#ifndef PARSER_
#define PARSER_

#include <iostream>
#include <pcl/console/parse.h>

struct PARSE_STATUS{
    bool simple = true;
    bool rgb = true; 
    bool custom_c = false;
    bool normals = false;
    bool shapes = false; 
    bool viewports = false;
    bool interaction_customization = false;
};

void printUsage (const char* progName);

PARSE_STATUS checkParameters(int argc, char** argv, PARSE_STATUS paserStatus);

#endif