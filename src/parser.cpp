#include "parser.h"

void printUsage (const char* progName)
{
  std::cout << "\n\nUsage: "<<progName<<" [options]\n\n"
            << "Options:\n"
            << "-------------------------------------------\n"
            << "-h           this help\n"
            << "-s           Simple visualisation example\n"
            << "-r           RGB colour visualisation example\n"
            << "-c           Custom colour visualisation example\n"
            << "-n           Normals visualisation example\n"
            << "-a           Shapes visualisation example\n"
            << "-v           Viewports example\n"
            << "-i           Interaction Customization example\n"
            << "\n\n";
}

PARSE_STATUS checkParameters(int argc, char** argv, PARSE_STATUS paserStatus)
{
  if (pcl::console::find_argument (argc, argv, "-h") >= 0)
    {
        printUsage(argv[0]);
        return paserStatus;
    }

    if (pcl::console::find_argument (argc, argv, "-s") >= 0)
    {
        paserStatus.simple = true;
        std::cout << "Simple visualisation example\n";
    }
    else if (pcl::console::find_argument (argc, argv, "-c") >= 0)
    {
        paserStatus.custom_c = true;
        std::cout << "Custom colour visualisation example\n";
    }
    else if (pcl::console::find_argument (argc, argv, "-r") >= 0)
    {
        paserStatus.rgb = true;
        std::cout << "RGB colour visualisation example\n";
    }
    else if (pcl::console::find_argument (argc, argv, "-n") >= 0)
    {
        paserStatus.normals = true;
        std::cout << "Normals visualisation example\n";
    }
    else if (pcl::console::find_argument (argc, argv, "-a") >= 0)
    {
        paserStatus.shapes = true;
        std::cout << "Shapes visualisation example\n";
    }
    else if (pcl::console::find_argument (argc, argv, "-v") >= 0)
    {
        paserStatus.viewports = true;
        std::cout << "Viewports example\n";
    }
    else if (pcl::console::find_argument (argc, argv, "-i") >= 0)
    {
        paserStatus.interaction_customization = true;
        std::cout << "Interaction Customization example\n";
    }
    else
    {
        printUsage (argv[0]);
        return paserStatus;
    }
    return paserStatus;
}