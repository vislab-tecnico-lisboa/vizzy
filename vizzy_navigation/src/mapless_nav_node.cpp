#include <mapless_nav/mapless_navigator.hpp>


int main(int argc, char **argv)
{

    ros::init(argc, argv, "mapless_navigation_node");
    ros::NodeHandle nh;

    MaplessNavigator mapless_navigator(nh);


    ros::Rate rate(50);

    while (ros::ok())
    {
        ros::spinOnce();
        try
        {
            mapless_navigator.doControlBase();
        }
        catch(const std::exception& e)
        {
            std::cerr << e.what() << '\n';
        }
        
        rate.sleep();
    }

 

    return 0;
}


