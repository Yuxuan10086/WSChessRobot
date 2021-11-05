#include<ros/ros.h>
#include<string>
#include<geometry_msgs/Point.h>
int column, row, showGridWidth;
// 可落子位置为(0, 0)到(column, row)，不包括四个顶角
void callback(const geometry_msgs::Point::ConstPtr &msg, int color)
{
    if(int(msg->x * 10) % 10 != 0 || int(msg->y * 10) % 10 != 0 || msg->x > column || msg->y > row || msg->x < -2 || msg->y < -2)
        throw "Illegal input";
    if(int(msg->x) == -1 && int(msg->y) != -1)
        throw "Illegal input";
    if(int(msg->x) == -2 && int(msg->y) != -2)
        throw "Illegal input";
    ros::param::set("piece" + std::to_string(int(msg->x)) + '_' + std::to_string(int(msg->y)), color);
}
void aiCallback(const geometry_msgs::Point::ConstPtr &msg){ callback(msg, 1); }
void humanCallback(const geometry_msgs::Point::ConstPtr &msg){ callback(msg, -1); }
int main(int argc, char **argv)
{
    ros::init(argc, argv, "game");
    ros::NodeHandle nh;
    ros::param::get("column", column);
    ros::param::get("row", row);
    ros::param::get("showGridWidth", showGridWidth);
    for(int y = 0; y <= column; y++)
        for(int x = 0; x <= row; x++)
            ros::param::set("piece" + std::to_string(x) + '_' + std::to_string(y), 0);

    ros::Subscriber ai_sub = nh.subscribe("ai", 1000, aiCallback);
    ros::Subscriber human_sub = nh.subscribe("human", 1000, humanCallback);
    ros::spin();

    return 0;
}