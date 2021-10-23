#include <stdio.h>
#include "ros/ros.h"
#include "nav_msgs/MapMetaData.h"
#include "nav_msgs/GetMap.h"
#include "Astar.h"
#include <visualization_msgs/Marker.h>
#include "commute/plan_path.h"
#define MAP_IDX(sx, i, j) ((sx) * (j) + (i))

Astar astar;
void recv(const nav_msgs::OccupancyGrid &map);
bool plan_path(commute::plan_pathRequest &req,
               commute::plan_pathResponse &res);

int main(int argc, char **argv)

{
  ros::init(argc, argv, "plan_server");
  ros::NodeHandle n;
  ros::ServiceServer service = n.advertiseService("plan_path", plan_path);
  ros::Subscriber sub = n.subscribe("map", 10, recv);
  ros::Rate loop_rate(10);

  astar.map_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  while (1)

  {
    ros::spinOnce();
    loop_rate.sleep();
  }
}

void recv(const nav_msgs::OccupancyGrid &map)
{

  for (int x = 0; x < 1984; x++)
  {
    for (int y = 0; y < 1984; y++)
    {
      astar.map[x][y] = map.data[MAP_IDX(map.info.width, x, y)];
    }
  }
}
bool plan_path(commute::plan_pathRequest &req,
               commute::plan_pathResponse &res)
{
  if (req.a == 1)
  {
    int stx = 0;
    int sty = 0;
    int edx = 0;
    int edy = 0;
    stx = static_cast<int>(req.stx * 20);
    sty = static_cast<int>(req.sty * 20);
    edx = static_cast<int>(req.edx * 20);
    edy = static_cast<int>(req.edy * 20);
    stx += 1000;
    sty += 1000;
    edx += 1000;
    edy += 1000;
    astar.edx = edx;
    astar.edy = edy;
    //转换为整数

    //转换为整数

    Node *startPos = new Node(stx, sty); //这里需要转换为整数
    Node *endPos = new Node(edx, edy);
    astar.marker.header.stamp = ros::Time::now();
    astar.target.header.stamp = ros::Time::now();
    astar.search(startPos, endPos);

    astar.darw_result();
    //赋值给res
    res.posx.resize(astar.cnt0);
    res.posy.resize(astar.cnt0);
    int i = 0;
    res.max_num = astar.cnt0;
    astar.cnt0--;

    for (astar.cnt0; astar.cnt0 >= 0; astar.cnt0--)
    {
      res.posx[i] = 0.05 * (astar.path_saver[0][astar.cnt0] - 1000);
      res.posy[i] = 0.05 * (astar.path_saver[1][astar.cnt0] - 1000);

      i = i + 1;
    }
    astar.reset();
  }

  else
  {
  }
  return true;
}
