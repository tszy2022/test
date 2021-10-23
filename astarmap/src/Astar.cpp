#include "Astar.h"
Astar::Astar()
{
marker.type = visualization_msgs::Marker::POINTS;
marker.header.frame_id = "/map";
//marker.header.stamp = ros::Time::now();
marker.pose.orientation.w = 1.0;
marker.scale.x = 0.05;
marker.scale.y = 0.05;
marker.scale.z = 0.1;
marker.color.g = 1;
marker.color.a = 1;
marker.lifetime = ros::Duration(60);
objmarker.z=0;

  target.ns ="mark_target";
   marker.ns = "marker_traj";
            marker.action = target.action = visualization_msgs::Marker::ADD;
			target.id = 0;
			marker.id=1;
target.type = visualization_msgs::Marker::POINTS;
target.header.frame_id = "/map";
//target.header.stamp = ros::Time::now();

target.pose.orientation.w = 1.0;
target.scale.x = 0.05;
target.scale.y = 0.05;
target.scale.z = 0.1;
target.color.g = 0;
target.color.b=1.0;
target.color.a = 1;
target.lifetime = ros::Duration(60);
objtarget.z=0.1;
}
Astar::~Astar()
{
}
void Astar::search( Node* startPos,Node* endPos )
{
	if (startPos->x < 0 || startPos->x > row || startPos->y < 0 || startPos->y >col
		||
		endPos->x < 0 || endPos->x > row || endPos->y < 0 || endPos->y > col)
		return ;
	Node* current;
	this->startPos = startPos;
	this->endPos = endPos;
	openList.push_back(startPos);
	//主要是这块，把开始的节点放入openlist后开始查找旁边的8个节点，如果坐标超长范围或在closelist就return 如果已经存在openlist就对比当前节点到遍历到的那个节点的G值和当前节点到原来父节点的G值 如果原来的G值比较大 不用管 否则重新赋值G值 父节点 和f 如果是新节点 加入到openlist直到opellist为空或找到终点
	while(openList.size() > 0)
	{
		current = openList[0];
		if (current->x == endPos->x && current->y == endPos->y)
		{
			cout<<"find the path"<<endl;
		//	printMap();
			printPath(current);
			openList.clear();
			closeList.clear();
            //reset()
            //cnt0=0;
            //cnt1=0;

            //reset()
			break;
		}
		NextStep(current);
		closeList.push_back(current);
		openList.erase(openList.begin());
		sort(openList.begin(),openList.end(),compare);
	}
}
void Astar::checkPoit( int x,int y,Node* father,int g)
{
	if (x < 0 || x > row || y < 0 || y > col)
		return;
	if (this->unWalk(x,y))
		return;
	if (isContains(&closeList,x,y) != -1)
		return;
	int index;
	if ((index = isContains(&openList,x,y)) != -1)
	{
		Node *point = openList[index];
		if (point->g > father->g + g)
		{
			point->father = father;
			point->g = father->g + g;
			point->f = point->g + point->h;
		}
	}
	else
	{
		Node * point = new Node(x,y,father);
		countGHF(point,endPos,g);
		openList.push_back(point);
	}
}
void Astar::NextStep( Node* current )
{
	checkPoit(current->x - 1,current->y,current,WeightW);//左
	checkPoit(current->x + 1,current->y,current,WeightW);//右
	checkPoit(current->x,current->y + 1,current,WeightW);//上
	checkPoit(current->x,current->y - 1,current,WeightW);//下
	checkPoit(current->x - 1,current->y + 1,current,WeightWH);//左上
	checkPoit(current->x - 1,current->y - 1,current,WeightWH);//左下
	checkPoit(current->x + 1,current->y - 1,current,WeightWH);//右下
	checkPoit(current->x + 1,current->y + 1,current,WeightWH);//右上
}
int Astar::isContains(vector<Node*>* Nodelist, int x,int y )
{
	for (int i = 0;i < Nodelist->size();i++)
	{
		if (Nodelist->at(i)->x == x && Nodelist->at(i)->y == y)
		{
			return i;
		}
	}
	return -1;
}
void Astar::countGHF( Node* sNode,Node* eNode,int g)
{
	int h = abs(sNode->x - eNode->x) * WeightW + abs(sNode->y - eNode->y) * WeightW;
	int currentg = sNode->father->g + g;
	int f = currentg + h;
	sNode->f = f;
	sNode->h = h;
	sNode->g = currentg;
}
bool Astar::compare( Node* n1,Node* n2 )
{
	//printf("%d,%d",n1->f,n2->f);
	return n1->f < n2->f;
}
bool Astar::unWalk( int x,int y)
{
	//if (map[x][y] == 100 || map[x][y] == -1)
	if (map[x][y] == 100 )
		return true;
	return false;
}
void Astar::printPath( Node* current )
{
    cnt0++;
    cnt1++;
	if (current->father != NULL)
    {
		printPath(current->father);
    }
	cnt0--;
    path_saver[0][cnt0]=current->x;
    path_saver[1][cnt0]=current->y;
	//printf("(%d,%d)",current->x,current->y);
}
void Astar::printMap()
{
	for(int i=0;i<=row;i++){
		for(int j=0;j<=col;j++){
			printf("%d ",map[i][j]);
		}
		printf("\n");
	}
}
void Astar::reset()
{
	vector<Node*> rqt;
	openList=rqt;
	closeList=rqt;
	cnt0=0;
	cnt1=0;
}
void Astar::darw_result()

{
int cnt10=cnt1;
cnt0=cnt1;
//printf("cnt0  cnt1 num:%d %d \n",cnt0,cnt1);
cnt10--;
ros::Rate loop_rate(60);
  for (cnt10; cnt10 >=0; cnt10--)
  {
      objmarker.x=0.05*(path_saver[0][cnt10]-1000);
	  objmarker.y=0.05*(path_saver[1][cnt10]-1000);
	  objmarker.z=0;
	  //printf("objmarker.x= %f objmarker.y= %f pathx:%d pathy:%d  \n",objmarker.x,objmarker.y,path_saver[0][cnt10],path_saver[1][cnt10]);
	  marker.points.push_back(objmarker);
	  map_pub.publish(marker);
	 loop_rate.sleep();
    
  }
      objtarget.x=0.05*(edx-1000);
	  objtarget.y=0.05*(edy-1000);
	   printf("edx %d edy %d\n",edx,edy);
	  objtarget.z=0.2;
	  target.points.push_back(objtarget);
	  map_pub.publish(target);

}