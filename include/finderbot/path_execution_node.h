#include <finderbot/planner.h>
#include <finderbot/map_utils.h>
#include <math.h>

class Executor
{
int current_row_;
int current_col_;
double current_theta_;
geometry_msgs::Twist cmd_;

  public:
	Executor();
	~Executor();
	
};