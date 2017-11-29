#include <planner.h>
#include <finderbot/map_utils.h>

// Test 1: 
// 	 source == goal == (0,0),
// 	 no obstacles

// Test 2:
// 	 source == (0,0),
//   goal == (10,10),
//   no obstacles

// Test 3:
//   source == (0,0),
//   goal == (10,0),
//   wall from (3,3) to (3,-3)