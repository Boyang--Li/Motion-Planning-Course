boyang@boyang-thinkpad-p52s:~/code/Motion-Planning-Course/chap2_ros/src/grid_path_searcher$ git status
On branch master
Your branch is up-to-date with 'origin/master'.
Changes not staged for commit:
  (use "git add/rm <file>..." to update what will be committed)
  (use "git checkout -- <file>..." to discard changes in working directory)

	modified:   CMakeLists.txt
	modified:   include/backward.hpp
	deleted:    include/graph_searcher.h
	modified:   launch/demo.launch
	modified:   launch/rviz_config/demo.rviz
	modified:   src/demo_node.cpp
	deleted:    src/graph_searcher.cpp

Untracked files:
  (use "git add <file>..." to include in what will be committed)

	include/Astar_searcher.h
	include/JPS_searcher.h
	include/JPS_utils.h
	include/node.h
	src/Astar_searcher.cpp
	src/read_only/

no changes added to commit (use "git add" and/or "git commit -a")


boyang@boyang-thinkpad-p52s:~/code/Motion-Planning-Course/chap2_ros/src/grid_path_searcher$ git diff
diff --git a/CMakeLists.txt b/CMakeLists.txt
index 46ff451..a1bc37e 100644
--- a/CMakeLists.txt
+++ b/CMakeLists.txt
@@ -30,7 +30,10 @@ set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS} -O3 -Wall") # -Wextra -Werror
 
 add_executable( demo_node 
     src/demo_node.cpp
-    src/graph_searcher.cpp)
+    src/Astar_searcher.cpp
+    src/read_only/JPS_utils.cpp
+    src/read_only/JPS_searcher.cpp
+    )
 
 target_link_libraries(demo_node 
     ${catkin_LIBRARIES}
diff --git a/include/backward.hpp b/include/backward.hpp
old mode 100755
new mode 100644
diff --git a/include/graph_searcher.h b/include/graph_searcher.h
deleted file mode 100644
index a097728..0000000
--- a/include/graph_searcher.h
+++ /dev/null
@@ -1,136 +0,0 @@
-#include <iostream>
-#include <ros/ros.h>
-#include <ros/console.h>
-#include <Eigen/Eigen>
-#include "backward.hpp"
-
-#define inf 1>>20
-struct GridNode;
-typedef GridNode* GridNodePtr;
-
-///Search and prune neighbors for JPS 3D
-struct JPS3DNeib {
-       // for each (dx,dy,dz) these contain:
-       //    ns: neighbors that are always added
-       //    f1: forced neighbors to check
-       //    f2: neighbors to add if f1 is forced
-       int ns[27][3][26];
-       int f1[27][3][12];
-       int f2[27][3][12];
-       // nsz contains the number of neighbors for the four different types of moves:
-       // no move (norm 0):        26 neighbors always added
-       //                          0 forced neighbors to check (never happens)
-       //                          0 neighbors to add if forced (never happens)
-       // straight (norm 1):       1 neighbor always added
-       //                          8 forced neighbors to check
-       //                          8 neighbors to add if forced
-       // diagonal (norm sqrt(2)): 3 neighbors always added
-       //                          8 forced neighbors to check
-       //                          12 neighbors to add if forced
-       // diagonal (norm sqrt(3)): 7 neighbors always added
-       //                          6 forced neighbors to check
-       //                          12 neighbors to add if forced
-       static constexpr int nsz[4][2] = {{26, 0}, {1, 8}, {3, 12}, {7, 12}};
-       JPS3DNeib();
-       private:
-       void Neib(int dx, int dy, int dz, int norm1, int dev, int& tx, int& ty, int& tz);
-       void FNeib( int dx, int dy, int dz, int norm1, int dev,
-           int& fx, int& fy, int& fz,
-           int& nx, int& ny, int& nz);
-};
-
-struct GridNode
-{     
-    int id;        // 1--> open set, -1 --> closed set
-    Eigen::Vector3d coord; 
-    Eigen::Vector3i dir;   // direction of expanding
-    Eigen::Vector3i index;
-       
-    bool is_path;
-    double gScore, fScore;
-    GridNodePtr cameFrom;
-    std::multimap<double, GridNodePtr>::iterator nodeMapIt;
-
-    GridNode(Eigen::Vector3i _index, Eigen::Vector3d _coord){  
-               id = 0;
-               is_path = false;
-               index = _index;
-               coord = _coord;
-               dir   = Eigen::Vector3i::Zero();
-
-               gScore = inf;
-               fScore = inf;
-               cameFrom = NULL;
-    }
-
-    GridNode(){};
-    ~GridNode(){};
-};
-
-class gridPathFinder
-{
-       private:
-               double getDiagHeu(GridNodePtr node1, GridNodePtr node2);
-               double getManhHeu(GridNodePtr node1, GridNodePtr node2);
-               double getEuclHeu(GridNodePtr node1, GridNodePtr node2);
-               double getHeu(GridNodePtr node1, GridNodePtr node2);
-
-               std::vector<GridNodePtr> retrievePath(GridNodePtr current);
-
-               double resolution, inv_resolution;
-               double tie_breaker = 1.0 + 1.0 / 10000;
-
-               std::vector<GridNodePtr> expandedNodes;
-               std::vector<GridNodePtr> gridPath;
-               std::vector<GridNodePtr> endPtrList;
-               
-               int GLX_SIZE, GLY_SIZE, GLZ_SIZE;
-               int GLXYZ_SIZE, GLYZ_SIZE;
-               double gl_xl, gl_yl, gl_zl;
-               double gl_xu, gl_yu, gl_zu;
-       
-               Eigen::Vector3i goalIdx;
-
-               uint8_t * data;
-
-               GridNodePtr *** GridNodeMap;
-               std::multimap<double, GridNodePtr> openSet;
-               JPS3DNeib * jn3d;
-
-               bool jump(const Eigen::Vector3i & curIdx, const Eigen::Vector3i & expDir, Eigen::Vector3i & neiIdx);
-               
-               inline void getJpsSucc(GridNodePtr currentPtr, std::vector<GridNodePtr> & neighborPtrSets, std::vector<double> & edgeCostSets, int num_iter);
-               inline void getSucc   (GridNodePtr currentPtr, std::vector<GridNodePtr> & neighborPtrSets, std::vector<double> & edgeCostSets);
-               inline bool hasForced(const Eigen::Vector3i & idx, const Eigen::Vector3i & dir);
-
-               inline bool isOccupied(const int & idx_x, const int & idx_y, const int & idx_z) const;
-               inline bool isOccupied(const Eigen::Vector3i & index) const;
-               inline bool isFree(const int & idx_x, const int & idx_y, const int & idx_z) const;
-               inline bool isFree(const Eigen::Vector3i & index) const;
-
-               inline Eigen::Vector3d gridIndex2coord(const Eigen::Vector3i & index) const;
-               inline Eigen::Vector3i coord2gridIndex(const Eigen::Vector3d & pt) const;
-
-       public:
-
-               std::vector<Eigen::Vector3d> debugNodes;
-               gridPathFinder( ){                              
-               jn3d = new JPS3DNeib();
-               };
-
-               ~gridPathFinder(){
-                       delete jn3d;
-               };
-
-               void initGridMap(double _resolution, Eigen::Vector3d global_xyz_l, Eigen::Vector3d global_xyz_u, int max_x_id, int max_y_id, int max_z_id);
-               void setObs(const double coord_x, const double coord_y, const double coord_z);
-
-               void graphSearch(Eigen::Vector3d start_pt, Eigen::Vector3d end_pt, bool use_jps = false);
-               void resetGrid(GridNodePtr ptr);
-               void resetUsedGrids();
-
-               Eigen::Vector3d coordRounding(const Eigen::Vector3d & coord) const;
-               std::vector<Eigen::Vector3d> getPath();
-               std::vector<Eigen::Vector3d> getVisitedNodes();
-               std::vector<Eigen::Vector3d> getCloseNodes();
-};
\ No newline at end of file
diff --git a/launch/demo.launch b/launch/demo.launch
index 56119ea..51f6859 100644
--- a/launch/demo.launch
+++ b/launch/demo.launch
@@ -1,8 +1,8 @@
 <launch>
 
-<arg name="map_size_x" default="20.0"/>
-<arg name="map_size_y" default="20.0"/>
-<arg name="map_size_z" default=" 3.0"/>
+<arg name="map_size_x" default="10.0"/>
+<arg name="map_size_y" default="10.0"/>
+<arg name="map_size_z" default=" 2.0"/>
 
 <arg name="start_x" default=" 0.0"/>
 <arg name="start_y" default=" 0.0"/>
diff --git a/launch/rviz_config/demo.rviz b/launch/rviz_config/demo.rviz
index 3b20e00..f37c8a5 100644
--- a/launch/rviz_config/demo.rviz
+++ b/launch/rviz_config/demo.rviz
@@ -4,16 +4,11 @@ Panels:
     Name: Displays
     Property Tree Widget:
       Expanded:
-        - /Global Options1
-        - /Status1
         - /AllMap1/Autocompute Value Bounds1
-        - /ClosedNodes1
-        - /ClosedNodes1/Namespaces1
-        - /gridPath1
+        - /visitedNodes1/Namespaces1
         - /gridPath1/Namespaces1
-        - /Marker1
       Splitter Ratio: 0.609022558
-    Tree Height: 517
+    Tree Height: 566
   - Class: rviz/Selection
     Name: Selection
   - Class: rviz/Tool Properties
@@ -50,7 +45,7 @@ Visualization Manager:
         Y: 0
         Z: 0
       Plane: XY
-      Plane Cell Count: 200
+      Plane Cell Count: 50
       Reference Frame: <Fixed Frame>
       Value: true
     - Class: rviz/Axes
@@ -63,9 +58,9 @@ Visualization Manager:
     - Alpha: 1
       Autocompute Intensity Bounds: true
       Autocompute Value Bounds:
-        Max Value: 3.9000001
-        Min Value: 0
-        Value: false
+        Max Value: 1.89999998
+        Min Value: 0.100000001
+        Value: true
       Axis: Z
       Channel Name: intensity
       Class: rviz/PointCloud2
@@ -92,24 +87,8 @@ Visualization Manager:
       Value: true
     - Class: rviz/Marker
       Enabled: false
-      Marker Topic: /demo_node/closed_nodes_vis
-      Name: ClosedNodes
-      Namespaces:
-        {}
-      Queue Size: 100
-      Value: false
-    - Class: rviz/Marker
-      Enabled: false
-      Marker Topic: /fast_flight_node/close_nodes_sequence_vis
-      Name: closeNodesSequence
-      Namespaces:
-        {}
-      Queue Size: 100
-      Value: false
-    - Class: rviz/Marker
-      Enabled: false
-      Marker Topic: /fast_flight_node/open_nodes_vis
-      Name: OpenNodes
+      Marker Topic: /demo_node/visited_nodes_vis
+      Name: visitedNodes
       Namespaces:
         {}
       Queue Size: 100
@@ -119,18 +98,9 @@ Visualization Manager:
       Marker Topic: /demo_node/grid_path_vis
       Name: gridPath
       Namespaces:
-        demo_node/astar_path: false
         demo_node/jps_path: true
       Queue Size: 100
       Value: true
-    - Class: rviz/Marker
-      Enabled: true
-      Marker Topic: /demo_node/debug_nodes_vis
-      Name: Marker
-      Namespaces:
-        demo_node/debug_info: true
-      Queue Size: 100
-      Value: true
   Enabled: true
   Global Options:
     Background Color: 255; 255; 127
@@ -149,16 +119,16 @@ Visualization Manager:
   Views:
     Current:
       Class: rviz/Orbit
-      Distance: 6.79373455
+      Distance: 9.74612236
       Enable Stereo Rendering:
         Stereo Eye Separation: 0.0599999987
         Stereo Focal Distance: 1
         Swap Stereo Eyes: false
         Value: false
       Focal Point:
-        X: 2.45372701
-        Y: 2.52708316
-        Z: 0.181677282
+        X: 2.19087362
+        Y: 0.484706104
+        Z: 0.618929148
       Focal Shape Fixed Size: true
       Focal Shape Size: 0.0500000007
       Invert Z Axis: false
@@ -167,7 +137,7 @@ Visualization Manager:
       Pitch: 1.56979632
       Target Frame: <Fixed Frame>
       Value: Orbit (rviz)
-      Yaw: 0.126217589
+      Yaw: 3.77436399
     Saved:
       - Class: rviz/Orbit
         Distance: 79.2448425
@@ -237,10 +207,10 @@ Visualization Manager:
 Window Geometry:
   Displays:
     collapsed: false
-  Height: 920
+  Height: 1028
   Hide Left Dock: false
   Hide Right Dock: false
-  QMainWindow State: 000000ff00000000fd0000000400000000000001a20000036bfc020000000bfb000000100044006900730070006c00610079007301000000130000023f000000c600fffffffb0000000a005600690065007700730100000258000001260000009e00fffffffb0000001200530065006c0065006300740069006f006e00000000000000009b0000005c00fffffffb000000120056006900650077007300200054006f006f02000001df000002110000018500000122fb000000200054006f006f006c002000500072006f0070006500720074006900650073003203000002880000011d000002210000017afb0000001e0054006f006f006c002000500072006f007000650072007400690065007300000000e3000001c60000005c00fffffffb0000002000730065006c0065006300740069006f006e00200062007500660066006500720200000138000000aa0000023a00000294fb00000014005700690064006500530074006500720065006f02000000e6000000d2000003ee0000030bfb0000000c004b0069006e0065006300740200000186000001060000030c00000261fb0000000a0049006d006100670065020000037e000000e30000016a00000089fb0000000a0049006d0061006700650200000593000001f50000016a000000ed000000010000010f000002c4fc0200000002fb0000001e0054006f006f006c002000500072006f00700065007200740069006500730100000041000000780000000000000000fb0000001200530065006c0065006300740069006f006e010000025a000000b20000000000000000000000020000073f000000ddfc0100000002fb0000000c00430061006d006500720061020000012e00000138000002d9000001a9fb0000000a00560069006500770073030000004e00000080000002e10000019700000003000005730000003efc0100000002fb0000000800540069006d00650000000000000005730000024400fffffffb0000000800540069006d00650100000000000004500000000000000000000004610000036b00000004000000040000000800000008fc0000000100000002000000010000000a0054006f006f006c00730000000000ffffffff0000000000000000
+  QMainWindow State: 000000ff00000000fd0000000400000000000001a2000003befc020000000bfb000000100044006900730070006c006100790073010000002800000277000000d700fffffffb0000000a0056006900650077007301000002a500000141000000ad00fffffffb0000001200530065006c0065006300740069006f006e00000000000000009b0000006100fffffffb000000120056006900650077007300200054006f006f02000001df000002110000018500000122fb000000200054006f006f006c002000500072006f0070006500720074006900650073003203000002880000011d000002210000017afb0000001e0054006f006f006c002000500072006f007000650072007400690065007300000000e3000001c60000006100fffffffb0000002000730065006c0065006300740069006f006e00200062007500660066006500720200000138000000aa0000023a00000294fb00000014005700690064006500530074006500720065006f02000000e6000000d2000003ee0000030bfb0000000c004b0069006e0065006300740200000186000001060000030c00000261fb0000000a0049006d006100670065020000037e000000e30000016a00000089fb0000000a0049006d0061006700650200000593000001f50000016a000000ed000000010000010f000002c4fc0200000002fb0000001e0054006f006f006c002000500072006f00700065007200740069006500730100000041000000780000000000000000fb0000001200530065006c0065006300740069006f006e010000025a000000b20000000000000000000000020000073f000000ddfc0100000002fb0000000c00430061006d006500720061020000012e00000138000002d9000001a9fb0000000a00560069006500770073030000004e00000080000002e10000019700000003000005730000003efc0100000002fb0000000800540069006d00650000000000000005730000030000fffffffb0000000800540069006d0065010000000000000450000000000000000000000597000003be00000004000000040000000800000008fc0000000100000002000000010000000a0054006f006f006c00730100000000ffffffff0000000000000000
   Selection:
     collapsed: false
   Time:
@@ -249,6 +219,6 @@ Window Geometry:
     collapsed: false
   Views:
     collapsed: false
-  Width: 1545
-  X: 905
-  Y: 325
+  Width: 1855
+  X: 142
+  Y: 69
diff --git a/src/demo_node.cpp b/src/demo_node.cpp
index ce79e40..0a66f8b 100644
--- a/src/demo_node.cpp
+++ b/src/demo_node.cpp
@@ -14,7 +14,8 @@
 #include <visualization_msgs/MarkerArray.h>
 #include <visualization_msgs/Marker.h>
 
-#include "graph_searcher.h"
+#include "Astar_searcher.h"
+#include "JPS_searcher.h"
 #include "backward.hpp"
 
 using namespace std;
@@ -37,18 +38,16 @@ int _max_x_id, _max_y_id, _max_z_id;
 
 // ros related
 ros::Subscriber _map_sub, _pts_sub;
-ros::Publisher  _grid_path_vis_pub, _debug_nodes_vis_pub, _closed_nodes_vis_pub, _open_nodes_vis_pub, _close_nodes_sequence_vis_pub, _grid_map_vis_pub;
+ros::Publisher  _grid_path_vis_pub, _visited_nodes_vis_pub, _grid_map_vis_pub;
 
-gridPathFinder * _path_finder = new gridPathFinder();
+AstarPathFinder * _astar_path_finder     = new AstarPathFinder();
+JPSPathFinder   * _jps_path_finder       = new JPSPathFinder();
 
 void rcvWaypointsCallback(const nav_msgs::Path & wp);
 void rcvPointCloudCallBack(const sensor_msgs::PointCloud2 & pointcloud_map);
 
-void visDebugNodes( vector<Vector3d> nodes );
 void visGridPath( vector<Vector3d> nodes, bool is_use_jps );
-void visCloseNode( vector<Vector3d> nodes );
-void visOpenNode( vector<Vector3d> nodes );
-void visCloseNodeSequence( vector<Vector3d> nodes );
+void visVisitedNode( vector<Vector3d> nodes );
 void pathFinding(const Vector3d start_pt, const Vector3d target_pt);
 
 void rcvWaypointsCallback(const nav_msgs::Path & wp)
@@ -59,9 +58,9 @@ void rcvWaypointsCallback(const nav_msgs::Path & wp)
     Vector3d target_pt;
     target_pt << wp.poses[0].pose.position.x,
                  wp.poses[0].pose.position.y,
-                 1.0;//wp.poses[0].pose.position.z;
+                 wp.poses[0].pose.position.z;
 
-    ROS_INFO("[jps_node] receive the way-points");
+    ROS_INFO("[node] receive the planning target");
     pathFinding(_start_pt, target_pt); 
 }
 
@@ -77,33 +76,21 @@ void rcvPointCloudCallBack(const sensor_msgs::PointCloud2 & pointcloud_map)
     
     if( (int)cloud.points.size() == 0 ) return;
 
-    pcl::PointXYZ pt, pt_inf;
-    int inf_step   = round(_cloud_margin * _inv_resolution);
-    int inf_step_z = max(1, inf_step / 2);
+    pcl::PointXYZ pt;
     for (int idx = 0; idx < (int)cloud.points.size(); idx++)
     {    
         pt = cloud.points[idx];        
-        for(int x = -inf_step ; x <= inf_step; x ++ )
-        {
-            for(int y = -inf_step ; y <= inf_step; y ++ )
-            {
-                for(int z = -inf_step_z; z <= inf_step_z; z ++ )
-                {
-                    double inf_x = pt.x + x * _resolution;
-                    double inf_y = pt.y + y * _resolution;
-                    double inf_z = pt.z + z * _resolution;
-                    _path_finder->setObs(inf_x, inf_y, inf_z);
-
-                    Vector3d cor_inf = _path_finder->coordRounding(Vector3d(inf_x, inf_y, inf_z));
-
-                    pt_inf.x = cor_inf(0);
-                    pt_inf.y = cor_inf(1);
-                    pt_inf.z = cor_inf(2);
-                    cloud_vis.points.push_back(pt_inf);
-
-                }
-            }
-        }
+
+        // set obstalces into grid map for path planning
+        _astar_path_finder->setObs(pt.x, pt.y, pt.z);
+        _jps_path_finder->setObs(pt.x, pt.y, pt.z);
+
+        // for visualize only
+        Vector3d cor_round = _astar_path_finder->coordRounding(Vector3d(pt.x, pt.y, pt.z));
+        pt.x = cor_round(0);
+        pt.y = cor_round(1);
+        pt.z = cor_round(2);
+        cloud_vis.points.push_back(pt);
     }
 
     cloud_vis.width    = cloud_vis.points.size();
@@ -120,20 +107,41 @@ void rcvPointCloudCallBack(const sensor_msgs::PointCloud2 & pointcloud_map)
 
 void pathFinding(const Vector3d start_pt, const Vector3d target_pt)
 {
-    _path_finder->graphSearch(start_pt, target_pt, true);
-    auto grid_path   = _path_finder->getPath();
-    visGridPath (grid_path, true);
-    visDebugNodes(_path_finder->debugNodes);
+    //Call A* to search for a path
+    _astar_path_finder->AstarGraphSearch(start_pt, target_pt);
 
-    _path_finder->resetUsedGrids();
+    //Retrieve the path
+    auto grid_path     = _astar_path_finder->getPath();
+    auto visited_nodes = _astar_path_finder->getVisitedNodes();
 
-    _path_finder->graphSearch(start_pt, target_pt, false);
-    grid_path   = _path_finder->getPath();
+    //Visualize the result
     visGridPath (grid_path, false);
-    auto close_nodes = _path_finder->getCloseNodes();
-    visCloseNode(close_nodes);
-    
-    _path_finder->resetUsedGrids();
+    visVisitedNode(visited_nodes);
+
+    //Reset map for next call
+    _astar_path_finder->resetUsedGrids();
+
+    //_use_jps = 0 -> Do not use JPS
+    //_use_jps = 1 -> Use JPS
+    //you just need to change the #define value of _use_jps
+#define _use_jps 0
+#if _use_jps
+    {
+        //Call JPS to search for a path
+        _jps_path_finder -> JPSGraphSearch(start_pt, target_pt);
+
+        //Retrieve the path
+        auto grid_path     = _jps_path_finder->getPath();
+        auto visited_nodes = _jps_path_finder->getVisitedNodes();
+
+        //Visualize the result
+        visGridPath   (grid_path, _use_jps);
+        visVisitedNode(visited_nodes);
+
+        //Reset map for next call
+        _jps_path_finder->resetUsedGrids();
+    }
+#endif
 }
 
 int main(int argc, char** argv)
@@ -146,10 +154,7 @@ int main(int argc, char** argv)
 
     _grid_map_vis_pub             = nh.advertise<sensor_msgs::PointCloud2>("grid_map_vis", 1);
     _grid_path_vis_pub            = nh.advertise<visualization_msgs::Marker>("grid_path_vis", 1);
-    _debug_nodes_vis_pub          = nh.advertise<visualization_msgs::Marker>("debug_nodes_vis", 1);
-    _closed_nodes_vis_pub         = nh.advertise<visualization_msgs::Marker>("closed_nodes_vis",   1);
-    _open_nodes_vis_pub           = nh.advertise<visualization_msgs::Marker>("open_nodes_vis",     1);
-    _close_nodes_sequence_vis_pub = nh.advertise<visualization_msgs::Marker>("close_nodes_sequence_vis", 10);
+    _visited_nodes_vis_pub        = nh.advertise<visualization_msgs::Marker>("visited_nodes_vis",1);
 
     nh.param("map/cloud_margin",  _cloud_margin, 0.0);
     nh.param("map/resolution",    _resolution,   0.2);
@@ -171,9 +176,12 @@ int main(int argc, char** argv)
     _max_y_id = (int)(_y_size * _inv_resolution);
     _max_z_id = (int)(_z_size * _inv_resolution);
 
-    _path_finder  = new gridPathFinder();
-    _path_finder  -> initGridMap(_resolution, _map_lower, _map_upper, _max_x_id, _max_y_id, _max_z_id);
+    _astar_path_finder  = new AstarPathFinder();
+    _astar_path_finder  -> initGridMap(_resolution, _map_lower, _map_upper, _max_x_id, _max_y_id, _max_z_id);
 
+    _jps_path_finder    = new JPSPathFinder();
+    _jps_path_finder    -> initGridMap(_resolution, _map_lower, _map_upper, _max_x_id, _max_y_id, _max_z_id);
+    
     ros::Rate rate(100);
     bool status = ros::ok();
     while(status) 
@@ -183,50 +191,11 @@ int main(int argc, char** argv)
         rate.sleep();
     }
 
-    delete _path_finder;
+    delete _astar_path_finder;
+    delete _jps_path_finder;
     return 0;
 }
 
-void visDebugNodes( vector<Vector3d> nodes )
-{   
-    visualization_msgs::Marker node_vis; 
-    node_vis.header.frame_id = "world";
-    node_vis.header.stamp = ros::Time::now();
-    
-    node_vis.ns = "demo_node/debug_info";
-
-    node_vis.type = visualization_msgs::Marker::CUBE_LIST;
-    node_vis.action = visualization_msgs::Marker::ADD;
-    node_vis.id = 0;
-
-    node_vis.pose.orientation.x = 0.0;
-    node_vis.pose.orientation.y = 0.0;
-    node_vis.pose.orientation.z = 0.0;
-    node_vis.pose.orientation.w = 1.0;
-
-    node_vis.color.a = 0.5;
-    node_vis.color.r = 0.0;
-    node_vis.color.g = 0.0;
-    node_vis.color.b = 0.0;
-
-    node_vis.scale.x = _resolution;
-    node_vis.scale.y = _resolution;
-    node_vis.scale.z = _resolution;
-
-    geometry_msgs::Point pt;
-    for(int i = 0; i < int(nodes.size()); i++)
-    {
-        Vector3d coord = nodes[i];
-        pt.x = coord(0);
-        pt.y = coord(1);
-        pt.z = coord(2);
-
-        node_vis.points.push_back(pt);
-    }
-
-    _debug_nodes_vis_pub.publish(node_vis);
-}
-
 void visGridPath( vector<Vector3d> nodes, bool is_use_jps )
 {   
     visualization_msgs::Marker node_vis; 
@@ -279,12 +248,12 @@ void visGridPath( vector<Vector3d> nodes, bool is_use_jps )
     _grid_path_vis_pub.publish(node_vis);
 }
 
-void visCloseNode( vector<Vector3d> nodes )
+void visVisitedNode( vector<Vector3d> nodes )
 {   
     visualization_msgs::Marker node_vis; 
     node_vis.header.frame_id = "world";
     node_vis.header.stamp = ros::Time::now();
-    node_vis.ns = "demo_node/closed_nodes";
+    node_vis.ns = "demo_node/expanded_nodes";
     node_vis.type = visualization_msgs::Marker::CUBE_LIST;
     node_vis.action = visualization_msgs::Marker::ADD;
     node_vis.id = 0;
@@ -313,79 +282,5 @@ void visCloseNode( vector<Vector3d> nodes )
         node_vis.points.push_back(pt);
     }
 
-    _closed_nodes_vis_pub.publish(node_vis);
-}
-
-void visOpenNode( vector<Vector3d> nodes )
-{   
-    visualization_msgs::Marker node_vis; 
-    node_vis.header.frame_id = "world";
-    node_vis.header.stamp = ros::Time::now();
-    node_vis.ns = "demo_node/visited_nodes";
-    node_vis.type = visualization_msgs::Marker::CUBE_LIST;
-    node_vis.action = visualization_msgs::Marker::ADD;
-    node_vis.id = 0;
-
-    node_vis.pose.orientation.x = 0.0;
-    node_vis.pose.orientation.y = 0.0;
-    node_vis.pose.orientation.z = 0.0;
-    node_vis.pose.orientation.w = 1.0;
-    node_vis.color.a = 0.3;
-    node_vis.color.r = 0.0;
-    node_vis.color.g = 1.0;
-    node_vis.color.b = 0.0;
-
-    node_vis.scale.x = _resolution;
-    node_vis.scale.y = _resolution;
-    node_vis.scale.z = _resolution;
-
-    geometry_msgs::Point pt;
-    for(int i = 0; i < int(nodes.size()); i++)
-    {
-        Vector3d coord = nodes[i];
-        pt.x = coord(0);
-        pt.y = coord(1);
-        pt.z = coord(2);
-
-        node_vis.points.push_back(pt);
-    }
-
-    _open_nodes_vis_pub.publish(node_vis);
-}
-
-void visCloseNodeSequence( vector<Vector3d> nodes )
-{   
-    visualization_msgs::Marker node_vis; 
-    node_vis.header.frame_id = "world";
-    node_vis.header.stamp = ros::Time::now();
-    node_vis.ns = "demo_node/animation_of_close_nodes";
-    node_vis.type = visualization_msgs::Marker::CUBE_LIST;
-    node_vis.action = visualization_msgs::Marker::ADD;
-    node_vis.id = 0;
-
-    node_vis.pose.orientation.x = 0.0;
-    node_vis.pose.orientation.y = 0.0;
-    node_vis.pose.orientation.z = 0.0;
-    node_vis.pose.orientation.w = 1.0;
-    node_vis.color.a = 1.0;
-    node_vis.color.r = 0.0;
-    node_vis.color.g = 0.0;
-    node_vis.color.b = 0.0;
-
-    node_vis.scale.x = _resolution;
-    node_vis.scale.y = _resolution;
-    node_vis.scale.z = _resolution;
-
-    geometry_msgs::Point pt;
-    for(int i = 0; i < int(nodes.size()); i++)
-    {
-        usleep(50000);
-        Vector3d coord = nodes[i];
-        pt.x = coord(0);
-        pt.y = coord(1);
-        pt.z = coord(2);
-
-        node_vis.points.push_back(pt);
-        _close_nodes_sequence_vis_pub.publish(node_vis);
-    }
+    _visited_nodes_vis_pub.publish(node_vis);
 }
\ No newline at end of file
diff --git a/src/graph_searcher.cpp b/src/graph_searcher.cpp
deleted file mode 100644
index 63d226a..0000000
--- a/src/graph_searcher.cpp
+++ /dev/null
@@ -1,820 +0,0 @@
-#include <graph_searcher.h>
-
-using namespace std;
-using namespace Eigen;
-
-void gridPathFinder::initGridMap(double _resolution, Vector3d global_xyz_l, Vector3d global_xyz_u, int max_x_id, int max_y_id, int max_z_id)
-{   
-    gl_xl = global_xyz_l(0);
-    gl_yl = global_xyz_l(1);
-    gl_zl = global_xyz_l(2);
-
-    gl_xu = global_xyz_u(0);
-    gl_yu = global_xyz_u(1);
-    gl_zu = global_xyz_u(2);
-    
-    GLX_SIZE = max_x_id;
-    GLY_SIZE = max_y_id;
-    GLZ_SIZE = max_z_id;
-    GLYZ_SIZE  = GLY_SIZE * GLZ_SIZE;
-    GLXYZ_SIZE = GLX_SIZE * GLYZ_SIZE;
-
-    resolution = _resolution;
-    inv_resolution = 1.0 / _resolution;    
-
-    data = new uint8_t[GLXYZ_SIZE];
-    memset(data, 0, GLXYZ_SIZE * sizeof(uint8_t));
-    
-    GridNodeMap = new GridNodePtr ** [GLX_SIZE];
-    for(int i = 0; i < GLX_SIZE; i++){
-        GridNodeMap[i] = new GridNodePtr * [GLY_SIZE];
-        for(int j = 0; j < GLY_SIZE; j++){
-            GridNodeMap[i][j] = new GridNodePtr [GLZ_SIZE];
-            for( int k = 0; k < GLZ_SIZE;k++){
-                Vector3i tmpIdx(i,j,k);
-                Vector3d pos = gridIndex2coord(tmpIdx);
-                GridNodeMap[i][j][k] = new GridNode(tmpIdx, pos);
-            }
-        }
-    }
-}
-
-void gridPathFinder::resetGrid(GridNodePtr ptr)
-{
-    ptr->id = 0;
-    ptr->cameFrom = NULL;
-    ptr->gScore = inf;
-    ptr->fScore = inf;
-}
-
-void gridPathFinder::resetUsedGrids()
-{   
-    //ROS_WARN("expandedNodes size : %d", expandedNodes.size()); 
-    for(auto tmpPtr:expandedNodes)
-        resetGrid(tmpPtr);
-
-    GridNodePtr tmpPtr = NULL;
-    for(auto ptr:openSet){   
-        tmpPtr = ptr.second;
-        resetGrid(tmpPtr);
-    }
-
-    for(auto ptr:gridPath)
-        ptr->is_path = false;
-
-    expandedNodes.clear();
-}
-
-void gridPathFinder::setObs(const double coord_x, const double coord_y, const double coord_z)
-{   
-    if( coord_x < gl_xl  || coord_y < gl_yl  || coord_z <  gl_zl || 
-        coord_x >= gl_xu || coord_y >= gl_yu || coord_z >= gl_zu )
-        return;
-
-    int idx_x = static_cast<int>( (coord_x - gl_xl) * inv_resolution);
-    int idx_y = static_cast<int>( (coord_y - gl_yl) * inv_resolution);
-    int idx_z = static_cast<int>( (coord_z - gl_zl) * inv_resolution);      
-
-    data[idx_x * GLYZ_SIZE + idx_y * GLZ_SIZE + idx_z] = 1;
-}
-
-double gridPathFinder::getDiagHeu(GridNodePtr node1, GridNodePtr node2)
-{   
-    double dx = abs(node1->index(0) - node2->index(0));
-    double dy = abs(node1->index(1) - node2->index(1));
-    double dz = abs(node1->index(2) - node2->index(2));
-
-    double h = 0.0;
-    int diag = min(min(dx, dy), dz);
-    dx -= diag;
-    dy -= diag;
-    dz -= diag;
-
-    if (dx == 0) {
-        h = 1.0 * sqrt(3.0) * diag + sqrt(2.0) * min(dy, dz) + 1.0 * abs(dy - dz);
-    }
-    if (dy == 0) {
-        h = 1.0 * sqrt(3.0) * diag + sqrt(2.0) * min(dx, dz) + 1.0 * abs(dx - dz);
-    }
-    if (dz == 0) {
-        h = 1.0 * sqrt(3.0) * diag + sqrt(2.0) * min(dx, dy) + 1.0 * abs(dx - dy);
-    }
-    return h;
-}
-
-double gridPathFinder::getManhHeu(GridNodePtr node1, GridNodePtr node2)
-{   
-    double dx = abs(node1->index(0) - node2->index(0));
-    double dy = abs(node1->index(1) - node2->index(1));
-    double dz = abs(node1->index(2) - node2->index(2));
-
-    return dx + dy + dz;
-}
-
-double gridPathFinder::getEuclHeu(GridNodePtr node1, GridNodePtr node2)
-{   
-    return (node2->index - node1->index).norm();
-}
-
-double gridPathFinder::getHeu(GridNodePtr node1, GridNodePtr node2)
-{
-    return tie_breaker * getDiagHeu(node1, node2);
-    //return getEuclHeu(node1, node2);
-}
-
-vector<GridNodePtr> gridPathFinder::retrievePath(GridNodePtr current)
-{   
-    vector<GridNodePtr> path;
-    path.push_back(current);
-
-    while(current->cameFrom != NULL)
-    {   
-        current->is_path = true;
-        current = current -> cameFrom;
-        path.push_back(current);
-    }
-
-    current->is_path = true;
-
-    return path;
-}
-
-vector<Vector3d> gridPathFinder::getVisitedNodes()
-{   
-    vector<Vector3d> visited_nodes;
-    for(int i = 0; i < GLX_SIZE; i++)
-        for(int j = 0; j < GLY_SIZE; j++)
-            for(int k = 0; k < GLZ_SIZE; k++)
-            {   
-                if(GridNodeMap[i][j][k]->id != 0)
-                //if(GridNodeMap[i][j][k]->id == -1)
-                    visited_nodes.push_back(GridNodeMap[i][j][k]->coord);
-            }
-
-    ROS_WARN("visited_nodes size : %d", visited_nodes.size());
-    return visited_nodes;
-}
-
-vector<Vector3d> gridPathFinder::getCloseNodes()
-{   
-    vector<Vector3d> vec;
-    for(auto tmpPtr:expandedNodes)
-    {   
-        if( !tmpPtr->is_path )
-            vec.push_back(tmpPtr->coord);
-    }
-
-    return vec;
-}
-
-vector<Vector3d> gridPathFinder::getPath() 
-{   
-    vector<Vector3d> path;
-
-    for(auto ptr: gridPath)
-        path.push_back(ptr->coord);
-
-    reverse(path.begin(), path.end());
-    return path;
-}
-
-inline Vector3d gridPathFinder::gridIndex2coord(const Vector3i & index) const
-{
-    Vector3d pt;
-
-    pt(0) = ((double)index(0) + 0.5) * resolution + gl_xl;
-    pt(1) = ((double)index(1) + 0.5) * resolution + gl_yl;
-    pt(2) = ((double)index(2) + 0.5) * resolution + gl_zl;
-
-    return pt;
-}
-
-inline Vector3i gridPathFinder::coord2gridIndex(const Vector3d & pt) const
-{
-    Vector3i idx;
-    idx <<  min( max( int( (pt(0) - gl_xl) * inv_resolution), 0), GLX_SIZE - 1),
-            min( max( int( (pt(1) - gl_yl) * inv_resolution), 0), GLY_SIZE - 1),
-            min( max( int( (pt(2) - gl_zl) * inv_resolution), 0), GLZ_SIZE - 1);                  
-  
-    return idx;
-}
-
-Eigen::Vector3d gridPathFinder::coordRounding(const Eigen::Vector3d & coord) const
-{
-    return gridIndex2coord(coord2gridIndex(coord));
-}
-
-inline bool gridPathFinder::isOccupied(const Eigen::Vector3i & index) const
-{
-    return isOccupied(index(0), index(1), index(2));
-}
-
-inline bool gridPathFinder::isFree(const Eigen::Vector3i & index) const
-{
-    return isFree(index(0), index(1), index(2));
-}
-
-inline bool gridPathFinder::isOccupied(const int & idx_x, const int & idx_y, const int & idx_z) const 
-{
-    return  (idx_x >= 0 && idx_x < GLX_SIZE && idx_y >= 0 && idx_y < GLY_SIZE && idx_z >= 0 && idx_z < GLZ_SIZE && 
-            (data[idx_x * GLYZ_SIZE + idx_y * GLZ_SIZE + idx_z] == 1));
-}
-
-inline bool gridPathFinder::isFree(const int & idx_x, const int & idx_y, const int & idx_z) const 
-{
-    return (idx_x >= 0 && idx_x < GLX_SIZE && idx_y >= 0 && idx_y < GLY_SIZE && idx_z >= 0 && idx_z < GLZ_SIZE && 
-           (data[idx_x * GLYZ_SIZE + idx_y * GLZ_SIZE + idx_z] < 1));
-}
-
-inline void gridPathFinder::getJpsSucc(GridNodePtr currentPtr, vector<GridNodePtr> & neighborPtrSets, vector<double> & edgeCostSets, int num_iter)
-{   
-    neighborPtrSets.clear();
-    edgeCostSets.clear();
-    const int norm1 = abs(currentPtr->dir(0)) + abs(currentPtr->dir(1)) + abs(currentPtr->dir(2));
-
-    int num_neib  = jn3d->nsz[norm1][0];
-    int num_fneib = jn3d->nsz[norm1][1];
-    int id = (currentPtr->dir(0) + 1) + 3 * (currentPtr->dir(1) + 1) + 9 * (currentPtr->dir(2) + 1);
-
-    for( int dev = 0; dev < num_neib + num_fneib; ++dev) {
-        Vector3i neighborIdx;
-        Vector3i expandDir;
-
-        if( dev < num_neib) {
-            expandDir(0) = jn3d->ns[id][0][dev];
-            expandDir(1) = jn3d->ns[id][1][dev];
-            expandDir(2) = jn3d->ns[id][2][dev];
-            
-            if( !jump(currentPtr->index, expandDir, neighborIdx) )  
-                continue;
-        }
-        else {
-            int nx = currentPtr->index(0) + jn3d->f1[id][0][dev - num_neib];
-            int ny = currentPtr->index(1) + jn3d->f1[id][1][dev - num_neib];
-            int nz = currentPtr->index(2) + jn3d->f1[id][2][dev - num_neib];
-            
-            if( isOccupied(nx, ny, nz) ) {
-                expandDir(0) = jn3d->f2[id][0][dev - num_neib];
-                expandDir(1) = jn3d->f2[id][1][dev - num_neib];
-                expandDir(2) = jn3d->f2[id][2][dev - num_neib];
-                
-                if( !jump(currentPtr->index, expandDir, neighborIdx) ) 
-                    continue;
-            }
-            else
-                continue;
-        }
-
-        if( num_iter == 1 )
-            debugNodes.push_back(gridIndex2coord(neighborIdx));
-
-        GridNodePtr nodePtr = GridNodeMap[neighborIdx(0)][neighborIdx(1)][neighborIdx(2)];
-        nodePtr->dir = expandDir;
-        
-        neighborPtrSets.push_back(nodePtr);
-        edgeCostSets.push_back(
-            sqrt(
-            (neighborIdx(0) - currentPtr->index(0)) * (neighborIdx(0) - currentPtr->index(0)) +
-            (neighborIdx(1) - currentPtr->index(1)) * (neighborIdx(1) - currentPtr->index(1)) +
-            (neighborIdx(2) - currentPtr->index(2)) * (neighborIdx(2) - currentPtr->index(2))   ) 
-            );
-    }
-}
-
-inline void gridPathFinder::getSucc(GridNodePtr currentPtr, vector<GridNodePtr> & neighborPtrSets, vector<double> & edgeCostSets)
-{   
-    neighborPtrSets.clear();
-    edgeCostSets.clear();
-    Vector3i neighborIdx;
-    for(int dx = -1; dx < 2; dx++){
-        for(int dy = -1; dy < 2; dy++){
-            for(int dz = -1; dz < 2; dz++){
-                
-                if( dx == 0 && dy == 0 && dz ==0 )
-                    continue; 
-
-                neighborIdx(0) = (currentPtr -> index)(0) + dx;
-                neighborIdx(1) = (currentPtr -> index)(1) + dy;
-                neighborIdx(2) = (currentPtr -> index)(2) + dz;
-
-                if(    neighborIdx(0) < 0 || neighborIdx(0) >= GLX_SIZE
-                    || neighborIdx(1) < 0 || neighborIdx(1) >= GLY_SIZE
-                    || neighborIdx(2) < 0 || neighborIdx(2) >= GLZ_SIZE){
-                    continue;
-                }
-
-                neighborPtrSets.push_back(GridNodeMap[neighborIdx(0)][neighborIdx(1)][neighborIdx(2)]);
-                edgeCostSets.   push_back(sqrt(dx * dx + dy * dy + dz * dz));
-            }
-        }
-    }
-}
-
-void gridPathFinder::graphSearch(Vector3d start_pt, Vector3d end_pt, bool use_jps)
-{   
-    ros::Time time_1 = ros::Time::now();    
-    debugNodes.clear();
-
-    Vector3i start_idx = coord2gridIndex(start_pt);
-    Vector3i end_idx   = coord2gridIndex(end_pt);
-
-    goalIdx = end_idx;
-
-    start_pt = gridIndex2coord(start_idx);
-    end_pt   = gridIndex2coord(end_idx);
-
-    GridNodePtr startPtr = new GridNode(start_idx, start_pt);
-    GridNodePtr endPtr   = new GridNode(end_idx,   end_pt);
-
-    openSet.clear();
-
-    GridNodePtr neighborPtr = NULL;
-    GridNodePtr currentPtr  = NULL;
-
-    startPtr -> gScore = 0;
-    startPtr -> fScore = getHeu(startPtr, endPtr);
-    startPtr -> id = 1; //put start node in open set
-    startPtr -> coord = start_pt;
-    openSet.insert( make_pair(startPtr -> fScore, startPtr) ); //put start in open set
-
-    double tentative_gScore;
-
-    int num_iter = 0;
-    vector<GridNodePtr> neighborPtrSets;
-    vector<double> edgeCostSets;
-
-    // we only cover 3d case in this project.
-    while ( !openSet.empty() )
-    {   
-        num_iter ++;
-        currentPtr = openSet.begin() -> second;
-
-        if( currentPtr->index == goalIdx )
-        {
-            ros::Time time_2 = ros::Time::now();
-
-            if( use_jps )
-                ROS_WARN("[JPS]{sucess} Time in JPS is %f ms, path cost if %f m", (time_2 - time_1).toSec() * 1000.0, currentPtr->gScore * resolution );
-            else
-                ROS_WARN("[A*]{sucess}  Time in A*  is %f ms, path cost if %f m", (time_2 - time_1).toSec() * 1000.0, currentPtr->gScore * resolution );
-            
-            gridPath = retrievePath(currentPtr);
-            return;
-        }         
-        openSet.erase(openSet.begin());
-        currentPtr -> id = -1; //move current node from open set to closed set.
-        expandedNodes.push_back(currentPtr);
-        
-        if(!use_jps)
-            getSucc(currentPtr, neighborPtrSets, edgeCostSets);
-        else
-            getJpsSucc(currentPtr, neighborPtrSets, edgeCostSets, num_iter);
-
-        for(int i = 0; i < (int)neighborPtrSets.size(); i++){
-            neighborPtr = neighborPtrSets[i];
-            if( isOccupied(neighborPtr->index) || neighborPtr -> id == -1)
-                continue;
-
-            double edge_cost = edgeCostSets[i];            
-            tentative_gScore = currentPtr -> gScore + edge_cost; 
-
-            if(neighborPtr -> id != 1){ //discover a new node
-                neighborPtr -> id        = 1;
-                neighborPtr -> cameFrom  = currentPtr;
-                neighborPtr -> gScore    = tentative_gScore;
-                neighborPtr -> fScore    = neighborPtr -> gScore + getHeu(neighborPtr, endPtr); 
-                neighborPtr -> nodeMapIt = openSet.insert( make_pair(neighborPtr->fScore, neighborPtr) ); //put neighbor in open set and record it.
-                continue;
-            }
-            else if(tentative_gScore <= neighborPtr-> gScore){ //in open set and need update
-                neighborPtr -> cameFrom = currentPtr;
-                neighborPtr -> gScore = tentative_gScore;
-                neighborPtr -> fScore = tentative_gScore + getHeu(neighborPtr, endPtr); 
-                openSet.erase(neighborPtr -> nodeMapIt);
-                neighborPtr -> nodeMapIt = openSet.insert( make_pair(neighborPtr->fScore, neighborPtr) ); //put neighbor in open set and record it.
-
-                // if change its parents, update the expanding direction
-                for(int i = 0; i < 3; i++){
-                    neighborPtr->dir(i) = neighborPtr->index(i) - currentPtr->index(i);
-                    if( neighborPtr->dir(i) != 0)
-                        neighborPtr->dir(i) /= abs( neighborPtr->dir(i) );
-                }
-            }
-            
-        }
-    }
-
-    ros::Time time_2 = ros::Time::now();
-
-    if((time_2 - time_1).toSec() > 0.1)
-        ROS_WARN("Time consume in A star path finding is %f", (time_2 - time_1).toSec() );
-}
-
-bool gridPathFinder::jump(const Vector3i & curIdx, const Vector3i & expDir, Vector3i & neiIdx)
-{
-    neiIdx = curIdx + expDir;
-
-    if( !isFree(neiIdx) )
-        return false;
-
-    if( neiIdx == goalIdx )
-        return true;
-
-    if( hasForced(neiIdx, expDir) )
-        return true;
-
-    const int id = (expDir(0) + 1) + 3 * (expDir(1) + 1) + 9 * (expDir(2) + 1);
-    const int norm1 = abs(expDir(0)) + abs(expDir(1)) + abs(expDir(2));
-    int num_neib = jn3d->nsz[norm1][0];
-
-    for( int k = 0; k < num_neib - 1; ++k ){
-        Vector3i newNeiIdx;
-        Vector3i newDir(jn3d->ns[id][0][k], jn3d->ns[id][1][k], jn3d->ns[id][2][k]);
-        if( jump(neiIdx, newDir, newNeiIdx) ) 
-            return true;
-    }
-
-    return jump(neiIdx, expDir, neiIdx);
-}
-
-inline bool gridPathFinder::hasForced(const Vector3i & idx, const Vector3i & dir)
-{
-    int norm1 = abs(dir(0)) + abs(dir(1)) + abs(dir(2));
-    int id    = (dir(0) + 1) + 3 * (dir(1) + 1) + 9 * (dir(2) + 1);
-
-    switch(norm1)
-    {
-        case 1:
-            // 1-d move, check 8 neighbors
-            for( int fn = 0; fn < 8; ++fn ){
-                int nx = idx(0) + jn3d->f1[id][0][fn];
-                int ny = idx(1) + jn3d->f1[id][1][fn];
-                int nz = idx(2) + jn3d->f1[id][2][fn];
-                if( isOccupied(nx, ny, nz) )
-                    return true;
-            }
-            return false;
-
-        case 2:
-            // 2-d move, check 8 neighbors
-            for( int fn = 0; fn < 8; ++fn ){
-                int nx = idx(0) + jn3d->f1[id][0][fn];
-                int ny = idx(1) + jn3d->f1[id][1][fn];
-                int nz = idx(2) + jn3d->f1[id][2][fn];
-                if( isOccupied(nx, ny, nz) )
-                    return true;
-            }
-            return false;
-
-        case 3:
-            // 3-d move, check 6 neighbors
-            for( int fn = 0; fn < 6; ++fn ){
-                int nx = idx(0) + jn3d->f1[id][0][fn];
-                int ny = idx(1) + jn3d->f1[id][1][fn];
-                int nz = idx(2) + jn3d->f1[id][2][fn];
-                if( isOccupied(nx, ny, nz) )
-                    return true;
-            }
-            return false;
-
-        default:
-            return false;
-    }
-}
-
-constexpr int JPS3DNeib::nsz[4][2];
-JPS3DNeib::JPS3DNeib() 
-{
-    int id = 0;
-    for(int dz = -1; dz <= 1; ++ dz) {
-        for(int dy = -1; dy <= 1; ++ dy) {
-            for(int dx = -1; dx <= 1; ++ dx) {
-                int norm1 = abs(dx) + abs(dy) + abs(dz);
-            
-                for(int dev = 0; dev < nsz[norm1][0]; ++ dev)
-                    Neib(dx,dy,dz,norm1,dev, ns[id][0][dev], ns[id][1][dev], ns[id][2][dev]);
-            
-                for(int dev = 0; dev < nsz[norm1][1]; ++ dev){
-                    FNeib(dx,dy,dz,norm1,dev,
-                    f1[id][0][dev],f1[id][1][dev], f1[id][2][dev],
-                    f2[id][0][dev],f2[id][1][dev], f2[id][2][dev]);
-                }
-                
-                id ++;
-            }
-        }
-    }
-}
-
-
-void JPS3DNeib::Neib(int dx, int dy, int dz, int norm1, int dev,
-    int& tx, int& ty, int& tz)
-{
-    switch(norm1)
-    {
-        case 0:
-            switch(dev)
-            {
-                case 0: tx=1; ty=0; tz=0; return;
-                case 1: tx=-1; ty=0; tz=0; return;
-                case 2: tx=0; ty=1; tz=0; return;
-                case 3: tx=1; ty=1; tz=0; return;
-                case 4: tx=-1; ty=1; tz=0; return;
-                case 5: tx=0; ty=-1; tz=0; return;
-                case 6: tx=1; ty=-1; tz=0; return;
-                case 7: tx=-1; ty=-1; tz=0; return;
-                case 8: tx=0; ty=0; tz=1; return;
-                case 9: tx=1; ty=0; tz=1; return;
-                case 10: tx=-1; ty=0; tz=1; return;
-                case 11: tx=0; ty=1; tz=1; return;
-                case 12: tx=1; ty=1; tz=1; return;
-                case 13: tx=-1; ty=1; tz=1; return;
-                case 14: tx=0; ty=-1; tz=1; return;
-                case 15: tx=1; ty=-1; tz=1; return;
-                case 16: tx=-1; ty=-1; tz=1; return;
-                case 17: tx=0; ty=0; tz=-1; return;
-                case 18: tx=1; ty=0; tz=-1; return;
-                case 19: tx=-1; ty=0; tz=-1; return;
-                case 20: tx=0; ty=1; tz=-1; return;
-                case 21: tx=1; ty=1; tz=-1; return;
-                case 22: tx=-1; ty=1; tz=-1; return;
-                case 23: tx=0; ty=-1; tz=-1; return;
-                case 24: tx=1; ty=-1; tz=-1; return;
-                case 25: tx=-1; ty=-1; tz=-1; return;
-            }
-        case 1:
-            tx = dx; ty = dy; tz = dz; return;
-        case 2:
-            switch(dev){
-                case 0:
-                    if(dz == 0){
-                        tx = 0; ty = dy; tz = 0; return;
-                    }else{
-                        tx = 0; ty = 0; tz = dz; return;
-                    }
-                case 1:
-                    if(dx == 0){
-                        tx = 0; ty = dy; tz = 0; return;
-                    }else{
-                        tx = dx; ty = 0; tz = 0; return;
-                    }
-                case 2:
-                    tx = dx; ty = dy; tz = dz; return;
-            }
-        case 3:
-            switch(dev){
-                case 0: tx = dx; ty =  0; tz =  0; return;
-                case 1: tx =  0; ty = dy; tz =  0; return;
-                case 2: tx =  0; ty =  0; tz = dz; return;
-                case 3: tx = dx; ty = dy; tz =  0; return;
-                case 4: tx = dx; ty =  0; tz = dz; return;
-                case 5: tx =  0; ty = dy; tz = dz; return;
-                case 6: tx = dx; ty = dy; tz = dz; return;
-            }
-    }
-}
-
-void JPS3DNeib::FNeib( int dx, int dy, int dz, int norm1, int dev,
-                          int& fx, int& fy, int& fz,
-                          int& nx, int& ny, int& nz)
-{
-    switch(norm1)
-    {
-        case 1:
-            switch(dev){
-                case 0: fx= 0; fy= 1; fz = 0; break;
-                case 1: fx= 0; fy=-1; fz = 0; break;
-                case 2: fx= 1; fy= 0; fz = 0; break;
-                case 3: fx= 1; fy= 1; fz = 0; break;
-                case 4: fx= 1; fy=-1; fz = 0; break;
-                case 5: fx=-1; fy= 0; fz = 0; break;
-                case 6: fx=-1; fy= 1; fz = 0; break;
-                case 7: fx=-1; fy=-1; fz = 0; break;
-            }
-            nx = fx; ny = fy; nz = dz;
-            // switch order if different direction
-            if(dx != 0){
-                fz = fx; fx = 0;
-                nz = fz; nx = dx;
-            }
-
-            if(dy != 0){
-                fz = fy; fy = 0;
-                nz = fz; ny = dy;
-            }
-            return;
-        case 2:
-            if(dx == 0){
-                switch(dev){
-                    case 0:
-                        fx = 0; fy = 0; fz = -dz;
-                        nx = 0; ny = dy; nz = -dz;
-                        return;
-                    case 1:
-                        fx = 0; fy = -dy; fz = 0;
-                        nx = 0; ny = -dy; nz = dz;
-                        return;
-                    case 2:
-                        fx = 1; fy = 0; fz = 0;
-                        nx = 1; ny = dy; nz = dz;
-                        return;
-                    case 3:
-                        fx = -1; fy = 0; fz = 0;
-                        nx = -1; ny = dy; nz = dz;
-                        return;
-                    case 4:
-                        fx = 1; fy = 0; fz = -dz;
-                        nx = 1; ny = dy; nz = -dz;
-                        return;
-                    case 5:
-                        fx = 1; fy = -dy; fz = 0;
-                        nx = 1; ny = -dy; nz = dz;
-                        return;
-                    case 6:
-                        fx = -1; fy = 0; fz = -dz;
-                        nx = -1; ny = dy; nz = -dz;
-                        return;
-                    case 7:
-                        fx = -1; fy = -dy; fz = 0;
-                        nx = -1; ny = -dy; nz = dz;
-                        return;
-                    // Extras
-                    case 8:
-                        fx = 1; fy = 0; fz = 0;
-                        nx = 1; ny = dy; nz = 0;
-                        return;
-                    case 9:
-                        fx = 1; fy = 0; fz = 0;
-                        nx = 1; ny = 0; nz = dz;
-                        return;
-                    case 10:
-                        fx = -1; fy = 0; fz = 0;
-                        nx = -1; ny = dy; nz = 0;
-                        return;
-                    case 11:
-                        fx = -1; fy = 0; fz = 0;
-                        nx = -1; ny = 0; nz = dz;
-                        return;
-                }
-            }
-            else if(dy == 0){
-                switch(dev){
-                    case 0:
-                        fx = 0; fy = 0; fz = -dz;
-                        nx = dx; ny = 0; nz = -dz;
-                        return;
-                    case 1:
-                        fx = -dx; fy = 0; fz = 0;
-                        nx = -dx; ny = 0; nz = dz;
-                        return;
-                    case 2:
-                        fx = 0; fy = 1; fz = 0;
-                        nx = dx; ny = 1; nz = dz;
-                        return;
-                    case 3:
-                        fx = 0; fy = -1; fz = 0;
-                        nx = dx; ny = -1;nz = dz;
-                        return;
-                    case 4:
-                        fx = 0; fy = 1; fz = -dz;
-                        nx = dx; ny = 1; nz = -dz;
-                        return;
-                    case 5:
-                        fx = -dx; fy = 1; fz = 0;
-                        nx = -dx; ny = 1; nz = dz;
-                        return;
-                    case 6:
-                        fx = 0; fy = -1; fz = -dz;
-                        nx = dx; ny = -1; nz = -dz;
-                        return;
-                    case 7:
-                        fx = -dx; fy = -1; fz = 0;
-                        nx = -dx; ny = -1; nz = dz;
-                        return;
-                    // Extras
-                    case 8:
-                        fx = 0; fy = 1; fz = 0;
-                        nx = dx; ny = 1; nz = 0;
-                        return;
-                    case 9:
-                        fx = 0; fy = 1; fz = 0;
-                        nx = 0; ny = 1; nz = dz;
-                        return;
-                    case 10:
-                        fx = 0; fy = -1; fz = 0;
-                        nx = dx; ny = -1; nz = 0;
-                        return;
-                    case 11:
-                        fx = 0; fy = -1; fz = 0;
-                        nx = 0; ny = -1; nz = dz;
-                        return;
-                }
-            }
-            else{// dz==0
-                switch(dev){
-                    case 0:
-                        fx = 0; fy = -dy; fz = 0;
-                        nx = dx; ny = -dy; nz = 0;
-                        return;
-                    case 1:
-                        fx = -dx; fy = 0; fz = 0;
-                        nx = -dx; ny = dy; nz = 0;
-                        return;
-                    case 2:
-                        fx =  0; fy = 0; fz = 1;
-                        nx = dx; ny = dy; nz = 1;
-                        return;
-                    case 3:
-                        fx =  0; fy = 0; fz = -1;
-                        nx = dx; ny = dy; nz = -1;
-                        return;
-                    case 4:
-                        fx = 0; fy = -dy; fz = 1;
-                        nx = dx; ny = -dy; nz = 1;
-                        return;
-                    case 5:
-                        fx = -dx; fy = 0; fz = 1;
-                        nx = -dx; ny = dy; nz = 1;
-                        return;
-                    case 6:
-                        fx = 0; fy = -dy; fz = -1;
-                        nx = dx; ny = -dy; nz = -1;
-                        return;
-                    case 7:
-                        fx = -dx; fy = 0; fz = -1;
-                        nx = -dx; ny = dy; nz = -1;
-                        return;
-                    // Extras
-                    case 8:
-                        fx =  0; fy = 0; fz = 1;
-                        nx = dx; ny = 0; nz = 1;
-                        return;
-                    case 9:
-                        fx = 0; fy = 0; fz = 1;
-                        nx = 0; ny = dy; nz = 1;
-                        return;
-                    case 10:
-                        fx =  0; fy = 0; fz = -1;
-                        nx = dx; ny = 0; nz = -1;
-                        return;
-                    case 11:
-                        fx = 0; fy = 0; fz = -1;
-                        nx = 0; ny = dy; nz = -1;
-                        return;
-                }
-            }
-        case 3:
-            switch(dev){
-                case 0:
-                    fx = -dx; fy = 0; fz = 0;
-                    nx = -dx; ny = dy; nz = dz;
-                    return;
-                case 1:
-                    fx = 0; fy = -dy; fz = 0;
-                    nx = dx; ny = -dy; nz = dz;
-                    return;
-                case 2:
-                    fx = 0; fy = 0; fz = -dz;
-                    nx = dx; ny = dy; nz = -dz;
-                    return;
-                // Need to check up to here for forced!
-                case 3:
-                    fx = 0; fy = -dy; fz = -dz;
-                    nx = dx; ny = -dy; nz = -dz;
-                    return;
-                case 4:
-                    fx = -dx; fy = 0; fz = -dz;
-                    nx = -dx; ny = dy; nz = -dz;
-                    return;
-                case 5:
-                    fx = -dx; fy = -dy; fz = 0;
-                    nx = -dx; ny = -dy; nz = dz;
-                    return;
-                // Extras
-                case 6:
-                    fx = -dx; fy = 0; fz = 0;
-                    nx = -dx; ny = 0; nz = dz;
-                    return;
-                case 7:
-                    fx = -dx; fy = 0; fz = 0;
-                    nx = -dx; ny = dy; nz = 0;
-                    return;
-                case 8:
-                    fx = 0; fy = -dy; fz = 0;
-                    nx = 0; ny = -dy; nz = dz;
-                    return;
-                case 9:
-                    fx = 0; fy = -dy; fz = 0;
-                    nx = dx; ny = -dy; nz = 0;
-                    return;
-                case 10:
-                    fx = 0; fy = 0; fz = -dz;
-                    nx = 0; ny = dy; nz = -dz;
-                    return;
-                case 11:
-                    fx = 0; fy = 0; fz = -dz;
-                    nx = dx; ny = 0; nz = -dz;
-                    return;
-            }
-    }
-}
\ No newline at end of file
(END)
