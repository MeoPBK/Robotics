// A mapping node for educational purposes

// roscpp
#include "ros/ros.h"

// Messages and services that we need
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/GetMap.h"
#include "nav_msgs/OccupancyGrid.h"
#include "std_srvs/Empty.h"

// For transform support
#include "tf/transform_listener.h"
#include "tf/message_filter.h"
#include "message_filters/subscriber.h"

// someone was lazy here...
#define SWAP(a, b)  {a ^= b; b ^= a; a ^= b;}

constexpr int map_square_side = 1000;
using namespace std;

class MappingNode {

public:
	MappingNode(ros::NodeHandle n);
	~MappingNode();

private:
	ros::NodeHandle nodeHandle;

	// we use the OccupancyGrid structure representing the reflection map
	nav_msgs::OccupancyGrid map;

	// subscribe to laser, use a msg_filter to ensure that we can associate an odometry pose with the scan
	tf::TransformListener tfListener;
	message_filters::Subscriber<sensor_msgs::LaserScan> laserScanSubscriber;
	tf::MessageFilter<sensor_msgs::LaserScan> laserScanFilter;

	// in order to visualize the map, we have to publish it
	ros::Publisher mapPublisher;
	ros::ServiceServer mapService;

	// Message and service callbacks
	void laserReceived( const sensor_msgs::LaserScanConstPtr& laser_scan );
	bool mapRequested( nav_msgs::GetMap::Request& req, nav_msgs::GetMap::Response& res );

	///////////////////////////////////////////////////
	// <helpful methods>

	// implementation of the bresenham algorithm: http://en.wikipedia.org/wiki/Bresenham%27s_line_algorithm
	//  returns all points from the starting point (x0, y0) and end point (x1, y1)
	//  starting and end point are included in the returned result.
	vector<pair<int, int> > bresenhamLine( int x0, int y0, int x1, int y1 );
	
	// use this method to update the reflection map
	void updateReflectionMapValue(int x, int y, double value);

	// </helpful methods>
	///////////////////////////////////////////////////
	
	///////////////////////////////////////////////////
	// <your declarations>

    pair<int, int> coordinatesToIndices(float x, float y);
    int hits[map_square_side][map_square_side];
    int misses[map_square_side][map_square_side];
	// </your declarations>
	///////////////////////////////////////////////////
};

MappingNode::MappingNode(ros::NodeHandle n) :
	nodeHandle(n),
	laserScanSubscriber(nodeHandle, "scan", 5),
	laserScanFilter(laserScanSubscriber, tfListener, "odom", 5),
    hits {0},
    misses {0}

{
	// initialize map meta data
	// map is 20 x 20 meters, resolution is 0.02 meters per grid cell
	this->map.info.resolution = 0.02;
	this->map.info.width = 1000;
	this->map.info.height = 1000;
	this->map.info.origin.position.x = -this->map.info.resolution * this->map.info.width / 2.0;
	this->map.info.origin.position.y = -this->map.info.resolution * this->map.info.height / 2.0;
	this->map.info.origin.position.z = 0.0;
	this->map.info.origin.orientation.x = 0.0;
	this->map.info.origin.orientation.y = 0.0;
	this->map.info.origin.orientation.z = 0.0;
	this->map.info.origin.orientation.w = 1.0;
	this->map.data.resize( this->map.info.width * this->map.info.height );

	// initialize map
	for (unsigned int i = 0; i < this->map.info.width; i++) {
		for (unsigned int j = 0; j < this->map.info.height; j++) {
			int index = j*this->map.info.height + i;
			this->map.data[index] = -1; // -1 means unknown
		}
	}
	///////////////////////////////////////////////////
	// <your code>
    // Initializing the two arrays using list initialization up there
	// </your code>
	///////////////////////////////////////////////////

	// subscribe to laser scan
	laserScanFilter.registerCallback( boost::bind( &MappingNode::laserReceived, this, _1 ) );
//	laserScanFilter.setTolerance(ros::Duration(0.01));

	// publish the map we're building
	this->mapPublisher = this->nodeHandle.advertise<nav_msgs::OccupancyGrid>( "map", 2 );
	this->mapService = this->nodeHandle.advertiseService( "map", &MappingNode::mapRequested, this );

	// publish the unknown map for the first time
	this->mapPublisher.publish( this->map );
}

MappingNode::~MappingNode() {
}


void MappingNode::laserReceived( const sensor_msgs::LaserScanConstPtr& laserScan ) {
	// what's the robot's odometry when this scan was taken?
	tf::Stamped<tf::Pose> ident( tf::Transform( tf::createIdentityQuaternion(), tf::Vector3(0,0,0) ), laserScan->header.stamp, "laser" );
	tf::Stamped<tf::Pose> odomPose;
	try {
		this->tfListener.transformPose( "odom", ident, odomPose );
	}
	catch( tf::TransformException e ) {
		ROS_WARN( "Failed to compute odom pose, skipping scan (%s)", e.what() );
		return;
	}

    // transform pose of robot to map coordinates
    int robotX = odomPose.getOrigin().x() / this->map.info.resolution + this->map.info.width / 2;
    int robotY = odomPose.getOrigin().y() / this->map.info.resolution + this->map.info.height / 2;
    double robotTheta = tf::getYaw( odomPose.getRotation() );

    ///////////////////////////////////////////////////
    // <your code>
    const float angle_increment = laserScan->angle_increment;
    const float angle_min = laserScan->angle_min;
    const float angle_max = laserScan->angle_max;
    const float range_min = laserScan->range_min;
    const float range_max = laserScan->range_max;
    vector<pair<float, float>> scanned_points_map_frame {};
    float angle = angle_min;
    int i = 0;
    while (angle < angle_max) {
        const float r = laserScan->ranges[i];
        if (!isnan(r) && r <= range_max && r >= range_min) {
            // Decode the scanned points (which are in laser frame)
            // The suffix '_laser' means laser frame, '_map' - map frame, assuming laser and odom frame are the same
            const float ray_x_laser = r * cos(angle);
            const float ray_y_laser = r * sin(angle);
            /* Transform the data from laser frame to map frame:
             *
             * | ray_x_map |     | cos(robotTheta) -sin(robotTheta) robot_odom_x |     | ray_x_laser |
             * | ray_y_map |  =  | sin(robotTheta)  cos(robotTheta) robot_odom_y |  *  | ray_y_laser |
             * |     1     |     |       0                 0             1       |     |      1      |
             *
             */
            const float ray_x_map = cos(robotTheta) * ray_x_laser - sin(robotTheta) * ray_y_laser + odomPose.getOrigin().x();
            const float ray_y_map = sin(robotTheta) * ray_x_laser + cos(robotTheta) * ray_y_laser + odomPose.getOrigin().y();
            scanned_points_map_frame.push_back(pair<float, float>(ray_x_map, ray_y_map));
        }
        i++;
        angle += angle_increment;
    }
    for(const auto& point : scanned_points_map_frame)
    {
        // For each point, get all grids that are passed by the ray and update the corresponding values
        const auto beam_end_indices = coordinatesToIndices(get<0>(point), get<1>(point));

        // Check if the point isn't outside of the map
        if (get<0>(beam_end_indices) >= map_square_side || get<0>(beam_end_indices) < 0
                || get<1>(beam_end_indices) >= map_square_side || get<1>(beam_end_indices) < 0) {
            continue;
        }

        // Get all cells touched by the laser beam
        auto beam_path = bresenhamLine(robotX, robotY, get<0>(beam_end_indices), get<1>(beam_end_indices));

        // Check if the line direction needs to be reversed
        const bool steep = abs(get<1>(beam_end_indices) - robotY) > abs(get<0>(beam_end_indices) - robotX);
        if ((!steep && robotX > get<0>(beam_end_indices)) || (steep && robotY > get<1>(beam_end_indices))) {
            reverse(beam_path.begin(), beam_path.end());
        }

        // The last element of the vector is the cell where the beam hit the obstacle
        const auto hit_cell = beam_path.back();
        const int hit_cell_i = get<0>(hit_cell);
        const int hit_cell_j = get<1>(hit_cell);

        // Update the hit cell and remove it from the list
        hits[hit_cell_i][hit_cell_j]++;
        beam_path.pop_back();

        // The beam went through all the remaining cells - update them too
        for (const auto& missed_cell : beam_path) {
            misses[get<0>(missed_cell)][get<1>(missed_cell)]++;
        }

        // Finally, update the changed occupancy grid values
        const float hit_cell_value = float(hits[hit_cell_i][hit_cell_j]) / float(hits[hit_cell_i][hit_cell_j] + misses[hit_cell_i][hit_cell_j]);
        updateReflectionMapValue(hit_cell_i, hit_cell_j, hit_cell_value);
        for (const auto& missed_cell : beam_path) {
            const int i = get<0>(missed_cell);
            const int j = get<1>(missed_cell);
            updateReflectionMapValue(i, j, float(hits[i][j]) / float(hits[i][j] + misses[i][j]));
        }
    }
    // </your code>
	///////////////////////////////////////////////////


	// publish result
	this->map.header.stamp = ros::Time::now();
	this->map.header.frame_id = "/odom";
	this->mapPublisher.publish( this->map );
}

void MappingNode::updateReflectionMapValue(int i, int j, double value) {
	int index = j*this->map.info.width + i;
	this->map.data[index] = 100*value; // *100 is just for better visibility in visualization
}

vector<pair<int, int> > MappingNode::bresenhamLine( int x0, int y0, int x1, int y1 ) {
	vector<pair<int, int> > result;

	bool steep = abs(y1 - y0) > abs(x1 - x0);

	if( steep ) {
		SWAP( x0, y0 );
		SWAP( x1, y1 );
	}
	if( x0 > x1 ) {
		SWAP( x0, x1 );
		SWAP( y0, y1 );
	}

	int deltax = x1 - x0;
	int deltay = abs(y1 - y0);
	int error = deltax / 2;
	int y = y0;
	int ystep = ( y0 < y1 ) ? 1 : -1;

	for( int x = x0; x <= x1; x++ ) {
		if ( steep ) {
			// cell (y,x) is crossed
			result.push_back(pair<int, int>(y, x));
		} else {
			// cell (x,y) is crossed
			result.push_back(pair<int, int>(x, y));
		}

		error = error - deltay;
		if( error < 0 ) {
			y += ystep;
			error += deltax;
		}
	}

	return result;
}

bool MappingNode::mapRequested( nav_msgs::GetMap::Request &request, nav_msgs::GetMap::Response &response ) {
	if( this->map.info.width && this->map.info.height )	{
		this->map.header.stamp = ros::Time::now();
		response.map = this->map;
		return true;
	}
	else
		return false;
}

pair<int, int> MappingNode::coordinatesToIndices(float x, float y) {
    int i_value = x / map.info.resolution + map.info.width / 2;
    int j_value = y / map.info.resolution + map.info.height / 2;
    return pair<int, int>(i_value, j_value);
}


int main( int argc, char** argv ) {
	ros::init( argc, argv, "mapper" );
	ros::NodeHandle nh;

	MappingNode mn( nh );

	ros::spin();

	return 0;
}
