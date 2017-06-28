#ifndef __CLUSTERING_LIB_H
#define __CLUSTERING_LIB_H

#include <pcl/point_types.h>

namespace leo {
	struct cloud {
		int id;
		pcl::PointXYZ center_point;
		pcl::PointCloud<pcl::PointXYZ> points;
	};

	typedef struct cloud cloud;
}

#endif