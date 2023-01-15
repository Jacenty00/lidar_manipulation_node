/*
 * geometry.cpp
 *
 *  Created on: Apr 11, 2013
 *      Author: jorge
 */

#include "../../include/yocs_math_toolkit/common.hpp"
#include "../../include/yocs_math_toolkit/geometry.hpp"

namespace mtk
{

	double wrapAngle(double a)
	{
		a = fmod(a + M_PI, 2 * M_PI);
		if (a < 0.0)
			a += 2.0 * M_PI;
		return a - M_PI;
	}

	double roll(const tf::Transform &tf)
	{
		double roll, pitch, yaw;
		tf::Matrix3x3(tf.getRotation()).getRPY(roll, pitch, yaw);
		return roll;
	}

	double roll(geometry_msgs::Pose pose)
	{
		tf::Transform tf;
		pose2tf(pose, tf);
		return roll(tf);
	}

	double roll(geometry_msgs::PoseStamped pose)
	{
		return roll(pose.pose);
	}

	double pitch(const tf::Transform &tf)
	{
		double roll, pitch, yaw;
		tf::Matrix3x3(tf.getRotation()).getRPY(roll, pitch, yaw);
		return pitch;
	}

	double pitch(geometry_msgs::Pose pose)
	{
		tf::Transform tf;
		pose2tf(pose, tf);
		return pitch(tf);
	}

	double pitch(geometry_msgs::PoseStamped pose)
	{
		return pitch(pose.pose);
	}

	double yaw(const tf::Transform &tf)
	{
		return tf::getYaw(tf.getRotation());
	}

	double yaw(geometry_msgs::Pose pose)
	{
		return tf::getYaw(pose.orientation);
	}

	double yaw(geometry_msgs::PoseStamped pose)
	{
		return yaw(pose.pose);
	}

	double distance2D(double x, double y)
	{
		return std::sqrt(std::pow(x, 2) + std::pow(y, 2));
	}

	double distance2D(const tf::Point &p)
	{
		return std::sqrt(std::pow(p.x(), 2) + std::pow(p.y(), 2));
	}

	double distance2D(double ax, double ay, double bx, double by)
	{
		return std::sqrt(std::pow(ax - bx, 2) + std::pow(ay - by, 2));
	}

	double distance2D(const tf::Point &p1, const tf::Point &p2)
	{
		return std::sqrt(std::pow(p2.x() - p1.x(), 2) + std::pow(p2.y() - p1.y(), 2));
	}

	double distance2D(geometry_msgs::Point a, geometry_msgs::Point b)
	{
		return distance2D(tf::Vector3(a.x, a.y, a.z), tf::Vector3(b.x, b.y, b.z));
	}

	double distance2D(geometry_msgs::Pose a, geometry_msgs::Pose b)
	{
		return distance2D(a.position, b.position);
	}

	double distance2D(const tf::Transform &a, const tf::Transform &b)
	{
		return distance2D(a.getOrigin(), b.getOrigin());
	}

	double distance3D(double x, double y, double z)
	{
		return std::sqrt(std::pow(x, 2) + std::pow(y, 2) + std::pow(z, 2));
	}

	double distance3D(const tf::Point &p)
	{
		return std::sqrt(std::pow(p.x(), 2) + std::pow(p.y(), 2) + std::pow(p.z(), 2));
	}

	double distance3D(double ax, double ay, double az, double bx, double by, double bz)
	{
		return std::sqrt(std::pow(ax - bx, 2) + std::pow(ay - by, 2) + std::pow(az - bz, 2));
	}

	double distance3D(const tf::Point &p1, const tf::Point &p2)
	{
		return std::sqrt(std::pow(p2.x() - p1.x(), 2) + std::pow(p2.y() - p1.y(), 2) + std::pow(p2.z() - p1.z(), 2));
	}

	double distance3D(geometry_msgs::Point a, geometry_msgs::Point b)
	{
		return distance3D(tf::Vector3(a.x, a.y, a.z), tf::Vector3(b.x, b.y, b.z));
	}

	double distance3D(geometry_msgs::Pose a, geometry_msgs::Pose b)
	{
		return distance3D(a.position, b.position);
	}

	double distance3D(const tf::Transform &a, const tf::Transform &b)
	{
		return distance3D(a.getOrigin(), b.getOrigin());
	}

	double heading(const tf::Vector3 &p)
	{
		return std::atan2(p.y(), p.x());
	}

	double heading(geometry_msgs::Point p)
	{
		return heading(tf::Vector3(p.x, p.y, p.z));
	}

	double heading(geometry_msgs::Pose p)
	{
		return heading(p.position);
	}

	double heading(const tf::Transform &t)
	{
		return heading(t.getOrigin());
	}

	double heading(const tf::Vector3 &a, const tf::Vector3 &b)
	{
		return std::atan2(b.y() - a.y(), b.x() - a.x());
	}

	double heading(geometry_msgs::Point a, geometry_msgs::Point b)
	{
		return heading(tf::Vector3(a.x, a.y, a.z), tf::Vector3(b.x, b.y, b.z));
	}

	double heading(geometry_msgs::Pose a, geometry_msgs::Pose b)
	{
		return heading(a.position, b.position);
	}

	double heading(const tf::Transform &a, const tf::Transform &b)
	{
		return heading(a.getOrigin(), b.getOrigin());
	}

	double minAngle(const tf::Quaternion &a, const tf::Quaternion &b)
	{
		return tf::angleShortestPath(a, b);
	}

	double minAngle(geometry_msgs::Quaternion a, geometry_msgs::Quaternion b)
	{
		return minAngle(tf::Quaternion(a.x, a.y, a.z, a.w), tf::Quaternion(b.x, b.y, b.z, b.w));
	}

	double minAngle(geometry_msgs::Pose a, geometry_msgs::Pose b)
	{
		return minAngle(a.orientation, b.orientation);
	}

	double minAngle(const tf::Transform &a, const tf::Transform &b)
	{
		return minAngle(a.getRotation(), b.getRotation());
	}

	bool sameFrame(const geometry_msgs::PoseStamped &a, const geometry_msgs::PoseStamped &b)
	{
		return sameFrame(a.header.frame_id, b.header.frame_id);
	}

	bool sameFrame(const std::string &frame_a, const std::string &frame_b)
	{
		if (frame_a.length() == 0 && frame_b.length() == 0)
		{
			ROS_WARN("Comparing two empty frame ids (considered as the same frame)");
			return true;
		}

		if (frame_a.length() == 0 || frame_b.length() == 0)
		{
			ROS_WARN("Comparing %s%s with an empty frame id (considered as different frames)",
					 frame_a.c_str(), frame_b.c_str());
			return false;
		}

		int start_a = frame_a.at(0) == '/' ? 1 : 0;
		int start_b = frame_b.at(0) == '/' ? 1 : 0;

		return frame_a.compare(start_a, frame_a.length(), frame_b, start_b, frame_b.length()) == 0;
	}

	// –––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––

	double distance(const Vector2D &p1, const Vector2D &p2)
	{
		// Calculate the Euclidean distance between two points
		double dx = p1.x - p2.x;
		double dy = p1.y - p2.y;
		return std::sqrt(dx * dx + dy * dy);
	}

	// –––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––

	double shortestDistance(const Vector2D &point, double s1x, double s1y, double s2x, double s2y,
							double s3x, double s3y, double s4x, double s4y)
	{

		Square square;

		square.p1 = {s1x, s1y};
		square.p2 = {s2x, s2y};
		square.p3 = {s3x, s3y};
		square.p4 = {s4x, s4y};

		// Check if the point is inside the square
		if (point.x >= square.p1.x && point.x <= square.p2.x &&
			point.y >= square.p1.y && point.y <= square.p3.y)
		{
			// Point is inside the square, so return the distance to the closest boundary point
			double min_distance = distance(point, square.p1);
			min_distance = std::min(min_distance, distance(point, square.p2));
			min_distance = std::min(min_distance, distance(point, square.p3));
			min_distance = std::min(min_distance, distance(point, square.p4));
			return min_distance;
		}
		else
		{
			// Point is outside the square, so calculate the distance to each side and return the minimum
			double min_distance = distance(point, Vector2D{square.p1.x, square.p1.y});
			min_distance = std::min(min_distance, distance(point, Vector2D{square.p2.x, square.p1.y}));
			min_distance = std::min(min_distance, distance(point, Vector2D{square.p2.x, square.p3.y}));
			min_distance = std::min(min_distance, distance(point, Vector2D{square.p1.x, square.p3.y}));
			return min_distance;
		}
	}

	// –––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––

	double pointSegmentDistance(double px, double py, double s1x, double s1y, double s2x, double s2y)
	{
		// Return minimum distance between line segment s1-s2 and point p

		double l = distance2D(s1x, s1y, s2x, s2y); // i.e. |p2 - p1|^2
		if (l == 0.0)
			return distance2D(px, py, s1x, s1y); // s1 == s2 case

		// Consider the line extending the segment, parameterized as s1 + t (s2 - s1).
		// We find projection of point p onto the line.
		// It falls where t = [(p - s1) . (s2 - s1)] / |s2 - s1|^2
		double t = (-s1x * (s2x - s1x) - s1y * (s2y - s1y)) / l;

		if (t < 0.0) // Beyond the s1 end of the segment
			return distance2D(s1x, s1y);

		if (t > 1.0) // Beyond the s2 end of the segment
			return distance2D(s2x, s2y);

		// Projection falls on the segment
		return distance2D(s1x + t * (s2x - s1x), s1y + t * (s2y - s1y));
	}

	// –––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––

	bool raySegmentIntersection(double r1x, double r1y, double r2x, double r2y,
								double s1x, double s1y, double s2x, double s2y,
								double &ix, double &iy, double &distance)
	{
		double r, s, d;
		// Make sure the lines aren't parallel
		if ((r2y - r1y) / (r2x - r1x) != (s2y - s1y) / (s2x - s1x))
		{
			d = (((r2x - r1x) * (s2y - s1y)) - (r2y - r1y) * (s2x - s1x));
			if (d != 0)
			{
				r = (((r1y - s1y) * (s2x - s1x)) - (r1x - s1x) * (s2y - s1y)) / d;
				s = (((r1y - s1y) * (r2x - r1x)) - (r1x - s1x) * (r2y - r1y)) / d;
				if (r >= 0)
				{
					if (s >= 0 && s <= 1)
					{
						ix = r1x + r * (r2x - r1x);
						iy = r1y + r * (r2y - r1y);
						distance = distance2D(ix, iy);
						return true;
					}
				}
			}
		}
		return false;
	}

	// –––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––

	bool rayCircleIntersection(double rx, double ry, double cx, double cy, double radius,
							   double &ix, double &iy, double &distance)
	{
		double a = rx * rx + ry * ry;
		double bBy2 = rx * cx + ry * cy;
		double c = cx * cx + cy * cy - radius * radius;

		double pBy2 = bBy2 / a;
		double q = c / a;

		double discriminant = pBy2 * pBy2 - q;
		if (discriminant < 0)
			return false;

		// if disc == 0 ... dealt with later
		double tmpSqrt = std::sqrt(discriminant);
		double abScalingFactor1 = -pBy2 + tmpSqrt;
		double abScalingFactor2 = -pBy2 - tmpSqrt;

		ix = -rx * abScalingFactor1;
		iy = -ry * abScalingFactor1;
		distance = distance2D(ix, iy);

		// discard the backward-pointing half of the ray
		if ((ix * rx < 0.0) && (iy * ry < 0.0))
			return false;

		if (discriminant == 0) // abScalingFactor1 == abScalingFactor2
			return true;

		// Check if the second intersection point is close (naively inefficient)
		double i2x = -rx * abScalingFactor2;
		double i2y = -ry * abScalingFactor2;
		double distance2 = distance2D(i2x, i2y);

		if (distance2 < distance)
		{
			ix = i2x;
			iy = i2y;
			distance = distance2;
		}

		return true;
	}
	// –––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––

	Vector2D Multiply(const Vector2D &v, double scalar)
	{
		Vector2D result;
		result.x = v.x * scalar;
		result.y = v.y * scalar;
		return result;
	}

	// –––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––

	void calculateRayDirection(Ray &ray, const Vector2D &start, const Vector2D &end)
	{
		ray.origin = start;
		ray.direction.x = end.x - start.x;
		ray.direction.y = end.y - start.y;

		double length = std::sqrt(ray.direction.x * ray.direction.x + ray.direction.y * ray.direction.y);
		ray.direction.x /= length;
		ray.direction.y /= length;
	}

	// –––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––

	// Calculates the intersection point between a ray and a line defined by two points
	bool intersectRayLine(const Ray &ray, const Vector2D &A, const Vector2D &B, Vector2D &intersection)
	{
		// Calculate the intersection point of the ray and the line
		double t = (B.y - A.y) * (ray.origin.x - A.x) - (B.x - A.x) * (ray.origin.y - A.y);
		t /= (B.x - A.x) * ray.direction.y - (B.y - A.y) * ray.direction.x;
		intersection = ray.origin + Multiply(ray.direction, t);

		// Check if the intersection point is within the bounds of the line segment
		return (std::min(A.x, B.x) <= intersection.x && intersection.x <= std::max(A.x, B.x)) &&
			   (std::min(A.y, B.y) <= intersection.y && intersection.y <= std::max(A.y, B.y));
	}

	// –––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––

	// Check if a point is on a line segment defined by two points
	bool isPointOnLineSegment(const Vector2D &point, const Vector2D &A, const Vector2D &B)
	{
		// Check if the point is within the bounds of the line segment
		return (std::min(A.x, B.x) <= point.x && point.x <= std::max(A.x, B.x)) &&
			   (std::min(A.y, B.y) <= point.y && point.y <= std::max(A.y, B.y));
	}

	// –––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––

	bool intersectRaySquare(const Ray &ray, double s1x, double s1y, double s2x, double s2y,
							double s3x, double s3y, double s4x, double s4y,
							double &ix, double &iy, double &distance)
	{

		Square square;

		square.p1 = {s1x, s1y};
		square.p2 = {s2x, s2y};
		square.p3 = {s3x, s3y};
		square.p4 = {s4x, s4y};

		// Check if the ray intersects the first side of the square
		Vector2D intersection;
		if (intersectRayLine(ray, square.p1, square.p2, intersection))
		{
			// Check if the intersection point is within the bounds of the side
			if (isPointOnLineSegment(intersection, square.p1, square.p2))
			{
				ix = intersection.x;
				iy = intersection.y;
				distance = distance2D(ix, iy);
				return true;
			}
		}

		// Check if the ray intersects the second side of the square
		if (intersectRayLine(ray, square.p2, square.p3, intersection))
		{
			// Check if the intersection point is within the bounds of the side
			if (isPointOnLineSegment(intersection, square.p2, square.p3))
			{
				ix = intersection.x;
				iy = intersection.y;
				distance = distance2D(ix, iy);
				return true;
			}
		}

		// Check if the ray intersects the second side of the square
		if (intersectRayLine(ray, square.p3, square.p4, intersection))
		{
			// Check if the intersection point is within the bounds of the side
			if (isPointOnLineSegment(intersection, square.p3, square.p4))
			{
				ix = intersection.x;
				iy = intersection.y;
				distance = distance2D(ix, iy);
				return true;
			}
		}

		// Check if the ray intersects the second side of the square
		if (intersectRayLine(ray, square.p4, square.p1, intersection))
		{
			// Check if the intersection point is within the bounds of the side
			if (isPointOnLineSegment(intersection, square.p4, square.p1))
			{
				ix = intersection.x;
				iy = intersection.y;
				distance = distance2D(ix, iy);
				return true;
			}
		}

		// If the ray does not intersect any of the sides, return a default Vector2D object
		return false;
	}

} /* namespace mtk */
