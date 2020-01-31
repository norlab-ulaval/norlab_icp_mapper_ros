#include <Eigen/Dense>

typedef Eigen::Matrix<float, Eigen::Dynamic, 1> Vector;
typedef Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> Matrix;

class Trajectory
{
public:
	Trajectory(int dimension);
	
	void addPoint(Vector point);
	
	void save(std::string filename);

private:
	int dimension;
	Matrix points;
};
