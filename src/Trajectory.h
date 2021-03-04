#include <Eigen/Dense>

typedef Eigen::Matrix<float, Eigen::Dynamic, 1> Vector;
typedef Eigen::Matrix<std::int64_t, 1, Eigen::Dynamic> Int64Matrix;
typedef Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> Matrix;

class Trajectory
{
public:
	Trajectory(int dimension);
	
	void addPoint(Vector point, std::int64_t timeStamp);
	
	void save(std::string filename);

private:
	int dimension;
	Int64Matrix times;
	Matrix points;
};
