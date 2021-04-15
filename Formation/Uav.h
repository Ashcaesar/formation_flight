#ifndef UAV_H_
#define UAV_H_
#include "funciton.h"

class Uav {
public:
	axis m_acceleration;
	axis m_velocity;
	axis m_position;
	double m_phi;
	double m_theta;
	Uav() :m_acceleration{ 0,0,0 }, m_velocity{ 0,0,0 }, m_position{ 0,0,0 }, m_phi(0), m_theta(0)
	{}
	void update_acceleration(axis);
	void update_velocity();
	void update_position();
	void update_angle();
};

#endif // !UAV_H_

