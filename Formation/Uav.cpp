#include "Uav.h"
#include <cmath>
#include "funciton.h"
#include "parameter.h"

//���¼��ٶ�
void Uav::update_acceleration(axis acce) {
	m_acceleration = acce;
	limit(m_acceleration, Max_acceleration);
	return;
}
//�����ٶ�
void Uav::update_velocity() {
	m_velocity = m_velocity + delt * m_acceleration;
	limit(m_velocity, Max_velocity);
	return;
}
//����λ��
void Uav::update_position() {
	m_position = m_position + delt * m_velocity;
	return;
}

//���º���ƫ�ǡ����
void Uav::update_angle() {
	m_phi = atan2(m_velocity.z, sqrt(pow(m_velocity.x, 2) + pow(m_velocity.y, 2)));
	m_theta = atan2(m_velocity.y, m_velocity.x);
	return;
}