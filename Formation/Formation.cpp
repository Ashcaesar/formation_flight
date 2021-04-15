#include <iostream>
#include <iomanip>
#include <fstream>
#include <ctime>
#include "Uav.h"
#include "funciton.h"
#include "parameter.h"

using namespace std;
Uav uav[SIZE + 1];
Uav *p = uav;

int main()
{
	double dis_to_target, angle_to_target;
	axis p_cg = { 0,0,0 }, temp_acceleration = { 0,0,0 };
	srand((unsigned)time(NULL));
	ofstream outfile_index, outfile_pos, outfile_vel, outfile_acce, outfile_form, outfile_formdispersion;
	outfile_index.open("index.dat", ios::out | ios::trunc);
	outfile_pos.open("position.dat", ios::out | ios::trunc);
	outfile_vel.open("velocity.dat", ios::out | ios::trunc);
	outfile_acce.open("acceleration.dat", ios::out | ios::trunc);
	outfile_form.open("finalpos.dat", ios::out | ios::trunc);
	outfile_formdispersion.open("formdispersion.dat", ios::out | ios::trunc);

	/*-------------------------集结部分-------------------------*/
	cout << "*-------------------------集结模块-------------------------*" << endl;
	double t = 0;
	int num_crash = 0;
	initial_assemble();
	while (t<=100) {
		for (int i = 1; i <= SIZE; i++) {
			outfile_pos << setprecision(5) << (p + i)->m_position.x << " " << (p + i)->m_position.y << " " << (p + i)->m_position.z << endl;
			outfile_vel << setprecision(5) << (p + i)->m_velocity.x << " " << (p + i)->m_velocity.y << " " << (p + i)->m_velocity.z << endl;
			outfile_acce << setprecision(5) << (p + i)->m_acceleration.x << " " << (p + i)->m_acceleration.y << " " << (p + i)->m_acceleration.z << endl;
		}
		p_cg = get_avr_position();
		dis_to_target = get_dis(p_cg, p_assemble);
		angle_to_target = get_angle(p_cg, p_assemble);
		outfile_index << setprecision(5) << dispersion_of_vel() << " " << dispersion_of_pos() << " " << dis_to_target << " " << angle_to_target << " " << get_min_dis() << endl;
		if (dis_to_target < 50 && angle_to_target < 30) break;
		for (int i = 1; i <= SIZE; i++) (p + i)->update_acceleration(get_acceleration_assemble(i));
		for (int i = 1; i <= SIZE; i++) {
			(p + i)->update_velocity();
			(p + i)->update_position();
			(p + i)->update_angle();
		}
		num_crash += crash();
		t += delt;
	}
	cout << "集结完成，用时" << t << "秒，发生" << num_crash << "次碰撞" << endl << endl;

	/*-------------------------编队部分-------------------------*/
	cout << "*-------------------------编队模块-------------------------*" << endl;
	t = 0;
	num_crash = 0;
	initial_form();
	while (t <= 100) {
		for (int i = 1; i <= SIZE; i++) {
			outfile_pos << setprecision(5) << (p + i)->m_position.x << " " << (p + i)->m_position.y << " " << (p + i)->m_position.z << endl;
			outfile_vel << setprecision(5) << (p + i)->m_velocity.x << " " << (p + i)->m_velocity.y << " " << (p + i)->m_velocity.z << endl;
			outfile_acce << setprecision(5) << (p + i)->m_acceleration.x << " " << (p + i)->m_acceleration.y << " " << (p + i)->m_acceleration.z << endl;
		}
		p_cg = get_avr_position();
		dis_to_target = get_dis(p_cg, p_final);
		angle_to_target = get_angle(p_cg, p_final);
		outfile_index << setprecision(5) << dispersion_of_vel() << " " << dispersion_of_pos() << " " << dis_to_target << " " << angle_to_target << " " << get_min_dis() << endl;
		outfile_formdispersion << dispersion_of_form() << endl;
		if (dis_to_target < 50 && angle_to_target < 20) break;
		for (int i = 1; i <= SIZE; i++) (p + i)->m_acceleration = get_acceleration_form(i);
		for (int i = 0; i <= SIZE; i++) {
			(p + i)->m_velocity = (p + i)->m_velocity + delt * (p + i)->m_acceleration;
			(p + i)->update_position();
			(p + i)->update_angle();
		}
		num_crash += crash();
		t += delt;
	}
	cout << "编队完成，用时" << t << "秒，发生" << num_crash << "次碰撞" << endl;
	for (int i = 1; i <= SIZE; i++) outfile_form << setprecision(5) << (p + i)->m_position.x << " " << (p + i)->m_position.y << " " << (p + i)->m_position.z << endl;
	outfile_pos.close();
	outfile_vel.close();
	outfile_acce.close();
	outfile_index.close();
	outfile_form.close();
}

