#include <iostream>
#include <iomanip>
#include <cmath>
#include <Eigen/Dense>
#include "Uav.h"
#include "funciton.h"
#include "parameter.h"

using namespace std;
using namespace Eigen;

extern Uav* p;
Matrix<int, SIZE + 1, SIZE + 1> A;
Isometry3d T;
Vector3d form[SIZE + 1];
axis pos_axis[SIZE + 1];
axis form_axis[SIZE + 1];

/*-------------------------ͨ�ú���-------------------------*/
//�ṹ�����������
axis operator +(const axis& a, const axis& b) {
	axis result;
	result.x = a.x + b.x;
	result.y = a.y + b.y;
	result.z = a.z + b.z;
	return result;
}
axis operator -(const axis &a, const axis &b) {
	axis result;
	result.x = a.x - b.x;
	result.y = a.y - b.y;
	result.z = a.z - b.z;
	return result;
}
double operator *(const axis &a, const axis &b) {
	return (a.x*b.x + a.y*b.y + a.z*b.z);
}
axis operator *(const double &n, const axis &a) {
	axis result;
	result.x = n * a.x;
	result.y = n * a.y;
	result.z = n * a.z;
	return result;
}
axis operator /(const axis &a, const double &n) {
	axis result;
	result.x = a.x / n;
	result.y = a.y / n;
	result.z = a.z / n;
	return result;
}

//���ֵ����
void limit(axis &num, axis &max) {
	if (num.x > max.x) num.x = max.x;
	else if (num.x < -max.x) num.x = -max.x;
	if (num.y > max.y) num.y = max.y;
	else if (num.y < -max.y) num.y = -max.y;
	if (num.z > max.z) num.z = max.z;
	else if (num.z < -max.z) num.z = -max.z;
}

//������� 2D��3D
double get_dis(double x, double y) {
	return sqrt(pow(x, 2) + pow(y, 2));
}
double get_dis(axis a, axis b) {
	return sqrt(pow(a.x - b.x, 2) + pow(a.y - b.y, 2) + pow(a.z - b.z, 2));
}

//����ƽ������
axis get_avr_position() {
	axis result = { 0,0,0 };
	for (int i = 1; i <= SIZE; i++) result = result + (p + i)->m_position;
	result = result / SIZE;
	return result;
}

//����ƽ���ٶ�
axis get_avr_velocity() {
	axis result = { 0,0,0 };
	for (int i = 1; i <= SIZE; i++) result = result + (p + i)->m_velocity;
	result = result / SIZE;
	return result;
}

//����Ŀ��ƫ���
double get_angle(axis now, axis target) {
	double v, dis, angle;
	axis vector_v = { 0,0,0 }, vector_dis = { 0,0,0 };
	vector_v = get_avr_velocity();
	v = get_dis(vector_v, p_origin);
	vector_dis = target - now;
	dis = get_dis(vector_dis, p_origin);
	angle = acos((vector_v*vector_dis) / (v*dis));
	angle = abs(angle) * 180 / PI;
	return angle;
}

//������С�������
double get_min_dis() {
	double dis, min_dis = 1000;
	for (int i = 1; i <= SIZE; i++) {
		for (int j = 1; j <= SIZE; j++) {
			if (i == j) continue;
			dis = get_dis((p + i)->m_position, (p + j)->m_position);
			if (dis < min_dis) min_dis = dis;
		}
	}
	return min_dis;
}

//�����ٶ���ɢ��
double dispersion_of_vel() {
	double dis = 0, result = 0;
	for (int i = 1; i <= SIZE; i++) {
		for (int j = 1; j <= SIZE; j++) {
			if (i == j) continue;
			dis = get_dis((p + i)->m_velocity, (p + j)->m_velocity);
			result += pow(dis, 2);
		}
	}
	result = sqrt(result) / SIZE;
	return result;
}

//����λ����ɢ��
double dispersion_of_pos() {
	double dis = 0, result = 0;
	for (int i = 1; i <= SIZE; i++) {
		for (int j = 1; j <= SIZE; j++) {
			if (i == j) continue;
			dis = get_dis((p + i)->m_position, (p + j)->m_position);
			result += pow(dis, 2);
		}
	}
	result = sqrt(result) / SIZE;
	return result;
}

//�����ײ
int crash() {
	int num = 0;
	double dis;
	for (int i = 1; i <= SIZE; i++) {
		for (int j = 1; j <= SIZE; j++) {
			if (i == j) continue;
			dis = get_dis((p + i)->m_position, (p + j)->m_position);
			if (dis < D_min) num++;
		}
	}
	return num;
}

/*-------------------------���Ჿ��-------------------------*/
//��ʼ�����˻���������֤��ɰ�ȫ����
void initial_assemble() {
	int n;
	do {
		n = 0;
		for (int i = 0; i <= SIZE; i++) {
			(p + i)->m_position.x = rand() % 200;
			(p + i)->m_position.y = rand() % 200;
			(p + i)->m_position.z = 0;
		}
		n = crash();
	} while (n != 0);
	for (int i = 1; i <= SIZE; i++) {
		(p + i)->m_velocity.z = rand() % 5;
		(p + i)->m_phi = atan2((p + i)->m_velocity.z, sqrt(pow((p + i)->m_velocity.x, 2) + pow((p + i)->m_velocity.y, 2)));
		(p + i)->m_theta = atan2((p + i)->m_velocity.y, (p + i)->m_velocity.x);
	}
	cout << "���˻���ʼ�����" << endl;
	return;
}

//C-Sģ���ٶ�ƥ����
axis f_speedmatch(int i) {
	int num = 0;
	double dis, aij;
	axis result = { 0,0,0 };
	for (int j = 1; j <= SIZE; j++) {
		if (j == i) continue;
		dis = get_dis((p + i)->m_position, (p + j)->m_position);
		if (dis <= NEIGHBOR) {
			num++;
			aij = H / pow(1 + pow(dis, 2), B);
			result = result + aij * ((p + j)->m_velocity - (p + i)->m_velocity);
		}
	}
	result = result / num;
	return result;
}

//C-Sģ�ͳ�����
axis f_repulsion(int i) {
	double dis, f;
	axis result = { 0,0,0 };
	for (int j = 1; j <= SIZE; j++) {
		if (j == i) continue;
		dis = get_dis((p + i)->m_position, (p + j)->m_position);
		if (dis <= D_max) f = K1 * (D_max - dis);
		else if (dis <= NEIGHBOR) f = K2 * (D_max - dis);
		else f = K2 * (D_max - NEIGHBOR);
		result = result + f * ((p + i)->m_position - (p + j)->m_position) / (dis + 1e-6);
	}
	return result;
}

//C-Sģ��Ŀ��������
axis f_target(int i) {
	double dis, g;
	axis result = { 0,0,0 };
	dis = get_dis((p + i)->m_position, p_assemble);
	g = K3 * pow(dis, ALPHA);
	result = g * (p_assemble - (p + i)->m_position) / (dis + 1e-6);
	return result;
}

//������ٶ���
axis get_acceleration_assemble(int i) {
	axis result;
	result = f_speedmatch(i) + f_repulsion(i) + f_target(i);
	return result;
}

/*-------------------------��Ӳ���-------------------------*/
//����ŷʽ�任����
void get_rotation_matrix() {
	T = Isometry3d::Identity();
	AngleAxisd rotation_vector_x(p->m_phi, Vector3d(1, 0, 0));
	AngleAxisd rotation_vector_z(p->m_theta - PI / 2, Vector3d(0, 0, 1));
	T.rotate(rotation_vector_z);
	T.rotate(rotation_vector_x);
	//��������ϵ
	T.pretranslate(Vector3d(0, 0, 0));
	for (int i = 0; i <= SIZE; i++) {
		form_axis[i].x = (T * form[i])[0];
		form_axis[i].y = (T * form[i])[1];
		form_axis[i].z = (T * form[i])[2];
	}
	//��������ϵ
	T.pretranslate(Vector3d(p->m_position.x, p->m_position.y, p->m_position.z));
	for (int i = 0; i <= SIZE; i++) {
		pos_axis[i].x = (T * form[i])[0];
		pos_axis[i].y = (T * form[i])[1];
		pos_axis[i].z = (T * form[i])[2];
	}
}

//��Ӳ�����ʼ��
void initial_form() {
	//��ʼ�����ⳤ������
	cout << "��ʼ�����ⳤ������:" << endl;
	double speed;
	p->m_position = get_avr_position();
	p->m_phi = atan2(p_final.z - p->m_position.z, sqrt(pow(p_final.x - p->m_position.x, 2) + pow(p_final.y - p->m_position.y, 2)));
	p->m_theta = atan2(p_final.y - p->m_position.y, p_final.x - p->m_position.x);
	speed = get_dis(get_avr_velocity(), p_origin);
	p->m_velocity.x = speed * cos(p->m_phi)*cos(p->m_theta);
	p->m_velocity.y = speed * cos(p->m_phi)*sin(p->m_theta);
	p->m_velocity.z = speed * sin(p->m_phi);
	p->m_acceleration = { 0,0,0 };
	cout << "����:(" << setw(5) << fixed << setprecision(2) << p->m_position.x << "," << p->m_position.y << "," << p->m_position.z << ")" << endl;
	cout << "�ٶ�:(" << setw(5) << fixed << setprecision(2) << p->m_velocity.x << "," << p->m_velocity.y << "," << p->m_velocity.z << ")" << endl;
	cout << "ƫ����:" << p->m_theta * 180 / PI << "�� " << "������:" << p->m_phi * 180 / PI << "��" << endl;
	cout << endl;
	//��ʼ�����λ��
	double d = D_form;
	Vector3d new_form[SIZE + 1];
	form[0] << 0, 0, 0;
	form[1] << -2 * d, 2 * d, 0;	form[2] << -d, 2 * d, 0;	form[3] << 0, 2 * d, 0;		form[4] << d, 2 * d, 0;		form[5] << 2 * d, 2 * d, 0;
	form[6] << -2 * d, d, 0;		form[7] << -d, d, 0;		form[8] << 0, d, 0;			form[9] << d, d, 0;			form[10] << 2 * d, d, 0;
	form[11] << -2 * d, 0, 0;		form[12] << -d, 0, 0;		form[13] << 0, 0, 0;		form[14] << d, 0, 0;		form[15] << 2 * d, 0, 0;
	form[16] << -2 * d, -d, 0;		form[17] << -d, -d, 0;		form[18] << 0, -d, 0;		form[19] << d, -d, 0;		form[20] << 2 * d, -d, 0;
	form[21] << -2 * d, -2 * d, 0;	form[22] << -d, -2 * d, 0;	form[23] << 0, -2 * d, 0;	form[24] << d, -2 * d, 0;	form[25] << 2 * d, -2 * d, 0;
	get_rotation_matrix();
	//�����Ӧ���˻�
	for (int i = 1; i <= SIZE; i++) {
		int min_id = -1;
		double dis, min_dis = 1000;
		for (int j = i; j <= SIZE; j++) {
			dis = get_dis(pos_axis[i], (p + j)->m_position);
			if (dis < min_dis) {
				min_dis = dis;
				min_id = j;
			}
		}
		swap(*(p+i), *(p+min_id));
		//cout << "���" << setw(2) << setfill('0') << i << "��λ���ѷ����" << setw(2) << setfill('0') << min_id << "�����˻�" << endl;
	}
	initial_adjmatrix();
	
}

//�ڽӾ���ֵ
void initial_adjmatrix() {
	A.setZero();
	for (int j = 0; j <= SIZE; j++) {
		if (j == 0) continue;
		A(j, 0) = 1;
		if (j%mat == 1 || j % mat == 2) A(j, j + 1) = 1;
		else if (j%mat == 0 || j % mat == 4) A(j, j - 1) = 1;
		else {
			A(j, j - 1) = 1;
			A(j, j + 1) = 1;
		}
		if (j <= 2 * mat) A(j, j + mat) = 1;
		else if (j > 3 * mat) A(j, j - mat) = 1;
		else {
			A(j, j + mat) = 1;
			A(j, j - mat) = 1;
		}
	}
	
	return;
}

//��Ӳ��ּ��ٶ���
axis f_form(int i) {
	axis result = { 0,0,0 };
	for (int j = 0; j <= SIZE; j++) {
		result = result + A(i, j)*(((form_axis[i] - (p + i)->m_position) - (form_axis[j] - (p + j)->m_position)) + 2 * ((p + j)->m_velocity - (p + i)->m_velocity));
	}
	return result;
}

//��Ӳ��ֳ�����
axis f_repulsion_form(int i) {
	double dis, f;
	axis result = { 0,0,0 };
	for (int j = 1; j <= SIZE; j++) {
		if (j == i) continue;
		dis = get_dis((p + i)->m_position, (p + j)->m_position);
		if (dis <= D_form) f = K4 * (D_form - dis);
		else if (dis <= 2 * D_form) f = K5 * (D_form - dis);
		else f = -K5 * D_form;
		result = result + f * ((p + i)->m_position - (p + j)->m_position) / (dis + 1e-6);
	}
	return result;
}

//������ٶ���
axis get_acceleration_form(int i) {
	axis result = { 0,0,0 };
	result = f_form(i) + f_repulsion_form(i) + p->m_acceleration;
	return result;
}

//���������ɢ��
double dispersion_of_form() {
	double result = 0;
	get_rotation_matrix();
	for (int i = 1; i <= SIZE; i++) result += get_dis(pos_axis[i], (p + i)->m_position);
	result /= SIZE;
	return result;
}