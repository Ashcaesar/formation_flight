#ifndef FUNCTION_H_
#define FUNCTION_H_

struct axis{
	double x, y, z;
	friend axis operator +(const axis &, const axis &);
	friend axis operator -(const axis &, const axis &);
	friend double operator *(const axis &, const axis &);
	friend axis operator *(const double &, const axis &);
	friend axis operator /(const axis &, const double &);
};

void limit(axis &, axis &);
double get_dis(double, double);
double get_dis(axis, axis);
axis get_avr_position();
axis get_avr_velocity();
double get_angle(axis, axis);
double get_min_dis();
double dispersion_of_vel();
double dispersion_of_pos();
int crash();

void initial_assemble();
axis f_speedmatch(int);
axis f_repulsion(int);
axis f_target(int);
axis get_acceleration_assemble(int);

void initial_form();
void initial_adjmatrix();
axis f_repulsion_form(int);
axis f_form(int);
axis get_acceleration_form(int);
double dispersion_of_form();

#endif // 

