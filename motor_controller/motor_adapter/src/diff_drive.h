/*******************************************************************************
 * Copyright (c) 2019  Carnegie Mellon University
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *******************************************************************************/

/*
 * Diff Drive
 * 
 * Author: Daisuke Sato <daisukes@cmu.edu>
 */

#ifndef MOTOR_ADAPTER_DIFF_DRIVE
#define MOTOR_ADPATER_DIFF_DRIVE

#include <mutex>

#define _USE_MATH_DEFINES
#include <math.h>

template<typename T>
struct LR{
    T l;
    T r;

LR(): l(0), r(0){}
    
LR(T l_, T r_): l(l_), r(r_)
    {}

    LR<T>& operator=(LR<T> a) {
	l = a.l;
	r = a.r;
	return *this;
    };
    
    LR<T> operator+(const LR<T> &a) const {
	return LR<T>(l + a.l, r + a.r);
    }

    LR<T> operator-(const LR<T> &a) const {
	return LR<T>(l - a.l, r - a.r);
    }

    LR<double> operator*(const double &a) const {
	return LR<double>(l * a, r * a);
    }

    LR<double> operator/(const double &a) const {
	return LR<double>(l / a, r / a);
    }
};

typedef LR<double> LRdouble;
typedef LR<int> LRint;

struct Pose{
    double x;
    double y;
    double a;

Pose(): x(0), y(0), a(0)
    {}
Pose(double x_, double y_, double a_):
    x(x_), y(y_), a(a_)
    {}
    
    void forward(const LRdouble &lr) {
	x += lr.l * cos(a);
	y += lr.l * sin(a);
	a += lr.r;
    }
};

namespace MotorAdapter {
    
    class DiffDrive {
    public:
	DiffDrive(const double &bias);
	~DiffDrive();

	void set(const double &bias);
	void update(const double &left, const double &right, const double &currentTime);

	Pose& pose();
	LRdouble& velocity();
	void y(const double &y);

    private:
	std::mutex stateMutex_;
	
	double bias_;

	double lastTime_;
	LRdouble lastLR_;

	LRdouble lastVel_;
	Pose pose_;
	
	bool initialized_;
    };
}

#endif
