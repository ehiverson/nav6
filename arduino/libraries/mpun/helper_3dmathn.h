// I2C device class (I2Cdev) demonstration Arduino sketch for MPU6050 class, 3D math helper
// 6/5/2012 by Jeff Rowberg <jeff@rowberg.net>
// Updates should (hopefully) always be available at https://github.com/jrowberg/i2cdevlib
//
// Changelog:
//     2012-06-05 - add 3D math helper file to DMP6 example sketch

/* ============================================
I2Cdev device library code is placed under the MIT license
Copyright (c) 2012 Jeff Rowberg

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
===============================================
*/

#ifndef _HELPER_3DMATH_H
#define _HELPER_3DMATH_H
#include <cmath>

// angle in radians = angle in degrees * Pi / 180
const float degrees2radians = M_PI / 180.0;
// angle in degrees = angle in radians * 180 / Pi
const float radians2degrees = 180.0 / M_PI;


class Quaternion {
    public:
        float w;
        float x;
        float y;
        float z;
        
        Quaternion() {
            w = 1.0f;
            x = 0.0f;
            y = 0.0f;
            z = 0.0f;
			
        }
        
        Quaternion(float nw, float nx, float ny, float nz) {
            w = nw;
            x = nx;
            y = ny;
            z = nz;
			
        }
		
		void init(float nw, float nx, float ny, float nz) {
            w = nw;
            x = nx;
            y = ny;
            z = nz;
			
        }

        Quaternion multiply(Quaternion q) {
            // Quaternion multiplication is defined by:
            //     (Q1 * Q2).w = (w1w2 - x1x2 - y1y2 - z1z2)
            //     (Q1 * Q2).x = (w1x2 + x1w2 + y1z2 - z1y2)
            //     (Q1 * Q2).y = (w1y2 - x1z2 + y1w2 + z1x2)
            //     (Q1 * Q2).z = (w1z2 + x1y2 - y1x2 + z1w2
            return Quaternion(
                w*q.w - x*q.x - y*q.y - z*q.z,  // new w
                w*q.x + x*q.w + y*q.z - z*q.y,  // new x
                w*q.y - x*q.z + y*q.w + z*q.x,  // new y
                w*q.z + x*q.y - y*q.x + z*q.w); // new z
        }

        Quaternion conjugate() {
            return Quaternion(w, -x, -y, -z);
        }
        
        float magnitude() {
            return sqrt(w*w + x*x + y*y + z*z);
        }
        
        void normalize() {
            float m = magnitude();
            w /= m;
            x /= m;
            y /= m;
            z /= m;
        }
		
};


class VectorInt16 {
    public:
        int16_t x;
        int16_t y;
        int16_t z;

        VectorInt16() {
            x = 0;
            y = 0;
            z = 0;
        }
        
        VectorInt16(int16_t nx, int16_t ny, int16_t nz) {
            x = nx;
            y = ny;
            z = nz;
        }
		
		void init(int16_t nx, int16_t ny, int16_t nz) {
            x = nx;
            y = ny;
            z = nz;
        }

        float magnitude() {
            return sqrt(x*x + y*y + z*z);
        }

        void normalize() {
            float m = magnitude();
            x /= m;
            y /= m;
            z /= m;
        }
        
        
        VectorInt16 rotate(Quaternion q) {
            // http://www.cprogramming.com/tutorial/3d/quaternions.html
            // http://www.euclideanspace.com/maths/algebra/realNormedAlgebra/quaternions/transforms/index.htm
            // http://content.gpwiki.org/index.php/OpenGL:Tutorials:Using_Quaternions_to_represent_rotation
            // ^ or: http://webcache.googleusercontent.com/search?q=cache:xgJAp3bDNhQJ:content.gpwiki.org/index.php/OpenGL:Tutorials:Using_Quaternions_to_represent_rotation&hl=en&gl=us&strip=1
        
            // P_out = q * P_in * conj(q)
            // - P_out is the output vector
            // - q is the orientation quaternion
            // - P_in is the input vector (a*aReal)
            // - conj(q) is the conjugate of the orientation quaternion (q=[w,x,y,z], q*=[w,-x,-y,-z])
            Quaternion p(0, x, y, z);

            // quaternion multiplication: q * p, stored back in p
            p = q.multiply(p);

            // quaternion multiplication: p * conj(q), stored back in p
            p = p.multiply(q.conjugate());

            // p quaternion is now [0, x', y', z']
			return VectorInt16(p.x,p.y,p.z);
        }
};

class VectorFloat {
    public:
        float x;
        float y;
        float z;

        VectorFloat() {
            x = 0;
            y = 0;
            z = 0;
        }
        
        VectorFloat(float nx, float ny, float nz) {
            x = nx;
            y = ny;
            z = nz;
        }
		
		void init(float nx, float ny, float nz) {
            x = nx;
            y = ny;
            z = nz;
        }

        float magnitude() {
            return sqrt(x*x + y*y + z*z);
        }

        void normalize() {
            float m = magnitude();
            x /= m;
            y /= m;
            z /= m;
        }
        
        VectorFloat rotate(Quaternion q) {
            Quaternion p(0, x, y, z);

            // quaternion multiplication: q * p, stored back in p
            p=q.multiply(p);

            // quaternion multiplication: p * conj(q), stored back in p
            p=p.multiply(q.conjugate());

            // p quaternion is now [0, x', y', z']
            return VectorFloat(p.x,p.y,p.z);
        }
		VectorFloat subtract(VectorFloat vec){
			return VectorFloat(x-vec.x,y-vec.y,z-vec.z);
		}
		VectorFloat add(VectorFloat vec){
			return VectorFloat(x+vec.x,y+vec.y,z+vec.z);
		}
		VectorFloat multiply(float scale){
			return VectorFloat(x*scale,y*scale,z*scale);
		}
};


void getEuler(float *data, Quaternion *q) {

	data[0] = atan2(2 * q->x * q->y - 2 * q->w * q->z, 2 * q->w * q->w + 2 * q->x * q->x - 1); // psi
	data[1] = -asin(2 * q->x * q->z + 2 * q->w * q->y);                      // theta
	data[2] = atan2(2 * q->y * q->z - 2 * q->w * q->x, 2 * q->w * q->w + 2 * q->z * q->z - 1); // phi
}
/* These next two functions converts the orientation matrix (see
* gyro_orientation) to a scalar representation for use by the DMP.
* NOTE: These functions are borrowed from Invensense's MPL.
*/
unsigned short inv_row_2_scale(const signed char *row) {

	unsigned short b;

	if (row[0] > 0)
		b = 0;
	else if (row[0] < 0)
		b = 4;
	else if (row[1] > 0)
		b = 1;
	else if (row[1] < 0)
		b = 5;
	else if (row[2] > 0)
		b = 2;
	else if (row[2] < 0)
		b = 6;
	else
		b = 7;      // error
	return b;
}


unsigned short inv_orientation_matrix_to_scalar(const signed char *mtx) {

	unsigned short scalar;

	/*
	XYZ  010_001_000 Identity Matrix
	XZY  001_010_000
	YXZ  010_000_001
	YZX  000_010_001
	ZXY  001_000_010
	ZYX  000_001_010
	*/

	scalar = inv_row_2_scale(mtx);
	scalar |= inv_row_2_scale(mtx + 3) << 3;
	scalar |= inv_row_2_scale(mtx + 6) << 6;


	return scalar;
}

Quaternion qfromYaw(VectorFloat vec,float yawradians)
{
	float a=sin(yawradians/2);
	Quaternion q(cos(yawradians/2),a*vec.x,a*vec.y,a*vec.z);
	return q;
	
	
	
}


VectorFloat ellipseTransform(VectorFloat vec,float invw[],float offsets[])
{
	
	vec.x-=offsets[0];
    vec.y-=offsets[1];
    vec.z-=offsets[2];
	VectorFloat v(
    invw[0]*vec.x+invw[1]*vec.y+invw[2]*vec.z,
    invw[3]*vec.x+invw[4]*vec.y+invw[5]*vec.z,
    invw[6]*vec.x+invw[7]*vec.y+invw[8]*vec.z
    );
	return v;
}

VectorFloat gravity(Quaternion q)
{
	//Uses the quaternion from the MPU9150 sensor to obtain a vector that points
	//away from the ground.This vector is in the MPU9150's rotational frame of reference.
	return VectorFloat(2 * (q.x * q.z - q.w * q.y), 2 * (q.w * q.x + q.y * q.z), q.w * q.w - q.x * q.x - q.y * q.y + q.z * q.z);
}

#endif /* _HELPER_3DMATH_H */