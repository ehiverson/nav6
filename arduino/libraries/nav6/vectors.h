
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
        
        
        void rotate(Quaternion q) {
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
            x = p.x;
            y = p.y;
            z = p.z;
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

        float magnitude() {
            return sqrt(x*x + y*y + z*z);
        }

        void normalize() {
            float m = magnitude();
            x /= m;
            y /= m;
            z /= m;
        }
        
        void rotate(Quaternion q) {
            Quaternion p(0, x, y, z);

            // quaternion multiplication: q * p, stored back in p
            p = q.multiply(p);

            // quaternion multiplication: p * conj(q), stored back in p
            p = p.multiply(q.conjugate());

            // p quaternion is now [0, x', y', z']
            x = p.x;
            y = p.y;
            z = p.z;
        }
};
