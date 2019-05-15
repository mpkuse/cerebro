#pragma once

template <typename T>
T NormalizeAngle(const T& angle_degrees) {
  if (angle_degrees > T(180.0))
  	return angle_degrees - T(360.0);
  else if (angle_degrees < T(-180.0))
  	return angle_degrees + T(360.0);
  else
  	return angle_degrees;
};

class AngleLocalParameterization {
 public:

  template <typename T>
  bool operator()(const T* theta_radians, const T* delta_theta_radians,
                  T* theta_radians_plus_delta) const {
    *theta_radians_plus_delta =
        NormalizeAngle(*theta_radians + *delta_theta_radians);

    return true;
  }

  static ceres::LocalParameterization* Create() {
    return (new ceres::AutoDiffLocalParameterization<AngleLocalParameterization,
                                                     1, 1>);
  }
};

template <typename T>
void YawPitchRollToRotationMatrix(const T yaw, const T pitch, const T roll, T R[9])
{

	T y = yaw / T(180.0) * T(M_PI);
	T p = pitch / T(180.0) * T(M_PI);
	T r = roll / T(180.0) * T(M_PI);


	R[0] = cos(y) * cos(p);
	R[1] = -sin(y) * cos(r) + cos(y) * sin(p) * sin(r);
	R[2] = sin(y) * sin(r) + cos(y) * sin(p) * cos(r);
	R[3] = sin(y) * cos(p);
	R[4] = cos(y) * cos(r) + sin(y) * sin(p) * sin(r);
	R[5] = -cos(y) * sin(r) + sin(y) * sin(p) * cos(r);
	R[6] = -sin(p);
	R[7] = cos(p) * sin(r);
	R[8] = cos(p) * cos(r);
};

template <typename T>
void RotationMatrixTranspose(const T R[9], T inv_R[9])
{
	inv_R[0] = R[0];
	inv_R[1] = R[3];
	inv_R[2] = R[6];
	inv_R[3] = R[1];
	inv_R[4] = R[4];
	inv_R[5] = R[7];
	inv_R[6] = R[2];
	inv_R[7] = R[5];
	inv_R[8] = R[8];
};

template <typename T>
void RotationMatrixRotatePoint(const T R[9], const T t[3], T r_t[3])
{
	r_t[0] = R[0] * t[0] + R[1] * t[1] + R[2] * t[2];
	r_t[1] = R[3] * t[0] + R[4] * t[1] + R[5] * t[2];
	r_t[2] = R[6] * t[0] + R[7] * t[1] + R[8] * t[2];
};

class PNPEulerAngleError
{
public:
    PNPEulerAngleError( Vector3d _w_X, Vector2d _uv_normed_cords ): w_X(_w_X), uv_normed_cords(_uv_normed_cords)
    {}

        //    //      minimize_{R,t} \sum_i (  PI( c_(R|t)_w * w_X[i] ) - u[i] )

    template <typename T>
    bool operator()( const T* yaw, const T* pitch, const T* roll, const T* tx, const T* ty , const T* tz, T * residue ) const
    {
        T R[9]; // c_T_a.
        YawPitchRollToRotationMatrix( yaw[0], pitch[0], roll[0], R );

        T out[3];
        out[0] = R[0] * T(w_X(0)) + R[1] * T(w_X(1)) + R[2] * T(w_X(2)) + tx[0];
        out[1] = R[3] * T(w_X(0)) + R[4] * T(w_X(1)) + R[5] * T(w_X(2)) + ty[0];
        out[2] = R[6] * T(w_X(0)) + R[7] * T(w_X(1)) + R[8] * T(w_X(2)) + tz[0];


        // Simple
        residue[0] = out[0] / out[2] - T(uv_normed_cords(0));
        residue[1] = out[1] / out[2] - T(uv_normed_cords(1));

        // Weight by Z. A point that is far away need to be down weighted
        // just search plot 1/ (1 + exp( x-10) )  on google. also called sigmoid function
        T wt =  T( 1.0 / ( 1.0 + exp(w_X(2) - 10.) ) );
        residue[0] *= wt;
        residue[1] *= wt;

        return true;
    }

    static ceres::CostFunction* Create( Vector3d _w_X, Vector2d _uv_normed_cords )
	{
	  return (new ceres::AutoDiffCostFunction<
	          PNPEulerAngleError, 2, 1,1,1,  1,1,1 >(
	          	new PNPEulerAngleError( _w_X, _uv_normed_cords ) ) );
	}


private:
    const Vector3d w_X;
    const Vector2d uv_normed_cords;
};



class P3PEulerAngleError
{
public:
    // [Input]:
    //      _a_X: 3dpts expressed in co-ordinate frame of a
    //      _b_X: 3dpts expressed in co-ordinate frame of b
    P3PEulerAngleError( Vector3d _a_X, Vector3d _b_X ): a_X(_a_X), b_X(_b_X)
    {}

        //    //      minimize_{R,t} \sum_i   b_(R|t)_a * a_X[i]  - b_X[i]

    template <typename T>
    bool operator()( const T* yaw, const T* pitch, const T* roll, const T* tx, const T* ty , const T* tz, T * residue ) const
    {
        T R[9]; // b_T_a.
        YawPitchRollToRotationMatrix( yaw[0], pitch[0], roll[0], R );

        T out[3];
        out[0] = R[0] * T(a_X(0)) + R[1] * T(a_X(1)) + R[2] * T(a_X(2)) + tx[0];
        out[1] = R[3] * T(a_X(0)) + R[4] * T(a_X(1)) + R[5] * T(a_X(2)) + ty[0];
        out[2] = R[6] * T(a_X(0)) + R[7] * T(a_X(1)) + R[8] * T(a_X(2)) + tz[0];


        // Simple
        residue[0] = out[0] - T(b_X(0));
        residue[1] = out[1] - T(b_X(1));
        residue[2] = out[2] - T(b_X(2));


        // Weight by Z. A point that is far away need to be down weighted
        // just search plot 1/ (1 + exp( x-10) )  on google. also called sigmoid function
        T wt =  T( 1.0 / ( 1.0 + exp(a_X(2) - 10.) ) );
        residue[0] *= wt;
        residue[1] *= wt;
        residue[2] *= wt;

        return true;
    }

    static ceres::CostFunction* Create( Vector3d _a_X, Vector3d _b_X )
	{
	  return (new ceres::AutoDiffCostFunction<
	          P3PEulerAngleError, 3, 1,1,1,  1,1,1 >(
	          	new P3PEulerAngleError( _a_X, _b_X ) ) );
	}


private:
    const Vector3d a_X;
    const Vector3d b_X;
};


/*
struct PNPFourDOFError
{
	PNPFourDOFError(double t_x, double t_y, double t_z, double relative_yaw, double pitch_i, double roll_i)
				  :t_x(t_x), t_y(t_y), t_z(t_z), relative_yaw(relative_yaw), pitch_i(pitch_i), roll_i(roll_i){}

	template <typename T>
	bool operator()(const T* const yaw_i, const T* ti, const T* yaw_j, const T* tj, T* residuals) const
	{
		T t_w_ij[3];
		t_w_ij[0] = tj[0] - ti[0];
		t_w_ij[1] = tj[1] - ti[1];
		t_w_ij[2] = tj[2] - ti[2];

		// euler to rotation
		T w_R_i[9];
		YawPitchRollToRotationMatrix(yaw_i[0], T(pitch_i), T(roll_i), w_R_i);
		// rotation transpose
		T i_R_w[9];
		RotationMatrixTranspose(w_R_i, i_R_w);
		// rotation matrix rotate point
		T t_i_ij[3];
		RotationMatrixRotatePoint(i_R_w, t_w_ij, t_i_ij);

		residuals[0] = (t_i_ij[0] - T(t_x));
		residuals[1] = (t_i_ij[1] - T(t_y));
		residuals[2] = (t_i_ij[2] - T(t_z));
		residuals[3] = NormalizeAngle(yaw_j[0] - yaw_i[0] - T(relative_yaw));

		return true;
	}

	static ceres::CostFunction* Create(const double t_x, const double t_y, const double t_z,
									   const double relative_yaw, const double pitch_i, const double roll_i)
	{
	  return (new ceres::AutoDiffCostFunction<
	          PNPFourDOFError, 4, 1, 3, 1, 3>(
	          	new PNPFourDOFError(t_x, t_y, t_z, relative_yaw, pitch_i, roll_i)));
	}

	double t_x, t_y, t_z;
	double relative_yaw, pitch_i, roll_i;

};
*/
