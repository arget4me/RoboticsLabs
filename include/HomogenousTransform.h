#ifndef HOMOGENOUS_TRANSFORM_HEADER
#define HOMOGENOUS_TRANSFORM_HEADER

#define PI 3.141593f

namespace ROBOTICS_LAB
{


    typedef struct
    {
        union
        {
            float data[3];
            struct{
                float x;
                float y;
                float z;
            };
        };
    }Vec3;

    typedef struct
    {
        union
        {
            float data[4];
            Vec3 to_vec3;
            struct{
                float x;
                float y;
                float z;
                float w;
            };
        };
    }Vec4;

    typedef struct 
    {
        union
        {
            float data[16];
            Vec4 column[4];
        };
    }HomogenousTransform;

    

    inline HomogenousTransform get_identity_matrix();

    inline HomogenousTransform get_transform_from_eulerZYX(const Vec3& xyz_input);

    inline HomogenousTransform get_transform_from_angle_axis(const Vec4& axis_angle);

    inline HomogenousTransform get_transform_from_quaternion(const Vec4& quaternion);

    void get_angle_axis_from_transform(Vec4* axis_angle_out, const HomogenousTransform&  transform);

    void get_RPY(const HomogenousTransform& transform, Vec3* rpy_out, float sqrt_select = 1);

    void get_quaternion_from_angle_axis(Vec4* quaternion_out, const Vec4& axis_angle);

    void get_quaternion_from_transform(Vec4* quaternion_out, const HomogenousTransform&transform);

    HomogenousTransform get_inverse(const HomogenousTransform&  transform);

    Vec4 apply_transform(const HomogenousTransform& transform, const Vec4& vector);

    HomogenousTransform apply_transform(const HomogenousTransform&  transform, const HomogenousTransform&  other_transform);

    inline void print_homogenous_transform(const HomogenousTransform& transform);

    inline void print_vector(Vec4& vector);

    void test_homogenous_transform();
};

//#define HOMOGENOUS_TRANSFORM_IMPLEMENTATION //Define this before the include in the file you want the implementation
#ifdef HOMOGENOUS_TRANSFORM_IMPLEMENTATION
    #include <iostream>
    #include <cmath>
    #include <cstdlib>
    #include <eigen3/Eigen/Dense>

    bool compare_values(float value_left, float value_right, int num_decimals)
    {
        float decimals = 1;
        for(int i = 0; i < num_decimals; i++)
        {
            decimals = decimals * 10;
        }
        float value_a = roundf(value_left * decimals) / decimals;
        float value_b = roundf(value_right * decimals) / decimals;

        return (value_a == value_b);
    }

    inline ROBOTICS_LAB::HomogenousTransform ROBOTICS_LAB::get_identity_matrix()
    {
        return {
            1, 0, 0, 0, //col 0
            0, 1, 0, 0, //col 1
            0, 0, 1, 0, //col 2
            0, 0, 0, 1, //col 3
        };
    };

    inline ROBOTICS_LAB::HomogenousTransform ROBOTICS_LAB::get_transform_from_eulerZYX(const Vec3& xyz_input)
    {
        const float& Z = xyz_input.z; 
        const float& Y = xyz_input.y; 
        const float& X = xyz_input.x; 
        return {
            cos(Y) * cos(Z), cos(Y) * sin(Z), -sin(Y), 0, //col 0
            cos(Z) * sin(X) * sin(Y) - cos(X) * sin(Z), cos(X) * cos(Z) + sin(X) * sin(Y) * sin(Z), cos(Y) * sin(X), 0, //col 1
            sin(X) * sin(Z) + cos(X) * cos(Z) * sin(Y), cos(X) * sin(Y) * sin(Z) - cos(Z) * sin(X), cos(X) * cos(Y), 0, //col 2
            0, 0, 0, 1, //col 3
        };
    }


    inline ROBOTICS_LAB::HomogenousTransform ROBOTICS_LAB::get_transform_from_angle_axis(const Vec4& axis_angle)
    {
        const Vec3& axis = axis_angle.to_vec3;
        const float& angle = axis_angle.w;
        return {
            axis.x*axis.x * (1 - cos(angle)) + cos(angle), axis.x * axis.y * (1 - cos(angle)) + axis.z * sin(angle), axis.x * axis.z * (1 - cos(angle)) - axis.y * sin(angle), 0, //col 0
            axis.x * axis.y * (1 - cos(angle)) - axis.z * sin(angle), axis.y * axis.y * (1 - cos(angle)) + cos(angle), axis.y * axis.z * (1 - cos(angle)) + axis.x * sin(angle), 0, //col 1
            axis.x * axis.z * (1 - cos(angle)) + axis.y * sin(angle), axis.y * axis.z * (1 - cos(angle)) - axis.x * sin(angle), axis.z * axis.z * (1 - cos(angle)) + cos(angle), 0, //col 2
            0, 0, 0, 1, //col 3
        };
    }

    inline ROBOTICS_LAB::HomogenousTransform ROBOTICS_LAB::get_transform_from_quaternion(const Vec4& quat)
    {
        const float& x = quat.x;
        const float& y = quat.y;
        const float& z = quat.z;
        const float& n = quat.w;
        
        return {
            2.0f * (n*n + x*x) - 1.0f, 2.0f * (x*y + n*z), 2.0f * (x*z - n*y), 0, //col 0
            2.0f * (x*y - n*z), 2.0f * (n*n +y*y) - 1.0f, 2.0f * (y*z + n*x), 0, //col 1
            2.0f * (x*z + n*y), 2.0f * (y*z - n*x), 2.0f * (n*n + z*z) - 1.0f, 0, //col 2
            0, 0, 0, 1, //col 3
        };
    }

    void ROBOTICS_LAB::get_angle_axis_from_transform(Vec4* axis_angle_out, const HomogenousTransform&  transform)
    {
        const float rz = transform.column[0].data[1] - transform.column[1].data[0];
        const float ry = transform.column[2].data[0] - transform.column[0].data[2];
        const float rx = transform.column[1].data[2] - transform.column[2].data[1];
        float sin_theta = sqrt(rx*rx + ry*ry + rz*rz);
        float cos_theta = transform.column[0].data[0] + transform.column[1].data[1] + transform.column[2].data[2] - 1;
        
        float theta = atan2(sin_theta, cos_theta);//@Note: sin_theta and cos_theta includes factor 0.5f but this is factored out in tan = sin/cos

        if(compare_values(fabs(theta), 0.0f, 4))
        {
            std::cout << "Singular case, Undefined axis.\n";
            axis_angle_out->w = 0.0f;
            //axis is Undefined
        }
        else if(compare_values(fabs(fabs(theta) - PI), 0.0f, 4))
        {

            std::cout << "Singular case, theta = +-pi " << theta <<  "\n" ;
            constexpr bool NEGATIVE = true;
            bool sign_xy = (transform.column[1].data[0] < 0.0f) == NEGATIVE;

            bool sign_xz = (transform.column[2].data[0] < 0.0f) == NEGATIVE;

            bool sign_yz = (transform.column[2].data[1] < 0.0f) == NEGATIVE;


            Vec4 axis = { 
                sqrt((transform.column[0].data[0] + 1.0f) / 2.0f),
                sqrt((transform.column[1].data[1] + 1.0f) / 2.0f),
                sqrt((transform.column[2].data[2] + 1.0f) / 2.0f),
                theta
            };

            // Resolve signs ambiguities: axis.y == -axis.y || +axis.y
            axis.y = 
            - axis.y *  (sign_xy || !sign_xz && sign_yz) 
            + axis.y * !(sign_xy || !sign_xz && sign_yz); 

            // Resolve signs ambiguities: axis.z == -axis.z || +axis.z
            axis.z = 
            - axis.z *  (sign_xz || !sign_xy && sign_yz) 
            + axis.z * !(sign_xz || !sign_xy && sign_yz);

            float length = sqrt(axis.x*axis.x + axis.y*axis.y + axis.z*axis.z);
            axis.x /= length;
            axis.y /= length;
            axis.z /= length;

            *axis_angle_out = axis;

        }
        else
        {
            Vec4 axis = {
                0.5f * sin(theta) * rx,
                0.5f * sin(theta) * ry,
                0.5f * sin(theta) * rz,
                theta
            };
            float length = sqrt(axis.x*axis.x + axis.y*axis.y + axis.z*axis.z);
            axis.x /= length;
            axis.y /= length;
            axis.z /= length;

            *axis_angle_out = axis;
        }
        
    }

    void ROBOTICS_LAB::get_RPY(const ROBOTICS_LAB::HomogenousTransform& transform, Vec3* rpy_out, float sqrt_select)
    {
        if(sqrt_select != 1.0f)sqrt_select = -1.0f;

        float cos2_theta = transform.column[1].data[2]*transform.column[1].data[2] + transform.column[2].data[2]*transform.column[2].data[2];
        float cos_theta = sqrt_select * sqrt(cos2_theta);
        if(fabs(cos_theta) >= 1e-6)
        {
            float sin_theta = -transform.column[0].data[2];
            float theta = atan2(sin_theta, cos_theta);
            cos_theta = cos(theta);
            if(fabs(cos_theta) >= 1e-6)
            {
                float sin_psi = transform.column[1].data[2] / cos_theta;
                float cos_psi = transform.column[2].data[2] / cos_theta;
                
                float sin_phi = transform.column[0].data[1] / cos_theta;
                float cos_phi = transform.column[0].data[0] / cos_theta;

                float psi = atan2(sin_psi, cos_psi);
                float phi = atan2(sin_phi, cos_phi);

                rpy_out->x = psi;
                rpy_out->y = theta;
                rpy_out->z = phi;
            }
            else
            {
                std::cout << "[->]Cos(theta) = 0\n";
            }
        }
        else
        {
            std::cout << "Cos(theta) = 0\n";
        }
    }

    void ROBOTICS_LAB::get_quaternion_from_angle_axis(Vec4* quaternion_out, const Vec4& axis_angle)
    {
        Vec4 axis = axis_angle;
        float length = sqrt(axis.x*axis.x + axis.y*axis.y + axis.z*axis.z);
        if(!compare_values(length, 0.0f, 4))
        {
            axis.x /= length;
            axis.y /= length;
            axis.z /= length;
        }

        float sin_theta_half = sin(axis.w / 2);
        float cos_theta_half = cos(axis.w / 2);

        quaternion_out->x = axis.x * sin_theta_half;
        quaternion_out->y = axis.y * sin_theta_half;
        quaternion_out->z = axis.z * sin_theta_half;
        quaternion_out->w = cos_theta_half;
    }

    void ROBOTICS_LAB::get_quaternion_from_transform(Vec4* quaternion_out, const ROBOTICS_LAB::HomogenousTransform& transform)
    {
        Vec4 axis = {0};
        get_angle_axis_from_transform(&axis, transform);
        //std::cout << "Quat from axis-angle:\n";
        //print_vector(axis);

        float sin_theta_half = sin(axis.w / 2);
        float cos_theta_half = cos(axis.w / 2);

        quaternion_out->x = axis.x * sin_theta_half;
        quaternion_out->y = axis.y * sin_theta_half;
        quaternion_out->z = axis.z * sin_theta_half;
        quaternion_out->w = cos_theta_half;
    }

    ROBOTICS_LAB::HomogenousTransform ROBOTICS_LAB::get_inverse(const HomogenousTransform& transform)
    {
        HomogenousTransform inverse_transform = get_identity_matrix();
        for(int col = 0; col < 3; col++)
        {
            for(int row = 0; row < 3; row++)
            {
                inverse_transform.column[col].data[row] = transform.column[row].data[col];
            }
        }

        for(int row = 0; row < 3; row++)
        {
            for(int col = 0; col < 3; col++)
            {
                inverse_transform.column[3].data[row] -= inverse_transform.column[col].data[row] * transform.column[3].data[col];
            }
        }
        
        return inverse_transform;
    }

    ROBOTICS_LAB::Vec4 ROBOTICS_LAB::apply_transform(const ROBOTICS_LAB::HomogenousTransform& transform, const ROBOTICS_LAB::Vec4& vector)
    {
        Vec4 result_vector = {0, 0, 0, 0};
        
        for(int row = 0; row < 4; row++)
        {
            for(int col = 0; col < 4; col++)
            {
                result_vector.data[row] += transform.column[col].data[row] * vector.data[col];
            }
        }

        return result_vector;
    }

    ROBOTICS_LAB::HomogenousTransform ROBOTICS_LAB::apply_transform(const ROBOTICS_LAB::HomogenousTransform& transform, const ROBOTICS_LAB::HomogenousTransform& other_transform)
    {

        HomogenousTransform result_transform = {0};
        for(int col = 0; col < 4; col++)
        {
            Vec4 result_column = apply_transform(transform, other_transform.column[col]);
            //for(int row = 0; row < 4; row++)
            {
                result_transform.column[col] = result_column;
            }
        }

        return result_transform;
    }

    inline void ROBOTICS_LAB::print_homogenous_transform(const ROBOTICS_LAB::HomogenousTransform& transform)
    {
        std::cout << "[ ";
        for(int row = 0; row < 4; row++)
        {
            for(int col = 0; col < 4; col++)
            {
                float value = transform.column[col].data[row];
                value = roundf(value * 1e4) * 1e-4;
                if(col % 4 != 3)
                    std::cout << value << ", ";
                else
                    std::cout << value << ";\n";
            }
        }
        std::cout << "]\n";

    }

    inline void ROBOTICS_LAB::print_vector(ROBOTICS_LAB::Vec4& vector)
    {
        std::cout << "Vector = \n";

        for(int row = 0; row < 4; row++)
        {
            float value = vector.data[row];
            value = roundf(value * 1e4) * 1e-4;
            std::cout << value << "\n";
        }
    }

    static inline void display_test_output(bool state, char* test_name)
    {
        if(state)
        {
            std::cout << "[Passed]:  ";
        }
        else
        {
            std::cout << "[Failed];  ";
        }
        std::cout << test_name << "\n";
    }

    void ROBOTICS_LAB::test_homogenous_transform()
    {
        std::cout <<"Testing Homogenous Transform\n--------------------TEST START--------------------\n";


        {//Vec4 data layout
            ROBOTICS_LAB::Vec4 vector;
            for(int i = 0; i < 4; i++)
            {
                vector.data[i] = i+1;
            }
            bool passed_vec4_data = vector.x == 1
            && vector.y == 2
            && vector.z == 3
            && vector.w == 4;
            display_test_output(passed_vec4_data, "Vec4 data layout");
        }

        ROBOTICS_LAB::HomogenousTransform transform = get_identity_matrix();
        {//matrix identity
            bool passed_matrix_identity = true;
            for(int col = 0; col < 4; col++)
            {   
                for(int row = 0; row < 4; row++)
                {
                    if(row == col)
                    {
                        if(transform.column[col].data[row] != 1)
                        {
                            passed_matrix_identity = false;
                            break;
                        }
                    }
                    else
                    {
                        if(transform.column[col].data[row] != 0)
                        {
                            passed_matrix_identity = false;
                            break;
                        }
                    }
                }
            }
            display_test_output(passed_matrix_identity, "matrix identity");
        }

        {//matrix data layout
                for(int col = 0; col < 4; col++)
                {
                    for(int i = 0; i < 4; i++)
                    {
                        transform.column[col].data[i] = col * 4 + i+1;
                    }
                }

                bool passed_matrix_data = true;
                for(int i = 0; i < 16; i++)
                {
                    if(transform.data[i] != i+1)
                    {
                        passed_matrix_data = false;
                        break;
                    }
                }
                display_test_output(passed_matrix_data, "matrix data layout");
        }

        {//matrix inverse
            HomogenousTransform control_transform = {
                0, 1, 0, 0, //col 0
               -1, 0, 0, 0, //col 1
                0, 0, 1, 0, //col 2
                3, 8, 0, 1, //col 3
            };
            transform = control_transform;
            transform = get_inverse(control_transform);

            bool passed_matrix_inverse = passed_matrix_inverse = transform.column[3].x == -control_transform.column[3].y 
            && transform.column[3].y == control_transform.column[3].x;
            

            transform = get_inverse(transform);
            for(int i = 0; i < 16; i++)
            {
                if(transform.data[i] != control_transform.data[i])
                {
                    passed_matrix_inverse = false;
                    break;
                }
            }
            display_test_output(passed_matrix_inverse, "matrix inverse");
        }

        {//Apply Transform to Vector
            transform = {
                0, 1, 0, 0, //col 0
               -1, 0, 0, 0, //col 1
                0, 0, 1, 0, //col 2
                1, 1, 0, 1, //col 3
            };

            Vec4 vector = {1, 1, 0, 1};
            Vec4 result_vector = apply_transform(transform, vector);
            bool passed_transform_vextor = result_vector.x == 0 
            && result_vector.y == 2
            && result_vector.z == 0
            && result_vector.w == 1;
            display_test_output(passed_transform_vextor, "Apply Transform to Vector"); 
        }

        {//Apply Transform to Transform
            HomogenousTransform transform_0 = {
                1, 0, 0, 0, //col 0
                0, 1, 0, 0, //col 1
                0, 0, 1, 0, //col 2
                1, 1, 0, 1, //col 3
            };

            HomogenousTransform transform_1 = {
                0, 1, 0, 0, //col 0
               -1, 0, 0, 0, //col 1
                0, 0, 1, 0, //col 2
                0, 0, 0, 1, //col 3
            };
            transform = apply_transform(transform_0, transform_1);

            Vec4 vector = {1, 1, 0, 1};
            Vec4 result_vector = apply_transform(transform, vector);
            bool passed_transform_transform = result_vector.x == 0 
            && result_vector.y == 2
            && result_vector.z == 0
            && result_vector.w == 1;
            display_test_output(passed_transform_transform, "Apply Transform to Transform"); 
        }

        {//Transform from euler ZYX
            transform = get_transform_from_eulerZYX({0});
            HomogenousTransform I = get_identity_matrix();
            bool passed_euler_ZYX = true;
            for(int i = 0; i < 16; i++)
            {
                if(transform.data[i] != I.data[i])
                {
                    passed_euler_ZYX = false;
                    break;
                }
            }
            Vec4 XYZ = {PI/2, PI/2, PI/2, 1};
            transform = get_transform_from_eulerZYX(XYZ.to_vec3);
            Eigen::Matrix3f check_transform;
            check_transform = Eigen::AngleAxisf(XYZ.z, Eigen::Vector3f::UnitZ())
                * Eigen::AngleAxisf(XYZ.y, Eigen::Vector3f::UnitY())
                * Eigen::AngleAxisf(XYZ.x, Eigen::Vector3f::UnitX());
            for(int col = 0; col < 3; col++)
            {
                for(int row = 0; row < 3; row++)
                {
                    float value_left = transform.column[col].data[row];
                    float value_right = check_transform(row, col);

                    value_left = roundf(value_left * 1e4) * 1e-4;
                    value_right = roundf(value_right * 1e4) * 1e-4;

                    if(value_left != value_right)
                    {
                        std::cout << "Failed at (" << row << ", " << col << ")\n" << value_left << " != " << value_right << "\n";
                        passed_euler_ZYX = false;
                        break;
                    }
                }
            }
            display_test_output(passed_euler_ZYX, "Transform from euler ZYX");
        }

        {//RPY from Transform
            bool passed_transform_to_rpy = true;
            Vec4 XYZ = {PI/1.0f, PI/4.0f, PI/2.0f, 1};
            transform = get_transform_from_eulerZYX(XYZ.to_vec3);
            //std::cout << "Euler XYZ angles: (transform from eulerZYX)\n";
            //print_vector(XYZ);
            

            int strikes = 0;
            float options[2] = {1, -1};
            for(int k = 0; k < 2; k++)
            {
                Vec4 RPY = {0, 0, 0, 1};
                get_RPY(transform, &RPY.to_vec3, options[k]);
                //std::cout << "RPY angles:\n";
                //print_vector(RPY);
                for(int i = 0; i < 3; i++)
                {
                    float value_left = XYZ.data[i];
                    float value_right = RPY.data[i];

                    value_left = roundf(value_left * 1e6) * 1e-6;
                    value_right = roundf(value_right * 1e6) * 1e-6;

                    //-PI and +PI should be reset to 0.0f if close
                    if(fabs(fabs(value_left) - PI) <= 1e-4)value_left = 0.0f;
                    if(fabs(fabs(value_right) - PI) <= 1e-4)value_right = 0.0f;

                    if(value_left != value_right && (PI - value_left) != value_right)
                    {
                        strikes++;
                        break;
                    }
                }
            }

            if(strikes > 1)
            {
                std::cout << "Num strikes for RPY test = " << strikes << " (max 1 strike allowed)\n";
                passed_transform_to_rpy = false;
            }

            display_test_output(passed_transform_to_rpy, "RPY from Transform");
        }

        {
            bool passed_angle_axis = true;
            {
                Vec4 axis = {1, 0, 0, PI/8};
                //std::cout << "Angle in: " << angle << "\n";
                //print_vector(axis);
                transform = get_transform_from_angle_axis(axis);
                //print_homogenous_transform(transform);
                


                axis = {0};
                get_angle_axis_from_transform(&axis, transform);
                //print_vector(axis);
                //std::cout << "Angle out: "<<angle << "\n\n";
                if(!(compare_values(axis.x, 1, 4) &&
                    compare_values(axis.y, 0, 4) &&
                    compare_values(axis.z, 0, 4) &&
                    compare_values(axis.w, PI/8, 4)))
                {
                    passed_angle_axis = false;
                }
            }

            {
                Vec4 axis = {1/sqrt(2), 1/sqrt(2), 0, PI/2};
                //std::cout << "Angle in: " << angle << "\n";
                //print_vector(axis);
                transform = get_transform_from_angle_axis(axis);
                //print_homogenous_transform(transform);
                


                axis = {0};
                get_angle_axis_from_transform(&axis, transform);
                //print_vector(axis);
                //std::cout << "Angle out: "<<angle << "\n\n";
                if(!(compare_values(axis.x, 1/sqrt(2), 4) &&
                    compare_values(axis.y, 1/sqrt(2), 4) &&
                    compare_values(axis.z, 0, 4) &&
                    compare_values(axis.w, PI/2, 4)))
                {
                    passed_angle_axis = false;
                }
            }
            display_test_output(passed_angle_axis, "Angle axis & Transform");
        }

        {
            bool passed_quaternion = true;
            Vec4 axis = {0, 1, 0, PI / 3.0f};
            transform = get_transform_from_angle_axis(axis);

            Vec4 quaternion = {0};
            get_quaternion_from_transform(&quaternion, transform);
            HomogenousTransform quat_transform = get_transform_from_quaternion(quaternion);

            for(int i = 0; i < 16; i++)
            {
                if(!compare_values(transform.data[i], quat_transform.data[i], 4))
                {
                    passed_quaternion = false;
                    break;
                }
            }

            display_test_output(passed_quaternion, "Quaternion & transform");
        }



        std::cout <<"--------------------TEST STOP--------------------\n";
    }

    

#endif









#endif