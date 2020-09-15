#ifndef HOMOGENOUS_TRANSFORM_HEADER
#define HOMOGENOUS_TRANSFORM_HEADER
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

    void get_RPY(HomogenousTransform&, Vec3* rpy_out, float sqrt_select = 1);

    HomogenousTransform get_inverse(HomogenousTransform& transform);

    Vec4 apply_transform(HomogenousTransform& transform, Vec4& vector);

    HomogenousTransform apply_transform(HomogenousTransform& transform, HomogenousTransform& other_transform);

    inline void print_homogenous_transform(HomogenousTransform& transform);

    inline void print_vector(Vec4& vector);

    void test_homogenous_transform();
};

//#define HOMOGENOUS_TRANSFORM_IMPLEMENTATION //Define this before the include in the file you want the implementation
#ifdef HOMOGENOUS_TRANSFORM_IMPLEMENTATION
    #include <iostream>
    #include <cmath>
    #include <cstdlib>
    #include <eigen3/Eigen/Dense>

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

    void ROBOTICS_LAB::get_RPY(ROBOTICS_LAB::HomogenousTransform& transform, Vec3* rpy_out, float sqrt_select)
    {
        if(sqrt_select != 1.0f)sqrt_select = -1.0f;

        float cos2_theta = transform.column[1].data[2]*transform.column[1].data[2] + transform.column[2].data[2]*transform.column[2].data[2];
        float cos_theta = sqrt_select * sqrt(cos2_theta);
        if(cos_theta != 0)
        {
            float sin_theta = -transform.column[0].data[2];
            float theta = atan2(sin_theta, cos_theta);
            cos_theta = cos(theta);
            if(cos_theta != 0)
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
        }
    }

    ROBOTICS_LAB::HomogenousTransform ROBOTICS_LAB::get_inverse(HomogenousTransform& transform)
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

    ROBOTICS_LAB::Vec4 ROBOTICS_LAB::apply_transform(ROBOTICS_LAB::HomogenousTransform& transform, ROBOTICS_LAB::Vec4& vector)
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

    ROBOTICS_LAB::HomogenousTransform ROBOTICS_LAB::apply_transform(ROBOTICS_LAB::HomogenousTransform& transform, ROBOTICS_LAB::HomogenousTransform& other_transform)
    {

        HomogenousTransform result_transform = {0};
        for(int col = 0; col < 4; col++)
        {
            Vec4 result_column = apply_transform(transform, other_transform.column[col]);
            for(int row = 0; row < 4; row++)
            {
                result_transform.column[col].data[row] = result_column.data[row];
            }
        }

        return result_transform;
    }

    inline void ROBOTICS_LAB::print_homogenous_transform(ROBOTICS_LAB::HomogenousTransform& transform)
    {
        for(int row = 0; row < 4; row++)
        {
            std::cout << "Row " << row << " = ";
            for(int col = 0; col < 4; col++)
            {
                float value = transform.column[col].data[row];
                value = roundf(value * 1e4) * 1e-4;
                std::cout << value << " ";
            }
            std::cout << "\n";
        }

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
            Vec4 XYZ = {3.1415/2, 3.1415/2, 3.1415/2, 1};
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
            Vec4 XYZ = {3.1415f/2.0f, 3.1415f/2.0f, 3.1415f/4.0f, 1};
            transform = get_transform_from_eulerZYX(XYZ.to_vec3);
            

            int strikes = 0;
            float options[2] = {1, -1};
            for(int k = 0; k < 2; k++)
            {
                Vec4 RPY = {0, 0, 0, 1};
                get_RPY(transform, &RPY.to_vec3, options[k]);
                for(int i = 0; i < 3; i++)
                {
                    float value_left = XYZ.data[i];
                    float value_right = RPY.data[i];

                    value_left = roundf(value_left * 1e4) * 1e-4;
                    value_right = roundf(value_right * 1e4) * 1e-4;

                    if(value_left != value_right)
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


        std::cout <<"--------------------TEST STOP--------------------\n";
    }

#endif









#endif