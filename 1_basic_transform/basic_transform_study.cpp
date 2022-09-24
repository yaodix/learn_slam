#include <iostream>

#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Geometry"

using namespace std;
void test1() {
    Eigen::Vector3d B_in_coor_O(3, 4, 0);  // 机器人B在坐标系O中的坐标：

    // 坐标系B到坐标O的转换矩阵：
    Eigen::Matrix4d coor_B_to_coor_O;
    double theta = M_PI ;
    coor_B_to_coor_O << cos(theta), sin(theta), 0, B_in_coor_O(0),
                        -sin(theta),  cos(theta), 0, B_in_coor_O(1),
                                0,          0,        1,              0,
                                0,          0,        0,              1;

    Eigen::Matrix4d coor_O_to_coor_B = Eigen::Matrix4d::Identity();  // 坐标系O到坐标B的转换矩阵:
    coor_O_to_coor_B.topLeftCorner(3, 3) = coor_B_to_coor_O.topLeftCorner(3, 3).inverse();
    coor_O_to_coor_B.topRightCorner(3, 1) =  -1 * coor_B_to_coor_O.topLeftCorner(3, 3).inverse() *
                                                 coor_B_to_coor_O.topRightCorner(3, 1);   

    Eigen::Vector4d A_in_coor_O(1, 3, 0, 1);  // 机器人A在坐标系O中的坐标： -M_PI / 2
    Eigen::Vector4d A_end_in_coor_O(1, 2, 0, 1);  // 机器人A在坐标系O中的坐标：
   
    Eigen::Vector4d A_in_coor_B;  // 求机器人A在机器人B中的坐标：
    Eigen::Vector4d A_end_in_coor_B;  // 求机器人A在机器人B中的end坐标：
    // TODO
    A_in_coor_B = coor_O_to_coor_B * A_in_coor_O;
    A_end_in_coor_B = coor_O_to_coor_B * A_end_in_coor_O;
    Eigen::Matrix3d coor_A_to_coor_O;

    Eigen::Vector2f v1{ 1, 0};
    Eigen::Vector2f v2{A_end_in_coor_B(0)-A_in_coor_B(0), A_end_in_coor_B(1)-A_in_coor_B(1)};

    double dot = v1(0)*v2(0) + v1(1)*v2(1);      // dot product between [v1(0), v1(1)] and [v2(0), v2(y)]
    double det = v1(0)*v2(1) - v1(1)*v2(0);      // determinant
    double angle = atan2(det, dot);  // atan2(y, x) or atan2(sin, cos)    // end your code here

    cout << "The right answer is BA: 2 1 1.5708" << endl;
    cout << "Your answer is BA: " << v1.transpose() << endl;
    cout << "Your answer is BA: " << v2.transpose() << endl;
    cout << "Your answer is BA angle: " <<angle << endl;
}

void test2() {
 // 机器人B在坐标系O中的坐标：
    Eigen::Vector3d B(3, 4, M_PI);

    // 坐标系B到坐标O的转换矩阵：
    Eigen::Matrix3d TOB;
    TOB << cos(B(2)), -sin(B(2)), B(0),
           sin(B(2)),  cos(B(2)), B(1),
              0,          0,        1;

    // 坐标系O到坐标B的转换矩阵:
    Eigen::Matrix3d TBO = TOB.inverse();

    // 机器人A在坐标系O中的坐标：
    Eigen::Vector3d A(1, 3, -M_PI / 2);

    // 求机器人A在机器人B中的坐标：
    Eigen::Vector3d BA;
    // TODO 参照第一课PPT
    // start your code here (5~10 lines)
    Eigen::Matrix3d TOA;
    TOA << cos(A(2)), -sin(A(2)), A(0),
    sin(A(2)), cos(A(2)) , A(1),
    0,0,1;

    Eigen::Matrix3d TBA = TBO * TOA;
    BA[0] = TBA(0, 2);
    BA[1] = TBA(1, 2);
    BA[2] = std::atan2(TBA(0, 1), TBA(0, 0));


    // end your code here

    cout << "The right answer is BA: 2 1 1.5708" << endl;
    cout << "Your answer is BA: " << BA.transpose() << endl;


}


int main(int argc, char** argv) {
  test1();
  // test2();
  return 0;
}


