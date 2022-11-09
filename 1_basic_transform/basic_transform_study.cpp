// 理解里程计中不同位置机器人坐标变换

#include <iostream>

#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Geometry"

using namespace std;

// 右手坐标系，四指方向(逆时针)为正旋转方向
void test1() {
    Eigen::Vector3d B_in_coor_O(3, 4, M_PI);  // 机器人B在坐标系O中的坐标：

    // 坐标系B到坐标O的转换矩阵：
    Eigen::Matrix4d coor_B_to_coor_O;
    // O->B旋转角度与B->O相反， 坐标系旋转参考https://blog.csdn.net/u012686154/article/details/88854386
    double theta = -B_in_coor_O(2);  
    coor_B_to_coor_O << cos(theta),   sin(theta),   0, B_in_coor_O(0),
                        -sin(theta),  cos(theta),   0, B_in_coor_O(1),
                                0,        0,        1,       0,
                                0,        0,        0,       1;

    Eigen::Matrix4d coor_O_to_coor_B = Eigen::Matrix4d::Identity();  // 坐标系O到坐标B的转换矩阵:
    coor_O_to_coor_B.topLeftCorner(3, 3) = coor_B_to_coor_O.topLeftCorner(3, 3).inverse();
    coor_O_to_coor_B.topRightCorner(3, 1) =  -1 * coor_B_to_coor_O.topLeftCorner(3, 3).inverse() *
                                                 coor_B_to_coor_O.topRightCorner(3, 1);   

    Eigen::Vector4d A_in_coor_O(1, 3, -M_PI / 2, 1);  // 机器人A在坐标系O中的齐次坐标
   
    Eigen::Vector4d A_in_coor_B;  // 求机器人A在机器人B中的坐标：
    // TODO
    A_in_coor_B = coor_O_to_coor_B * A_in_coor_O;  // 利用坐标旋转关系求解坐标点，更容易理解。

    double angle =  A_in_coor_O(2) - B_in_coor_O(2);  // A相对B的变化： angleA - angleB

    cout << "The right answer is BA: 2 1 1.5708" << endl;
    cout << "Your answer is BA: " << A_in_coor_B(0) << " " << A_in_coor_B(1) << " " << angle<< endl;
}

// 参考答案， 感觉不太好理解，实际验证的角度结果与test1相反
// 查询发现 slam中默认顺时针为正？ 课程讲的真的糊
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

   // 轨迹推算方式求解
     Eigen::Matrix3d R;
    R << cos(B(2)), -sin(B(2)), 0,
         sin(B(2)),  cos(B(2)), 0,
         0      , 0           , 1;
    //  解算
    Eigen::Vector3d d_pos = R.inverse() * (A - B);


    cout << "The right answer is BA: 2 1 1.5708" << endl;
    cout << "Your answer is BA: " << BA.transpose() << endl;
    cout << "Your answer is d_pos: " << d_pos.transpose() << endl;
}


int main(int argc, char** argv) {
  test1();
  test2();
  return 0;
}


