#include "../include/calib_odom/Odom_Calib.hpp"


//设置数据长度,即多少数据计算一次
void OdomCalib::Set_data_len(int len)
{
    data_len = len;
    A.conservativeResize(len*3,9);
    b.conservativeResize(len*3);
    A.setZero();
    b.setZero();
}


/*
输入:里程计和激光数据

TODO:
构建最小二乘需要的超定方程组
Ax = b

*/
bool OdomCalib::Add_Data(Eigen::Vector3d Odom,Eigen::Vector3d scan)
{

    if(now_len<INT_MAX)
    {
        // TODO: 构建超定方程组
        A.block(3 * now_len + 0, 0, 1, 3) = Odom.transpose().head(3);
        A.block(3 * now_len + 1, 3, 1, 3) = Odom.transpose().head(3);
        A.block(3 * now_len + 2, 6, 1, 3) = Odom.transpose().head(3);

        b.block(now_len*3,0,3, 1) = scan.head(3);
        //end of TODO
        now_len++;
        return true;
    }
    else
    {
        return false;
    }
}

/*
 * TODO:
 * 求解线性最小二乘Ax=b
 * 返回得到的矫正矩阵
*/
Eigen::Matrix3d OdomCalib::Solve()
{
    Eigen::Matrix3d correct_matrix;

    //TODO: 求解线性最小二乘
    Eigen::VectorXd x2 = A.matrix().colPivHouseholderQr().solve(b.matrix());
    correct_matrix << x2[0], x2[1], x2[2],
                      x2[3], x2[4], x2[5],
                      x2[6], x2[7], x2[8] ;
    //end of TODO

    return correct_matrix;
}

/* 用于判断数据是否满
 * 数据满即可以进行最小二乘计算
*/
bool OdomCalib::is_full()
{
    if(now_len%data_len==0&&now_len>=1)
    {
        now_len = data_len;
        return true;
    }
    else
        return false;
}

/*
 * 数据清零
*/
void OdomCalib::set_data_zero()
{
    A.setZero();
    b.setZero();
}
