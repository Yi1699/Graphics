#include <memory>
#include <spdlog/spdlog.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/sinks/basic_file_sink.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <cmath>
#include <iostream>
#include "formatter.hpp"
#define P 2213515299    //设置学号

using Eigen::Matrix2d;
using Eigen::Vector2d;
using std::cout;
Vector2d fgrad(Vector2d x)
{
    Vector2d xy;
    xy << 2.0*x(0), 2.0*x(1);
    return xy;
}   //梯度向量

int main()
{
    auto console_sink = std::make_shared<spdlog::sinks::stdout_color_sink_st>();
    auto file_sink = std::make_shared<spdlog::sinks::basic_file_sink_st>("optimizer.log", true);

    Vector2d x(P%827, P%1709);
    Vector2d x1(P%827, P%1709);
    Matrix2d hessian;
    hessian << 2.0, 0.0, 0.0, 2.0;
    double t = 0.5;
    double level = 0.01;
    
    spdlog::logger my_logger("Point logger", {console_sink, file_sink});
    my_logger.set_level(spdlog::level::debug);

    do{
        x=x1;
        x1 = x - t * hessian.inverse() * fgrad(x);
        my_logger.debug("{}", x);
    }while((x1 - x).norm() >= level);
    my_logger.info("{}", pow(x(0), 2.0) + pow(x(1), 2.0));
    return 0;
}
