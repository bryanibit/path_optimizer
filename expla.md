1. Blue line of referenceline is conneced to direction of start and end state
2. Smoothed referenceline is connected with obstacles and direction.
3. Smooth trajectories cost function is used with three or four 项（与障碍物距离、平滑度等）

reference_path_:是PathOptimizer成员变量，是“path_optimizer.solve(reference_path, &result_path)”后的结果：
1.reference_path_smoother->solve：即经过bspline插值，smooth，DP和postSmooth后的结果
2.segmentSmoothedPath
3.optimizePath(final_path)
最后一步主要优化的是result_path，对应绿色线，reference_path_:黑色的线


reference_points:手动选择的红点
std::vector<double> x_list_, y_list_, s_list_:是b-spline插值后的点，在reference_path_smoother.hpp里
在reference_path_smoother.hpp中函数segmentRawReference通过使用b-spline的结果（s_list_, x_list_, y_list_），将s的间距变为1.0后重新生成x_list和y_list，并计算curvature（k_list），heading（angle of referenceline）

在TensionSmoother::smooth函数中，x_list, y_list, s_list, angle_list, k_list是根据上面的s_list_, x_list_, y_list_计算heading和curvature得到的

在TensionSmoother::smooth()函数中，x_list_，y_list_,s_list_（ReferencePathSmooth类成员）这几个变量又被osqp后的ref点重新赋值，osqp是通过上面x_list, y_list, s_list, angle_list, k_list这些量优化的。
也就是x_list_, y_list_等，先存储bspline的结果，后存osqp优化后的结果。


path_optimizer::solve()由一下函数构成
ReferencePathSmoother->solve()   segmentSmoothedPath()  optimizePath(final_path)
         |
         |
        \/
bSpline()  smooth() graphSearchDp()  postSmooth()
