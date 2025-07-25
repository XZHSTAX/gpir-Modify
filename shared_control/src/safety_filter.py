import osqp
import numpy as np
from scipy import sparse

def cbf_filter(u_star, p_ego, p_other):
    """安全过滤器，基于控制障碍函数（CBF）确保安全。

    Args:
        u_star (np.array): 原始控制输入，u_star = [v_des, omega_mix]。
        p_ego (np.array): 自车中心点的坐标，p_ego = [x_ego, y_ego, psi_ego]。
        p_other (np.array): 二维列表，包含场景中所有其他车辆的中心点坐标，
                              p_other = [[x1, y1, psi1], [x2, y2, psi2], ...]。

    Returns:
        np.array: 经过安全滤波后的控制量 u = [v, omega]。
    """
    l = 2.875 / 4.0
    d_safe = 2.875 / 2.0 + 0.5
    w_v = 1.0
    w_omega = 1.0
    alpha_gamma = 1.0

    # 1. 计算自车和障碍物车辆的圆心
    s_values = [0, l, -l]
    ego_circles = np.array([[p_ego[0] + s * np.cos(p_ego[2]), p_ego[1] + s * np.sin(p_ego[2])] for s in s_values])

    # 2. 构建QP问题
    P = sparse.csc_matrix([[w_v, 0], [0, w_omega]])
    q = np.array([-w_v * u_star[0], -w_omega * u_star[1]])

    constraints = []
    for other_veh in p_other:
        other_circles = np.array([[other_veh[0] + s * np.cos(other_veh[2]), other_veh[1] + s * np.sin(other_veh[2])] for s in s_values])
        for i in range(len(ego_circles)):
            for j in range(len(other_circles)):
                p_i = ego_circles[i]
                p_j = other_circles[j]
                s_i = s_values[i]

                # 计算 A_ij
                A_ij_row1 = 2 * (p_i[0] - p_j[0]) * np.cos(p_ego[2]) + 2 * (p_i[1] - p_j[1]) * np.sin(p_ego[2])
                A_ij_row2 = 2 * (p_i[0] - p_j[0]) * (-s_i * np.sin(p_ego[2])) + 2 * (p_i[1] - p_j[1]) * (s_i * np.cos(p_ego[2]))
                A_ij = np.array([A_ij_row1, A_ij_row2])

                # 计算 b_ij
                b_ij = alpha_gamma * (np.linalg.norm(p_i - p_j)**2 - d_safe**2)

                # osqp 约束格式: l <= Ax <= u
                # 我们有 A_ij u + b_ij >= 0  =>  -A_ij u <= b_ij
                # 所以 A = -A_ij, u = b_ij, l = -inf
                constraints.append((-A_ij, b_ij))

    if not constraints:
        return u_star

    A_matrix = sparse.csc_matrix(np.vstack([c[0] for c in constraints]))
    u_bound = np.array([c[1] for c in constraints])
    l_bound = np.full_like(u_bound, -np.inf)

    # 3. 求解QP
    prob = osqp.OSQP()
    prob.setup(P, q, A_matrix, l_bound, u_bound, verbose=False)
    res = prob.solve()

    if res.info.status == 'solved':
        return res.x
    else:
        # 如果求解失败，返回原始控制量
        return u_star

if __name__ == '__main__':
    # 测试用例
    u_star_test = np.array([10.0, 0.1])
    p_ego_test = np.array([0.0, 0.0, 0.0])
    p_other_test = np.array([[10.0, 0.5, 0.0]])

    u_safe = cbf_filter(u_star_test, p_ego_test, p_other_test)
    print(f"原始控制: {u_star_test}")
    print(f"安全控制: {u_safe}")

    # 另一个测试用例：障碍物在正前方很近的位置
    p_other_test_2 = np.array([[3.0, 0.0, 0.0]])
    u_safe_2 = cbf_filter(u_star_test, p_ego_test, p_other_test_2)
    print(f"\n场景2：障碍物在前方")
    print(f"原始控制: {u_star_test}")
    print(f"安全控制: {u_safe_2}")